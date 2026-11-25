#include <Arduino.h>
#include "driver/pulse_cnt.h"

// --- Configuration & Constants ---
#define LOOP_FREQ_HZ 50
#define DT 0.02f
#define TICKS_PER_REV 660.0f
#define RAD_PER_TICK ( (2.0f * PI) / TICKS_PER_REV )

// L298N PWM Resolution (8-bit)
#define PWM_RES 8
#define PWM_FREQ 20000
#define MAX_PWM 255.0f

// --- Motor & Encoder Pin Assignments ---
const int PIN_ENA[4] = {19, 18, 16, 17};
const int PIN_IN1[4] = {5, 13, 23, 14};
const int PIN_IN2[4] = {15, 12, 4, 2};
const int PIN_ENC_A[4] = {36, 35, 32, 27};
const int PIN_ENC_B[4] = {39, 34, 25, 26};

// --- Servo (MG995, open loop) ---
// Wiring: servo signal on the pins below; power the servos from a SEPARATE 5-6 V
// supply (an MG995 can draw >1 A when stalled) and join its GND to the ESP32 GND.
#define NUM_SERVOS 3
#define SERVO_FREQ 50
#define SERVO_RES 16                 // 16-bit duty at 50 Hz -> ~0.3 us resolution
#define SERVO_PERIOD_US 20000UL
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
const int PIN_SERVO[NUM_SERVOS] = {21, 22, 33};
// Explicit LEDC channels 8..10 (low-speed group): separate timers from the 20 kHz
// motor PWM, which lives on the auto-allocated channels 0..3.
const int SERVO_LEDC_CH[NUM_SERVOS] = {8, 9, 10};

// --- Link Watchdog ---
// If no valid packet (any type) arrives for LINK_TIMEOUT_MS, every motor PWM and
// servo pulse is forced to zero until the host talks to us again. Keep this in
// line with the host's comm_timeout_ms (default 500).
#define LINK_TIMEOUT_MS 500

// --- Thread Safety ---
SemaphoreHandle_t state_mutex;
float target_velocities[4] = {0, 0, 0, 0}; // rad/s
float current_velocities[4] = {0, 0, 0, 0}; // rad/s
uint32_t last_rx_ms = 0;     // millis() of the last valid packet (guarded by state_mutex)
bool link_lost = true;       // true until the first valid packet (guarded by state_mutex)

// --- Servo Output ---
class ServoOutput {
private:
    int pin;
public:
    explicit ServoOutput(int p_pin, int channel) : pin(p_pin) {
        ledcAttachChannel(pin, SERVO_FREQ, SERVO_RES, channel);
        ledcWrite(pin, 0);  // no pulse until the host commands one
    }
    // 0 = output off (servo limp); otherwise pulse width clamped to 500..2500 us.
    void setPulseUs(uint16_t us) {
        if (us == 0) { ledcWrite(pin, 0); return; }
        if (us < SERVO_MIN_US) us = SERVO_MIN_US;
        if (us > SERVO_MAX_US) us = SERVO_MAX_US;
        ledcWrite(pin, (uint32_t)(((uint64_t)us << SERVO_RES) / SERVO_PERIOD_US));
    }
    void release() { ledcWrite(pin, 0); }
};

ServoOutput* servos[NUM_SERVOS];

// --- Serial TX Payload (Unchanged) ---
struct __attribute__((packed)) TxSerialPacket {
    uint8_t header[2] = {0xAA, 0x55};
    float velocities[4];
    uint16_t checksum;
};

// --- Motor Controller Class ---
class MotorController {
private:
    pcnt_unit_handle_t pcnt_unit;
    int ena, in1, in2;

    // PID Parameters
    float Kff = 0.0f;
    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float max_integral = 40.0f; // Clamping for anti-windup

    // State
    float integral = 0.0f;
    float prev_meas = 0.0f;
    float filtered_vel = 0.0f;
    const float alpha = 0.6f;

public:
    MotorController(int p_ena, int p_in1, int p_in2, int enc_a, int enc_b) :
        ena(p_ena), in1(p_in1), in2(p_in2) {

        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        ledcAttach(ena, PWM_FREQ, PWM_RES);
        ledcWrite(ena, 0);

        pcnt_unit_config_t unit_config = { .low_limit = -32768, .high_limit = 32767 };
        pcnt_new_unit(&unit_config, &pcnt_unit);

        pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
        pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

        pcnt_chan_config_t chan_config = { .edge_gpio_num = enc_a, .level_gpio_num = enc_b };
        pcnt_channel_handle_t pcnt_chan = NULL;
        pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan);

        pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_unit_enable(pcnt_unit);
        pcnt_unit_clear_count(pcnt_unit);
        pcnt_unit_start(pcnt_unit);
    }

    void setGains(float ff, float p, float i, float d, float clamp) {
        Kff = ff; Kp = p; Ki = i; Kd = d; max_integral = clamp;
    }

    void updatePID(float p, float i, float d) {
        Kp = p; Ki = i; Kd = d;
    }

    float update(float target_rad_s, bool enabled = true) {
        int raw_ticks = 0;
        pcnt_unit_get_count(pcnt_unit, &raw_ticks);
        pcnt_unit_clear_count(pcnt_unit);

        float raw_vel = (raw_ticks * RAD_PER_TICK) / DT;
        filtered_vel = (alpha * raw_vel) + ((1.0f - alpha) * filtered_vel);

        if (!enabled) {
            // Watchdog tripped: H-bridge off, PWM 0, PID state reset (no integral kick
            // on recovery). Velocity is still measured so telemetry stays truthful.
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            ledcWrite(ena, 0);
            integral = 0.0f;
            prev_meas = filtered_vel;
            return filtered_vel;
        }

        float error = target_rad_s - filtered_vel;

        // 1. Accumulate the actual PWM effort (Bumpless Transfer)
        integral += Ki * error * DT;

        // 2. Anti-Windup now strictly clamps the maximum PWM contribution
        if (integral > max_integral) integral = max_integral;
        if (integral < -max_integral) integral = -max_integral;

        float derivative = -(filtered_vel - prev_meas) / DT;
        prev_meas = filtered_vel;

        // 3. Remove Ki from this line, as it is already baked into 'integral'
        float output = (Kff * target_rad_s) + (Kp * error) + integral + (Kd * derivative);

        // L298N Deadband Compensation
        float final_pwm = 0.0f;
        if (output > 0.1f) {
            final_pwm = output + 45.0f;
        } else if (output < -0.1f) {
            final_pwm = output - 45.0f;
        }

        if (final_pwm > MAX_PWM) final_pwm = MAX_PWM;
        if (final_pwm < -MAX_PWM) final_pwm = -MAX_PWM;

        if (final_pwm >= 0) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            ledcWrite(ena, (uint32_t)final_pwm);
        } else {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            ledcWrite(ena, (uint32_t)-final_pwm);
        }

        return filtered_vel;
    }
};

MotorController* motors[4];

// --- Core 1: Strict Real-Time PID Loop ---
void pidControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / LOOP_FREQ_HZ);

    float local_targets[4] = {0, 0, 0, 0};
    bool lost = true;  // fail safe if the mutex is never obtained

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
            // Watchdog: runs here (core 1) so it still fires if the comms task stalls.
            if (!link_lost && (uint32_t)(millis() - last_rx_ms) > LINK_TIMEOUT_MS) {
                link_lost = true;
            }
            if (link_lost) {
                // Latched until the next valid packet. Zero the setpoints and
                // release the servos (re-asserted every cycle; it is idempotent).
                memset(target_velocities, 0, sizeof(target_velocities));
                for (int i = 0; i < NUM_SERVOS; i++) servos[i]->release();
            }
            lost = link_lost;
            memcpy(local_targets, target_velocities, sizeof(target_velocities));
            xSemaphoreGive(state_mutex);
        }

        float local_currents[4];
        for (int i = 0; i < 4; i++) {
            local_currents[i] = motors[i]->update(lost ? 0.0f : local_targets[i], !lost);
        }

        if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
            memcpy(current_velocities, local_currents, sizeof(current_velocities));
            xSemaphoreGive(state_mutex);
        }
    }
}

// --- Core 0: Serial Communications ---
uint16_t computeChecksum(const uint8_t* data, size_t length) {
    uint16_t sum1 = 0, sum2 = 0;
    for (size_t i = 0; i < length; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}

// Waits (max 10 ms) until `n` bytes are buffered. Returns false on timeout.
static bool waitForBytes(int n) {
    uint32_t t0 = millis();
    while (Serial.available() < n && (millis() - t0 < 10)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return Serial.available() >= n;
}

// Marks the link alive. Caller must hold state_mutex.
static inline void markLinkAlive() {
    last_rx_ms = millis();
    link_lost = false;
}

// Reads and handles ONE host packet if a header is pending. Packets (all start AA 55,
// checksum = Fletcher-16 over type + payload):
//   0x01 velocity : 4 x float rad/s                  (21 B total)
//   0x02 PID gains: motor_id u8, kp, ki, kd floats   (18 B total)
//   0x03 servos   : 3 x uint16 pulse us, 0 = off     (11 B total)
static void handleHostPacket() {
    if (Serial.read() != 0xAA || Serial.peek() != 0x55) return;
    Serial.read(); // Consume 0x55
    uint8_t type = Serial.read();

    if (type == 0x01) {
        if (!waitForBytes(18)) return;
        uint8_t payload[16];
        Serial.readBytes((char*)payload, 16);
        uint16_t checksum;
        Serial.readBytes((char*)&checksum, 2);

        uint8_t crc_buf[17];
        crc_buf[0] = type;
        memcpy(&crc_buf[1], payload, 16);
        if (computeChecksum(crc_buf, 17) == checksum) {
            if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                memcpy(target_velocities, payload, 16);
                markLinkAlive();
                xSemaphoreGive(state_mutex);
            }
        }
    }
    else if (type == 0x02) {
        if (!waitForBytes(15)) return;
        uint8_t payload[13]; // MotorID (1) + 3x Floats (12)
        Serial.readBytes((char*)payload, 13);
        uint16_t checksum;
        Serial.readBytes((char*)&checksum, 2);

        uint8_t crc_buf[14];
        crc_buf[0] = type;
        memcpy(&crc_buf[1], payload, 13);
        if (computeChecksum(crc_buf, 14) == checksum) {
            if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                markLinkAlive();
                xSemaphoreGive(state_mutex);
            }
            uint8_t motor_id = payload[0];
            if (motor_id < 4) {
                float p, i, d;
                memcpy(&p, &payload[1], 4);
                memcpy(&i, &payload[5], 4);
                memcpy(&d, &payload[9], 4);
                motors[motor_id]->updatePID(p, i, d);
            }
        }
    }
    else if (type == 0x03) {
        if (!waitForBytes(8)) return;
        uint8_t payload[6]; // 3 x uint16 pulse width (us), little endian
        Serial.readBytes((char*)payload, 6);
        uint16_t checksum;
        Serial.readBytes((char*)&checksum, 2);

        uint8_t crc_buf[7];
        crc_buf[0] = type;
        memcpy(&crc_buf[1], payload, 6);
        if (computeChecksum(crc_buf, 7) == checksum) {
            // Applied under the mutex so it can't race the watchdog's release().
            if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                markLinkAlive();
                for (int s = 0; s < NUM_SERVOS; s++) {
                    uint16_t us;
                    memcpy(&us, &payload[2 * s], 2);
                    servos[s]->setPulseUs(us);
                }
                xSemaphoreGive(state_mutex);
            }
        }
    }
    // Unknown type: header + type already consumed; the parser re-syncs on the next 0xAA 0x55.
}

void serialCommsTask(void *pvParameters) {
    TxSerialPacket tx_packet;

    while (true) {
        // Drain every pending packet (host sends velocity + servo [+ gains] per cycle);
        // bounded so telemetry TX can't be starved by a flooding host.
        for (int n = 0; n < 8 && Serial.available() >= 3; n++) {
            handleHostPacket();
        }

        // Transmit current velocities
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            memcpy(tx_packet.velocities, current_velocities, sizeof(current_velocities));
            xSemaphoreGive(state_mutex);
        }

        tx_packet.checksum = computeChecksum((uint8_t*)tx_packet.velocities, 16);
        Serial.write((uint8_t*)&tx_packet, sizeof(TxSerialPacket));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(500000);
    state_mutex = xSemaphoreCreateMutex();
    last_rx_ms = millis();

    for (int i = 0; i < 4; i++) {
        motors[i] = new MotorController(PIN_ENA[i], PIN_IN1[i], PIN_IN2[i], PIN_ENC_A[i], PIN_ENC_B[i]);
        // Initial defaults match the requested Cruising Profile placeholders
        motors[i]->setGains(15.0f, 2.0f, 0.5f, 0.0f, 40.0f);
    }

    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i] = new ServoOutput(PIN_SERVO[i], SERVO_LEDC_CH[i]);
    }

    xTaskCreatePinnedToCore(serialCommsTask, "CommsTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(pidControlTask, "PIDTask", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}
