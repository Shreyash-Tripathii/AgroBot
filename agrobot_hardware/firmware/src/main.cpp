#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

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

// --- Comms ---
#define RX_BUF_SIZE 512              // host -> ESP32 reassembly buffer (frames <= 21 B)
#define COMMS_POLL_MS 2              // RX poll period (low command latency)
#define TELEMETRY_PERIOD_US 10000LL  // wheel telemetry stays at 100 Hz
#define STATUS_PERIOD_US 1000000LL   // diagnostics frame (AA 56) at 1 Hz
#define STATUS_VERSION 1             // 1 = understands 0x03 servo frames

// --- Thread Safety ---
SemaphoreHandle_t state_mutex;
float target_velocities[4] = {0, 0, 0, 0}; // rad/s
float current_velocities[4] = {0, 0, 0, 0}; // rad/s
uint32_t last_rx_ms = 0;     // millis() of the last valid packet (guarded by state_mutex)
bool link_lost = true;       // true until the first valid packet (guarded by state_mutex)
uint16_t servo_us_cmd[NUM_SERVOS] = {0, 0, 0}; // latest host pulse widths, 0 = off (state_mutex)

// Diagnostics reported in the status frame. Each is written by ONE task only.
volatile uint16_t stat_watchdog_trips = 0;     // PID task
volatile uint16_t stat_pid_mutex_misses = 0;   // PID task
volatile uint16_t stat_pid_max_period_us = 0;  // PID task, max wake-to-wake time of the last second

// --- Servo Output ---
class ServoOutput {
private:
    int pin;
    uint16_t last_us = 0;   // last pulse written (0 = off): LEDC is only touched on change
public:
    explicit ServoOutput(int p_pin, int channel) : pin(p_pin) {
        ledcAttachChannel(pin, SERVO_FREQ, SERVO_RES, channel);
        ledcWrite(pin, 0);  // no pulse until the host commands one
    }
    // 0 = output off (servo limp); otherwise pulse width clamped to 500..2500 us.
    void setPulseUs(uint16_t us) {
        if (us != 0) {
            if (us < SERVO_MIN_US) us = SERVO_MIN_US;
            if (us > SERVO_MAX_US) us = SERVO_MAX_US;
        }
        if (us == last_us) return;
        last_us = us;
        ledcWrite(pin, us == 0 ? 0 : (uint32_t)(((uint64_t)us << SERVO_RES) / SERVO_PERIOD_US));
    }
    void release() { setPulseUs(0); }
};

ServoOutput* servos[NUM_SERVOS];

// --- Serial TX Payload (Unchanged) ---
struct __attribute__((packed)) TxSerialPacket {
    uint8_t header[2] = {0xAA, 0x55};
    float velocities[4];
    uint16_t checksum;
};

// --- Diagnostics frame, ESP32 -> host, 1 Hz (23 B). Own header (AA 56) so the AA 55
// telemetry stream is untouched. Checksum = Fletcher-16 over the 19 bytes after the header.
struct __attribute__((packed)) StatusPacket {
    uint8_t header[2] = {0xAA, 0x56};
    uint8_t version = STATUS_VERSION;
    uint32_t uptime_ms;
    uint16_t watchdog_trips;      // cumulative
    uint16_t rx_frames;           // cumulative valid host frames
    uint16_t rx_bad_checksum;     // cumulative
    uint16_t rx_skipped_bytes;    // cumulative
    uint16_t max_pass_us;         // longest RX parse pass in the last second
    uint16_t max_pid_period_us;   // longest PID wake-to-wake gap in the last second (20000 nominal)
    uint16_t pid_mutex_misses;    // cumulative
    uint16_t checksum;
};
static_assert(sizeof(StatusPacket) == 23, "StatusPacket must be 23 bytes");

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
    uint16_t local_servo[NUM_SERVOS] = {0, 0, 0};
    bool lost = true;  // fail safe if the mutex is never obtained

    int64_t last_wake_us = esp_timer_get_time();
    uint16_t window_max_period = 0;
    int window_cycles = 0;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Timing diagnostics: wake-to-wake gap, max over each 1 s window.
        int64_t now_us = esp_timer_get_time();
        int64_t period_us = now_us - last_wake_us;
        last_wake_us = now_us;
        uint16_t p = period_us > 65535 ? 65535 : (uint16_t)period_us;
        if (p > window_max_period) window_max_period = p;
        if (++window_cycles >= LOOP_FREQ_HZ) {
            stat_pid_max_period_us = window_max_period;
            window_max_period = 0;
            window_cycles = 0;
        }

        if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
            // Watchdog: runs here (core 1) so it still fires if the comms task stalls.
            if (!link_lost && (uint32_t)(millis() - last_rx_ms) > LINK_TIMEOUT_MS) {
                link_lost = true;
                stat_watchdog_trips = stat_watchdog_trips + 1;
            }
            if (link_lost) {
                // Latched until the next valid packet: zero the setpoints and the servo
                // commands (so stale pulses can't resume without a fresh 0x03 frame).
                memset(target_velocities, 0, sizeof(target_velocities));
                memset(servo_us_cmd, 0, sizeof(servo_us_cmd));
            }
            lost = link_lost;
            memcpy(local_targets, target_velocities, sizeof(target_velocities));
            memcpy(local_servo, servo_us_cmd, sizeof(servo_us_cmd));
            xSemaphoreGive(state_mutex);
        } else {
            stat_pid_mutex_misses = stat_pid_mutex_misses + 1;
        }

        // All LEDC servo writes happen here (never under the mutex, never in the comms
        // task). setPulseUs() only touches the hardware when the pulse changes.
        for (int i = 0; i < NUM_SERVOS; i++) {
            servos[i]->setPulseUs(lost ? 0 : local_servo[i]);
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

// Marks the link alive. Caller must hold state_mutex.
static inline void markLinkAlive() {
    last_rx_ms = millis();
    link_lost = false;
}

// Host -> ESP32 frame sizes (all start AA 55, Fletcher-16 over type + payload):
//   0x01 velocity : 4 x float rad/s                  (21 B total)
//   0x02 PID gains: motor_id u8, kp, ki, kd floats   (18 B total)
//   0x03 servos   : 3 x uint16 pulse us, 0 = off     (11 B total)
static size_t frameSize(uint8_t type) {
    switch (type) {
        case 0x01: return 21;
        case 0x02: return 18;
        case 0x03: return 11;
        default:   return 0;
    }
}

// Reassembly buffer + counters (comms task only).
static uint8_t rx_buf[RX_BUF_SIZE];
static size_t rx_len = 0;
static uint16_t rx_frames = 0, rx_bad_ck = 0, rx_skipped = 0, pass_max_us = 0;

// Non-blocking: moves whatever the UART has into rx_buf, parses every complete frame
// (junk is skipped in one sweep, partial frames wait for the next pass, nothing
// sleeps), then commits only the NEWEST velocity / servo frame under the mutex so a
// backlog can never replay stale commands.
static void processRx() {
    int64_t t0 = esp_timer_get_time();

    int avail = Serial.available();
    if (avail > 0) {
        size_t space = RX_BUF_SIZE - rx_len;
        size_t n = (size_t)avail < space ? (size_t)avail : space;
        rx_len += Serial.readBytes((char*)rx_buf + rx_len, n);
    }

    float latest_vel[4];
    uint16_t latest_servo[NUM_SERVOS];
    bool got_vel = false, got_servo = false, got_any = false;

    size_t pos = 0;
    while (rx_len - pos >= 3) {
        if (rx_buf[pos] != 0xAA || rx_buf[pos + 1] != 0x55) { pos++; rx_skipped++; continue; }
        uint8_t type = rx_buf[pos + 2];
        size_t n = frameSize(type);
        if (n == 0) { pos += 2; rx_skipped += 2; continue; }   // unknown type: re-sync
        if (rx_len - pos < n) break;                           // partial frame: wait

        uint16_t checksum;
        memcpy(&checksum, &rx_buf[pos + n - 2], 2);
        if (computeChecksum(&rx_buf[pos + 2], n - 4) != checksum) {
            rx_bad_ck++;  // false header inside a payload, or corruption: slip one byte
            pos++;
            rx_skipped++;
            continue;
        }

        const uint8_t* payload = &rx_buf[pos + 3];
        if (type == 0x01) {
            memcpy(latest_vel, payload, 16);
            got_vel = true;
        } else if (type == 0x02) {
            uint8_t motor_id = payload[0];
            if (motor_id < 4) {
                float p, i, d;
                memcpy(&p, &payload[1], 4);
                memcpy(&i, &payload[5], 4);
                memcpy(&d, &payload[9], 4);
                motors[motor_id]->updatePID(p, i, d);
            }
        } else {  // 0x03
            memcpy(latest_servo, payload, sizeof(latest_servo));
            got_servo = true;
        }
        got_any = true;
        rx_frames++;
        pos += n;
    }

    if (pos > 0) {
        rx_len -= pos;
        memmove(rx_buf, rx_buf + pos, rx_len);
    }

    if (got_any && xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        if (got_vel) memcpy(target_velocities, latest_vel, sizeof(latest_vel));
        if (got_servo) memcpy(servo_us_cmd, latest_servo, sizeof(latest_servo));
        markLinkAlive();
        xSemaphoreGive(state_mutex);
    }

    int64_t dt = esp_timer_get_time() - t0;
    uint16_t d = dt > 65535 ? 65535 : (uint16_t)dt;
    if (d > pass_max_us) pass_max_us = d;
}

void serialCommsTask(void *pvParameters) {
    TxSerialPacket tx_packet;
    int64_t last_tx_us = 0, last_status_us = 0;

    while (true) {
        processRx();

        int64_t now = esp_timer_get_time();

        // Wheel telemetry at 100 Hz (unchanged format)
        if (now - last_tx_us >= TELEMETRY_PERIOD_US) {
            last_tx_us = now;
            if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                memcpy(tx_packet.velocities, current_velocities, sizeof(current_velocities));
                xSemaphoreGive(state_mutex);
            }
            tx_packet.checksum = computeChecksum((uint8_t*)tx_packet.velocities, 16);
            Serial.write((uint8_t*)&tx_packet, sizeof(TxSerialPacket));
        }

        // Diagnostics + capability announcement at 1 Hz
        if (now - last_status_us >= STATUS_PERIOD_US) {
            last_status_us = now;
            StatusPacket st;
            st.uptime_ms = (uint32_t)(now / 1000);
            st.watchdog_trips = stat_watchdog_trips;
            st.rx_frames = rx_frames;
            st.rx_bad_checksum = rx_bad_ck;
            st.rx_skipped_bytes = rx_skipped;
            st.max_pass_us = pass_max_us;
            st.max_pid_period_us = stat_pid_max_period_us;
            st.pid_mutex_misses = stat_pid_mutex_misses;
            pass_max_us = 0;
            st.checksum = computeChecksum((uint8_t*)&st + 2, sizeof(StatusPacket) - 4);
            Serial.write((uint8_t*)&st, sizeof(StatusPacket));
        }

        vTaskDelay(pdMS_TO_TICKS(COMMS_POLL_MS));
    }
}

void setup() {
    Serial.setRxBufferSize(1024);  // must precede begin(); headroom only
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
