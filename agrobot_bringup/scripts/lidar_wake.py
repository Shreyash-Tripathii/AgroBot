#!/usr/bin/env python3
"""
lidar_wake.py -- bring an RPLidar A1 into a known-good state, then exec the real command.

    lidar_wake.py [--port /dev/ttyUSB0] [--baud 115200] [--timeout 90] -- sllidar_node --ros-args ...
    lidar_wake.py --off --port /dev/ttyUSB0          # stop the scan and the motor, then exit

Why: on the A1's CP2102 adapter the motor is switched by DTR (DTR low = motor on). After the motor is
switched on, or after the port is opened, the lidar's MCU answers nothing for 1.5 s up to several
seconds (spin-up / supply dip), while sllidar_node sends GET_DEVICE_INFO a few ms after opening the
port and exits with SL_RESULT_OPERATION_TIMEOUT (0x80008002) or SL_RESULT_INVALID_DATA (0x80008000).
So this wrapper does what the driver does not:

  1. open with the motor OFF, settle 2 s, STOP any scan left over from a crashed driver
  2. motor ON, query GET_DEVICE_INFO until --stable complete, valid answers arrive in a row
  3. check GET_HEALTH; on a protection stop / error / long silence: RESET the lidar and repeat
  4. leave the motor running (HUPCL cleared) and exec the driver, which then finds a ready lidar

It always ends by exec'ing the command (even if the lidar never woke up), so the driver's own error
messages and the caller's restart logic still apply.
"""
import argparse
import glob
import os
import sys
import termios
import time

import serial

STOP, RESET, GET_INFO, GET_HEALTH = b'\xA5\x25', b'\xA5\x40', b'\xA5\x50', b'\xA5\x52'
INFO_HDR = b'\xA5\x5A\x14\x00\x00\x00\x04'
HEALTH_HDR = b'\xA5\x5A\x03\x00\x00\x00\x06'


def log(msg):
    print(f'[lidar_wake] {msg}', file=sys.stderr, flush=True)


def ask(ser, cmd, header, payload_len, wait):
    """Send cmd; return the payload of a complete response, or None."""
    ser.reset_input_buffer()
    ser.write(cmd)
    buf, end = b'', time.monotonic() + wait
    while time.monotonic() < end:
        buf += ser.read(64)
        i = buf.find(header)
        if i >= 0 and len(buf) >= i + len(header) + payload_len:
            return buf[i + len(header):i + len(header) + payload_len]
    return None


def wake(port, baud, timeout, stable):
    """True once `stable` GET_INFO queries in a row are answered with the motor running."""
    deadline = time.monotonic() + timeout
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.05
    ser.dtr, ser.rts = True, False            # DTR high = motor OFF
    ser.exclusive = True
    ser.open()
    try:
        time.sleep(2.0)                       # the lidar is mute for ~1.5 s after the DTR/RTS edges of open()
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            ser.dtr = True                    # motor off
            ser.write(STOP)                   # end a scan left over from a crashed driver
            time.sleep(0.1)
            t0 = time.monotonic()
            alive = None
            while time.monotonic() < min(deadline, t0 + 8.0) and alive is None:
                alive = ask(ser, GET_INFO, INFO_HDR, 20, 0.3)
            if alive is None:
                log(f'attempt {attempt}: silent for 8 s with the motor off, RESET')
                ser.write(RESET)
                time.sleep(4.0)
                continue
            ser.dtr = False                   # motor on
            t0 = time.monotonic()
            good, tries, info = 0, 0, None
            while time.monotonic() < min(deadline, t0 + 20.0) and good < stable:
                r = ask(ser, GET_INFO, INFO_HDR, 20, 0.3)
                tries += 1
                if r is None:
                    good = 0
                    time.sleep(0.2)
                else:
                    good, info = good + 1, r
                    time.sleep(0.15)
            if good >= stable:
                health = ask(ser, GET_HEALTH, HEALTH_HDR, 3, 1.0)
                if health is not None and health[0] == 0:
                    log(f'attempt {attempt}: ready after {time.monotonic() - t0:.1f} s with the motor on '
                        f'({tries} queries, fw {info[2]}.{info[1]:02d}, hw {info[3]})')
                    ser.write(STOP)
                    time.sleep(0.05)
                    return True
                log(f'attempt {attempt}: health status {None if health is None else health.hex()}, RESET')
                ser.dtr = True
                ser.write(RESET)
                time.sleep(4.0)
            else:
                log(f'attempt {attempt}: not stable with the motor on ({tries} queries), '
                    f'motor off for 3 s and again')
                ser.dtr = True
                time.sleep(3.0)
        return False
    finally:
        try:                                  # keep DTR (motor) as is when the port closes
            attrs = termios.tcgetattr(ser.fd)
            attrs[2] &= ~termios.HUPCL
            termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
        except Exception:
            pass
        ser.close()


def detect_port(baud, timeout, window=6.0):
    """Find the RPLidar among /dev/ttyUSB*: motor off, settle, repeated GET_INFO. None if absent."""
    deadline = time.monotonic() + timeout
    while True:
        for port in sorted(glob.glob('/dev/ttyUSB*')):
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout = port, baud, 0.05
            ser.dtr, ser.rts, ser.exclusive = True, False, True
            try:
                ser.open()
                time.sleep(1.5)
                end = time.monotonic() + window
                while time.monotonic() < end:
                    if ask(ser, GET_INFO, INFO_HDR, 20, 0.4) is not None:
                        return port
            except (serial.SerialException, OSError):
                pass
            finally:
                if ser.is_open:
                    ser.close()
        if time.monotonic() >= deadline:
            return None
        time.sleep(1.0)


def motor_off(port, baud=115200):
    """Stop scanning and switch the motor off (DTR high), e.g. after a manual stop."""
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        ser.dtr, ser.rts = True, False
        ser.write(STOP)
        time.sleep(0.1)
        ser.close()
    except (serial.SerialException, OSError):
        pass


def main():
    argv = sys.argv[1:]
    cmd = []
    if '--' in argv:
        i = argv.index('--')
        argv, cmd = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--stable', type=int, default=3, help='consecutive good answers required')
    ap.add_argument('--off', action='store_true', help='just stop the lidar and its motor, then exit')
    a = ap.parse_args(argv)
    if a.off:
        motor_off(a.port, a.baud)
        return
    try:
        ok = wake(a.port, a.baud, a.timeout, a.stable)
        log('lidar ready' if ok else f'WARNING: lidar did not wake within {a.timeout:.0f} s')
    except (serial.SerialException, OSError) as e:
        log(f'WARNING: {e}')
    if cmd:
        os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    main()
