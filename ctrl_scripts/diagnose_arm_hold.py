#!/usr/bin/env python3
"""Monitor RoArm feedback while checking hold stability.

Run this with slave_arm/slave_armSimple stopped because it opens the same
serial port exclusively.
"""

import argparse
import json
import time

import serial


FEEDBACK_COMMAND = b'{"T":105}\n'
TORQUE_ON_COMMAND = b'{"T":210,"cmd":1}\n'
TORQUE_KEYS = ("torswitchB", "torswitchS", "torswitchE", "torswitchH")


def read_json_line(ser, deadline):
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        try:
            value = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if isinstance(value, dict):
            return value
    return None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--recover", action="store_true",
                        help="send T=210 when a torque switch reports 0")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval < 0.05:
        raise SystemExit("--interval must be at least 0.05 seconds")

    min_voltage = None
    last_position = None
    last_torque = None
    missed = 0
    with serial.Serial(args.port, 115200, timeout=0.05, exclusive=True) as ser:
        ser.reset_input_buffer()
        print("time voltage min_voltage torque x y z t dz warning", flush=True)
        try:
            while True:
                started = time.monotonic()
                ser.write(FEEDBACK_COMMAND)
                ser.flush()
                feedback = read_json_line(ser, started + args.interval)
                warning = []
                if not feedback or feedback.get("T") != 1051:
                    missed += 1
                    print(f"{time.strftime('%H:%M:%S')} feedback_missed={missed}",
                          flush=True)
                else:
                    missed = 0
                    voltage = feedback.get("v")
                    voltage_text = "?"
                    if isinstance(voltage, (int, float)):
                        voltage = voltage / 100.0
                        min_voltage = voltage if min_voltage is None else min(min_voltage, voltage)
                        voltage_text = f"{voltage:.2f}"
                        if voltage < 10.5:
                            warning.append("LOW_VOLTAGE")

                    torque = tuple(feedback.get(key) for key in TORQUE_KEYS)
                    if any(value == 0 for value in torque):
                        warning.append("TORQUE_OFF")
                        if args.recover and torque != last_torque:
                            ser.write(TORQUE_ON_COMMAND)
                            ser.flush()
                            warning.append("T210_SENT")
                    last_torque = torque

                    position = {
                        key: feedback.get(key) for key in ("x", "y", "z", "t")
                        if isinstance(feedback.get(key), (int, float))
                    }
                    dz = None
                    if last_position and "z" in position and "z" in last_position:
                        dz = float(position["z"]) - float(last_position["z"])
                        if dz < -2.0:
                            warning.append("Z_DROPPING")
                    if position:
                        last_position = position
                    print(
                        f"{time.strftime('%H:%M:%S')} {voltage_text} "
                        f"{'?' if min_voltage is None else f'{min_voltage:.2f}'} "
                        f"{torque} {position.get('x', '?')} {position.get('y', '?')} "
                        f"{position.get('z', '?')} {position.get('t', '?')} "
                        f"{'?' if dz is None else f'{dz:.3f}'} "
                        f"{','.join(warning)}",
                        flush=True,
                    )
                remaining = args.interval - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            print("stopped", flush=True)


if __name__ == "__main__":
    main()
