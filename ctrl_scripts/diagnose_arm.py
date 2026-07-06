#!/usr/bin/env python3
"""RoArm-M2-S의 전압과 torque-lock 상태를 실시간 진단한다."""

import argparse
import json
import time

import serial


FEEDBACK_COMMAND = b'{"T":105}\n'
TORQUE_ON_COMMAND = b'{"T":210,"cmd":1}\n'
TORQUE_KEYS = ("torswitchB", "torswitchS", "torswitchE", "torswitchH")


def parse_args():
    parser = argparse.ArgumentParser(
        description="UGV 동작 중 RoArm 전압/torque 상태 감시"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument(
        "--recover",
        action="store_true",
        help="torque switch가 0이면 torque ON 명령을 한 번 전송",
    )
    return parser.parse_args()


def read_json_line(ser, deadline):
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        try:
            value = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if value.get("T") == 1051:
            return value
    return None


def main():
    args = parse_args()
    if args.interval < 0.1:
        raise SystemExit("--interval은 0.1초 이상이어야 합니다.")

    minimum_voltage = None
    last_torque_state = None
    missed = 0

    with serial.Serial(
        args.port,
        baudrate=115200,
        timeout=0.1,
        exclusive=True,
    ) as ser:
        ser.reset_input_buffer()
        print("팔 감시 시작. 이제 다른 터미널에서 UGV를 움직이세요. 종료: Ctrl+C")

        try:
            while True:
                started = time.monotonic()
                ser.write(FEEDBACK_COMMAND)
                ser.flush()
                feedback = read_json_line(ser, started + args.interval)

                if feedback is None:
                    missed += 1
                    print(
                        f"{time.strftime('%H:%M:%S')} 응답 없음 ({missed}회 연속) "
                        "- arm 제어기 재부팅/USB 끊김 가능"
                    )
                else:
                    missed = 0
                    voltage = feedback.get("v")
                    voltage = voltage / 100.0 if isinstance(voltage, (int, float)) else None
                    if voltage is not None:
                        minimum_voltage = (
                            voltage
                            if minimum_voltage is None
                            else min(minimum_voltage, voltage)
                        )

                    torque_state = tuple(feedback.get(key) for key in TORQUE_KEYS)
                    warning = []
                    if any(value == 0 for value in torque_state):
                        warning.append("TORQUE OFF 감지")
                        if args.recover and torque_state != last_torque_state:
                            ser.write(TORQUE_ON_COMMAND)
                            ser.flush()
                            warning.append("torque ON 재전송")
                    if voltage is not None and voltage < 10.5:
                        warning.append("전압 강하")

                    voltage_text = "?" if voltage is None else f"{voltage:.2f}V"
                    minimum_text = "?" if minimum_voltage is None else f"{minimum_voltage:.2f}V"
                    print(
                        f"{time.strftime('%H:%M:%S')} voltage={voltage_text} "
                        f"min={minimum_text} torque={torque_state}"
                        + ("  !!! " + ", ".join(warning) if warning else "")
                    )
                    last_torque_state = torque_state

                remaining = args.interval - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            print("\n감시 종료")


if __name__ == "__main__":
    main()
