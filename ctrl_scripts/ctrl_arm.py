import serial
import threading
import time


TORQUE_ON_COMMAND = '{"T":210,"cmd":1}\n'
DEFA_OFF_COMMAND = (
    '{"T":112,"mode":0,"b":1000,"s":1000,"e":1000,"h":1000}\n'
)
TORQUE_KEEPALIVE_INTERVAL = 0.2

def open_serial(port, baudrate, timeout):
    try:
        # 동일 포트를 다른 제어 프로그램이 동시에 여는 사고를 즉시 감지합니다.
        ser = serial.Serial(
            port, baudrate=baudrate, timeout=timeout, exclusive=True
        )
        print("Port opened successfully.")
        return ser
    except Exception as e:
        print(f"Failed to open port: {e}")
        exit()

def write_command(ser, command, write_lock):
    with write_lock:
        ser.write(command.encode())
        ser.flush()


def torque_watchdog(ser, write_lock, stop_event):
    """UGV 동작 중에도 RoArm torque lock을 계속 유지한다."""
    while not stop_event.wait(TORQUE_KEEPALIVE_INTERVAL):
        try:
            write_command(ser, TORQUE_ON_COMMAND, write_lock)
        except (serial.SerialException, OSError) as exc:
            print(f"\nTorque watchdog stopped: {exc}")
            stop_event.set()
            return


def send_command(ser, write_lock, command):
    if not command.endswith("\n"):
        command += "\n"  # 명령어 끝에 개행 문자 추가
    write_command(ser, command, write_lock)
    print(f"Sent command: {repr(command)}")

    response = ser.readline()  # 응답 읽기
    print(f"Raw response: {response}")
    if response:
        try:
            print(f"Decoded response: {response.decode('utf-8')}")
        except UnicodeDecodeError:
            print(f"Non-UTF-8 response: {response}")
    else:
        print("No response received.")

def main():
    # Serial 포트 설정
    port = "/dev/ttyUSB0"  # 연결된 포트
    baudrate = 115200      # Baudrate 설정
    timeout = 2            # Timeout 설정
    ser = open_serial(port, baudrate, timeout)
    write_lock = threading.Lock()
    watchdog_stop = threading.Event()

    # 기존의 DEFA/torque limit 설정을 해제하고 torque lock을 계속 유지한다.
    write_command(ser, DEFA_OFF_COMMAND, write_lock)
    write_command(ser, TORQUE_ON_COMMAND, write_lock)
    watchdog = threading.Thread(
        target=torque_watchdog,
        args=(ser, write_lock, watchdog_stop),
        daemon=True,
    )
    watchdog.start()
    print("Torque watchdog enabled (200 ms). DEFA disabled.")

    # 명령 루프 시작
    while True:
        try:
            user_input = input("\nInput command (or type 'quit' to exit): ")
            if user_input.lower() == "quit":
                print("Exiting...")
                break

            # 명령어 전송
            send_command(ser, write_lock, user_input)
            time.sleep(0.5)  # 대기 시간 추가
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print(f"Error during communication: {e}")

    watchdog_stop.set()
    watchdog.join(timeout=1.0)

    # Serial 포트 닫기
    try:
        ser.close()
        print("Serial port closed.")
    except Exception as e:
        print(f"Error closing serial port: {e}")

if __name__ == "__main__":
    main()
