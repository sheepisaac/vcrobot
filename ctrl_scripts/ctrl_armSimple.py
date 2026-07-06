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
        print("✅ Serial port opened successfully.")
        return ser
    except Exception as e:
        print(f"❌ Failed to open serial port: {e}")
        exit()

def write_command(ser, command, write_lock):
    with write_lock:
        ser.write(command.encode())
        ser.flush()


def torque_watchdog(ser, write_lock, stop_event):
    """다른 동작 중 torque lock이 풀려도 즉시 다시 활성화한다."""
    while not stop_event.wait(TORQUE_KEEPALIVE_INTERVAL):
        try:
            write_command(ser, TORQUE_ON_COMMAND, write_lock)
        except (serial.SerialException, OSError) as exc:
            print(f"\n❌ Torque watchdog stopped: {exc}")
            stop_event.set()
            return


def send_command(ser, write_lock, x, y, z, t, spd=0.75):
    cmd = f'{{"T":104,"x":{x},"y":{y},"z":{z},"t":{t},"spd":{spd}}}\n'
    write_command(ser, cmd, write_lock)
    print(f"➡️ Sent command: {cmd.strip()}")

    response = ser.readline()
    if response:
        try:
            print("⬅️ Response:", response.decode('utf-8').strip())
        except UnicodeDecodeError:
            print("⚠️ Non-UTF8 response:", response)

def main():
    port = "/dev/ttyUSB0"  # 필요 시 ttyAMA0 으로 변경
    baudrate = 115200
    timeout = 2
    ser = open_serial(port, baudrate, timeout)
    write_lock = threading.Lock()
    watchdog_stop = threading.Event()

    # 저장되어 있던 외력 적응/낮은 torque limit을 해제하고 torque lock을 켠다.
    write_command(ser, DEFA_OFF_COMMAND, write_lock)
    write_command(ser, TORQUE_ON_COMMAND, write_lock)
    watchdog = threading.Thread(
        target=torque_watchdog,
        args=(ser, write_lock, watchdog_stop),
        daemon=True,
    )
    watchdog.start()
    print("🔒 Torque watchdog enabled (200 ms). DEFA disabled.")

    print("🦾 Input x, y, z, t values to control the arm.")
    print("📌 Format: x y z t (e.g., 100 0 480 4.1)")
    print("⛔ Type 'quit' to exit.")

    while True:
        try:
            user_input = input("\nInput x, y, z, t: ")
            if user_input.strip().lower() == 'quit':
                print("👋 Exiting...")
                break

            parts = user_input.strip().split()
            if len(parts) != 4:
                print("⚠️ Please enter exactly 4 values (x y z t).")
                continue

            x, y, z, t = map(float, parts)
            send_command(ser, write_lock, x, y, z, t)

        except KeyboardInterrupt:
            print("\n👋 Interrupted. Exiting...")
            break
        except Exception as e:
            print(f"❌ Error: {e}")

    watchdog_stop.set()
    watchdog.join(timeout=1.0)

    try:
        ser.close()
        print("🔌 Serial port closed.")
    except Exception as e:
        print(f"⚠️ Error closing serial port: {e}")

if __name__ == "__main__":
    main()
