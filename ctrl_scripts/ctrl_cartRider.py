import serial
import threading
import time
import curses

uart = None

fixed_speed = 0.05
# 모터의 즉시 역전은 큰 피크 전류를 만듭니다. 20 ms마다 이 값만큼 속도를
# 올리고, 정지/역전 사이에는 아래 시간만큼 0 명령을 유지합니다.
speed_ramp_step = 0.005
reverse_deadtime = 0.35
key_states = {'w': False, 'a': False, 's': False, 'd': False}
exit_event = threading.Event()
command_lock = threading.Lock()
target_speeds = (0.0, 0.0)

def target_from_keys():
    L, R = 0.0, 0.0

    if key_states['w']:
        L, R = fixed_speed, fixed_speed
    elif key_states['s']:
        L, R = -fixed_speed, -fixed_speed
    elif key_states['a']:
        L, R = -fixed_speed, fixed_speed
    elif key_states['d']:
        L, R = fixed_speed, -fixed_speed

    return L, R


def generate_command(left, right):
    if left == 0.0 and right == 0.0:
        # Ordinary stop only; T=0 also releases bus-servo torque on UGV.
        return '{"T":1,"L":0.0,"R":0.0}'
    return f'{{"T":1,"L":{left:.3f},"R":{right:.3f}}}'


def move_toward(current, target, step):
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current

def execute_command():
    prev_command = None  # Track last command sent
    current_left = 0.0
    current_right = 0.0
    reverse_allowed_at = time.monotonic()

    while not exit_event.is_set():
        try:
            with command_lock:
                target_left, target_right = target_speeds

            now = time.monotonic()
            stopped = target_left == 0.0 and target_right == 0.0
            reversing = (
                current_left * target_left < 0
                or current_right * target_right < 0
            )

            if stopped or reversing:
                # 정지 명령은 감속 ramp보다 안전을 우선해 즉시 보냅니다.
                current_left = current_right = 0.0
                reverse_allowed_at = now + reverse_deadtime
            elif now >= reverse_allowed_at:
                current_left = move_toward(
                    current_left, target_left, speed_ramp_step
                )
                current_right = move_toward(
                    current_right, target_right, speed_ramp_step
                )

            command = generate_command(current_left, current_right)
            if command != prev_command:
                uart.write(command.encode() + b'\n')
                uart.flush()
                prev_command = command
        except serial.SerialException as e:
            print(f"[ERROR] Serial write failed: {e}")
            exit_event.set()
            break

        time.sleep(0.02)

def input_command(stdscr):
    global target_speeds
    stdscr.nodelay(True)
    stdscr.keypad(True)
    stdscr.clear()
    stdscr.addstr("Use WASD to move, 'q' to quit.\n")

    hold_timeout = 1  # Hold movement state
    last_key_time = time.time()
    last_key_pressed = None

    while not exit_event.is_set():
        key = stdscr.getch()
        now = time.time()

        if key in (ord('w'), ord('a'), ord('s'), ord('d')):
            key_states.update({
                'w': key == ord('w'),
                'a': key == ord('a'),
                's': key == ord('s'),
                'd': key == ord('d'),
            })
            last_key_time = now
            last_key_pressed = key
        elif key == ord('q'):
            exit_event.set()
            break
        elif key == -1:
            if now - last_key_time > hold_timeout:
                key_states.update({'w': False, 'a': False, 's': False, 'd': False})
        else:
            # Unknown key — stop movement
            key_states.update({'w': False, 'a': False, 's': False, 'd': False})

        with command_lock:
            target_speeds = target_from_keys()

        time.sleep(0.02)


if __name__ == "__main__":
    # 다른 프로세스가 같은 UART를 실수로 함께 여는 것을 막습니다.
    uart = serial.Serial(
        '/dev/ttyS0', baudrate=115200, timeout=1, exclusive=True
    )
    executor_thread = threading.Thread(target=execute_command, daemon=True)
    executor_thread.start()

    curses.wrapper(input_command)

    exit_event.set()
    executor_thread.join()
    uart.write(b'{"T":0}\n')
    uart.flush()
    uart.close()
    print("Program terminated.")
