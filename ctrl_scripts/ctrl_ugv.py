import serial
import threading
import time

# ctrl_rovering2.py의 설정값 사용
SERIAL_PORT = '/dev/ttyS0'
BAUDRATE = 115200 #
SERIAL_TIMEOUT = 1  # 시리얼 작업 타임아웃 (초)
# 같은 명령을 20 ms마다 계속 쓰면 UART/MCU 부하만 늘어납니다. 명령이 바뀔 때
# 즉시 보내고, 연결 확인용 keep-alive만 낮은 주기로 보냅니다.
KEEPALIVE_INTERVAL = 0.5

# 전역 변수
# ctrl_rovering2.py와 유사하게 정지 명령으로 초기화
current_command = '{"T":0}'
command_lock = threading.Lock() # current_command 접근 동기화를 위한 Lock
exit_event = threading.Event()  # 스레드 종료 신호를 위한 Event

def cmd_parser(cmd_input_str):
    """
    사용자 입력 문자열을 파싱하여 로버의 JSON 명령어 형식으로 변환합니다.
    예상 입력: "go <L_SPEED> <R_SPEED>" 또는 "stop" [ctrl_ugv.py와 유사한 CLI 입력 스타일을 따름, cite: 2]
    속도값은 실수여야 합니다 (예: 0.5, -0.3).
    JSON 명령어 문자열 또는 None (입력값이 유효하지 않은 경우)을 반환합니다.
    """
    parts = cmd_input_str.strip().lower().split()
    
    if not parts:
        print("[정보] 입력된 명령어가 없습니다.")
        return None

    cmd_type = parts[0]

    if cmd_type == "stop":
        return '{"T":0}' # 유효한 JSON 형식의 정지 명령
    elif cmd_type == "go":
        if len(parts) == 3:
            try:
                l_speed = float(parts[1])
                r_speed = float(parts[2])
                # 로버 펌웨어가 속도 제한/스케일링을 처리해야 함
                # ctrl_rovering2.py에서 생성된 명령어 형식과 일치
                return f'{{"T":1,"L":{l_speed},"R":{r_speed}}}'
            except ValueError:
                print("[오류] 잘못된 속도값입니다. 속도는 숫자여야 합니다 (예: 'go 0.5 -0.5').")
                return None
        else:
            print("[오류] 잘못된 'go' 명령어 형식입니다. 사용법: 'go <L_SPEED> <R_SPEED>'.")
            return None
    else:
        print(f"[오류] 알 수 없는 명령어: '{cmd_type}'. 사용 가능한 명령어: 'go L R', 'stop', 'quit'.")
        return None

def execute_command_thread():
    """
    현재 명령어를 로버에 지속적으로 전송하는 스레드 함수입니다.
    시리얼 연결을 열고 관리합니다.
    각 명령어 전송 시 개행 문자가 추가됩니다. [ctrl_rovering2.py의 전송 방식 참조, cite: 1]
    종료 시 정지 명령어를 전송합니다.
    """
    global current_command # 이 스레드는 current_command를 읽음
    uart = None

    try:
        uart = serial.Serial(
            SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=SERIAL_TIMEOUT,
            exclusive=True,
        )
        print(f"[정보] 시리얼 포트 {SERIAL_PORT}가 {BAUDRATE} baud로 성공적으로 열렸습니다.")
    except serial.SerialException as e:
        print(f"[오류] 시리얼 포트 {SERIAL_PORT}를 여는 데 실패했습니다: {e}")
        exit_event.set()  # 다른 스레드에 종료 신호
        return

    last_sent_command = None
    last_sent_time = 0.0

    while not exit_event.is_set():
        command_to_send_on_this_loop = None # 스레드 안전성을 위한 로컬 복사본
        with command_lock:
            command_to_send_on_this_loop = current_command # 항상 현재 명령어를 가져옴

        # current_command는 "{T:0}"으로 초기화되므로, 이 명령어는
        # 사용자가 새 명령을 입력할 때까지 계속 전송됩니다.
        
        now = time.monotonic()
        should_send = (
            command_to_send_on_this_loop != last_sent_command
            or now - last_sent_time >= KEEPALIVE_INTERVAL
        )

        try:
            if should_send:
                uart.write(command_to_send_on_this_loop.encode() + b'\n')
                uart.flush()
                last_sent_command = command_to_send_on_this_loop
                last_sent_time = now
            # 디버깅을 위해 매번 전송되는 명령어를 출력하고 싶다면 아래 줄의 주석을 해제하세요.
            # print(f"[디버그] 전송됨: {command_to_send_on_this_loop}")

        except serial.SerialException as e:
            print(f"[오류] 시리얼 쓰기 실패: {e}")
            exit_event.set()
            break
        except Exception as e:
            print(f"[오류] 시리얼 쓰기 중 예상치 못한 오류 발생: {e}")
            exit_event.set()
            break
        
        # 명령어 반복 전송 간격. ctrl_rovering2.py의 값을 따름
        # ctrl_ugv.py는 0.1초 간격이었음
        time.sleep(0.02)


    # 이 스레드의 종료 절차
    if uart and uart.is_open:
        print("[정보] execute_command_thread: 종료 중. 로버에 최종 정지 명령어를 전송합니다.")
        try:
            final_stop_cmd = '{"T":0}'
            uart.write(final_stop_cmd.encode() + b'\n') #
            uart.flush()
            print(f"[정보] 최종 정지 명령어 '{final_stop_cmd}'가 전송되었습니다.")
            time.sleep(0.05)
        except serial.SerialException as se:
            print(f"[오류] 최종 정지 명령어 전송 실패: {se}")
        finally:
            print(f"[정보] 시리얼 포트 {SERIAL_PORT}를 닫습니다.")
            uart.close() # (두 파일 모두 uart 종료 로직 포함)
    elif uart and not uart.is_open:
         print("[경고] UART 객체는 존재하지만, 스레드 종료 시 포트가 열려있지 않았습니다.")
    else:
        print("[정보] 시리얼 포트가 열리지 않았으므로, 이 스레드에서 최종 정지 명령어를 보낼 필요가 없습니다.")


def input_command_main_loop():
    """
    CLI를 통해 사용자로부터 명령어 입력을 받는 메인 루프입니다 (메인 스레드에서 실행).
    전역 'current_command' 변수를 업데이트합니다.
    'quit', EOF, 또는 Ctrl+C 입력 시 'exit_event'를 설정합니다.
    """
    global current_command

    print("\n로버 커맨드 라인 인터페이스")
    print("----------------------------")
    print("로버에 명령어를 입력하세요:")
    print("  - 'go <왼쪽_속도> <오른쪽_속도>' (예: 'go 0.5 0.5' 또는 'go 0.3 -0.3')")
    print("  - 'stop' (로버 정지: 'go 0.0 0.0'과 동일)")
    print("  - 'quit' (프로그램 종료)")
    print("----------------------------")

    while not exit_event.is_set():
        try:
            user_input = input("RoverCmd> ") # [ctrl_ugv.py의 input 방식, cite: 2]
            cleaned_input = user_input.strip().lower()

            if cleaned_input == 'quit':
                print("[정보] 'quit' 명령어가 수신되었습니다. 종료를 시작합니다...")
                exit_event.set()
                break

            parsed_cmd = cmd_parser(user_input)

            if parsed_cmd:
                with command_lock:
                    current_command = parsed_cmd
                    print(f"[정보] 명령어가 다음으로 업데이트되었습니다: {current_command}")

        except EOFError:
            print("\n[정보] EOF가 감지되었습니다 (Ctrl+D). 종료를 시작합니다...")
            exit_event.set()
            break
        except KeyboardInterrupt:
            print("\n[정보] 키보드 인터럽트가 감지되었습니다 (Ctrl+C). 종료를 시작합니다...")
            exit_event.set()
            break
        except Exception as e:
            print(f"[오류] 입력 루프 중 예상치 못한 오류 발생: {e}")
            exit_event.set()
            break

if __name__ == "__main__":
    print("[정보] 로버 CLI 컨트롤러 프로그램을 시작합니다...")
    print("[정보] 팔을 함께 사용할 때는 torque watchdog이 포함된 ctrl_arm.py를 실행하세요.")

    # 명령어 실행 스레드 시작 (ctrl_ugv.py와 ctrl_rovering2.py 모두 스레드 사용)
    executor = threading.Thread(target=execute_command_thread)
    executor.start()

    input_command_main_loop()

    if not exit_event.is_set():
        print("[정보] 메인 입력 루프가 종료되었으므로, 다른 스레드를 위해 exit_event를 설정합니다.")
        exit_event.set()

    print("[정보] 명령어 실행 스레드가 종료될 때까지 대기 중...")
    executor.join(timeout=3.0) # [ctrl_ugv.py와 ctrl_rovering2.py 모두 join 사용, cite: 1, 2]

    if executor.is_alive():
        print("[경고] 명령어 실행 스레드가 타임아웃 후에도 정상적으로 종료되지 않았습니다.")
    else:
        print("[정보] 명령어 실행 스레드가 종료되었습니다.")

    print("[정보] 프로그램이 종료되었습니다.") # [ctrl_ugv.py와 ctrl_rovering2.py 모두 종료 메시지 출력, cite: 1, 2]
