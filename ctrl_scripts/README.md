# UGV와 arm 동시 구동 주의사항

- UGV는 `/dev/ttyS0`, arm은 `/dev/ttyUSB0`을 사용한다. 두 포트를 바꾸어
  연결하지 않는다.
- 각 스크립트는 포트를 exclusive 모드로 연다. 같은 포트를 두 프로그램이
  사용하면 두 번째 프로그램은 즉시 오류로 종료한다.
- `ctrl_ugv.py`는 변경된 명령을 즉시 보내고 0.5초 간격으로만 keep-alive를
  보낸다. 이전의 20ms 무한 송신으로 인한 불필요한 UART/MCU 부하를 제거했다.
- `ctrl_cartRider.py`는 속도를 단계적으로 올리고, 전진/후진 또는 좌/우 역전
  사이에 0.35초 정지 구간을 넣어 모터의 순간 역전 전류를 줄인다.
- `ctrl_arm.py`와 `ctrl_armSimple.py`는 시작 시 DEFA와 기존 torque limit을
  해제하고, 공식 torque-lock ON 명령을 0.2초마다 보내 팔의 torque가 풀리지
  않게 유지한다.

## 팔이 여전히 힘을 잃는 경우

UGV와 arm은 코드상 서로 다른 통신 포트를 쓴다. UGV 모터가 출발하거나
방향을 바꿀 때 arm 서보가 힘을 잃거나 재부팅된다면 원인은 통신 충돌이
아니라 순간 전압 강하일 가능성이 높다.

1. 바퀴를 바닥에서 띄우고 두 프로그램만 켠다. 이때 정상이고 바퀴에 부하를
   걸 때만 arm이 풀리면 전원 문제로 확정할 수 있다.
2. arm 서보 전원을 UGV 구동 모터 전원과 분리한다. 전압은 반드시 해당
   서보 사양에 맞추고, 제어기의 GND끼리만 공통으로 연결한다.
3. 전원을 분리할 수 없다면 정격 전압에서 모터의 stall current와 모든
   서보의 stall current 합보다 여유 있는 전원/BEC를 사용한다. 굵고 짧은
   전원선을 쓰고 arm 전원 입력 가까이에 제조사 권장 벌크 커패시터를 둔다.
4. 멀티미터보다 오실로스코프로 arm 전원 입력의 순간 최저 전압을 확인하는
   것이 정확하다. 서보/제어기 저전압 기준 아래로 내려가면 하드웨어 전원을
   보강해야 하며, 소프트웨어만으로 안전하게 해결할 수 없다.

아래 명령으로 팔 자체가 보고하는 전압과 torque switch를 확인할 수 있다.
진단 중에는 `ctrl_arm.py`/`ctrl_armSimple.py`를 동시에 실행하지 않는다.

```bash
python3 ~/ctrl_scripts/diagnose_arm.py
```

그 상태에서 다른 터미널로 UGV 제어 코드를 실행하고 문제를 재현한다.

- `voltage`가 크게 떨어지거나 `응답 없음`이 나오면 전원 강하/제어기 재부팅이다.
- 전압은 유지되면서 torque 값이 `(0, ...)`으로 변하면 torque-off 명령 또는
  설정 문제다. 원인 확인 후 임시 자동 복구는 `--recover` 옵션으로 시험한다.

## 여러 로봇 arm 동기 제어

각 로봇에서 기존 `ctrl_arm.py`를 종료한 뒤 아래 서버를 실행한다. 같은
`/dev/ttyUSB0`을 exclusive 모드로 사용하므로 두 프로그램을 동시에 실행하면
안 된다.

```bash
python3 ~/ctrl_scripts/slave_arm.py
```

서버 PC의 마스터는 기본적으로 `192.168.10.0/24`의 TCP `50210` 포트를
검색하고, `slave_arm` 프로토콜 응답을 보내는 로봇 2대를 자동 등록한다.

```bash
python ctrl_scripts/master_arm.py
```

다른 서브넷을 검색하거나 IP를 직접 지정할 수도 있다.

```bash
python3 ctrl_scripts/master_arm.py --discover-subnet 192.168.20.0/24
python3 ctrl_scripts/master_arm.py --robots 192.168.10.203 192.168.10.206
```

프롬프트에 RoArm JSON을 입력하면 두 로봇에 예약 전송된다. 시작과 종료
명령을 정확히 같은 간격으로 예약하려면 다음 형식을 쓴다.

```text
run 2.0 {"T":104,"x":100,"y":0,"z":480,"t":4.1,"spd":0.75} || {"T":104,"x":100,"y":0,"z":300,"t":4.1,"spd":0.75}
```

네트워크와 타이밍만 시험할 때는 로봇에서 `--dry-run`을 사용한다. 기본 TCP
포트는 `50210`이다. 방화벽에서 서버 PC가 이 포트로 접근할 수 있어야 한다.

### Simple 좌표 입력 모드

`ctrl_armSimple.py`처럼 `x y z t` 네 값만 입력하려면 각 로봇에서 다음을
실행한다.

```bash
python3 ~/ctrl_scripts/slave_armSimple.py
```

서버 PC에서는 다음을 실행하고 프롬프트에 `100 0 480 4.1` 형식으로 입력한다.

```bash
python3 ctrl_scripts/master_armSimple.py
```

기본 속도는 `0.75`이며 `--spd 0.5`처럼 바꿀 수 있다. 한 번만 실행할 수도
있다.

```bash
python3 ctrl_scripts/master_armSimple.py --position 100 0 480 4.1
```

`slave_arm.py`와 `slave_armSimple.py`는 같은 TCP 포트와 `/dev/ttyUSB0`을
사용하므로 동시에 실행하지 않는다. 두 Simple 파일도 공용 동기화 엔진을
사용하므로 clock sync, READY/COMMIT, 실행시각 보고 방식은 일반 모드와 같다.

Simple 모드는 기본적으로 명령 전송 후 다음 검증을 추가 수행한다.

1. `T=104` 직후 JSON 응답(직접 command ACK)을 수집한다.
2. `T=105`를 반복 전송해 `T=1051` 제어기 피드백을 확인한다.
3. 피드백의 `x/y/z/t`가 목표 허용오차 안에 연속 2회 들어와야 성공한다.

기본 위치 제한은 XYZ 8, tool angle 0.15이며 다음처럼 조절한다.

```bash
python3 master_armSimple.py --robots 192.168.10.203 192.168.10.206 \
  --xyz-tolerance 5 --t-tolerance 0.1 --verify-timeout 10
```

펌웨어가 `T=104` 직접 응답을 제공하는 것이 확인된 경우에만
`--require-command-ack`를 사용한다. 직접 ACK가 없어도 `T=1051` 피드백과 실제
목표 도달이 확인되면 기본 모드에서는 성공한다. 펌웨어 피드백에 Cartesian
좌표가 없다면 성공으로 간주하지 않고 `feedback_has_no_cartesian_position`과
실제 필드 목록을 출력한다.

PTP가 모든 장치에 외부에서 설정되어 system clock과 `CLOCK_TAI`가 동기화된
환경에서는 절대 TAI 시각 예약을 사용할 수 있다.

```bash
python3 master_armSimple.py --robots 192.168.10.203 192.168.10.206 \
  --clock-mode ptp --ptp-max-offset-ms 0.5
```

이 옵션은 `ptp4l`/`phc2sys`를 설치하거나 설정하지 않는다. 각 장치의 TAI
잔여 offset을 TCP로 측정해 기준을 넘으면 실행을 거부하며, 기준 안일 때만
동일한 절대 `CLOCK_TAI` 시각을 두 슬레이브에 전달한다.

## 여러 로봇 UGV 동기 이동

각 로봇에서 기존 `ctrl_ugv.py`/`ctrl_cartRider.py`를 종료하고 UGV 슬레이브를
실행한다. 로봇에는 `drive_sync.py`와 해당 slave 파일만 필요하다. TCP 포트는
`50220`, 시리얼은 `/dev/ttyS0`이다.

```bash
python3 ~/ctrl_scripts/slave_ugv.py
```

서버 PC에서 실행한다.

```bash
python3 master_ugv.py --robots 192.168.10.203 192.168.10.206
```

프롬프트 명령은 `go L R`, `stop`, `run SECONDS L R`이다. `run`은 시작과
정지를 모두 사전 예약하므로 두 로봇의 종료도 같은 시각에 수행된다.

```text
ugv-master> run 3 0.1 0.1
```

## 여러 로봇 CartRider 동기 이동

CartRider는 기존의 20ms ramp, 0.005 step, 역전 전 0.35초 정지를 유지한다.
TCP 포트는 `50221`이다.

```bash
# 각 로봇
python3 ~/ctrl_scripts/slave_cartRider.py

# 서버 PC
python3 master_cartRider.py --robots 192.168.10.203 192.168.10.206
```

원본과 동일한 curses 화면에서 `w`, `a`, `s`, `d`를 누른다. 키 입력이 1초
동안 없거나 알 수 없는 키를 누르면 정지하고, `q`로 정지 후 종료한다. 첫
UART step과 이후 20ms ramp 시간축은 공통 예약시각에 고정된다. 화면의
`UART skew`는 두 로봇의 실제 첫 nonzero 시리얼 전송시각 차이다.

두 이동 슬레이브는 0.2초 TCP heartbeat를 받고, 기본 0.8초 동안 heartbeat가
없거나 마스터 연결이 끊기면 즉시 `T=0`을 쓰고 예약된 미래 명령도 폐기한다.
한 슬레이브는 한 마스터에만 제어권을 준다. UGV와 CartRider는 같은
`/dev/ttyS0`을 사용하므로 동시에 실행하지 않는다. 네트워크만 시험할 때는
각 슬레이브에 `--dry-run`을 사용한다.

arm 슬레이브의 torque watchdog은 USB/arm 제어기가 순간 재부팅되어도 종료되지
않고 `/dev/ttyUSB0`을 다시 열어 `DEFA OFF`와 `TORQUE ON`을 재적용한다. arm과
UGV는 서로 다른 포트이므로 `slave_armSimple.py`와 `slave_ugv.py` 또는
`slave_cartRider.py`를 동시에 실행해야 torque 유지와 이동 제어가 함께 된다.
