# vcrobot control scripts

This directory contains robot-side and master-side control scripts for synchronized
RoArm and mobile-base control across multiple vcrobot units.

The synchronized scripts use a TCP prepare/commit protocol. The master estimates
each slave clock offset, sends a future execution timestamp, waits until every
slave is ready, and then commits the command. The goal is to make all robots
start the same motion as closely together as the local network and controller
latency allow.

## Hardware ports

Default serial devices:

- RoArm: `/dev/ttyUSB0`
- UGV / CartRider base: `/dev/ttyS0`

Each slave opens its serial port in exclusive mode. Do not run two scripts that
try to use the same serial device at the same time. For example,
`slave_ugv.py` and `slave_cartRider.py` both use `/dev/ttyS0`, so they should not
run together. `slave_armSimple.py` can run together with either drive slave
because it uses `/dev/ttyUSB0`.

## Arm control

Run one arm slave on each robot:

```bash
cd ~/ctrl_scripts
python3 slave_arm.py
```

Or use the simple `x y z t` interface:

```bash
cd ~/ctrl_scripts
python3 slave_armSimple.py
```

From the master PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/ctrl_scripts
python3 master_armSimple.py --robots 192.168.10.203 192.168.10.206
```

Interactive input format:

```text
100 0 480 4.1
```

One-shot command:

```bash
python3 master_armSimple.py \
  --robots 192.168.10.203 192.168.10.206 \
  --position 100 0 480 4.1
```

The full JSON arm master also accepts RoArm JSON commands:

```bash
python3 master_arm.py --robots 192.168.10.203 192.168.10.206
```

Example JSON command:

```json
{"T":104,"x":100,"y":0,"z":480,"t":4.1,"spd":0.75}
```

`slave_arm.py` includes an arm torque watchdog. It sends periodic `T=210`
torque-keepalive commands while idle, but suppresses the keepalive briefly after
motion commands so that it does not disturb in-flight RoArm motion.

Useful options:

```bash
python3 slave_armSimple.py --torque-quiet-after-command 1.5
python3 slave_armSimple.py --torque-keepalive-interval 0.2
python3 slave_armSimple.py --defa-restore-interval 0
```

By default, the slave exits when the master disconnects. To keep the slave
listening for another master connection:

```bash
python3 slave_armSimple.py --keep-alive
```

## Arm hold diagnostics

Stop `slave_arm.py` / `slave_armSimple.py` before running this diagnostic because
it opens `/dev/ttyUSB0` exclusively.

```bash
cd ~/ctrl_scripts
python3 diagnose_arm_hold.py --interval 0.1 --recover
```

Watch for:

- `LOW_VOLTAGE`: likely power sag
- `TORQUE_OFF`: one or more servo torque switches are off
- `Z_DROPPING`: the arm height is dropping while it should hold position
- repeated `feedback_missed`: unstable serial/controller communication

If a robot arm drops or jitters only while the drive base is moving, check the
arm power supply, ground wiring, and voltage sag before assuming a software
timing problem.

## UGV drive control

Run the UGV slave on each robot:

```bash
cd ~/ctrl_scripts
python3 slave_ugv.py
```

From the master PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/ctrl_scripts
python3 master_ugv.py --robots 192.168.10.203 192.168.10.206
```

Interactive commands:

```text
go 0.1 0.1
run 3 0.1 0.1
stop
quit
```

The drive slave sends an emergency stop when the master disconnects or when the
TCP heartbeat times out.

## CartRider control

Run one CartRider slave on each robot:

```bash
cd ~/ctrl_scripts
python3 slave_cartRider.py
```

From the master PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/ctrl_scripts
python3 master_cartRider.py --robots 192.168.10.203 192.168.10.206
```

The master uses a curses-style keyboard interface similar to the original
`ctrl_cartRider.py`:

- `w`: forward
- `s`: backward
- `a`: left
- `d`: right
- `space`: stop
- `q`: quit

CartRider mode includes ramping and reverse dead-time to reduce current spikes
when changing direction.

## Clock modes

Default mode uses TCP round-trip measurements to estimate each slave clock
offset:

```bash
python3 master_armSimple.py --clock-mode estimated
```

If all robots and the master PC are configured with PTP and `CLOCK_TAI`, strict
TAI scheduling can be used:

```bash
python3 master_armSimple.py \
  --robots 192.168.10.203 192.168.10.206 \
  --clock-mode ptp --ptp-max-offset-ms 0.5
```

Use PTP mode only when system time synchronization has been verified.

## Safety notes

These scripts directly command robot arms and mobile bases. Test with low speed,
clear surroundings, and emergency power access. The code is provided for research
use and does not guarantee safe behavior on every robot, power supply, or network
configuration.
