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
run together. `slave_armSimple.py` uses `/dev/ttyUSB0`, but different Linux port
names alone do not prove the firmware/servo bus is independent.

## ArmSimple + CartRider torque-loss fix

Ordinary drive stops now send `{"T":1,"L":0.0,"R":0.0}`. Previously, drive
startup, idle keepalives, normal stops and direction reversal sent `{"T":0}`.
On official WAVESHARE UGV firmware, T=0 stops the wheels AND calls
`emergencyStopProcessing()`, broadcasting torque OFF to servo ID 254.
The drive's 0.5-second idle keepalive could therefore fight the arm's T=210
torque ON watchdog and repeatedly release/recover the arm.

Sources: [command handler](https://github.com/waveshareteam/ugv_base_ros/blob/main/ROS_Driver/uart_ctrl.h),
[servo implementation](https://github.com/waveshareteam/ugv_base_ros/blob/main/ROS_Driver/RoArm-M2_module.h).
Installed firmware and wiring still need robot-side confirmation.

Legacy T=0 ordinary commands from older drive masters are translated to the
wheel-only stop. Both drive masters import the stop constant through
`drive_master_sync.py`, so their interfaces are unchanged. Original standalone
`ctrl_cartRider.py` and `ctrl_ugv.py` ordinary stops are fixed as well.

**Safety behavior retained:** disconnect, heartbeat timeout and slave shutdown
still use the existing global T=0 emergency stop, which can release arm torque.
The existing arm watchdog recovery policy is also unchanged. Look for
`EMERGENCY STOP` in the drive terminal if release still occurs after updating.
This patch changes ordinary drive stops, not the fault/shutdown safety policy.

Drive state selection and UART writes are serialized so a stale background
movement packet cannot follow an emergency stop. The arm watchdog rechecks its
motion quiet window after acquiring the serial lock.

Deploy updated `drive_sync.py` and `slave_arm.py` to `~/ctrl_scripts/` on every
robot and restart both slaves. Update `drive_sync.py` on the PC too. Wrapper
scripts require no edits. Copy the original `ctrl_cartRider.py`/`ctrl_ugv.py`
if using those standalone controllers.

For the physical test, support the arm against unexpected drops, start armSimple,
then cartRider, first leave the wheels idle and then test normal stop/reverse.
No robot is moved by the regression tests:

```bash
python3 -m unittest discover -s ctrl_scripts -p 'test_drive_arm_coexistence.py' -v
```

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
