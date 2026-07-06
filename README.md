# vcrobot

Synchronized control and RealSense capture scripts for the vcrobot multi-robot
research platform.

This repository contains:

- synchronized RoArm control scripts
- synchronized UGV / CartRider drive scripts
- synchronized RealSense V4L2 / FFmpeg video capture scripts
- revision-level synchronization log analysis and plotting tools

The code was developed for experiments where multiple vcrobot units must start
robot motion or camera capture from a server PC with low timing skew.

## Hardware and third-party notices

This project is designed around the following hardware used in the vcrobot
research setup:

- **WAVESHARE UGV / mobile-base hardware**
- **WAVESHARE RoArm-style JSON serial command protocol**
- **Intel RealSense RGB camera hardware**

The control scripts send JSON commands that are compatible with the
WAVESHARE-style robot controller protocol used by the target robots. For
example, drive commands are sent in JSON form such as:

```json
{"T":1,"L":0.1,"R":0.1}
```

and arm commands use RoArm-style JSON command IDs such as `T=104`, `T=105`,
`T=112`, and `T=210`.

The RealSense capture path uses the camera as a V4L2 video device and records
frames through FFmpeg. The current V4L2 capture scripts do not vendor Intel
RealSense SDK source code. Earlier experimental ROS 2 / RealSense scripts are
kept under `realsense_scripts/old/` for reference.

Third-party names, hardware, firmware, protocols, SDKs, and tools remain under
their respective owners' terms:

- WAVESHARE hardware, firmware, documentation, and JSON command protocols are
  owned by WAVESHARE or their respective right holders. This repository only
  sends compatible commands for interoperability and does not include
  WAVESHARE firmware/source code.
- Intel RealSense hardware and Intel RealSense SDK / librealsense are owned by
  Intel / RealSense project maintainers. The upstream librealsense SDK is
  distributed under the Apache License 2.0.
- FFmpeg and Linux V4L2 utilities are external runtime dependencies and are not
  redistributed by this repository.

## Repository layout

```text
.
|-- ctrl_scripts/
|   |-- master_arm.py
|   |-- master_armSimple.py
|   |-- master_cartRider.py
|   |-- master_ugv.py
|   |-- slave_arm.py
|   |-- slave_armSimple.py
|   |-- slave_cartRider.py
|   |-- slave_ugv.py
|   `-- ...
|-- realsense_scripts/
|   |-- master_realsense_v4l2.py
|   |-- slave_realsense_v4l2.py
|   |-- input_parameters.txt
|   `-- old/
`-- analyze_revision_frame_errors.py
```

More detailed usage is documented in:

- [`ctrl_scripts/README.md`](ctrl_scripts/README.md)
- [`realsense_scripts/README.md`](realsense_scripts/README.md)

## Network model

The server PC runs a master script. Each robot runs a corresponding slave
script. The master connects to all slaves over TCP, estimates clock offsets,
sends a future scheduled execution time, waits until every slave is ready, and
then commits the command.

Typical robot IPs in the lab setup:

```text
vcrobot3: 192.168.10.203
vcrobot4: 192.168.10.204
vcrobot5: 192.168.10.205
vcrobot6: 192.168.10.206
```

Update the IP list according to the current network.

## Quick start: arm control

On each robot:

```bash
cd ~/ctrl_scripts
python3 slave_armSimple.py
```

On the server PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/ctrl_scripts
python3 master_armSimple.py --robots 192.168.10.203 192.168.10.206
```

Interactive command:

```text
100 0 480 4.1
```

One-shot command:

```bash
python3 master_armSimple.py \
  --robots 192.168.10.203 192.168.10.206 \
  --position 100 0 480 4.1
```

## Quick start: CartRider / UGV control

CartRider slave on each robot:

```bash
cd ~/ctrl_scripts
python3 slave_cartRider.py
```

CartRider master on the server PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/ctrl_scripts
python3 master_cartRider.py --robots 192.168.10.203 192.168.10.206
```

UGV slave on each robot:

```bash
cd ~/ctrl_scripts
python3 slave_ugv.py
```

UGV master on the server PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/ctrl_scripts
python3 master_ugv.py --robots 192.168.10.203 192.168.10.206
```

By default, movement slaves stop and exit when the master disconnects. Add
`--keep-alive` to keep a slave waiting for another master connection.

## Quick start: RealSense capture

Install dependencies on each robot:

```bash
sudo apt update
sudo apt install -y ffmpeg v4l-utils python3
```

Copy `realsense_scripts/input_parameters.txt` to each robot:

```bash
scp realsense_scripts/input_parameters.txt vcrobot3@192.168.10.203:~/realsense_scripts/
```

Start the slave on each robot:

```bash
cd ~/realsense_scripts
python3 slave_realsense_v4l2.py
```

Run the master on the server PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/realsense_scripts
python3 master_realsense_v4l2.py \
  --robots 192.168.10.203 192.168.10.205 192.168.10.206 \
  --target-frames 90
```

Example output file:

```text
Results/20260706/vcrobot3_1920x1080_yuv420_30fps_20260706_1603.yuv
```

The date and time in the output folder/file name are generated by the master PC,
not by each robot's local clock.

## Synchronization log analysis

Use `analyze_revision_frame_errors.py` to generate plots from revision folders
such as `r02`, `r03`, ..., `r11_20260702`.

Example:

```bash
cd /mnt/c/Research/vcrobot/scripts
python3 analyze_revision_frame_errors.py --revision-mean
python3 analyze_revision_frame_errors.py --frame-stddev-each
python3 analyze_revision_frame_errors.py --robot-stddev-each --overlay-frame-stddev
```

By default, the script scans the Results directory used during development. Use
script options to point it at a different results root when needed.

## Development notes

- Do not commit raw experiment videos, generated plots, or `__pycache__` files.
- Keep robot-specific IPs and camera parameters in config files where possible.
- Test new motion code at low speed before using multiple robots together.
- RealSense raw `.yuv` files do not contain embedded color metadata; use the
  `.meta.json` sidecar to record the intended interpretation.

## Safety disclaimer

This repository directly controls physical robot arms, mobile bases, and camera
hardware. The software is provided for research use. It does not guarantee safe
operation under all power, wiring, firmware, controller, or network conditions.
Always test in a clear area, keep emergency power access available, and verify
each robot individually before running synchronized multi-robot experiments.

## License

This project is released under the MIT License. See [`LICENSE`](LICENSE).

MIT is a permissive open-source license: others may use, copy, modify, merge,
publish, distribute, sublicense, and sell copies of the software, as long as the
copyright and license notice are included. The license also includes a warranty
disclaimer, which is important for research hardware code like this.
