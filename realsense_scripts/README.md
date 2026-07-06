# vcrobot RealSense capture scripts

This directory contains synchronized multi-robot RealSense RGB video capture
scripts based on V4L2 and FFmpeg.

The current capture path intentionally bypasses ROS 2 and `pyrealsense2` because
the Raspberry Pi / ROS 2 pipeline introduced too much frame timing jitter for
the target synchronization experiments. The V4L2 path opens the RealSense RGB
video node directly and records raw 8-bit YUV video.

## Files

- `master_realsense_v4l2.py`: server-PC master for synchronized capture
- `slave_realsense_v4l2.py`: robot-side slave that keeps the camera stream open
- `input_parameters.txt`: shared configuration file for both master and slaves
- `old/`: previous ROS 2 / earlier synchronization attempts kept for reference

## Capture design

Each slave opens the V4L2/FFmpeg camera stream as soon as
`slave_realsense_v4l2.py` starts. The camera is warmed up before the slave begins
listening for the master.

When the master sends a capture command, the slave does not reopen the camera.
Instead, it saves frames from the already-running stream starting at the
scheduled capture time. This avoids including sensor startup, format negotiation,
and early unstable frames in the recorded video.

The master sends its own session date/time to every slave. Output folders and
filenames therefore follow the master PC clock, not each robot's local clock.

Example output:

```text
Results/20260706/vcrobot3_1920x1080_yuv420_30fps_20260706_1603.yuv
Results/20260706/vcrobot3_1920x1080_yuv420_30fps_20260706_1603.yuv.csv
Results/20260706/vcrobot3_1920x1080_yuv420_30fps_20260706_1603.yuv.meta.json
```

## Dependencies

Install on each robot:

```bash
sudo apt update
sudo apt install -y ffmpeg v4l-utils python3
```

The RealSense RGB stream must be visible as a V4L2 device. For Intel RealSense
L515 in this setup, the FHD RGB node has typically been `/dev/video6`.

Check devices:

```bash
v4l2-ctl --list-devices
```

Check supported formats:

```bash
for d in /dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4 /dev/video5 /dev/video6 /dev/video7; do
  echo "===== $d ====="
  v4l2-ctl --list-formats-ext -d "$d" | grep -E "Pixel Format|Size|Interval|1920|1080|YUYV|MJPG"
done
```

Expected RGB node capability:

```text
Pixel Format: 'YUYV'
Size: Discrete 1920x1080
Interval: Discrete 0.033s (30.000 fps)
```

## Configuration

Copy `input_parameters.txt` to the server PC and every robot:

```bash
# server PC path
/mnt/c/Research/vcrobot/scripts/realsense_scripts/input_parameters.txt

# robot path
~/realsense_scripts/input_parameters.txt
```

Important defaults:

```ini
[master]
target_frames = 10
width = 1920
height = 1080
fps = 30
input_format = yuyv422
output_pix_fmt = yuv420p
lead = 2.0

[slave]
device = /dev/video6
width = 1920
height = 1080
fps = 30
ready_frames = 90
ring_frames = 300

[camera_controls]
white_balance_temperature_auto = 1
exposure_auto = 3
exposure_auto_priority = 0
power_line_frequency = 1
```

The current default does not force manual exposure or gain. This keeps the
RealSense default auto-exposure pipeline active while disabling exposure auto
priority so that the stream stays at 30 FPS.

## Slave usage

Run this on each robot:

```bash
cd ~/realsense_scripts
python3 slave_realsense_v4l2.py
```

Expected startup:

```text
Opening V4L2/ffmpeg stream and warming camera...
V4L2 recorder ready: /dev/video6, input=yuyv422, output=yuv420p, 1920x1080@30.0, stream already warm (...)
slave_realsense_v4l2 listening on 0.0.0.0:50322
```

By default, the slave exits when the master disconnects. To keep it running:

```bash
python3 slave_realsense_v4l2.py --keep-alive
```

## Master usage

Run this on the server PC:

```bash
cd /mnt/c/Research/vcrobot/scripts/realsense_scripts
python3 master_realsense_v4l2.py \
  --robots 192.168.10.203 192.168.10.205 192.168.10.206 \
  --target-frames 90
```

The master can also read robot IPs and capture settings from
`input_parameters.txt`:

```bash
python3 master_realsense_v4l2.py
```

Useful overrides:

```bash
python3 master_realsense_v4l2.py \
  --robots 192.168.10.203 192.168.10.205 192.168.10.206 \
  --target-frames 300 \
  --width 1920 --height 1080 --fps 30 \
  --lead 2.0 --timeout 90
```

## Output format

Raw video:

- pixel format: `yuv420p`
- bit depth: 8-bit
- resolution: usually `1920x1080`
- frame rate: usually `30 FPS`
- no container timestamps are embedded in the `.yuv` file

Sidecar files:

- `.csv`: frame index, stream frame index, local monotonic read timestamp,
  scheduled timestamp, and timestamp error
- `.ffmpeg.log`: FFmpeg stderr log
- `.meta.json`: resolution, pixel format, color metadata, and session timestamp

Open a raw YUV file with FFmpeg:

```bash
ffplay \
  -f rawvideo \
  -pixel_format yuv420p \
  -video_size 1920x1080 \
  -framerate 30 \
  Results/20260706/vcrobot3_1920x1080_yuv420_30fps_20260706_1603.yuv
```

## Timing analysis

The `.csv` file is the authoritative timing log for each raw video. Use the
project-level `analyze_revision_frame_errors.py` script for revision-level
plots and synchronization error summaries.

## Troubleshooting

If startup fails with a V4L2 control error, remove that control from
`input_parameters.txt`. Some RealSense firmware / kernel combinations reject
manual `gain`, `sharpness`, or color controls while auto exposure is enabled.

If the slave reports that the driver changed resolution, the selected device is
probably not the RGB FHD node. Check `v4l2-ctl --list-devices` and use the RGB
node, commonly `/dev/video6`.

If the saved file size does not match the requested frame count, the slave marks
the capture as failed. For `1920x1080 yuv420p`, one frame is:

```text
1920 * 1080 * 1.5 = 3,110,400 bytes
```

So 90 frames should be:

```text
279,936,000 bytes
```
