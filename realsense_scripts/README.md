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
`slave_realsense_v4l2.py` starts. The slave can accept a connection during warmup,
but the master waits for **every camera** to report `CAMERA READY` before clock
synchronization and scheduling a common capture time.

Readiness requires all of the following (default values):

- At least 90 received frames and 5 seconds elapsed since the first full frame.
- A further 2-second observation window with sampled mean Y varying by at most
  3 code values, and sampled mean U/V varying by at most 2 (8-bit values).
- Mean Y at least 25, to reject near-black startup frames, and a live stream.

Warmup frames are discarded, not counted toward `target_frames`. If stabilization
does not finish within the master's `ready_timeout` (60 seconds), capture fails
instead of saving startup frames. The slave does not independently postpone the
scheduled capture time to finish warming up.

When the master sends a capture command, the slave does not reopen the camera.
Instead, it saves frames from the already-running stream starting at the
scheduled capture time. This avoids including sensor startup, format negotiation,
and early unstable frames in the recorded video.

`target_frames = 100` means exactly 100 consecutive frames from the running
FFmpeg output after readiness and the scheduled gate. FFmpeg frame duplication
is disabled with `-vsync 0` (compatible with the older FFmpeg installed on the
robots; see [FFmpeg synchronization options](https://ffmpeg.org/ffmpeg.html)).
Buffer overrun, stream failure, short output, or capture timeout is a failure,
not a successful shorter recording. Partial files retain failure metadata.

Readiness is an image-stability heuristic, not color calibration or a guarantee
of exposure correctness. Automatic exposure/white balance remain enabled and
can still respond to lighting or scene changes during capture. Keep the scene
steady during warmup. A consistently dark or incorrectly colored scene cannot
be diagnosed from temporal stability alone. No denoising or color correction is
applied by this change.

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

Both scripts now default to the parameter file **beside the script**, regardless
of the shell's current directory. Missing files fail explicitly. `--params PATH`
selects a different file; CLI options still override file settings. Startup prints
the resolved parameter-file path so that old/missing configurations are visible.

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
ready_timeout = 60

[slave]
device = /dev/video6
width = 1920
height = 1080
fps = 30
ready_frames = 90
warmup_seconds = 5.0
stable_seconds = 2.0
luma_tolerance = 3.0
chroma_tolerance = 2.0
min_luma = 25.0
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
Parameters: /home/vcrobot3/realsense_scripts/input_parameters.txt
Opening V4L2/ffmpeg stream and warming camera...
slave_realsense_v4l2 listening on 0.0.0.0:50322
CAMERA READY: /dev/video6; discarded ... startup frames; Y/U/V=(...)
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

For a 100-frame check, update `slave_realsense_v4l2.py` and
`input_parameters.txt` on **every robot**, and update `master_realsense_v4l2.py`
on the PC. Start the slaves as above, then run:

```bash
python3 master_realsense_v4l2.py \
  --robots 192.168.10.203 192.168.10.205 192.168.10.206 \
  --target-frames 100
```

The master should print `ALL CAMERAS READY` before capture and `frames=100/100`
with `success=True` for each robot. At FHD I420, each output must be exactly
311,040,000 bytes. Warmup duration and discarded-frame counts are independent
of those 100 saved frames. The new master requires the matching updated slave
for the `camera_status` handshake.

## Output format

Raw video:

- pixel format: `yuv420p`
- bit depth: 8-bit
- resolution: usually `1920x1080`
- frame rate: usually `30 FPS`
- no container timestamps are embedded in the `.yuv` file

Sidecar files:

- `.csv`: frame index, stream frame index, local monotonic read timestamp,
  scheduled timestamp, timestamp error, and sampled `mean_y`, `mean_u`, `mean_v`
- `.ffmpeg.log`: FFmpeg stderr log
- `.meta.json`: resolution, pixel format, color metadata, session timestamp,
  requested/saved frame counts, success/failure, startup readiness measurements,
  and controls whose V4L2 set commands succeeded

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

The CSV timestamps mark complete FFmpeg pipe reads, not sensor exposure times.
`timestamp_error_ms` in this capture path is the elapsed time from the scheduled
gate, not a per-frame cross-camera synchronization error. Use a common clock
mapping and matching frames for cross-camera comparisons. Neither the readiness
barrier nor small pipe-read timing differences certify exposure synchronization
within 10 ms. Physical-camera timing still needs an independent measurement.

## Troubleshooting

If the master times out with `brightness_changing` or `color_changing`, keep the
scene steady and inspect the camera's auto controls. For unusually slow startup,
increase `[slave] warmup_seconds` and `[master] ready_timeout`. `image_too_dark`
means the mean-Y check failed; lower `min_luma` only for intentionally dark scenes.
These thresholds use sparse image statistics, not a physical lux measurement.

Plot `mean_y`, `mean_u`, and `mean_v` from the saved CSV to check whether image
brightness/color continue to drift during the recording. Readiness checks do
not lock exposure or white balance during capture.

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

## Regression tests (no camera required)

From the project `scripts` directory:

```bash
python3 -m unittest discover -s realsense_scripts -p 'test_capture_readiness.py' -v
```

The tests cover warmup ramps, dark frames, stale streams, the multi-camera
readiness barrier, and exact 100-frame recording through one persistent FFmpeg
pipe. FFmpeg integration tests use generated images and do not open hardware.
