"""Hardware-free regression tests: python3 -m unittest discover -s realsense_scripts -p 'test_*.py'."""
import csv
import json
from collections import deque
from pathlib import Path
import shutil
import tempfile
import time
import unittest
from unittest import mock

import master_realsense_v4l2 as master_module
import slave_realsense_v4l2 as slave


class ReadinessTests(unittest.TestCase):
    def test_brightness_and_color_ramp_not_ready(self):
        state = slave.FrameReadiness(3, .2, .2, 3, 2, 25)
        for i in range(20):
            state.observe(i, i * 50_000_000, (30 + i * 4, 90 + i * 3, 128))
            self.assertFalse(state.status(i * 50_000_000)["ready"])
        for i in range(20, 27):
            state.observe(i, i * 50_000_000, (110, 150, 128))
        self.assertTrue(state.status(26 * 50_000_000)["ready"])

    def test_stable_black_is_not_ready(self):
        state = slave.FrameReadiness(1, 0, .2)
        for i in range(20):
            state.observe(i, i * 50_000_000, (4, 128, 128))
        self.assertFalse(state.status(950_000_000)["ready"])
        self.assertEqual(state.reason, "image_too_dark")

    def test_elapsed_time_and_frames_both_required_and_stall_resets(self):
        state = slave.FrameReadiness(90, 5, 2)
        for i in range(100):
            state.observe(i, i * 1_000_000, (100, 128, 128))
        self.assertFalse(state.status(100_000_000)["ready"])
        for i in range(1, 91):
            state.observe(100 + i, 100_000_000 + i * 100_000_000, (100, 128, 128))
        self.assertTrue(state.status(9_100_000_000)["ready"])
        self.assertFalse(state.status(11_000_000_000)["ready"])
        state.observe(191, 11_000_000_000, (100, 128, 128))
        self.assertIsNone(state.ready_ns)

    def test_all_raw_pixel_layouts(self):
        cases = {
            "yuv420p": bytes([80] * 16 + [110] * 4 + [140] * 4),
            "nv12": bytes([80] * 16 + [110, 140] * 4),
            "yuyv422": bytes([80, 110, 80, 140] * 8),
        }
        for fmt, data in cases.items():
            self.assertEqual(slave.FrameReadiness.measure(data, 4, 4, fmt), (80, 110, 140))
        rgb = slave.FrameReadiness.measure(bytes([100, 120, 140] * 16), 4, 4, "rgb24")
        bgr = slave.FrameReadiness.measure(bytes([140, 120, 100] * 16), 4, 4, "bgr24")
        self.assertEqual(rgb, bgr)

    def test_parameter_file_not_silently_ignored_and_bom_supported(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "params.txt"
            for module in (slave, master_module):
                with self.assertRaises(FileNotFoundError):
                    module.parse_parameter_file(path)
            path.write_text("[slave]\nready_frames=120\n", encoding="utf-8-sig")
            self.assertEqual(slave.parse_parameter_file(path)["slave"]["ready_frames"], "120")


class FakeRobot:
    def __init__(self, endpoint, replies, events):
        self.endpoint = endpoint
        self.socket = mock.Mock()
        self.socket.gettimeout.return_value = None
        self.replies = deque(replies)
        self.events = events

    def send(self, message):
        self.events.append((self.endpoint, "send"))

    def receive(self):
        self.events.append((self.endpoint, "receive"))
        reply = self.replies.popleft() if len(self.replies) > 1 else self.replies[0]
        return {"type": "camera_status", "reason": "stable" if reply else "warming_up",
                "ready": reply}, 0


class MasterBarrierTests(unittest.TestCase):
    def test_waits_for_slowest_camera_and_queries_all_first(self):
        events = []
        robots = [FakeRobot("r1", [True, True], events),
                  FakeRobot("r2", [False, True], events)]
        with mock.patch.object(master_module.time, "sleep"):
            result = master_module.wait_for_cameras(mock.Mock(robots=robots), 1)
        self.assertTrue(all(x["ready"] for x in result))
        self.assertEqual(events[:2], [("r1", "send"), ("r2", "send")])
        self.assertEqual(len(events), 8)

    def test_timeout_does_not_allow_recording(self):
        robot = FakeRobot("r1", [False], [])
        with self.assertRaisesRegex(RuntimeError, "readiness timed out"):
            master_module.wait_for_cameras(mock.Mock(robots=[robot]), .01)


@unittest.skipUnless(shutil.which("ffmpeg"), "FFmpeg is required for pipe integration")
class PersistentStreamTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        ffmpeg = shutil.which("ffmpeg")

        def command(recorder):
            return [ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin",
                    "-re", "-f", "lavfi", "-i", "color=c=gray:s=64x48:r=30",
                    "-c:v", "rawvideo", "-pix_fmt", "yuv420p", "-vsync", "0",
                    "-f", "rawvideo", "pipe:1"]

        with mock.patch.object(slave.V4L2Recorder, "_stream_command", command):
            self.recorder = slave.V4L2Recorder(
                self.directory.name, 64, 48, 30, "yuyv422", "yuv420p",
                self.directory.name, False, ready_frames=3, ring_frames=60,
                warmup_seconds=.1, stable_seconds=.1)
        self.addCleanup(self.recorder.close)

    def job(self, frames, timeout=10):
        now = time.monotonic_ns()
        return slave.Job(now, "test", {"target_frames": frames, "capture_timeout": timeout,
                                     "session_date": "20260905", "session_stamp": "20260905_1603"},
                         scheduled_clock_ns=now)

    def test_one_stream_warmup_excluded_exact_100_frames(self):
        self.assertTrue(self.recorder.wait_ready(5))
        pid = self.recorder.proc.pid
        job = self.job(100)
        result = self.recorder.capture(job)
        self.assertTrue(result["success"], result)
        capture = result["capture"]
        self.assertEqual(capture["saved_frames"], 100)
        self.assertEqual(capture["file_size_bytes"], 64 * 48 * 3 // 2 * 100)
        with open(capture["csv_filename"]) as file:
            rows = list(csv.DictReader(file))
        indices = [int(row["stream_frame_idx"]) for row in rows]
        self.assertEqual(indices, list(range(indices[0], indices[0] + 100)))
        self.assertGreater(indices[0], self.recorder.readiness.ready_frame_idx)
        self.assertTrue(all(int(row["frame_read_mono_ns"]) >= job.execute_at_ns for row in rows))
        self.assertEqual(self.recorder.proc.pid, pid)
        self.assertIsNone(self.recorder.proc.poll())

    def test_timeout_flushes_partial_file_and_reports_failure(self):
        self.assertTrue(self.recorder.wait_ready(5))
        result = self.recorder.capture(self.job(100, .1))
        self.assertFalse(result["success"])
        self.assertEqual(result["reason"], "capture_timeout")
        capture = result["capture"]
        self.assertLess(capture["saved_frames"], 100)
        self.assertEqual(capture["file_size_bytes"], capture["saved_frames"] * 4608)
        meta = json.loads(Path(capture["meta_filename"]).read_text())
        self.assertFalse(meta["success"])

    def test_capture_before_ready_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "camera_not_ready"):
            self.recorder.capture(self.job(100))

    def test_missing_buffer_frame_is_not_silently_skipped(self):
        self.assertTrue(self.recorder.wait_ready(5))
        status = self.recorder.require_ready()
        now = time.monotonic_ns()
        start = self.recorder.frame_index + 1000
        # Freeze the consumer-visible ring for a deterministic missing-frame test.
        frames = deque([{"frame_idx": i, "read_mono_ns": now,
                         "data": bytes(4608), "image_metrics": (100, 128, 128)}
                        for i in (start, start + 2)], maxlen=60)
        import io
        with self.recorder.cv:
            with mock.patch.object(self.recorder, "frames", frames):
                with self.assertRaisesRegex(RuntimeError, "buffer_overrun"):
                    self.recorder._save_frames(io.BytesIO(), now, status, 2, 1, [])


if __name__ == "__main__":
    unittest.main()
