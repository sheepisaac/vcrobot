#!/usr/bin/env python3
"""Synchronized RealSense capture slave/runtime for ROS2 Image topics.

This module intentionally speaks the same TCP protocol as ctrl_scripts/master_arm.py:
HELLO -> SYNC -> PREPARE -> READY -> COMMIT -> EXECUTED.

The synchronized event is not "when the TCP packet arrives"; every robot pre-arms a
capture job and starts accepting frames at the timestamp sent by the master.
"""

import argparse
import csv
import heapq
import json
import os
import queue
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


PROTOCOL_VERSION = 2
TAI_CLOCK_ID = getattr(time, "CLOCK_TAI", 11)


def tai_available():
    if not hasattr(time, "clock_gettime_ns"):
        return False
    try:
        time.clock_gettime_ns(TAI_CLOCK_ID)
        return True
    except (OSError, ValueError):
        return False


def clock_now_ns(mode):
    if mode == "monotonic":
        return time.monotonic_ns()
    if mode == "tai" and tai_available():
        return time.clock_gettime_ns(TAI_CLOCK_ID)
    raise ValueError(f"unsupported clock mode: {mode}")


def ros_stamp_ns(msg):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value or None


def image_msg_to_bgr(bridge, msg, color_mode="cv_bridge"):
    """Convert a ROS Image message to an OpenCV BGR image explicitly.

    RealSense color topics commonly publish rgb8. OpenCV imwrite/video encoders
    expect BGR, so relying on implicit cv_bridge conversion can make saved images
    look blue on some robot environments.
    """
    encoding = (getattr(msg, "encoding", "") or "").lower()
    if color_mode == "cv_bridge":
        return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    if color_mode == "bgr":
        return image
    if color_mode == "rgb":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if color_mode == "swap":
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if encoding in ("bgr8", "bgr16"):
        return image
    if encoding in ("rgb8", "rgb16"):
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding in ("mono8", "8uc1"):
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    # Fallback to cv_bridge's converter for unusual encodings.
    return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def send_message(stream, message):
    stream.write((json.dumps(message, separators=(",", ":")) + "\n").encode())
    stream.flush()


def ros2_param_set(node_name, param_name, value, timeout=10.0):
    command = ["ros2", "param", "set", node_name, param_name, str(value)]
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout
        )
    except Exception as exc:
        print(
            f"WARNING: failed to set {node_name} {param_name}={value}: {exc}",
            flush=True,
        )
        return False
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        print(
            f"WARNING: ros2 param set failed: {node_name} "
            f"{param_name}={value}: {detail}",
            flush=True,
        )
        return False
    print(f"Set camera param: {node_name} {param_name}={value}", flush=True)
    return True


def configure_realsense_camera(node_name, auto_white_balance=True,
                               auto_exposure=True, white_balance=None,
                               exposure=None, param_timeout=10.0):
    if not node_name:
        return
    if auto_white_balance is not None:
        ros2_param_set(
            node_name, "rgb_camera.enable_auto_white_balance",
            "true" if auto_white_balance else "false",
            timeout=param_timeout,
        )
    if auto_exposure is not None:
        ros2_param_set(
            node_name, "rgb_camera.enable_auto_exposure",
            "true" if auto_exposure else "false",
            timeout=param_timeout,
        )
    if white_balance is not None:
        # librealsense accepts manual white balance only when auto WB is off.
        ros2_param_set(
            node_name, "rgb_camera.enable_auto_white_balance", "false",
            timeout=param_timeout,
        )
        ok = ros2_param_set(
            node_name, "rgb_camera.white_balance", f"{float(white_balance):.1f}",
            timeout=param_timeout,
        )
        if not ok:
            print(
                "WARNING: manual white balance failed; re-enabling auto WB",
                flush=True,
            )
            ros2_param_set(
                node_name, "rgb_camera.enable_auto_white_balance", "true",
                timeout=param_timeout,
            )
    if exposure is not None:
        ros2_param_set(
            node_name, "rgb_camera.enable_auto_exposure", "false",
            timeout=param_timeout,
        )
        ok = ros2_param_set(
            node_name, "rgb_camera.exposure", int(exposure),
            timeout=param_timeout,
        )
        if not ok:
            print(
                "WARNING: manual exposure failed; re-enabling auto exposure",
                flush=True,
            )
            ros2_param_set(
                node_name, "rgb_camera.enable_auto_exposure", "true",
                timeout=param_timeout,
            )


@dataclass
class CaptureRequest:
    job_id: str
    action: str
    accept_after_ns: int
    command: dict
    clock_mode: str
    scheduled_clock_ns: int
    done: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    reason: str = "pending"
    result: Optional[dict] = None
    candidate_frames: list = field(default_factory=list)
    selected_frame: Optional[dict] = None


class RealSenseCaptureNode(Node):
    def __init__(self, topic, queue_size=1, warmup_seconds=2.0,
                 warmup_frames=30, color_mode="cv_bridge"):
        super().__init__("sync_realsense_capture")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.requests = []
        self.active_videos = []
        self.frame_count_seen = 0
        self.first_callback_ns = None
        self.last_callback_ns = None
        self.last_ros_stamp_ns = None
        self.last_encoding = None
        self.warmup_seconds = max(0.0, float(warmup_seconds))
        self.warmup_frames = max(0, int(warmup_frames))
        self.color_mode = color_mode
        self.writer_queue = queue.Queue()
        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer_thread.start()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, int(queue_size)),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            Image, topic, self.listener_callback, qos
        )
        self.get_logger().info(
            f"Subscribed to {topic}; warming up for "
            f"{self.warmup_seconds:.1f}s and {self.warmup_frames} frames; "
            f"qos_depth={max(1, int(queue_size))}, reliability=best_effort"
        )

    def _writer_loop(self):
        while True:
            item = self.writer_queue.get()
            if item is None:
                self.writer_queue.task_done()
                return
            req, frame_index, msg, frame_info = item
            try:
                cv_image = image_msg_to_bgr(self.bridge, msg, self.color_mode)
                cv_image = cv2.resize(
                    cv_image, (req._video_width, req._video_height)
                )
                yuv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV_I420)
                req._video_file.write(yuv.tobytes())
                self._write_video_csv_row(req, frame_index, frame_info)
                req._video_written_frames += 1
                if req._video_written_frames >= req._video_frame_count:
                    req._video_file.close()
                    req._video_csv_file.close()
                    req.success = True
                    req.reason = "video_saved"
                    req.done.set()
                    with self.lock:
                        if req in self.active_videos:
                            self.active_videos.remove(req)
            except Exception as exc:
                try:
                    req._video_file.close()
                except Exception:
                    pass
                try:
                    req._video_csv_file.close()
                except Exception:
                    pass
                req.success = False
                req.reason = str(exc)
                req.done.set()
                with self.lock:
                    if req in self.active_videos:
                        self.active_videos.remove(req)
            finally:
                self.writer_queue.task_done()

    def submit(self, request):
        with self.lock:
            self.requests.append(request)

    def snapshot_status(self):
        with self.lock:
            age_ms = None
            if self.last_callback_ns is not None:
                age_ms = (time.monotonic_ns() - self.last_callback_ns) / 1e6
            warmup_elapsed_ms = None
            if self.first_callback_ns is not None:
                warmup_elapsed_ms = (time.monotonic_ns() - self.first_callback_ns) / 1e6
            return {
                "frames_seen": self.frame_count_seen,
                "last_callback_age_ms": age_ms,
                "last_ros_stamp_ns": self.last_ros_stamp_ns,
                "last_encoding": self.last_encoding,
                "warmup_ready": self._warmup_ready_locked(time.monotonic_ns()),
                "warmup_elapsed_ms": warmup_elapsed_ms,
                "warmup_required_frames": self.warmup_frames,
                "warmup_required_seconds": self.warmup_seconds,
            }

    def _warmup_ready_locked(self, now_ns):
        if self.first_callback_ns is None:
            return False
        elapsed = (now_ns - self.first_callback_ns) / 1e9
        return (
            self.frame_count_seen >= self.warmup_frames and
            elapsed >= self.warmup_seconds
        )

    def listener_callback(self, msg):
        callback_ns = time.monotonic_ns()
        stamp_ns = ros_stamp_ns(msg)
        with self.lock:
            self.frame_count_seen += 1
            if self.first_callback_ns is None:
                self.first_callback_ns = callback_ns
            self.last_callback_ns = callback_ns
            self.last_ros_stamp_ns = stamp_ns
            self.last_encoding = getattr(msg, "encoding", None)
            warmup_ready = self._warmup_ready_locked(callback_ns)
            ready = []
            keep = []
            for req in self.requests:
                if req.done.is_set():
                    continue
                if not warmup_ready:
                    keep.append(req)
                    continue
                if req.command.get("sync_mode", "gate") == "timestamp":
                    selected = self._timestamp_selection_locked(req, msg, callback_ns)
                    if selected is None:
                        keep.append(req)
                    else:
                        req.selected_frame = selected
                        ready.append(req)
                elif callback_ns >= req.accept_after_ns:
                    req.selected_frame = {
                        "msg": msg,
                        "callback_ns": callback_ns,
                        "ros_stamp_ns": stamp_ns,
                        "clock_ns": clock_now_ns(req.clock_mode),
                    }
                    ready.append(req)
                else:
                    keep.append(req)
            self.requests = keep
        for req in ready:
            frame = req.selected_frame or {
                "msg": msg, "callback_ns": callback_ns,
                "ros_stamp_ns": stamp_ns, "clock_ns": clock_now_ns(req.clock_mode),
            }
            self._save_request(req, frame["msg"], frame["callback_ns"])
        self._process_active_videos(msg, callback_ns)

    def _timestamp_selection_locked(self, req, msg, callback_ns):
        target_ns = int(req.command.get("target_ns", req.scheduled_clock_ns))
        collect_before_ms = float(req.command.get("collect_before_ms", 80.0))
        collect_after_ms = float(req.command.get("collect_after_ms", 80.0))
        frame = {
            "msg": msg,
            "callback_ns": callback_ns,
            "ros_stamp_ns": ros_stamp_ns(msg),
            "clock_ns": clock_now_ns(req.clock_mode),
        }
        frame_time_ns = int(frame["clock_ns"])
        if frame_time_ns < target_ns - int(collect_before_ms * 1e6):
            return None
        req.candidate_frames.append(frame)
        if frame_time_ns < target_ns + int(collect_after_ms * 1e6):
            return None
        return min(
            req.candidate_frames,
            key=lambda item: abs(int(item["clock_ns"]) - target_ns),
        )

    def _save_request(self, req, msg, callback_ns):
        try:
            command = req.command
            frame_info = req.selected_frame or {
                "msg": msg,
                "callback_ns": callback_ns,
                "ros_stamp_ns": ros_stamp_ns(msg),
                "clock_ns": clock_now_ns(req.clock_mode),
            }
            width = int(command.get("width", 1280))
            height = int(command.get("height", 720))
            output_dir = Path(command.get("output_dir", "./Results"))
            output_dir.mkdir(parents=True, exist_ok=True)
            frame_stamp = ros_stamp_ns(msg)
            if req.action == "image":
                cv_image = image_msg_to_bgr(self.bridge, msg, self.color_mode)
                cv_image = cv2.resize(cv_image, (width, height))
                frame_number = command.get("frame_number")
                if frame_number is None:
                    filename = output_dir / f"image_{req.job_id[:8]}.png"
                else:
                    filename = output_dir / f"image_{int(frame_number):04d}.png"
                ok = cv2.imwrite(str(filename), cv_image)
                req.success = bool(ok)
                req.reason = "image_saved" if ok else "image_write_failed"
                req.result = {
                    "filename": str(filename),
                    "saved_frame_clock_ns": frame_info["clock_ns"],
                    "callback_mono_ns": callback_ns,
                    "ros_stamp_ns": frame_stamp,
                    "encoding": getattr(msg, "encoding", None),
                    "late_from_accept_ms": (callback_ns - req.accept_after_ns) / 1e6,
                    "width": width,
                    "height": height,
                    "sync_mode": command.get("sync_mode", "gate"),
                    "target_ns": command.get("target_ns"),
                    "timestamp_error_ms": self._timestamp_error_ms(req, frame_info),
                    "candidate_count": len(req.candidate_frames),
                }
            elif req.action == "video":
                self._save_video(req, msg, callback_ns, output_dir)
            else:
                req.success = False
                req.reason = f"unknown action: {req.action}"
                req.result = {}
        except Exception as exc:
            req.success = False
            req.reason = str(exc)
            req.result = {}
            req.done.set()
        finally:
            if req.action == "video" and not req.done.is_set():
                return
            req.done.set()

    def _save_video(self, req, first_msg, first_callback_ns, output_dir):
        command = req.command
        first_frame_info = req.selected_frame or {
            "msg": first_msg,
            "callback_ns": first_callback_ns,
            "ros_stamp_ns": ros_stamp_ns(first_msg),
            "clock_ns": clock_now_ns(req.clock_mode),
        }
        width = int(command.get("width", 1280))
        height = int(command.get("height", 720))
        frame_count = int(command.get("frame_count", 1))
        fps = float(command.get("fps", 30))
        sample_mode = command.get("sample_mode", "every_frame")
        today = datetime.today().strftime("%Y%m%d")
        directory = output_dir / today
        directory.mkdir(parents=True, exist_ok=True)
        filename = directory / command.get(
            "filename", f"output_video_{width}x{height}_yuv420_{req.job_id[:8]}.yuv"
        )
        csv_filename = Path(str(filename) + ".csv")
        req._video_file = open(filename, "wb")
        req._video_csv_file = open(csv_filename, "w", newline="")
        req._video_csv = csv.writer(req._video_csv_file)
        req._video_csv.writerow([
            "frame_idx", "ros_stamp_ns", "callback_mono_ns", "clock_ns",
            "target_ns", "timestamp_error_ms"
        ])
        req._video_filename = filename
        req._video_width = width
        req._video_height = height
        req._video_fps = fps
        req._video_frame_count = frame_count
        frame_delay_ns = int((1.0 / fps) * 1e9)
        req.success = True
        req.reason = "video_saved" if frame_count <= 1 else "video_recording"
        req.result = {
            "filename": str(filename),
            "saved_frame_clock_ns": first_frame_info["clock_ns"],
            "callback_mono_ns": first_callback_ns,
            "ros_stamp_ns": ros_stamp_ns(first_msg),
            "encoding": getattr(first_msg, "encoding", None),
            "late_from_accept_ms": (first_callback_ns - req.accept_after_ns) / 1e6,
            "width": width,
            "height": height,
            "fps": fps,
            "frame_count": frame_count,
            "frames_written": 1,
            "sample_mode": sample_mode,
            "csv_filename": str(csv_filename),
            "sync_mode": command.get("sync_mode", "gate"),
            "target_ns": command.get("target_ns"),
            "timestamp_error_ms": self._timestamp_error_ms(req, first_frame_info),
            "candidate_count": len(req.candidate_frames),
        }
        if frame_count <= 1:
            req._video_written_frames = 0
            self.writer_queue.put((req, 0, first_msg, first_frame_info))
            return
        req._video_frame_delay_ns = frame_delay_ns
        req._video_sample_mode = sample_mode
        req._video_next_ns = (
            first_callback_ns + 1
            if sample_mode == "every_frame"
            else first_callback_ns + frame_delay_ns
        )
        req._video_frames_written = 1
        req._video_written_frames = 0
        self.writer_queue.put((req, 0, first_msg, first_frame_info))
        with self.lock:
            self.active_videos.append(req)

    def _process_active_videos(self, msg, callback_ns):
        with self.lock:
            active = list(self.active_videos)
        for req in active:
            if req.done.is_set() or callback_ns < req._video_next_ns:
                continue
            try:
                req._video_frames_written += 1
                frame_info = {
                    "msg": msg,
                    "callback_ns": callback_ns,
                    "ros_stamp_ns": ros_stamp_ns(msg),
                    "clock_ns": clock_now_ns(req.clock_mode),
                }
                self.writer_queue.put(
                    (req, req._video_frames_written - 1, msg, frame_info)
                )
                req._video_next_ns = (
                    callback_ns + 1
                    if req._video_sample_mode == "every_frame"
                    else callback_ns + req._video_frame_delay_ns
                )
                req.result["frames_written"] = req._video_frames_written
                req.result["last_callback_mono_ns"] = callback_ns
                req.result["last_frame_clock_ns"] = frame_info["clock_ns"]
                if req._video_frames_written >= req._video_frame_count:
                    with self.lock:
                        if req in self.active_videos:
                            self.active_videos.remove(req)
            except Exception as exc:
                try:
                    req._video_file.close()
                except Exception:
                    pass
                try:
                    req._video_csv_file.close()
                except Exception:
                    pass
                req.success = False
                req.reason = str(exc)
                req.done.set()
                with self.lock:
                    if req in self.active_videos:
                        self.active_videos.remove(req)

    @staticmethod
    def _timestamp_error_ms(req, frame_info):
        target_ns = req.command.get("target_ns")
        if target_ns is None:
            return None
        return (int(frame_info["clock_ns"]) - int(target_ns)) / 1e6

    def _write_video_csv_row(self, req, frame_idx, frame_info):
        req._video_csv.writerow([
            frame_idx,
            frame_info.get("ros_stamp_ns"),
            frame_info.get("callback_ns"),
            frame_info.get("clock_ns"),
            req.command.get("target_ns"),
            self._timestamp_error_ms(req, frame_info),
        ])


@dataclass(order=True)
class Job:
    execute_at_ns: int
    job_id: str = field(compare=False)
    command: str = field(compare=False)
    clock_mode: str = field(default="monotonic", compare=False)
    scheduled_clock_ns: int = field(default=0, compare=False)
    committed: bool = field(default=False, compare=False)
    owner: Optional["ClientSession"] = field(default=None, compare=False)


class Scheduler:
    def __init__(self, camera, capture_timeout):
        self.camera = camera
        self.capture_timeout = capture_timeout
        self.cv = threading.Condition()
        self.jobs = {}
        self.heap = []
        self.stopping = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def prepare(self, job_id, command, execute_at_ns, owner, clock_mode="monotonic"):
        parsed = json.loads(command)
        if not isinstance(parsed, dict):
            raise ValueError("camera command must be a JSON object")
        scheduled_clock_ns = int(execute_at_ns)
        parsed.setdefault("target_ns", scheduled_clock_ns)
        now = time.monotonic_ns()
        if clock_mode == "tai":
            execute_at_ns = now + (scheduled_clock_ns - clock_now_ns("tai"))
        elif clock_mode != "monotonic":
            raise ValueError(f"unsupported execution clock: {clock_mode}")
        if parsed.get("sync_mode") == "timestamp":
            execute_at_ns -= int(float(parsed.get("collect_before_ms", 80.0)) * 1e6)
        if execute_at_ns <= now + 20_000_000:
            raise ValueError("execution deadline is less than 20 ms away")
        with self.cv:
            if job_id in self.jobs:
                raise ValueError("duplicate job id")
            command = json.dumps(parsed, separators=(",", ":"))
            job = Job(execute_at_ns, job_id, command, clock_mode, scheduled_clock_ns, owner=owner)
            self.jobs[job_id] = job
            heapq.heappush(self.heap, job)
            self.cv.notify_all()

    def commit(self, job_id):
        with self.cv:
            job = self.jobs.get(job_id)
            if job is None:
                raise ValueError("unknown job id")
            job.committed = True
            self.cv.notify_all()

    def cancel(self, job_id):
        with self.cv:
            self.jobs.pop(job_id, None)
            self.cv.notify_all()

    def cancel_owner(self, owner):
        with self.cv:
            for job_id, job in list(self.jobs.items()):
                if job.owner is owner and not job.committed:
                    self.jobs.pop(job_id, None)
            self.cv.notify_all()

    def _run(self):
        while True:
            with self.cv:
                if self.stopping:
                    return
                while self.heap and self.jobs.get(self.heap[0].job_id) is not self.heap[0]:
                    heapq.heappop(self.heap)
                if not self.heap:
                    self.cv.wait()
                    continue
                job = self.heap[0]
                now = time.monotonic_ns()
                if not job.committed:
                    if now >= job.execute_at_ns:
                        heapq.heappop(self.heap)
                        self.jobs.pop(job.job_id, None)
                        continue
                    self.cv.wait((job.execute_at_ns - now) / 1e9)
                    continue
                remaining_ns = job.execute_at_ns - now
                if remaining_ns > 3_000_000:
                    self.cv.wait((remaining_ns - 3_000_000) / 1e9)
                    continue
                heapq.heappop(self.heap)
                self.jobs.pop(job.job_id, None)
            self._execute(job)

    def _execute(self, job):
        command = json.loads(job.command)
        action = command.get("action", "image")
        actual_ns = clock_now_ns(job.clock_mode)
        req = CaptureRequest(
            job.job_id, action, job.execute_at_ns, command,
            job.clock_mode, job.scheduled_clock_ns
        )
        self.camera.submit(req)
        completed = req.done.wait(self.capture_timeout)
        camera_status = self.camera.snapshot_status()
        verification = {
            "success": bool(completed and req.success),
            "reason": req.reason if completed else "capture_timeout",
            "scheduled_clock_ns": job.scheduled_clock_ns,
            "activation_clock_ns": actual_ns,
            "capture": req.result,
            "camera_status": camera_status,
        }
        job.owner.send({
            "type": "executed",
            "job_id": job.job_id,
            "scheduled_ns": job.scheduled_clock_ns,
            "actual_ns": actual_ns,
            "late_ns": actual_ns - job.scheduled_clock_ns,
            "verification": verification,
        })

    def close(self):
        with self.cv:
            self.stopping = True
            self.cv.notify_all()
        self.thread.join(timeout=1.0)


class ClientSession:
    def __init__(self, conn, address, scheduler):
        self.conn = conn
        self.address = address
        self.scheduler = scheduler
        self.stream = conn.makefile("rwb")
        self.send_lock = threading.Lock()

    def send(self, message):
        with self.send_lock:
            send_message(self.stream, message)

    def run(self):
        print(f"Master connected: {self.address}", flush=True)
        try:
            for raw_line in self.stream:
                received_ns = time.monotonic_ns()
                try:
                    message = json.loads(raw_line)
                    kind = message.get("type")
                    if kind == "hello":
                        self.send({
                            "type": "hello", "version": PROTOCOL_VERSION,
                            "robot": socket.gethostname(), "dry_run": False,
                            "tai_available": tai_available(),
                        })
                    elif kind == "sync":
                        clock_mode = message.get("clock_mode", "monotonic")
                        t1_ns = received_ns if clock_mode == "monotonic" else clock_now_ns(clock_mode)
                        self.send({
                            "type": "sync", "seq": message["seq"],
                            "t0_ns": message["t0_ns"], "t1_ns": t1_ns,
                            "t2_ns": clock_now_ns(clock_mode),
                            "clock_mode": clock_mode,
                        })
                    elif kind == "prepare":
                        self.scheduler.prepare(
                            message["job_id"], message["command"],
                            int(message["execute_at_ns"]), self,
                            message.get("clock_mode", "monotonic"),
                        )
                        self.send({"type": "ready", "job_id": message["job_id"]})
                    elif kind == "commit":
                        self.scheduler.commit(message["job_id"])
                        self.send({"type": "committed", "job_id": message["job_id"]})
                    elif kind == "cancel":
                        self.scheduler.cancel(message["job_id"])
                    else:
                        raise ValueError(f"unknown message type: {kind}")
                except Exception as exc:
                    self.send({"type": "error", "error": str(exc)})
        finally:
            self.scheduler.cancel_owner(self)
            try:
                self.stream.close()
            finally:
                self.conn.close()
            print(f"Master disconnected: {self.address}", flush=True)


def run_slave(port, topic="/camera/color/image_raw", listen="0.0.0.0",
              capture_timeout=8.0, queue_size=1,
              warmup_seconds=2.0, warmup_frames=30,
              camera_node="/camera/camera", configure_camera=False,
              auto_white_balance=True, auto_exposure=True,
              white_balance=None, exposure=None, color_mode="cv_bridge",
              param_timeout=10.0, ros_localhost_only=True):
    if ros_localhost_only:
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    if configure_camera:
        configure_realsense_camera(
            camera_node,
            auto_white_balance=auto_white_balance,
            auto_exposure=auto_exposure,
            white_balance=white_balance,
            exposure=exposure,
            param_timeout=param_timeout,
        )
    rclpy.init()
    node = RealSenseCaptureNode(
        topic, queue_size, warmup_seconds, warmup_frames, color_mode
    )
    scheduler = Scheduler(node, capture_timeout)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((listen, port))
    server.listen()
    print(f"slave_realsense listening on {listen}:{port}, topic={topic}", flush=True)

    def accept_loop():
        try:
            while rclpy.ok():
                conn, address = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                session = ClientSession(conn, address, scheduler)
                threading.Thread(target=session.run, daemon=True).start()
        except OSError:
            pass

    threading.Thread(target=accept_loop, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopping slave_realsense", flush=True)
    finally:
        server.close()
        scheduler.close()
        node.destroy_node()
        rclpy.shutdown()


def add_slave_args(parser, default_port):
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=default_port)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--capture-timeout", type=float, default=8.0)
    parser.add_argument("--queue-size", type=int, default=1,
                        help="ROS image subscriber queue depth; keep 1 for sync")
    parser.add_argument("--warmup-seconds", type=float, default=2.0,
                        help="seconds of incoming frames to discard before saving")
    parser.add_argument("--warmup-frames", type=int, default=30,
                        help="number of incoming frames to discard before saving")
    parser.add_argument("--camera-node", default="/camera/camera",
                        help="RealSense ROS2 camera node used for RGB params")
    parser.add_argument("--configure-camera", action="store_true",
                        help="set RealSense RGB auto exposure/white balance params")
    parser.add_argument("--no-auto-white-balance", action="store_true",
                        help="disable RGB auto white balance before capture")
    parser.add_argument("--no-auto-exposure", action="store_true",
                        help="disable RGB auto exposure before capture")
    parser.add_argument("--white-balance", type=int,
                        help="manual RGB white balance, e.g. 4600; disables auto WB")
    parser.add_argument("--exposure", type=int,
                        help="manual RGB exposure; disables auto exposure")
    parser.add_argument("--color-mode",
                        choices=("cv_bridge", "auto", "rgb", "bgr", "swap"),
                        default="cv_bridge",
                        help=("color conversion mode. cv_bridge matches the original "
                              "realsense_save_image_ros2.py; auto follows ROS "
                              "encoding; rgb forces RGB->BGR; bgr saves as-is; "
                              "swap flips red/blue for diagnosis"))
    parser.add_argument("--param-timeout", type=float, default=10.0,
                        help="seconds to wait for each ros2 param set command")
    parser.add_argument("--allow-ros-network", action="store_true",
                        help=("do not force ROS_LOCALHOST_ONLY=1 for this slave. "
                              "Leave this off when each robot should use only its "
                              "own local RealSense camera."))
    return parser


def parse_slave_args(default_port, description):
    parser = argparse.ArgumentParser(description=description)
    add_slave_args(parser, default_port)
    return parser.parse_args()
