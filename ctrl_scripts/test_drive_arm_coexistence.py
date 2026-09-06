"""Regression checks using fake serial; no physical robot is accessed."""
import importlib
import json
import socket
import sys
import threading
import time
import types
import unittest
from unittest import mock

import drive_sync as drive
import slave_arm as arm_module


class SerialModel:
    def __init__(self):
        self.packets = []
        self.torque = True

    def write(self, data):
        value = json.loads(data)
        self.packets.append(value)
        if value["T"] == 0:
            self.torque = False
        if value["T"] == 210:
            self.torque = bool(value["cmd"])
        return len(data)

    def flush(self):
        pass

    def close(self):
        pass


class DriveTests(unittest.TestCase):
    def setUp(self):
        self.serial = SerialModel()
        serial_module = types.SimpleNamespace(Serial=lambda *a, **kw: self.serial)
        with mock.patch.dict(sys.modules, serial=serial_module), \
                mock.patch.object(threading.Thread, "start"):
            self.driver = drive.DriveSerial("cart", "fake", 115200, 1, False, .5, .005, .35)
        self.driver.worker = mock.Mock()
        self.addCleanup(self.driver.close)

    def test_normal_start_idle_stop_and_reverse_preserve_torque(self):
        for _ in range(3):
            self.driver.last_write = 0
            self.driver._background_step(drive.STOP_COMMAND)
        self.driver.apply_at('{"T":1,"L":0.05,"R":0.05}', 0, "monotonic")
        self.driver.apply_at('{"T":1,"L":-0.05,"R":-0.05}', 0, "monotonic")
        self.assertEqual(self.serial.packets[-1], {"T": 1, "L": 0., "R": 0.})
        self.driver.apply_at('{"T":0}', 0, "monotonic")
        self.driver.apply_at(drive.STOP_COMMAND, 0, "monotonic")
        self.assertTrue(self.serial.torque)
        self.assertTrue(all(c["T"] == 1 for c in self.serial.packets))

    def test_safety_stop_and_shutdown_still_send_global_stop(self):
        self.driver.emergency_stop("fault")
        self.assertFalse(self.serial.torque)
        self.driver.last_write = 0
        self.driver._background_step(drive.STOP_COMMAND)
        self.assertEqual(self.serial.packets[-1], {"T": 0})
        self.driver.close()
        self.assertEqual(self.serial.packets[-1], {"T": 0})

    def test_disconnect_still_uses_safety_stop(self):
        local, remote = socket.socketpair()
        scheduler = mock.Mock(driver=self.driver)
        shutdown = threading.Event()
        session = drive.DriveSession(local, "fake", scheduler, shutdown)
        session.controls_motion = True
        remote.close()
        session.run()
        self.assertTrue(shutdown.is_set())
        self.assertEqual(self.serial.packets[-1], {"T": 0})

    def test_no_stale_movement_after_safety_stop(self):
        self.driver.apply_at('{"T":1,"L":0.05,"R":0.05}', time.monotonic_ns(), "monotonic")
        selected, release, stopped = threading.Event(), threading.Event(), threading.Event()
        original = self.driver._write

        def delayed_write(command):
            if threading.current_thread().name == "background-test":
                selected.set()
                if not release.wait(2):
                    raise RuntimeError("test writer timeout")
            return original(command)

        def stop():
            self.driver.emergency_stop("concurrent test")
            stopped.set()

        with mock.patch.object(self.driver, "_write", delayed_write):
            worker = threading.Thread(name="background-test", target=self.driver._background_step,
                                      args=(drive.STOP_COMMAND,))
            worker.start()
            self.assertTrue(selected.wait(1))
            stopper = threading.Thread(target=stop)
            stopper.start()
            self.assertFalse(stopped.wait(.05))
            release.set()
            worker.join(2)
            stopper.join(2)
            self.assertFalse(worker.is_alive())
            self.assertFalse(stopper.is_alive())
        self.assertEqual(self.serial.packets[-1], {"T": 0})

    def test_master_and_original_controller_stop_commands(self):
        from drive_master_sync import STOP_COMMAND
        expected = {"T": 1, "L": 0., "R": 0.}
        self.assertEqual(json.loads(STOP_COMMAND), expected)
        with mock.patch.dict(sys.modules, serial=types.SimpleNamespace(SerialException=OSError)):
            cart = importlib.import_module("ctrl_cartRider")
            ugv = importlib.import_module("ctrl_ugv")
        self.assertEqual(json.loads(cart.generate_command(0, 0)), expected)
        self.assertEqual(json.loads(ugv.cmd_parser("stop")), expected)
        self.assertEqual(json.loads(ugv.current_command), expected)


class ArmQuietTests(unittest.TestCase):
    def test_watchdog_rechecks_quiet_window_after_lock(self):
        arm = arm_module.ArmSerial.__new__(arm_module.ArmSerial)
        arm.lock = threading.Lock()
        arm.stop_event = threading.Event()
        arm.dry_run = False
        arm.serial = SerialModel()
        arm.quiet_until_monotonic = 0
        arm.torque_quiet_after_command = 1.5
        started = threading.Event()
        result = []

        def watchdog():
            started.set()
            result.append(arm._write_watchdog_command(arm_module.TORQUE_ON_COMMAND))

        with arm.lock:
            worker = threading.Thread(target=watchdog)
            worker.start()
            self.assertTrue(started.wait(1))
            arm._mark_controller_command('{"T":104,"x":100}')
        worker.join(2)
        self.assertEqual(result, [False])
        self.assertEqual(arm.serial.packets, [])
        arm.quiet_until_monotonic = 0
        self.assertTrue(arm._write_watchdog_command(arm_module.TORQUE_ON_COMMAND))
        self.assertEqual(arm.serial.packets[-1], {"T": 210, "cmd": 1})


if __name__ == "__main__":
    unittest.main()
