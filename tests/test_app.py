#!/usr/bin/env python3
"""Exercise the compiled daemon over a FIFO and test-only ioctl transport."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Daemon:
    def __init__(self, options=(), failure=None):
        self.temp = tempfile.TemporaryDirectory(prefix="jetson-daemon-test-")
        directory = Path(self.temp.name)
        self.log = directory / "commands"
        self.log.touch()
        encoder = directory / "encoder"
        os.mkfifo(encoder)
        self.writer = os.open(encoder, os.O_RDWR | os.O_NONBLOCK)
        motor, led = directory / "motor", directory / "led"
        motor.touch()
        led.touch()
        environment = os.environ.copy()
        environment["LD_PRELOAD"] = str(ROOT / "tests/mock_ioctl.so")
        environment["MOCK_LOG"] = str(self.log)
        if failure:
            environment["MOCK_FAIL"] = failure
        else:
            environment.pop("MOCK_FAIL", None)
        self.process = subprocess.Popen(
            [str(ROOT / "app/motor_daemon"), "--encoder", str(encoder),
             "--motor", str(motor), "--led", str(led), *options],
            env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True,
        )
        self.sequence = 0
        self.wait_for(lambda: "L 0" in self.lines())

    def lines(self):
        return self.log.read_text().splitlines()

    def wait_for(self, predicate):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process.poll() is not None:
                raise AssertionError(f"Daemon exited early: {self.process.communicate()}")
            time.sleep(0.01)
        raise AssertionError(f"Daemon timed out; log={self.lines()}")

    def raw_event(self, kind, value, sequence=None):
        self.sequence = self.sequence + 1 if sequence is None else sequence
        os.write(self.writer, struct.pack("<IiQII", kind, value,
                                         time.monotonic_ns(), self.sequence, 0))

    def event(self, kind, value):
        before = len([line for line in self.lines() if line.startswith("L ")])
        self.raw_event(kind, value)
        self.wait_for(lambda: len([line for line in self.lines()
                                  if line.startswith("L ")]) > before)

    def output(self):
        motors = [line for line in self.lines() if line.startswith("M ")]
        leds = [line for line in self.lines() if line.startswith("L ")]
        return tuple(map(int, motors[-1].split()[1:])), int(leds[-1].split()[1])

    def finish(self, expected=0, terminate=True):
        if terminate and self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
        stdout, stderr = self.process.communicate(timeout=5)
        if self.process.returncode != expected:
            raise AssertionError((self.process.returncode, stdout, stderr))
        assert self.output()[0][0:2] == (0, 0), self.lines()
        assert self.output()[1] == 0, self.lines()
        return stdout, stderr

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.communicate()
        os.close(self.writer)
        self.temp.cleanup()


class AppTests(unittest.TestCase):
    def daemon(self, *args, **kwargs):
        daemon = Daemon(*args, **kwargs)
        self.addCleanup(daemon.close)
        return daemon

    def test_limits_exact_duty_buttons_and_shutdown(self):
        daemon = self.daemon()
        self.assertEqual(daemon.output(), ((0, 0, 0), 0))
        daemon.event(1, -1)
        for _ in range(10):
            daemon.event(1, 1)
        self.assertEqual(daemon.output(), ((0, 0, 0), 0))
        daemon.event(2, 100)
        self.assertEqual(daemon.output(), ((1000, 1, 0), 8))
        for level in range(7, -1, -1):
            daemon.event(1, -1)
            self.assertEqual(daemon.output(), ((level * 125, 1, 0), level))
        daemon.event(1, -1)
        self.assertEqual(daemon.output(), ((0, 1, 0), 0))
        daemon.event(1, 1)
        daemon.event(2, 800)
        self.assertEqual(daemon.output(), ((125, 1, 1), 1))
        daemon.event(2, 100)
        self.assertEqual(daemon.output(), ((0, 0, 1), 0))
        daemon.event(2, 100)
        self.assertEqual(daemon.output(), ((125, 1, 1), 1))
        daemon.finish()

    def test_reverse_encoder_direction_mode(self):
        daemon = self.daemon(["--reverse-encoder", "--button-mode", "direction"])
        daemon.event(1, -1)
        self.assertEqual(daemon.output(), ((125, 1, 0), 1))
        daemon.event(2, 100)
        self.assertEqual(daemon.output(), ((125, 1, 1), 1))
        daemon.finish()

    def test_toggle_only_long_press(self):
        daemon = self.daemon(["--button-mode", "toggle"])
        daemon.event(1, 1)
        daemon.event(2, 1000)
        self.assertEqual(daemon.output(), ((125, 1, 0), 1))
        daemon.finish()

    def test_idle_heartbeat(self):
        daemon = self.daemon()
        daemon.wait_for(lambda: daemon.lines().count("PING") >= 2)
        daemon.finish()

    def test_sequence_gap_stops(self):
        daemon = self.daemon()
        daemon.raw_event(1, 1, sequence=2)
        _, stderr = daemon.finish(expected=1, terminate=False)
        self.assertIn("lost encoder event", stderr)

    def test_partial_record_stops(self):
        daemon = self.daemon()
        os.write(daemon.writer, b"partial")
        daemon.finish(expected=1, terminate=False)

    def test_unknown_event_stops(self):
        daemon = self.daemon()
        daemon.raw_event(99, 1)
        daemon.finish(expected=1, terminate=False)

    def test_output_failure_stops(self):
        for failure in ("motor", "led"):
            with self.subTest(failure=failure):
                daemon = self.daemon(failure=failure)
                daemon.event(1, 1)
                daemon.raw_event(2, 100)
                daemon.finish(expected=1, terminate=False)

    def test_heartbeat_and_overflow_failure_stop(self):
        for failure in ("heartbeat", "overflow"):
            with self.subTest(failure=failure):
                daemon = self.daemon(failure=failure)
                daemon.finish(expected=1, terminate=False)

    def test_bad_arguments(self):
        for arguments in (["--button-mode", "bad"], ["--long-press-ms", "0"],
                          ["--long-press-ms", "abc"], ["unexpected"]):
            run = subprocess.run([str(ROOT / "app/motor_daemon"), *arguments],
                                 capture_output=True, text=True, check=False)
            self.assertEqual(run.returncode, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
