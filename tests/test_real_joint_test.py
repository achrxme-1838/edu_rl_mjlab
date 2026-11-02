"""Exercise the C++ diagnostic offline; no Python SDK or DDS connection."""

import json
import os
import pty
import select
import signal
import subprocess
import termios
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "deploy/robots/a2/build/real_joint_test"


@unittest.skipUnless(BINARY.is_file(), "Build the C++ real_joint_test target first")
class CppJointDiagnosticTest(unittest.TestCase):
    def test_mapping_matches_export_and_works_outside_repo(self):
        manifest = json.loads(
            (
                ROOT
                / "deploy/robots/a2/config/policy/velocity/v0/exported/manifest.json"
            ).read_text()
        )
        for order in ("edu", "sdk"):
            result = subprocess.run(
                [str(BINARY), "--print-mapping", "--order", order],
                cwd="/tmp",
                text=True,
                capture_output=True,
                check=True,
                timeout=5,
            )
            rows = [line for line in result.stdout.splitlines() if " -> motor " in line]
            self.assertEqual(len(rows), 12)
            for index, row in enumerate(rows):
                label, mapping = row.split(" | EDU ")
                policy_id, motor_id = map(int, mapping.split(" -> motor "))
                self.assertEqual(
                    motor_id, manifest["policy_to_sdk_motor_ids"][policy_id]
                )
                self.assertEqual(policy_id if order == "edu" else motor_id, index)
                if policy_id == 0:
                    self.assertIn("left front hip", label)
                if policy_id == 1:
                    self.assertIn("right front hip", label)

    def test_invalid_motion_is_rejected_before_connecting(self):
        for options in (
            ["--amplitude", "0.16"],
            ["--amplitude", "nan"],
            ["--period", "1"],
            ["--stand-duration", "2"],
            ["--order", "wrong"],
            ["--connect-timeout", "0"],
        ):
            result = subprocess.run(
                [str(BINARY), "--print-mapping", *options],
                check=False,
                text=True,
                capture_output=True,
                timeout=5,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("ERROR:", result.stderr)
            self.assertNotIn("Connecting to A2", result.stdout)

    def run_terminal(self, interrupt=False):
        master, slave = pty.openpty()
        original_settings = termios.tcgetattr(slave)
        process = subprocess.Popen(
            [str(BINARY), "--dry-run", "--stand-duration", "3"],
            stdin=slave,
            stdout=slave,
            stderr=slave,
            cwd="/tmp",
            start_new_session=True,
        )
        output = bytearray()

        def wait_for(text):
            deadline = time.monotonic() + 10
            while text.encode() not in output:
                if time.monotonic() >= deadline:
                    self.fail(f"Timed out waiting for {text!r}: {output.decode()}")
                if select.select([master], [], [], 0.1)[0]:
                    output.extend(os.read(master, 65536))
                elif process.poll() is not None:
                    self.fail(f"Exited before {text!r}: {output.decode()}")

        try:
            wait_for("Moving to EDU default pose")
            if interrupt:
                process.send_signal(signal.SIGINT)
            else:
                wait_for("[TEST 1/12] expected: left front hip")
                os.write(master, b" ")
                wait_for("[RETURN 1/12]")
                wait_for("[TEST 2/12] expected: right front hip")
                os.write(master, b"q")
            wait_for("Dry run stopped (no robot commands sent).")
            self.assertEqual(process.wait(timeout=3), 0)
            self.assertEqual(termios.tcgetattr(slave), original_settings)
            self.assertNotIn("Connecting to A2", output.decode())
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3)
            os.close(master)
            os.close(slave)

    def test_space_advances_and_q_restores_terminal(self):
        self.run_terminal()

    def test_sigint_restores_terminal(self):
        self.run_terminal(interrupt=True)


if __name__ == "__main__":
    unittest.main()
