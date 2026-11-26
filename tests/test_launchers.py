"""Check launcher shell lifetime without starting MuJoCo or robot connections."""

import errno
import os
import pty
import select
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCHERS = ("sim2sim.sh", "sim2real.sh")


class LauncherTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="a2 launchers ")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / "scripts").mkdir()
        (self.root / "bin").mkdir()
        for name in LAUNCHERS:
            shutil.copy2(ROOT / "scripts" / name, self.root / "scripts" / name)
        self.controller = self.root / "deploy/robots/a2/build/a2_ctrl"
        self.controller.parent.mkdir(parents=True)
        for child in (self.root / "bin/python", self.controller):
            child.write_text(
                f"#!{sys.executable}\n"
                "import os, signal, sys\n"
                "assert sys.argv[-2:] == ['--test-argument', 'value with spaces']\n"
                "if sys.argv[0].endswith('/python'):\n"
                "    assert 'PYTHONPATH' not in os.environ\n"
                "    assert os.environ['PYTHONNOUSERSITE'] == '1'\n"
                "if os.environ.get('A2_LAUNCHER_TEST_WAIT'):\n"
                "    signal.signal(signal.SIGINT, signal.SIG_DFL)\n"
                "    print('CHILD_READY', flush=True)\n"
                "    signal.pause()\n"
                "print('CHILD_EXITED', flush=True)\n"
                "sys.exit(int(os.environ.get('A2_LAUNCHER_TEST_STATUS', '0')))\n"
            )
            child.chmod(0o755)
        self.env = {
            **os.environ,
            "PATH": str(self.root / "bin") + os.pathsep + os.environ["PATH"],
            "PYTHONPATH": "parent-pythonpath",
            "PYTHONNOUSERSITE": "0",
        }

    def run_launcher(self, name, invocation, status=0):
        return subprocess.run(
            [
                "/bin/bash", "--noprofile", "--norc", "-c",
                'before_options=$-; before_shellopts=$SHELLOPTS; '
                'script_dir=parent-dir; controller=parent-controller; '
                f'{invocation} "$1" --test-argument "value with spaces"; '
                'status=$?; '
                '[[ $- == "$before_options" && $SHELLOPTS == "$before_shellopts" '
                '&& $script_dir == parent-dir && $controller == parent-controller '
                '&& $PYTHONPATH == parent-pythonpath && $PYTHONNOUSERSITE == 0 ]] '
                '|| exit 99; '
                'printf "SHELL_SURVIVED\\n"; exit "$status"',
                "launcher-test", str(self.root / "scripts" / name),
            ],
            env={**self.env, "A2_LAUNCHER_TEST_STATUS": str(status)},
            capture_output=True, text=True, timeout=5,
        )

    def test_exit_status_and_parent_shell_are_preserved(self):
        for name in LAUNCHERS:
            for invocation in ("bash", "source", "."):
                for status in (0, 7):
                    with self.subTest(name=name, invocation=invocation, status=status):
                        result = self.run_launcher(name, invocation, status)
                        self.assertEqual(result.returncode, status, result.stderr)
                        self.assertEqual(result.stdout, "CHILD_EXITED\nSHELL_SURVIVED\n")

    def test_missing_controller_returns_to_shell(self):
        self.controller.unlink()
        for invocation in ("bash", "source", "."):
            with self.subTest(invocation=invocation):
                result = self.run_launcher("sim2real.sh", invocation)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertEqual(result.stdout, "SHELL_SURVIVED\n")
                self.assertIn("a2_ctrl", result.stderr)

    def test_ctrl_c_returns_to_interactive_shell(self):
        for name in LAUNCHERS:
            for invocation in ("bash", "source", "."):
                with self.subTest(name=name, invocation=invocation):
                    self.check_ctrl_c(name, invocation)

    def check_ctrl_c(self, name, invocation):
        pid, terminal = pty.fork()
        if pid == 0:
            os.execve(
                "/bin/bash", ["bash", "--noprofile", "--norc", "-i"],
                {**self.env, "PS1": "A2_PROMPT> ", "TERM": "dumb", "A2_LAUNCHER_TEST_WAIT": "1"},
            )

        def read_until(marker):
            output = b""
            deadline = time.monotonic() + 5
            while marker not in output and time.monotonic() < deadline:
                if select.select([terminal], [], [], 0.1)[0]:
                    try:
                        chunk = os.read(terminal, 4096)
                    except OSError as error:
                        if error.errno != errno.EIO:
                            raise
                        break  # The shell exited and closed its terminal.
                    if not chunk:
                        break
                    output += chunk
            self.assertIn(marker, output, output.decode(errors="replace"))

        try:
            read_until(b"A2_PROMPT> ")
            launcher = shlex.quote(str(self.root / "scripts" / name))
            os.write(terminal, f'{invocation} {launcher} --test-argument "value with spaces"\n'.encode())
            read_until(b"CHILD_READY\r\n")
            os.write(terminal, b"\x03")  # Terminal-generated SIGINT to the foreground job.
            read_until(b"A2_PROMPT> ")
            os.write(terminal, b"printf 'SHELL_%s\\n' SURVIVED\n")
            read_until(b"SHELL_SURVIVED\r\n")
        finally:
            os.close(terminal)
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os.waitpid(pid, 0)


if __name__ == "__main__":
    unittest.main()
