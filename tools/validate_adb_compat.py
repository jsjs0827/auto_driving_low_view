"""Host-only regression checks; never contacts ADB or an X5."""
import contextlib
import importlib.machinery
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading

ROOT = Path(__file__).resolve().parent.parent
HELPER = ROOT / "tools/adb_legacy_compat/adb"
loader = importlib.machinery.SourceFileLoader("b6_adb_compat", str(HELPER))
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)

FAKE_ADB = r'''
import os, re, shlex, subprocess, sys, time
assert sys.argv[-2] == "shell"
payload = sys.argv[-1]
lexer = shlex.shlex(payload, posix=True, punctuation_chars=True)
lexer.whitespace_split = True
tokens = list(lexer)
assert tokens[:3] == ["if", "sh", "-c"]
assert tokens[3] == os.environ["B6_FAKE_COMMAND"]
assert tokens[4] == ";"
marker = re.search(r"__B6_ADB_RC_[0-9a-f]{32}__=", payload).group(0)
mode = os.environ["B6_FAKE_MODE"]
status = int(os.environ["B6_FAKE_STATUS"])
if mode == "posix_execute":
    subprocess.run(["/bin/sh", "-c", payload])
    sys.exit(0)
if mode == "heartbeat":
    for i in range(100):
        print("waiting", flush=True)
        time.sleep(.05)
elif mode == "stdin":
    sys.stdout.buffer.write(sys.stdin.buffer.read())
else:
    print("model output", flush=True)
if mode not in ("missing", "transport"):
    suffix = str(status)
    prefix = ""
    if mode == "ansi": prefix = "\x1b[24;80R   "
    if mode == "caret_ansi": prefix = "^[[24;80R   "
    if mode == "cursor_save": prefix = "\x1b7\x1b[24;80R   "
    if mode == "malformed": suffix = "bad"
    if mode == "truncated": suffix = ""
    print(prefix + marker + suffix + "\r", flush=True)
    if mode == "duplicate": print(marker + suffix, flush=True)
sys.exit(1 if mode in ("transport", "transport_after_marker") else 0)
'''


def capture(base, command):
    buffer = io.BytesIO()
    stream = io.TextIOWrapper(buffer, encoding="utf-8", write_through=True)
    errors = io.StringIO()
    with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(errors):
        rc = module.checked_shell(base, command)
    text = buffer.getvalue().decode("utf-8")
    stream.detach()
    return rc, text, errors.getvalue()


def main():
    checks = []
    with tempfile.TemporaryDirectory(prefix="b6_adb_compat_") as scratch:
        fake = Path(scratch) / "fake_adb.py"
        fake.write_text(FAKE_ADB, encoding="utf-8")
        base = [sys.executable, str(fake)]
        command = "echo $$ > /userdata/example/job.pid; exec bin/ab_benchmark --source camera"
        os.environ["B6_FAKE_COMMAND"] = command
        cases = [
            ("success", "normal", 0, 0),
            ("remote_failure_7", "normal", 7, 7),
            ("remote_failure_42", "normal", 42, 42),
            ("remote_signal_status", "normal", 139, 139),
            ("actual_terminal_CSI", "ansi", 7, 7),
            ("rendered_terminal_CSI", "caret_ansi", 7, 7),
            ("legacy_cursor_save", "cursor_save", 7, 7),
            ("missing_marker", "missing", 0, 125),
            ("malformed_marker", "malformed", 0, 125),
            ("truncated_marker", "truncated", 0, 125),
            ("duplicate_marker", "duplicate", 0, 125),
            ("invalid_status", "normal", 999, 125),
            ("transport_failure", "transport", 0, 1),
            ("transport_failure_after_success_marker", "transport_after_marker", 0, 1),
        ]
        for name, mode, status, expected in cases:
            os.environ.update(B6_FAKE_MODE=mode, B6_FAKE_STATUS=str(status))
            rc, output, errors = capture(base, command)
            assert rc == expected, (name, rc, expected, errors)
            assert output.startswith("model output\n"), (name, repr(output))
            if mode not in ("malformed", "truncated"):
                assert output == "model output\n", (name, repr(output))
            if expected == 125:
                assert "missing or invalid remote exit marker" in errors
            checks.append({"name": name, "status": "passed", "exit_code": rc})

        command = "printf '%s\\n' \"quoted value\"; echo '$literal'; echo real-$$\nexit 7"
        os.environ.update(B6_FAKE_COMMAND=command, B6_FAKE_MODE="normal", B6_FAKE_STATUS="7")
        assert capture(base, command)[0] == 7
        checks.append({"name": "POSIX_quote_roundtrip_and_new_shell_for_PID", "status": "passed"})

        if os.name == "posix":
            for name, command, expected in [
                ("actual_shell_exit_7", "printf 'camera-check\\n'; exit 7", 7),
                ("actual_shell_errexit", "set -e; false; echo unreachable", 1),
                ("actual_shell_success", "printf 'camera-check\\n'; exit 0", 0),
            ]:
                os.environ.update(B6_FAKE_COMMAND=command, B6_FAKE_MODE="posix_execute", B6_FAKE_STATUS="0")
                rc, output, _ = capture(base, command)
                assert rc == expected, (name, rc, output)
                checks.append({"name": name, "status": "passed", "exit_code": rc})
            command = "printf 'recorded_pid=%s\\n' \"$$\"; exec sh -c 'printf \"exec_pid=%s\\n\" \"$$\"'"
            os.environ.update(B6_FAKE_COMMAND=command, B6_FAKE_MODE="posix_execute")
            rc, output, _ = capture(base, command)
            pairs = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)
            assert rc == 0 and pairs["recorded_pid"] == pairs["exec_pid"], (rc, output)
            checks.append({"name": "actual_shell_saved_PID_matches_exec_process", "status": "passed"})

        for args, index in [(["-s", "shell", "shell", "exit 7"], 2),
                            (["-H", "localhost", "-P", "5037", "push", "a", "b"], 4),
                            (["get-state"], 0)]:
            assert module.command_index(args) == index
        checks.append({"name": "ADB_global_options_and_serial_named_shell", "status": "passed"})

        os.environ.update(B6_FAKE_COMMAND="sh -s /userdata/example 123", B6_FAKE_MODE="stdin", B6_FAKE_STATUS="0")
        # The child process gives checked_shell an actual inherited OS stdin.
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--stdin-worker", str(fake)],
            input=b"cleanup-script-input\n", stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=10,
        )
        assert result.returncode == 0 and result.stdout == b"cleanup-script-input\n", result
        checks.append({"name": "cleanup_script_stdin_preserved", "status": "passed"})

        os.environ.update(B6_FAKE_COMMAND="sleep 5", B6_FAKE_MODE="heartbeat", B6_FAKE_STATUS="0")
        timer = threading.Timer(.4, lambda: signal.raise_signal(signal.SIGTERM))
        timer.start()
        try:
            rc, _, _ = capture(base, "sleep 5")
            assert rc == 143, rc
        finally:
            timer.join()
        checks.append({"name": "SIGTERM_forwarded_to_host_ADB_child", "status": "passed", "exit_code": rc})

    report = {
        "status": "passed", "checks_passed": len(checks), "checks": checks,
        "scope": "Host Python integration tests with a simulated legacy ADB client that returns 0 for remote failures.",
        "not_performed": ["X5 connection or inference", "GNU timeout integration", "real board signal cleanup"],
    }
    (ROOT / "validation/adb_compat_v4.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "passed", "checks_passed": len(checks)}))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--stdin-worker":
        sys.exit(module.checked_shell([sys.executable, sys.argv[2]], "sh -s /userdata/example 123"))
    main()
