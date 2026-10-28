"""One command to set up and run the active aero simulator on Windows, macOS or Linux.

    python aero.py               # set up if needed, then open the dashboard in your browser
    python aero.py setup         # (re)install, rebuild the C core and run all tests
    python aero.py test          # C smoke test + pytest suite
    python aero.py scenarios     # run every scripted scenario -> results/<id>/
    python aero.py scenarios 04  #   ...or only those whose file name contains "04"
    python aero.py clean         # delete .venv, build and results (next run starts fresh)

On Windows use `py aero.py ...`; on macOS use `python3 aero.py ...`.
Only Python 3.11+ is required. The C compiler (zig) is installed into .venv by pip,
so nobody needs Xcode, Visual Studio, MSYS2, CMake or make.
SIMULATION ONLY: synthetic thresholds and actuator physics.
"""
from __future__ import annotations

import sys

if sys.version_info < (3, 11):
    sys.exit(
        "This project needs Python 3.11 or newer, but you ran it with Python "
        f"{sys.version_info[0]}.{sys.version_info[1]}.\n"
        "Install the latest Python from https://www.python.org/downloads/ and try again."
    )

import argparse
import hashlib
import os
import shutil
import socket
import subprocess
import time
import venv
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
VENV = ROOT / ".venv"
BUILD = ROOT / "build"
REQUIREMENTS = ROOT / "requirements.txt"
WINDOWS = sys.platform == "win32"
VENV_PY = VENV / ("Scripts/python.exe" if WINDOWS else "bin/python")
LIB = BUILD / ("aero_core." + {"win32": "dll", "darwin": "dylib"}.get(sys.platform, "so"))
SMOKE_TEST = BUILD / ("test_core_c.exe" if WINDOWS else "test_core_c")
CORE_SOURCES = [ROOT / "core/src/aero_core.c"]
# Same strictness as the original CMake build: any warning fails the build.
CFLAGS = ["-std=c99", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-Wshadow", "-Werror", f"-I{ROOT / 'core/include'}"]
DASHBOARD_PORT = 8050


def step(msg: str) -> None:
    print(f"\n==> {msg}", flush=True)


def fail(msg: str) -> None:
    sys.exit(f"\nERROR: {msg}")


def run(*cmd, **kw) -> subprocess.CompletedProcess:
    return subprocess.run([str(c) for c in cmd], cwd=ROOT, **kw)


# --- setup ------------------------------------------------------------------

def ensure_venv() -> None:
    if not VENV_PY.exists():
        step("Creating Python environment in .venv (one time)")
        venv.EnvBuilder(with_pip=True, clear=True).create(VENV)
    stamp = VENV / ".requirements.sha256"
    want = hashlib.sha256(REQUIREMENTS.read_bytes()).hexdigest()
    if stamp.exists() and stamp.read_text() == want:
        return
    step("Installing Python packages (one time, needs internet; can take a minute)")
    if run(VENV_PY, "-m", "pip", "install", "--disable-pip-version-check", "-q", "-r", REQUIREMENTS).returncode:
        fail("pip could not install the packages. Check your internet connection and run `aero.py setup` again.")
    stamp.write_text(want)


def compile_c(output: Path, sources: list[Path], shared: bool) -> None:
    cmd = [VENV_PY, "-m", "ziglang", "cc", *CFLAGS, *(["-shared"] if shared else []), "-o", output, *sources]
    if run(*cmd).returncode:
        fail(f"compiling {output.name} failed (see the compiler messages above).")


def ensure_built(force: bool = False) -> None:
    inputs = [*CORE_SOURCES, *(ROOT / "core/include").glob("*.h"), ROOT / "core/tests/test_core.c", Path(__file__)]
    newest = max(p.stat().st_mtime for p in inputs)
    if not force and all(p.exists() and p.stat().st_mtime >= newest for p in (LIB, SMOKE_TEST)):
        return
    step("Building the C supervisor core (the first build takes ~30 s while the compiler warms up)")
    BUILD.mkdir(exist_ok=True)
    try:
        LIB.unlink(missing_ok=True)  # Windows locks a DLL that a running simulator has loaded
    except PermissionError:
        fail(f"{LIB.name} is in use. Close any running dashboard/simulator and try again.")
    compile_c(LIB, CORE_SOURCES, shared=True)
    compile_c(SMOKE_TEST, [ROOT / "core/tests/test_core.c", *CORE_SOURCES], shared=False)


def ensure_ready(force: bool = False) -> None:
    ensure_venv()
    ensure_built(force)


# --- commands ---------------------------------------------------------------

def cmd_setup(_args) -> int:
    ensure_ready(force=True)
    code = cmd_test(_args)
    if code == 0:
        print("\nSetup complete. Start the simulator with:  " + ("py aero.py" if WINDOWS else "python3 aero.py"))
    return code


def cmd_test(_args) -> int:
    ensure_ready()
    step("C smoke test")
    if run(SMOKE_TEST).returncode:
        return 1
    step("Python test suite")
    return run(VENV_PY, "-m", "pytest", "-q", "tests").returncode


def cmd_scenarios(args) -> int:
    ensure_ready()
    return run(VENV_PY, "scripts/run_scenario.py", *([args.filter] if args.filter else [])).returncode


def free_port(start: int) -> int:
    for port in range(start, start + 50):
        with socket.socket() as s:
            if s.connect_ex(("127.0.0.1", port)) != 0:
                return port
    fail("no free local port found for the dashboard.")


def cmd_dashboard(args) -> int:
    ensure_ready()
    port = free_port(args.port)
    url = f"http://127.0.0.1:{port}"
    server = subprocess.Popen([str(VENV_PY), "sim/dashboard/server.py", "--port", str(port)], cwd=ROOT)
    try:
        for _ in range(100):  # wait up to ~10 s for the server to accept connections
            if server.poll() is not None:
                fail("the dashboard server stopped during startup (see messages above).")
            with socket.socket() as s:
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    break
            time.sleep(0.1)
        if not args.no_browser:
            webbrowser.open(url)
        print(f"\nDashboard running at {url}  -  leave this window open; press Ctrl+C to stop.", flush=True)
        return server.wait()
    except KeyboardInterrupt:
        return 0
    finally:
        if server.poll() is None:
            server.terminate()


def cmd_clean(_args) -> int:
    for path in (VENV, BUILD, ROOT / "results", ROOT / ".pytest_cache"):
        if path.exists():
            print(f"removing {path.relative_to(ROOT)}")
            shutil.rmtree(path)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command")
    sub.add_parser("setup", help="install packages, rebuild the C core, run all tests")
    sub.add_parser("test", help="run the C smoke test and the pytest suite")
    sc = sub.add_parser("scenarios", help="run scripted scenarios into results/")
    sc.add_argument("filter", nargs="?", default="", help="substring of scenario file name")
    db = sub.add_parser("dashboard", help="open the live dashboard (also the default with no command)")
    db.add_argument("--port", type=int, default=DASHBOARD_PORT)
    db.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    sub.add_parser("clean", help="delete .venv, build and results")
    args = ap.parse_args()
    if args.command is None:
        args = db.parse_args([], namespace=args)
    handler = {"setup": cmd_setup, "test": cmd_test, "scenarios": cmd_scenarios,
               "clean": cmd_clean}.get(args.command, cmd_dashboard)
    return handler(args)


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUTF8", "1")  # consistent text encoding in child processes on Windows
    sys.exit(main())
