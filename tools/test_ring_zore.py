"""Run ring regressions and unchanged ordinary-following checks with host GCC.

Usage: python tools/test_ring_zore.py --compiler C:/msys64/mingw64/bin/gcc.exe
"""
import argparse
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=shutil.which("gcc") or "gcc")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = root / "demo/user/src"
    include = root / "demo/user/inc"
    output = args.output or root / "demo/mdk/Objects/host-tests/zore"
    output.mkdir(parents=True, exist_ok=True)
    suites = {
        "test_track_vision": ["track_vision"],
        "test_track_control": ["track_control"],
        "test_pid": ["pid"],
        "test_img_process": ["img_process"],
        "test_track_preprocess": ["img_process", "track_vision", "track_control"],
        "test_line_following": ["track_vision", "track_control", "pid"],
        "test_ring_control": ["ring_control"],
        "test_ring_routes": ["track_vision", "track_control", "ring_control"],
        "test_ring_isolation": ["track_vision", "track_control", "ring_control", "pid"],
        "test_ring_360_zore": [],
        "test_near_route": ["track_vision"],
        "test_ring_motion_chain": ["ring_control", "track_control"],
    }
    for name, dependencies in suites.items():
        print("RUN " + name, flush=True)
        executable = output / (name + ".exe")
        command = [args.compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                   "-Wshadow", "-pedantic", "-O2", "-I", str(include),
                   str(root / "tools" / (name + ".c"))]
        command += [str(source / (item + ".c")) for item in dependencies]
        if name == "test_ring_isolation":
            command += [str(root / "tools/fixtures/ordinary_before_zore.c")]
        if name == "test_ring_360_zore":
            command += [str(root / "tools/fixtures/ring_vision_before_360.c")]
        command += ["-lm", "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable.resolve())], check=True)
    print("PASS all 12 host suites", flush=True)


if __name__ == "__main__":
    main()
