#!/usr/bin/env python3
import argparse
import subprocess
import sys

PY = sys.executable or "python"
PROFILES = {
    "ai1": [PY, "beta_main.py", "--use-last", "--ai", "on", "--mode", "1", "--log", ""],
    "ai2": [PY, "beta_main.py", "--use-last", "--ai", "on", "--mode", "2", "--log", ""],
    "plain": [PY, "beta_main.py", "--use-last", "--ai", "off", "--log", ""],
    "preview": [PY, "beta_main.py", "--use-last", "--ai", "off", "--show-video", "--log", ""],
    "diag": [PY, "beta_main.py", "--use-last", "--ai", "auto", "--show-video", "--log", ""],
    "dry": [PY, "dry_main.py", "--use-last"],
}

def parse_args():
    parser = argparse.ArgumentParser(
        description="Run canned beta mission profiles.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "profile",
        choices=sorted(PROFILES.keys()),
        help="profile to execute",
    )
    parser.add_argument(
        "extra",
        nargs=argparse.REMAINDER,
        help="additional arguments appended to the command (use -- before them)",
    )
    return parser.parse_args()

def main():
    args = parse_args()
    cmd = PROFILES[args.profile][:]
    if args.extra:
        cmd.extend(args.extra)
    print(">>", " ".join(cmd))
    result = subprocess.run(cmd)
    if result.returncode:
        sys.exit(result.returncode)

if __name__ == "__main__":
    main()
