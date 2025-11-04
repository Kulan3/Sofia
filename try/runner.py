#!/usr/bin/env python3
import argparse
import subprocess
import sys

PY = sys.executable or "python"
PROFILES = {
    "mission": [PY, "beta_main.py", "--use-last", "--show-video", "--log", ""],
    "silent": [PY, "beta_main.py", "--use-last"],
    "preview": [PY, "beta_main.py", "--use-last", "--show-video"],
    "detect": [PY, "beta_detect.py"],
    "dry": [PY, "dry_main.py", "--use-last"],
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run canned beta mission profiles.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("profile", choices=sorted(PROFILES.keys()), help="profile to execute")
    parser.add_argument("extra", nargs=argparse.REMAINDER, help="append additional args (prefix with --)")
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
