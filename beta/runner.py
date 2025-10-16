import subprocess
import sys

modes = {
    "ai": ["python", "beta_main.py", "--use-last", "--ai", "on", '--log=""', "--show-video"],
    "none":   ["python", "beta_main.py", "--use-last", "--ai", "off", "--show-video"],
    "test":   ["python", "beta_main.py", "--log=debug.log"],
}

if len(sys.argv) < 2 or sys.argv[1] not in modes:
    print("Usage: python runner.py [ai|none|test]")
    sys.exit(1)

subprocess.run(modes[sys.argv[1]])
