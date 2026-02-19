#!/usr/bin/env python3

import sys
import signal
import subprocess
import atexit
from pathlib import Path


def _stop_all():
    try:
        subprocess.run(
            ["ros2", "service", "call", "/launch/mujoco_robot/stop", "std_srvs/srv/Trigger", "{}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3.0,
        )
    except Exception:
        pass
    try:
        subprocess.run(
            ["ros2", "service", "call", "/launch/control_stack/stop", "std_srvs/srv/Trigger", "{}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3.0,
        )
    except Exception:
        pass


def main() -> int:
    atexit.register(_stop_all)

    def _handler(signum, _frame):
        _stop_all()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    from streamlit.web import cli as stcli

    app_path = Path(__file__).with_name("app.py")
    sys.argv = ["streamlit", "run", str(app_path)]
    return stcli.main()


if __name__ == "__main__":
    raise SystemExit(main())
