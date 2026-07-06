#!/usr/bin/env python3
"""Robot-side synchronized server for master_armSimple.py.

The TCP clock synchronization, scheduler, torque watchdog, and serial writer are
shared with slave_arm.py so both arm controllers use exactly the same timing path.
"""

from slave_arm import main


if __name__ == "__main__":
    main()
