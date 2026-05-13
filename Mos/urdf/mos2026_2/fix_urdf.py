"""Produce a cleaned-up copy of mos2026_2.urdf for Isaac conversion.

Fix vs. the as-exported URDF:
  - Only the FL leg ships with effort=30 / velocity=30; every other joint
    has effort="0" velocity="0", which leaves PhysX nothing to drive.
    Bumped to effort=30 velocity=30 to match the FL leg.

Unlike mos2026, this export has no fixed-joint quirks (all 28 joints are
already type="revolute"), so we only touch effort/velocity.

Usage:
    python3 Mos/urdf/mos2026_2/fix_urdf.py
"""

from __future__ import annotations

import os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "urdf", "mos2026_2.urdf")
DST = os.path.join(HERE, "urdf", "mos2026_2_fixed.urdf")


def patch(text: str) -> str:
    text = text.replace('effort="0"', 'effort="30"')
    text = text.replace('velocity="0"', 'velocity="30"')
    return text


def main() -> None:
    with open(SRC) as f:
        original = f.read()
    fixed = patch(original)
    with open(DST, "w") as f:
        f.write(fixed)
    n_revolute = fixed.count('type="revolute"')
    n_fixed = fixed.count('type="fixed"')
    print(f"[OK] wrote {DST}")
    print(f"     revolute joints: {n_revolute}")
    print(f"     fixed joints:    {n_fixed}")


if __name__ == "__main__":
    main()
