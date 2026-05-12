"""Produce a cleaned-up copy of mos2026.urdf for Isaac conversion.

Fixes vs. the as-exported URDF:
  - rr_thigh_motor / rr_thigh are exported as type="fixed" (axis 0 0 0).
    Patched to type="revolute" with axis (0, 1, 0) — matches the right-side
    convention (fr_thigh_motor / fr_thigh both use axis 0 1 0).
  - Every joint except the FL leg ships with effort="0" velocity="0", which
    leaves PhysX nothing to drive. Bumped to effort=30 velocity=30 to match
    the FL leg.

Usage:
    python3 Mos/urdf/mos2026/fix_urdf.py
"""

from __future__ import annotations

import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "urdf", "mos2026.urdf")
DST = os.path.join(HERE, "urdf", "mos2026_fixed.urdf")


# Joints that need fixed -> revolute + non-zero axis. Axis follows right-side
# convention (0, 1, 0).
FIXED_TO_REVOLUTE = {
    "rr_thigh_motor": (0.0, 1.0, 0.0),
    "rr_thigh": (0.0, 1.0, 0.0),
}


JOINT_RE = re.compile(
    r'(<joint\s+name="(?P<name>[^"]+)"\s+type=)"fixed"(?P<body>.*?)</joint>',
    re.DOTALL,
)
AXIS_RE = re.compile(r'<axis\s+xyz="[^"]*"\s*/>')


def patch(text: str) -> str:
    def repl(match: re.Match) -> str:
        name = match.group("name")
        if name not in FIXED_TO_REVOLUTE:
            return match.group(0)
        ax = FIXED_TO_REVOLUTE[name]
        body = AXIS_RE.sub(
            f'<axis xyz="{ax[0]} {ax[1]} {ax[2]}"/>', match.group("body")
        )
        return f'{match.group(1)}"revolute"{body}</joint>'

    text = JOINT_RE.sub(repl, text)
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
