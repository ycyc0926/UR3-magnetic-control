#!/usr/bin/env python3
"""Legacy URScript preview; direct execution is disabled.

This file cannot evaluate the shared full-link/tool clearance policy. Real
motion must use the model-aware ROS 2 planners instead.
"""

from __future__ import annotations

import argparse
import math
import sys


DEFAULT_ROBOT_IP = "192.168.56.101"
MAX_DISTANCE_MM = 100.0
MAX_SPEED_MM_S = 50.0
MAX_ACCELERATION_MM_S2 = 250.0


def build_urscript(
    dx_m: float,
    dy_m: float,
    dz_m: float,
    speed_m_s: float,
    acceleration_m_s2: float,
    frame: str,
) -> str:
    offset = f"p[{dx_m:.9f},{dy_m:.9f},{dz_m:.9f},0,0,0]"
    if frame == "tool":
        target_expression = f"pose_trans(start_pose, {offset})"
    else:
        # Left multiplication applies the translation in the robot base frame.
        target_expression = f"pose_trans({offset}, start_pose)"

    return f"""def pc_guarded_linear_move():
  start_pose = get_actual_tcp_pose()
  target_pose = {target_expression}
  movel(target_pose, a={acceleration_m_s2:.6f}, v={speed_m_s:.6f})
end
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Preview one TCP straight-line move relative to the current pose. "
            "The legacy --execute option is retained only to return an explicit refusal."
        )
    )
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP)
    parser.add_argument("--frame", choices=("base", "tool"), default="base")
    parser.add_argument("--dx-mm", type=float, default=0.0)
    parser.add_argument("--dy-mm", type=float, default=0.0)
    parser.add_argument("--dz-mm", type=float, default=0.0)
    parser.add_argument("--speed-mm-s", type=float, default=20.0)
    parser.add_argument("--acceleration-mm-s2", type=float, default=100.0)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-path-clear", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    values = (
        args.dx_mm,
        args.dy_mm,
        args.dz_mm,
        args.speed_mm_s,
        args.acceleration_mm_s2,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("All distances, speed, and acceleration must be finite numbers.")

    distance_mm = math.sqrt(
        args.dx_mm**2 + args.dy_mm**2 + args.dz_mm**2
    )
    if distance_mm <= 0.0:
        raise ValueError("The requested displacement is zero.")
    if distance_mm > MAX_DISTANCE_MM:
        raise ValueError(
            f"Distance {distance_mm:.1f} mm exceeds the {MAX_DISTANCE_MM:.0f} mm guard limit."
        )
    if not 0.0 < args.speed_mm_s <= MAX_SPEED_MM_S:
        raise ValueError(f"Speed must be in (0, {MAX_SPEED_MM_S:.0f}] mm/s.")
    if not 0.0 < args.acceleration_mm_s2 <= MAX_ACCELERATION_MM_S2:
        raise ValueError(
            f"Acceleration must be in (0, {MAX_ACCELERATION_MM_S2:.0f}] mm/s^2."
        )

    script = build_urscript(
        args.dx_mm / 1000.0,
        args.dy_mm / 1000.0,
        args.dz_mm / 1000.0,
        args.speed_mm_s / 1000.0,
        args.acceleration_mm_s2 / 1000.0,
        args.frame,
    )
    print(
        f"Planned move: frame={args.frame}, "
        f"d=({args.dx_mm:.1f}, {args.dy_mm:.1f}, {args.dz_mm:.1f}) mm, "
        f"length={distance_mm:.1f} mm, speed={args.speed_mm_s:.1f} mm/s"
    )
    print("\nGenerated URScript:\n" + script)

    if not args.execute:
        print("PREVIEW ONLY: no command was sent to the robot.")
        return 0
    print(
        "REFUSED: this legacy direct-URScript path cannot enforce the global "
        "5 mm acrylic-bottom / 10 mm side clearance policy; use the ROS 2 "
        "model-aware planners.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
