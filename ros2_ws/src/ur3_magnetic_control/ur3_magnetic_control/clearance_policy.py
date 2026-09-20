"""Single source of truth for modeled workspace clearances."""

import math
from pathlib import Path

import yaml


DEFAULT_PROJECT_ROOT = Path("/home/yc/UR3")
BOUNDARIES = ("ceiling", "left", "right", "table")


def load_clearance_limits_m(project_root=DEFAULT_PROJECT_ROOT):
    """Load the one global policy; callers must not supply per-motion overrides."""
    path = Path(project_root) / "config" / "ur3_system.yaml"
    with path.open(encoding="utf-8") as stream:
        safety = yaml.safe_load(stream)["safety"]
    policy = safety["clearance_policy_m"]
    required = {"acrylic_bottom", "side", "table"}
    if set(policy) != required:
        raise ValueError(
            "clearance_policy_m must contain exactly acrylic_bottom, side and table"
        )
    values = {key: float(policy[key]) for key in required}
    if not all(math.isfinite(value) and value > 0.0 for value in values.values()):
        raise ValueError("clearance policy values must be finite and positive")
    return {
        "ceiling": values["acrylic_bottom"],
        "left": values["side"],
        "right": values["side"],
        "table": values["table"],
    }


def clearance_limits_mm(project_root=DEFAULT_PROJECT_ROOT):
    return {
        key: value * 1000.0
        for key, value in load_clearance_limits_m(project_root).items()
    }
