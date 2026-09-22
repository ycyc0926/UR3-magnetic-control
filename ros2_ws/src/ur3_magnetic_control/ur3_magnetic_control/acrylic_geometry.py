"""Shared finite-footprint model for the fixed acrylic underside."""

import math

import numpy as np


CEILING_REGION_MODEL = "world_y_at_or_above_lower_edge"
X_BOUNDARY_USAGE = "recorded_only_not_used_for_ceiling_exemption"


def acrylic_inner_lower_left_world_xy_m(table_geometry):
    """Return and validate the measured acrylic inner lower-left corner."""
    try:
        footprint = table_geometry["fixed_work_surface"]["acrylic_footprint"]
        corner = np.asarray(footprint["inner_lower_left_world_xy_m"], dtype=float)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("missing acrylic footprint measurement") from error
    if (corner.shape != (2,) or not np.isfinite(corner).all()
            or footprint.get("ceiling_region_model") != CEILING_REGION_MODEL
            or footprint.get("x_boundary_usage") != X_BOUNDARY_USAGE):
        raise ValueError("invalid acrylic footprint model")
    return tuple(float(value) for value in corner)


def acrylic_ceiling_gap_m(table_geometry, lower_world, upper_world):
    """Return the underside gap, or infinity when geometry is wholly below Y edge.

    The exemption is conservative: a shape is exempt only when its complete
    world-coordinate AABB satisfies max(Y) < 0.025 m. A shape touching or
    crossing the measured lower edge is still checked against the 5 mm global
    acrylic-bottom policy.
    """
    lower = np.asarray(lower_world, dtype=float)
    upper = np.asarray(upper_world, dtype=float)
    if (lower.shape != (3,) or upper.shape != (3,)
            or not np.isfinite([lower, upper]).all() or np.any(upper < lower)):
        raise ValueError("invalid world bounds for acrylic clearance")
    _, lower_edge_y = acrylic_inner_lower_left_world_xy_m(table_geometry)
    if upper[1] < lower_edge_y:
        return math.inf
    try:
        underside = float(
            table_geometry["fixed_work_surface"]["acrylic_bottom_world_z_m"]
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("invalid acrylic underside height") from error
    if not math.isfinite(underside):
        raise ValueError("invalid acrylic underside height")
    return underside - float(upper[2])


def modeled_boundary_gaps_m(table_geometry, side_geometry, lower_world, upper_world):
    """Return all globally governed plane gaps for one geometry AABB."""
    lower = np.asarray(lower_world, dtype=float)
    upper = np.asarray(upper_world, dtype=float)
    if (lower.shape != (3,) or upper.shape != (3,)
            or not np.isfinite([lower, upper]).all() or np.any(upper < lower)):
        raise ValueError("invalid world bounds for clearance calculation")
    try:
        left = float(side_geometry["left_inner_x_m"])
        right = float(side_geometry["right_inner_x_m"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("invalid acrylic side geometry") from error
    if not math.isfinite(left) or not math.isfinite(right) or left >= right:
        raise ValueError("invalid acrylic side geometry")
    return {
        "ceiling": acrylic_ceiling_gap_m(table_geometry, lower, upper),
        "left": float(lower[0]) - left,
        "right": right - float(upper[0]),
        "table": float(lower[2]),
    }
