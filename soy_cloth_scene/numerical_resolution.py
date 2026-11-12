"""Minimum gravity resolution for PBD's position-based velocity reconstruction.

With ``v = (x_new - x_old) / h`` after every substep, a gravity displacement
smaller than half a position ULP can round away, erasing acceleration instead
of accumulating it. This checks that minimum threshold for Earth gravity.
It is NOT a stability or accuracy guarantee: contacts, SPH, force integration,
roundoff bias, and coordinates beyond the supplied height need separate tests.
No Genesis initialization or scene mutation occurs here.
"""
from __future__ import annotations

import math
from numbers import Integral, Real

import numpy as np


class PBDGravityResolutionError(ValueError):
    """An unresolved gravity displacement, with the calculated metrics."""

    def __init__(self, metrics):
        self.metrics = dict(metrics)
        super().__init__(
            "PBD gravity displacement is below the minimum position resolution: "
            f"float{metrics['precision']}, h={metrics['h']:.9g} s, "
            f"max_height={metrics['max_height']:.9g} m, "
            f"gravity_step={metrics['gravity_step']:.9g} m, "
            f"ULP={metrics['ulp']:.9g} m, ratio={metrics['ratio']:.6g} "
            "(must be > 0.5). Position-based velocity reconstruction can erase "
            "gravity; use higher precision or a resolvable substep."
        )


def validate_pbd_gravity_resolution(dt, substeps, precision, max_height):
    """Return ``h, ulp, gravity_step, ratio``; reject unresolved combinations.

    ``dt`` is the outer step in seconds and ``substeps`` is a positive integer.
    ``precision`` is 32 or 64, either an integer or its string representation.
    ``max_height`` is the positive maximum absolute WORLD Z coordinate that
    cloth particles may reach, including margin; it is not height above the
    tabletop. The upward ULP at this bound is a conservative envelope estimate.

    ``gravity_step = 9.81*h*h`` and ``ratio = gravity_step/ulp``. A ratio at or
    below 0.5 is rejected, including the rounding-tie case. Other invalid inputs
    raise ValueError. A resolution error additionally exposes ``exc.metrics``.
    Passing only means gravity is not below this minimum rounding threshold;
    it does not establish correct free-fall, stable contacts, or stable fluid.
    """
    for name, value in (("dt", dt), ("max_height", max_height)):
        if isinstance(value, bool) or not isinstance(value, Real) \
                or not math.isfinite(float(value)) or value <= 0:
            raise ValueError(f"{name} must be a finite positive number")
    if isinstance(substeps, bool) or not isinstance(substeps, Integral) or substeps <= 0:
        raise ValueError("substeps must be a positive integer")
    if isinstance(precision, bool) or str(precision) not in ("32", "64"):
        raise ValueError("precision must be 32 or 64")
    bits = int(precision)
    dtype = np.float32 if bits == 32 else np.float64
    h = float(dt) / int(substeps)
    gravity_step = 9.81 * h * h
    with np.errstate(over="ignore", invalid="ignore"):
        coordinate = dtype(max_height)
        ulp = float(np.spacing(coordinate))
    if not (math.isfinite(h) and h > 0 and math.isfinite(gravity_step)
            and gravity_step > 0 and math.isfinite(ulp) and ulp > 0):
        raise ValueError("Step or height is outside the selected numerical range")
    metrics = {"h": h, "ulp": ulp, "gravity_step": gravity_step,
               "ratio": gravity_step / ulp, "precision": bits,
               "max_height": float(max_height), "minimum_ratio": 0.5}
    if metrics["ratio"] <= metrics["minimum_ratio"]:
        raise PBDGravityResolutionError(metrics)
    return metrics
