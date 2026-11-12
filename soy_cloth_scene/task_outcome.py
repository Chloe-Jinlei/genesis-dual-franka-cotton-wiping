"""Separate task completion from three known strict cotton-strain checks.

This classifier changes no threshold, actuator behavior, material value, or
original report field. In particular ``passed`` keeps its original meaning:
all original checks and no runtime failure. Only the three exact names below
may fail without blocking ``task_passed``. Coverage, grip geometry, mass,
contact, collision, capacity and tracking remain task requirements.

The schema is deliberately explicit and fail closed. Missing expected checks,
non-boolean check values, inconsistent raw ``passed``, malformed metadata and
unknown failed checks block task success. A nonempty failure blocks both new
success flags. These flags do not certify a calibrated material or force
closure. ``strict_material_passed`` covers only the three recorded directional
end-stage maxima, not every instant of the trajectory.
"""
from collections.abc import Mapping
from copy import deepcopy


STRICT_MATERIAL_CHECKS = (
    "dry_and_wet_lift_warp_extension_within_limit_and_residual",
    "dry_and_wet_lift_weft_extension_within_limit_and_residual",
    "dry_and_wet_lift_bias_extension_within_limit_and_residual",
)

TASK_CHECKS = (
    "complete_action_sequence",
    "all_liquid_absorbed",
    "both_robot_arms_moved",
    "two_contact_gated_final_grasps",
    "wet_cloth_clear_of_table",
    "absorbed_liquid_carried_up",
    "mass_conserved",
    "wet_mass_in_native_cloth",
    "finite_local_capacity",
    "robot_tracking_within_5mm",
    "no_arm_table_or_arm_arm_collision",
    "both_arms_pressed_cloth",
    "no_cloth_through_table",
    "laid_projected_area_at_least_85pct",
    "laid_near_table_area_at_least_85pct",
    "initial_and_final_grasps_preserve_material_lengths",
)


def classify_task_outcome(report):
    """Return an independent report copy with four classification fields.

    New fields are ``task_passed``, ``strict_material_passed``,
    ``material_warnings`` and explanatory ``task_blockers``. Existing fields,
    including raw ``passed`` and every check, remain unchanged. The source must
    be a mapping; malformed contents produce false success flags rather than
    being coerced using Python truthiness. Exact Python booleans are required,
    matching JSON true/false. The original report's raw-passed rule remains
    ``all(checks.values()) and failure is None``.
    """
    if not isinstance(report, Mapping):
        raise TypeError("Task outcome requires a report mapping")
    result = deepcopy(dict(report))
    blockers = []
    warnings = []
    checks = report.get("checks")
    schema_valid = isinstance(checks, Mapping)
    if not schema_valid:
        blockers.append("checks must be a mapping")
        checks = {}

    for name in TASK_CHECKS + STRICT_MATERIAL_CHECKS:
        if name not in checks:
            schema_valid = False
            blockers.append("missing expected check: " + name)
    for name, value in checks.items():
        if not isinstance(name, str) or type(value) is not bool:
            schema_valid = False
            blockers.append("invalid check name or non-boolean value: " + repr(name))
        elif value is False and name not in STRICT_MATERIAL_CHECKS:
            # New checks are task requirements by default; naming something
            # 'material' or 'warp' must not accidentally exempt a failure.
            blockers.append("failed task check: " + name)

    for name in STRICT_MATERIAL_CHECKS:
        if name not in checks:
            warnings.append({"check": name, "status": "missing",
                             "message": "Strict directional material evidence is missing."})
        elif type(checks[name]) is not bool:
            warnings.append({"check": name, "status": "invalid",
                             "message": "Strict directional material result is not a boolean."})
        elif checks[name] is False:
            warnings.append({"check": name, "status": "failed",
                             "message": "The recorded directional peak exceeds its unchanged limit plus numerical residual allowance."})

    failure = report.get("failure")
    failure_valid = "failure" in report and (failure is None or isinstance(failure, str))
    if not failure_valid:
        schema_valid = False
        blockers.append("missing or invalid failure field")
    elif failure:
        blockers.append("runtime failure: " + failure)
    failure_absent = failure_valid and (failure is None or failure == "")

    raw_passed = report.get("passed")
    if type(raw_passed) is not bool:
        schema_valid = False
        blockers.append("missing or non-boolean original passed field")
    elif isinstance(report.get("checks"), Mapping) and failure_valid:
        expected_raw_passed = all(type(value) is bool and value is True
                                  for value in checks.values()) and failure is None
        if raw_passed != expected_raw_passed:
            schema_valid = False
            blockers.append("original passed contradicts the original checks/failure rule")

    task_checks_pass = all(value is True for name, value in checks.items()
                          if name not in STRICT_MATERIAL_CHECKS)
    result.update(
        task_passed=bool(schema_valid and failure_absent and task_checks_pass),
        strict_material_passed=bool(schema_valid and failure_absent
                                   and all(checks.get(name) is True for name in STRICT_MATERIAL_CHECKS)),
        material_warnings=warnings,
        task_blockers=blockers,
    )
    return result
