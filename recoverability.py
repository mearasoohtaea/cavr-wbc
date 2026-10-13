"""Small, deterministic recoverability mechanism used by the STEP13 pilot.

This module intentionally contains no robot/model/solver imports.  It is the
analytic mechanism which sits *in front of* the pilot controller.  The
one-dimensional feature is a signed distance ``p_max - p`` and its associated
velocity.  For a finite braking authority ``a`` the stopping margin is

``h_stop = p_max - p - v**2 / (2*a)``.

The three pilot arms share the instantaneous feasibility result.  B1 applies
only the fault-aware soft task, P0 adds the instantaneous projection, and P1
adds the future stopping test.  P1 is therefore a filter/screening operation,
not a claim of closed-loop or constant-torque recovery.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
import math
from typing import Any, Callable, Mapping, Sequence


METHOD_B1 = "B1_FAULT_AWARE_SOFT_WBC"
METHOD_P0 = "P0_INSTANT_PROJECTION"
METHOD_P1 = "P1_RECOVERABILITY_FILTER"
METHODS = (METHOD_B1, METHOD_P0, METHOD_P1)
_METHOD_ALIASES = {
    "B1": METHOD_B1, "P0": METHOD_P0, "P1": METHOD_P1,
    "B1_FAULT_AWARE_SOFT": METHOD_B1,
    "P0_INSTANT": METHOD_P0,
    "P1_RECOVERABILITY": METHOD_P1,
}


class RecoverabilityInputError(ValueError):
    """Raised when a mechanism input is missing, non-finite or invalid."""


@dataclass(frozen=True)
class BrakingAssessment:
    """Result of the one-dimensional stopping-margin calculation."""

    p: float
    p_max: float
    v: float
    authority: float
    h_stop: float
    stopping_distance: float | None
    zero_authority: bool
    terminal_reachable: bool
    margin_class: str

    @property
    def safe(self) -> bool:
        """Whether the closed interval can reach zero velocity before ``p_max``."""

        return self.terminal_reachable and self.h_stop >= 0.0

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["safe"] = self.safe
        return result


@dataclass(frozen=True)
class MethodDecision:
    """Decision for one arm on one paired state.

    ``accepted`` refers to the command being accepted unchanged.  A P1 result
    can be ``DEGRADE`` (``accepted=False``) when a finite admissible speed can
    be requested; this keeps rejection and reduction distinct in the pilot
    ledger without pretending that a fallback is a certificate.
    """

    method: str
    accepted: bool
    action: str
    command_scale: float
    reason: str
    instant_feasible: bool
    projection_feasible: bool
    braking: BrakingAssessment

    @property
    def reduced(self) -> bool:
        return self.action == "DEGRADE"

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["braking"] = self.braking.as_dict()
        result["reduced"] = self.reduced
        return result


def _finite(name: str, value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise RecoverabilityInputError(f"{name} must be a finite scalar") from exc
    if not math.isfinite(number):
        raise RecoverabilityInputError(f"{name} must be a finite scalar")
    return number


def authority_acceleration(rho: float, *, a_max: float = 1.0) -> float:
    """Map an actuator health level to a non-negative braking authority.

    The pilot uses the deliberately transparent linear mapping ``a=rho*a_max``;
    the mapping is a mechanism assumption and is recorded with every result.
    ``rho`` is the normalized health level, not a solver tolerance.
    """

    health = _finite("rho", rho)
    maximum = _finite("a_max", a_max)
    if not 0.0 <= health <= 1.0:
        raise RecoverabilityInputError("rho must lie in [0, 1]")
    if maximum < 0.0:
        raise RecoverabilityInputError("a_max must be non-negative")
    return health * maximum


# Short alias used by case-manifest code and papers.
a_of_rho = authority_acceleration


def braking_margin(
    p: float,
    p_max: float,
    v: float,
    acceleration: float,
) -> BrakingAssessment:
    """Evaluate ``h_stop`` with an explicit and reproducible ``a(rho)``.

    At zero authority, a state at rest is holdable exactly when it is inside
    the boundary.  A non-zero speed has no finite stopping distance and is
    represented by ``h_stop=-inf``/``terminal_reachable=False``.  Negative
    authority is a malformed configuration and is rejected rather than being
    silently treated as zero.
    """

    position = _finite("p", p)
    boundary = _finite("p_max", p_max)
    velocity = _finite("v", v)
    authority = _finite("acceleration", acceleration)
    if authority < 0.0:
        raise RecoverabilityInputError("acceleration must be non-negative")
    distance = boundary - position
    if authority == 0.0:
        if velocity == 0.0:
            margin = distance
            stopping = 0.0
            reachable = margin >= 0.0
            category = "BOUNDARY" if margin == 0.0 else ("INTERIOR" if margin > 0 else "EXTERIOR")
        else:
            margin = float("-inf")
            stopping = None
            reachable = False
            category = "ZERO_AUTHORITY_DRIFT"
        return BrakingAssessment(position, boundary, velocity, authority, margin,
                                 stopping, True, reachable, category)

    stopping = velocity * velocity / (2.0 * authority)
    margin = distance - stopping
    reachable = margin >= 0.0
    category = "BOUNDARY" if margin == 0.0 else ("INTERIOR" if margin > 0 else "EXTERIOR")
    return BrakingAssessment(position, boundary, velocity, authority, margin,
                             stopping, False, reachable, category)


def h_stop(
    p: float,
    p_max: float,
    v: float,
    acceleration: float,
    *,
    detailed: bool = False,
) -> float | BrakingAssessment:
    """Return the signed ``h_stop`` margin (or its detailed assessment).

    ``detailed=True`` is convenient for ledgers; the default scalar return is
    deliberately identical to the equation printed in the STEP13 protocol.
    For zero authority and non-zero speed the scalar is ``-inf`` to represent
    the absence of a finite terminal set.
    """

    assessment = braking_margin(p, p_max, v, acceleration)
    return assessment if detailed else assessment.h_stop


def stopping_margin(p: float, p_max: float, v: float, acceleration: float) -> float:
    """Scalar alias for :func:`h_stop` used by result-table builders."""

    return float(h_stop(p, p_max, v, acceleration))


def recoverability_margin(
    p: float, p_max: float, v: float, rho: float, *, a_max: float = 1.0
) -> BrakingAssessment:
    """Convenience form applying the frozen ``a(rho)`` health mapping."""

    return braking_margin(p, p_max, v, authority_acceleration(rho, a_max=a_max))


def _as_state_values(state: Mapping[str, Any], *, p_max: float | None = None) -> tuple[float, float, float, float]:
    def first(*names: str, default: Any = None) -> Any:
        for name in names:
            if name in state:
                return state[name]
        if default is not None:
            return default
        raise RecoverabilityInputError(f"state is missing one of {names!r}")

    p = first("p", "position", "position_m")
    boundary = first("p_max", "boundary", "boundary_m", default=p_max)
    if boundary is None:
        raise RecoverabilityInputError("state is missing p_max/boundary")
    v = first("v", "velocity", "velocity_mps")
    rho = first("rho", "health", "health_level")
    return (_finite("p", p), _finite("p_max", boundary), _finite("v", v), _finite("rho", rho))


def evaluate_method(
    method: str,
    state: Mapping[str, Any] | None = None,
    *,
    p: float | None = None,
    p_max: float | None = None,
    v: float | None = None,
    rho: float | None = None,
    acceleration: float | None = None,
    a_max: float = 1.0,
    instant_feasible: bool = True,
    projection_feasible: bool = True,
    allow_degrade: bool = True,
) -> MethodDecision:
    """Apply B1, P0 or P1 semantics to one state.

    A state may be supplied as a mapping (the preferred case-level interface)
    or as explicit keyword values.  For P1, ``DEGRADE`` reports the largest
    admissible speed fraction when braking authority and distance permit one;
    it is still not an accepted unchanged command.
    """

    method = _METHOD_ALIASES.get(method, method)
    if method not in METHODS:
        raise RecoverabilityInputError(f"unknown method {method!r}")
    if state is not None:
        if not isinstance(state, Mapping):
            raise RecoverabilityInputError("state must be a mapping")
        p, p_max, v, rho = _as_state_values(state, p_max=p_max)
        instant_feasible = state.get("instant_feasible", instant_feasible)
        projection_feasible = state.get("projection_feasible", projection_feasible)
        if acceleration is None and "acceleration" in state:
            acceleration = state["acceleration"]
        if "a_max" in state:
            a_max = state["a_max"]
    if p is None or p_max is None or v is None or rho is None:
        raise RecoverabilityInputError("p, p_max, v and rho are required")
    if not isinstance(instant_feasible, bool) or not isinstance(projection_feasible, bool):
        raise RecoverabilityInputError("instant_feasible and projection_feasible must be bool")
    authority = authority_acceleration(rho, a_max=a_max) if acceleration is None else _finite("acceleration", acceleration)
    braking = braking_margin(p, p_max, v, authority)
    if method == METHOD_B1:
        accepted = instant_feasible
        action = "ACCEPT" if accepted else "REJECT"
        reason = "INSTANT_FEASIBLE" if accepted else "INSTANT_INFEASIBLE"
        scale = 1.0 if accepted else 0.0
    elif method == METHOD_P0:
        accepted = instant_feasible and projection_feasible
        action = "ACCEPT" if accepted else "REJECT"
        reason = "INSTANT_PROJECTION_FEASIBLE" if accepted else "INSTANT_PROJECTION_INFEASIBLE"
        scale = 1.0 if accepted else 0.0
    else:
        if not instant_feasible or not projection_feasible:
            accepted, action, reason, scale = False, "REJECT", "INSTANT_FILTER_INFEASIBLE", 0.0
        elif braking.safe:
            accepted, action, reason, scale = True, "ACCEPT", "BRAKING_MARGIN_NONNEGATIVE", 1.0
        elif allow_degrade and authority > 0.0 and p_max - float(p) > 0.0 and v != 0.0:
            max_speed = math.sqrt(max(0.0, 2.0 * authority * (p_max - float(p))))
            scale = min(1.0, max_speed / abs(float(v)))
            # A zero scale is still reported as a reduction; the pilot can
            # choose to terminate instead of issuing this degraded command.
            accepted, action, reason = False, "DEGRADE", "BRAKING_MARGIN_NEGATIVE"
        else:
            accepted, action, reason, scale = False, "REJECT", "NO_FINITE_RECOVERY", 0.0
    return MethodDecision(method, accepted, action, float(scale), reason,
                          instant_feasible, projection_feasible, braking)


def evaluate_b1(state: Mapping[str, Any] | None = None, **kwargs: Any) -> MethodDecision:
    return evaluate_method(METHOD_B1, state, **kwargs)


def evaluate_p0(state: Mapping[str, Any] | None = None, **kwargs: Any) -> MethodDecision:
    return evaluate_method(METHOD_P0, state, **kwargs)


def evaluate_p1(state: Mapping[str, Any] | None = None, **kwargs: Any) -> MethodDecision:
    return evaluate_method(METHOD_P1, state, **kwargs)


def four_leg_feature(
    state: Mapping[str, Any],
    *,
    position_key: str = "base_position_x",
    velocity_key: str = "base_velocity_x",
    p_max_key: str = "p_max_x",
    health_key: str = "health",
) -> dict[str, float]:
    """Map a four-leg state snapshot to the scalar pilot feature.

    The mapping is intentionally explicit and lossless for the pilot: the
    complete source snapshot remains in the case ledger, while this function
    records only which scalar components were consumed by the surrogate.
    """

    missing = [key for key in (position_key, velocity_key, p_max_key, health_key) if key not in state]
    if missing:
        raise RecoverabilityInputError(f"state missing feature keys: {missing}")
    return {
        "p": _finite(position_key, state[position_key]),
        "v": _finite(velocity_key, state[velocity_key]),
        "p_max": _finite(p_max_key, state[p_max_key]),
        "rho": _finite(health_key, state[health_key]),
    }


def evaluate_all_methods(state: Mapping[str, Any], **kwargs: Any) -> dict[str, MethodDecision]:
    """Evaluate the paired B1/P0/P1 arms without changing the input state."""

    return {method: evaluate_method(method, state, **kwargs) for method in METHODS}


def evaluate_batch(states: Sequence[Mapping[str, Any]], method: str, **kwargs: Any) -> list[MethodDecision]:
    """Deterministically map a pre-frozen sequence of states to decisions."""

    return [evaluate_method(method, state, **kwargs) for state in states]


__all__ = [
    "METHOD_B1", "METHOD_P0", "METHOD_P1", "METHODS",
    "RecoverabilityInputError", "BrakingAssessment", "MethodDecision",
    "authority_acceleration", "a_of_rho", "braking_margin", "h_stop", "stopping_margin", "recoverability_margin",
    "evaluate_method", "evaluate_b1", "evaluate_p0", "evaluate_p1",
    "four_leg_feature", "evaluate_all_methods", "evaluate_batch",
]
