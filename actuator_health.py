"""Single-source actuator health model for output torque capacity loss.

The model deliberately changes the physical output bounds, not the command
gain.  For actuator ``i`` it implements exactly

    u_health[i] = clip(u_raw[i], -rho[i] * limit[i],
                       +rho[i] * limit[i])

No simulator-specific transmission is applied here.  A future robot adapter
must convert this actuator-space output into generalized joint torque using the
audited transmission exactly once.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


Vector = tuple[float, ...]


def _finite_vector(name: str, values: Iterable[float]) -> Vector:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{name} must be a one-dimensional numeric iterable")
    try:
        vector = tuple(float(value) for value in values)
    except TypeError as exc:
        raise TypeError(
            f"{name} must be a one-dimensional numeric iterable"
        ) from exc
    except ValueError as exc:
        raise ValueError(f"{name} contains a non-numeric value") from exc

    if not vector:
        raise ValueError(f"{name} must not be empty")
    if not all(math.isfinite(value) for value in vector):
        raise ValueError(f"{name} must contain only finite values")
    return vector


def _clip_to_bounds(
    command: Vector, lower_bound: Vector, upper_bound: Vector
) -> tuple[Vector, tuple[bool, ...]]:
    applied = tuple(
        min(max(value, lower), upper)
        for value, lower, upper in zip(command, lower_bound, upper_bound)
    )
    saturated = tuple(
        value < lower or value > upper
        for value, lower, upper in zip(command, lower_bound, upper_bound)
    )
    return applied, saturated


def apply_physical_actuator_bounds(
    u_cmd: Iterable[float], torque_limits: Iterable[float]
) -> Vector:
    """Apply the healthy symmetric physical actuator limits.

    This function provides the correct comparison point for the rho=1 identity
    test: both the healthy path and the health-model path use physical clipping.
    """

    command = _finite_vector("u_cmd", u_cmd)
    limits = _finite_vector("torque_limits", torque_limits)
    if len(command) != len(limits):
        raise ValueError(
            "u_cmd and torque_limits must have the same actuator count"
        )
    if any(limit <= 0.0 for limit in limits):
        raise ValueError("torque_limits must be strictly positive")

    lower = tuple(-limit for limit in limits)
    applied, _ = _clip_to_bounds(command, lower, limits)
    return applied


@dataclass(frozen=True)
class ActuatorHealthResult:
    """Immutable result of one actuator-health application."""

    u_raw: Vector
    u_health: Vector
    rho: Vector
    saturation_mask: tuple[bool, ...]
    lower_bound: Vector
    upper_bound: Vector

    @property
    def u_cmd(self) -> Vector:
        """Deprecated compatibility alias for :attr:`u_raw`."""

        return self.u_raw

    @property
    def tau_applied(self) -> Vector:
        """Deprecated compatibility alias for :attr:`u_health`.

        This value is actuator-space command after health limiting.  It is not
        necessarily MuJoCo ``actuator_force`` or generalized joint torque.
        """

        return self.u_health

    def as_dict(self) -> dict[str, list[float] | list[bool]]:
        """Return a JSON-serializable representation."""

        return {
            "u_raw": list(self.u_raw),
            "u_health": list(self.u_health),
            # Deprecated keys remain for consumers of the STEP-01 scaffold.
            "u_cmd": list(self.u_raw),
            "tau_applied": list(self.u_health),
            "rho": list(self.rho),
            "saturation_mask": list(self.saturation_mask),
            "lower_bound": list(self.lower_bound),
            "upper_bound": list(self.upper_bound),
        }


class ActuatorHealthModel:
    """Oracle output-torque-capacity model used by STEP-01 and E0."""

    fault_model = "output_torque_capacity"

    def apply(
        self,
        u_cmd: Iterable[float],
        rho: Iterable[float],
        torque_limits: Iterable[float],
    ) -> ActuatorHealthResult:
        """Apply per-actuator health-scaled output bounds exactly once."""

        command = _finite_vector("u_cmd", u_cmd)
        health = _finite_vector("rho", rho)
        limits = _finite_vector("torque_limits", torque_limits)

        counts = {len(command), len(health), len(limits)}
        if len(counts) != 1:
            raise ValueError(
                "u_cmd, rho, and torque_limits must have the same actuator count"
            )
        if any(value < 0.0 or value > 1.0 for value in health):
            raise ValueError("rho values must lie in the closed interval [0, 1]")
        if any(limit <= 0.0 for limit in limits):
            raise ValueError("torque_limits must be strictly positive")

        upper = tuple(value * limit for value, limit in zip(health, limits))
        lower = tuple(-value for value in upper)
        applied, saturated = _clip_to_bounds(command, lower, upper)

        return ActuatorHealthResult(
            u_raw=command,
            u_health=applied,
            rho=health,
            saturation_mask=saturated,
            lower_bound=lower,
            upper_bound=upper,
        )
