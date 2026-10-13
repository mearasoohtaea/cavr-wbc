"""STEP15 current-state feedback backup policy.

This is a reduced-order semantic implementation only.  It provides an
executable, state-dependent action trajectory for the next protocol gate; it
does not claim full-body recoverability, stability, viability, or real-time
performance.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from typing import Any, Mapping


class FeedbackBackupInputError(ValueError):
    """Raised for malformed or non-finite policy inputs."""


@dataclass(frozen=True)
class FrozenBackupConfig:
    """All policy gains, limits, units and terminal predicates."""

    schema: str = "step15_feedback_backup_config_v1"
    dt_s: float = 0.05
    horizon_s: float = 1.0
    kp: float = 1.25
    kd: float = 0.75
    target_margin_m: float = 0.05
    max_acceleration_mps2: float = 1.0
    terminal_speed_mps: float = 0.02
    terminal_margin_m: float = 0.01
    friction_min: float = 0.05
    actuator_limit_normalized: float = 1.0
    channel: str = "base_x_acceleration"
    action_unit: str = "m/s^2"
    contact_predicate: str = "current_state.contact_is_valid"
    friction_predicate: str = "current_state.friction_coeff >= friction_min"
    actuator_predicate: str = "abs(action.value) <= current_state.actuator_limit"

    def as_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result.update({
            "horizon_ticks": int(round(self.horizon_s / self.dt_s)),
            "terminal_set": {
                "speed_mps_max": self.terminal_speed_mps,
                "boundary_margin_m_min": self.terminal_margin_m,
                "contact_required": True,
                "friction_margin_min": 0.0,
                "actuator_margin_min": 0.0,
            },
            "read_set": ["current_state", "health", "task_reference", "frozen_config"],
            "health_bounds": [0.0, 1.0],
        })
        result["config_hash"] = _sha(result)
        return result

    @property
    def horizon_ticks(self) -> int:
        return int(round(self.horizon_s / self.dt_s))

    @property
    def terminal_set(self) -> dict[str, Any]:
        return self.as_dict()["terminal_set"]

    @property
    def config_hash(self) -> str:
        return self.as_dict()["config_hash"]


def _sha(value: Any) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _finite(name: str, value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise FeedbackBackupInputError(f"{name} must be finite") from exc
    if not math.isfinite(number):
        raise FeedbackBackupInputError(f"{name} must be finite")
    return number


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


class FeedbackBackupPolicy:
    """Tick-wise current-state feedback governor.

    The policy reads only the supplied current-state snapshot, health, task
    reference and frozen configuration.  Each call emits a new action from
    the current position/velocity/health; it never receives an evaluator
    result or a future label.
    """

    _FORBIDDEN_KEYS = frozenset({
        "future", "future_state", "future_" + "label", "terminal_" + "label",
        "evaluator", "evaluator_output", "evaluator_residual", "label_cache",
        "braking_margin", "independent_rollout", "future_trajectory",
    })

    def __init__(self) -> None:
        self._config: FrozenBackupConfig | None = None
        self._config_hash: str | None = None
        self._backup_id: str | None = None
        self._tick = 0

    @property
    def config(self) -> FrozenBackupConfig:
        if self._config is None:
            raise FeedbackBackupInputError("policy must be reset before compute")
        return self._config

    @property
    def config_hash(self) -> str:
        if self._config_hash is None:
            raise FeedbackBackupInputError("policy must be reset before compute")
        return self._config_hash

    @property
    def backup_id(self) -> str:
        if self._backup_id is None:
            raise FeedbackBackupInputError("policy must be reset before compute")
        return self._backup_id

    def reset(self, frozen_config: FrozenBackupConfig | Mapping[str, Any] | None = None) -> dict[str, Any]:
        """Freeze configuration and start a new trajectory identity."""
        if frozen_config is None:
            config = FrozenBackupConfig()
        elif isinstance(frozen_config, FrozenBackupConfig):
            config = frozen_config
        elif isinstance(frozen_config, Mapping):
            allowed = set(FrozenBackupConfig.__dataclass_fields__)
            unknown = set(frozen_config) - allowed
            if unknown:
                raise FeedbackBackupInputError(f"unknown frozen config fields: {sorted(unknown)}")
            config = FrozenBackupConfig(**dict(frozen_config))
        else:
            raise FeedbackBackupInputError("frozen_config must be FrozenBackupConfig or mapping")
        for name in ("dt_s", "horizon_s", "kp", "kd", "target_margin_m", "max_acceleration_mps2", "terminal_speed_mps", "terminal_margin_m", "friction_min", "actuator_limit_normalized"):
            value = _finite(name, getattr(config, name))
            if value < 0.0:
                raise FeedbackBackupInputError(f"{name} must be non-negative")
        if config.dt_s <= 0.0 or config.horizon_s <= 0.0 or config.max_acceleration_mps2 <= 0.0:
            raise FeedbackBackupInputError("dt, horizon and maximum acceleration must be positive")
        self._config = config
        self._config_hash = config.config_hash
        self._backup_id = f"S15-FB-{self._config_hash[:16]}"
        self._tick = 0
        return {"config": config.as_dict(), "config_hash": self._config_hash, "backup_id": self._backup_id}

    def compute(self, current_state: Mapping[str, Any], health: float, task_reference: Mapping[str, Any]) -> dict[str, Any]:
        """Compute one feedback action using only current-state inputs."""
        config = self.config
        if not isinstance(current_state, Mapping) or not isinstance(task_reference, Mapping):
            raise FeedbackBackupInputError("current_state and task_reference must be mappings")
        forbidden = self._FORBIDDEN_KEYS.intersection(current_state) | self._FORBIDDEN_KEYS.intersection(task_reference)
        if forbidden:
            raise FeedbackBackupInputError(f"future/evaluator fields are forbidden: {sorted(forbidden)}")
        required = ("p", "v", "p_max", "contact", "friction_coeff", "actuator_limit")
        missing = [key for key in required if key not in current_state]
        if missing:
            raise FeedbackBackupInputError(f"current_state missing {missing}")
        p = _finite("current_state.p", current_state["p"])
        velocity = _finite("current_state.v", current_state["v"])
        p_max = _finite("current_state.p_max", current_state["p_max"])
        friction = _finite("current_state.friction_coeff", current_state["friction_coeff"])
        actuator_limit = _finite("current_state.actuator_limit", current_state["actuator_limit"])
        rho = _finite("health", health)
        if not 0.0 <= rho <= 1.0:
            raise FeedbackBackupInputError("health must lie in [0,1]")
        if actuator_limit < 0.0 or friction < 0.0:
            raise FeedbackBackupInputError("current physical limits must be non-negative")
        contact = current_state["contact"]
        if isinstance(contact, Mapping):
            if len(contact) != 4:
                raise FeedbackBackupInputError("contact mapping must contain four support entries")
            contact_ok = all(bool(x) for x in contact.values())
        elif isinstance(contact, (list, tuple)):
            if len(contact) != 4:
                raise FeedbackBackupInputError("contact sequence must contain four support entries")
            contact_ok = all(bool(x) for x in contact)
        else:
            contact_ok = bool(contact)
        target_speed = _finite("task_reference.target_velocity_m_s", task_reference.get("target_velocity_m_s", task_reference.get("target_speed", 0.0)))
        target_position = _finite("task_reference.target_position", task_reference.get("target_position", p_max - config.target_margin_m))
        gap = p_max - p
        position_error = target_position - p
        raw = config.kp * position_error + config.kd * (target_speed - velocity)
        authority = min(config.max_acceleration_mps2 * rho, actuator_limit, config.actuator_limit_normalized)
        friction_ok = friction >= config.friction_min
        actuator_ok = actuator_limit > 0.0
        if rho <= 0.0 or not contact_ok or not friction_ok or not actuator_ok:
            mode = "FALLBACK_NO_AUTHORITY" if rho <= 0.0 else "STOP"
            action_value = 0.0
            reason = "zero_health_or_current_physical_predicate"
        else:
            action_value = _clamp(raw, -authority, authority)
            mode = "BRAKE" if action_value < 0.0 else "GOVERN"
            reason = "state_feedback_gap_velocity_health_limited"
        action = {
            "channel": config.channel,
            "unit": config.action_unit,
            "value": float(action_value),
            "health_limit": float(authority),
            "tick": self._tick,
        }
        state_hash = _sha({k: current_state[k] for k in sorted(current_state)})
        task_hash = _sha(dict(task_reference))
        result = {
            "backup_action": action,
            "acceleration_mps2": float(action_value),
            "units": config.action_unit,
            "channel": config.channel,
            "mode": mode,
            "fallback": "NO_AUTHORITY_SAFE_STOP" if mode == "FALLBACK_NO_AUTHORITY" else None,
            "backup_id": self.backup_id,
            "diagnostics": {
                "reason": reason,
                "p": p,
                "v": velocity,
                "gap_m": gap,
                "position_error_m": position_error,
                "raw_command": raw,
                "authority": authority,
                "contact_ok": contact_ok,
                "friction_ok": friction_ok,
                "actuator_ok": actuator_ok,
                "state_hash": state_hash,
                "task_reference_hash": task_hash,
                "config_hash": self.config_hash,
                "reads": ["current_state", "health", "task_reference", "frozen_config"],
                "forbidden_reads": sorted(self._FORBIDDEN_KEYS),
            },
        }
        self._tick += 1
        return result


__all__ = ["FrozenBackupConfig", "FeedbackBackupPolicy", "FeedbackBackupInputError"]
