# CAVR-WBC

**Damage-conditioned task reconfiguration with recoverability-aware whole-body control.**

Research in progress by me. This repository releases the current first-partyme
Python source as a development snapshot. It studies how a legged robot can adjust
its task after actuator-capacity loss while preserving a feasible recovery path.

## Code

- actuator_health.py: actuator-capacity bounds applied exactly once.
- recoverability.py: deterministic stopping-margin and task-screening mechanisms.
- feedback_backup.py: state-dependent reduced-order backup policy.
- cavr_wbc_snapshot.zip: all 455 Python modules from the current development
  package, including platform interfaces, constrained control, independent
  evaluation, experiment runners and logging.
- SOURCE_MANIFEST.json and verify_snapshot.py: source provenance and SHA-256 checks.

The three readable root modules are unchanged copies of modules in the archive.
The archived package is the canonical development source.

## Quick start

~~~bash
python verify_snapshot.py
python -m zipfile -e cavr_wbc_snapshot.zip .
python -c "from cavr_wbc.models import ActuatorHealthModel; print(ActuatorHealthModel().apply([12.0], [0.5], [10.0]).u_health)"
~~~

The model-independent example uses Python's standard library. MuJoCo/QP modules
also require the packages in requirements.txt. Some advanced experiment runners
reference separately frozen configurations, native solver builds and artifact
folders; they are development code rather than a complete one-command paper reproduction.

## Current progress — 2026-10-13

Implemented actuator-path auditing, capacity-loss modeling, constrained WBC,
task-management/recovery components, paired counterfactual evaluation and replay.
The latest frozen batch has known outcomes for all 176 evaluation positions;
108 trajectories completed the full four-second horizon. These are engineering
and evaluation-completion counts across all comparison arms.

The negative and positive confirmation gates remain unmet. Full-algorithm
independent numerical reproduction remains unverified. No method-wide
superiority, global safety guarantee or completed hardware validation is claimed.

## Robot assets and external dependencies

Obtain Go2 assets separately from
[Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco), pinned upstream
commit 1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d.
The development adapter expects the Go2 scene under
third_party/unitree_mujoco_snapshot/unitree_robots/go2/scene.xml.
Those assets retain their upstream BSD-3-Clause license and are not redistributed
here. Native binaries and large experiment data are not included.

## 中文介绍

面向四足机器人执行器能力下降，研究任务重构、约束全身控制与可恢复性。
当前公开的是在研源码快照，包含实际已有的模型、控制、评测和日志模块。
已完成本批176个评测位置的处理；确认实验、完整独立复现及后续验证仍在推进。
本仓库不作为已经发表或已完成论文的正式代码发布。

## License

First-party code: MIT. External simulators, solvers, robot assets and their
licenses remain separate. CITATION.cff describes this software, not a published paper.
