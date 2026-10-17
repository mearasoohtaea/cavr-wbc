# CAVR-WBC

**Damage-conditioned task reconfiguration with recoverability-aware whole-body control.**

Research in progress. The current research direction is **task-constrained temporal
reachability of contact loads and anticipatory load allocation**. The central
question is whether a robot can realize a required load-transfer trajectory in
time while preserving its task after actuator-capacity loss.

This repository retains the previously released first-party Python development
snapshot. The research-direction update below does not replace that source release
or claim that the new controller and experiments have already been implemented.

## Code

- actuator_health.py: actuator-capacity bounds applied exactly once.
- recoverability.py: deterministic stopping-margin and task-screening mechanisms.
- feedback_backup.py: state-dependent reduced-order backup policy.
- cavr_wbc_snapshot.zip: all 455 Python modules from the released development
  package, including platform interfaces, constrained control, independent
  evaluation, experiment runners and logging.
- SOURCE_MANIFEST.json and verify_snapshot.py: source provenance and SHA-256 checks.

The three readable root modules are unchanged copies of modules in the archive.
The archived package is the canonical source of that released snapshot, identified
by SOURCE_MANIFEST.json; it is not a release of the new load-reachability method.

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

## Current progress — 2026-10-17

The previous task/contact joint-QP candidate (STEP68, J) is **NO_GO**: a distinct
mechanism and complete-task benefit over strong fixed controls have not been
established. STEP67 remains **INCOMPLETE**; its missing pulse results are not
reclassified as physical failures.

The new route studies **temporally executable contact-load paths under task and
shared actuator constraints**. A scalar interval-reachability example has been
derived, but its assumptions have not yet been validated on the original robot.
The Go2 model uses direct-torque motors without activation states, so an artificial
load-rate limit or actuator lag must not be introduced to manufacture a bottleneck.

**STEP69 is planned, not executed.** It will test actual reaction-force changes at
fixed states and the subsequent 100 ms response, with limits of 64 new QP calls,
500 integration steps and 400 explicit forward calls. This pilot does not test
four-second task improvement, establish inevitable failure after delayed control,
or demonstrate hard-real-time deployment.

See [research progress and direction](docs/RESEARCH_PROGRESS.md) for the preserved
outcomes, analytical starting point, budget status and requirements for a T-RO
contribution. Historical experimental results and the released source archive
remain unchanged.

## Earlier snapshot progress — 2026-10-13

Implemented actuator-path auditing, capacity-loss modeling, constrained WBC,
task-management/recovery components, paired counterfactual evaluation and replay.
At that snapshot, the frozen batch had known outcomes for all 176 evaluation positions;
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
当前主攻已转为任务约束下的接触载荷时序可达性与提前分配；STEP68旧候选保持NO_GO，
STEP67保持INCOMPLETE。STEP69仅完成规划，尚未执行，不宣称新方法、完整任务收益或实时部署已成立。
已公开的源码快照及此前176位评测记录作为历史资产保留，不代表新路线代码已经发布。
详见[研究进度与路线转变](docs/RESEARCH_PROGRESS.md)。
本仓库不作为已经发表或已完成论文的正式代码发布。

## License

First-party code: MIT. External simulators, solvers, robot assets and their
licenses remain separate. CITATION.cff describes this software, not a published paper.
