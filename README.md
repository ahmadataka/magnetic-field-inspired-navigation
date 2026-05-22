# Magnetic-Field-Inspired Navigation

This is a cleanroom reference implementation of the double-integrator branch of the
magnetic-field-inspired navigation method from the `double_integrator` ROS repo.

Current scope:

- 2D double-integrator dynamics
- goal-relaxation weighting inspired by `nodes/ataka_listener.py`
- obstacle avoidance field inspired by `nodes/collision_avoidance_okt.py` with `field=2`
- simple circular-obstacle scenarios for validating the control logic without ROS
- documented algorithm/model changes in `docs/algorithm_and_model_changes.md`

## Quickstart

Create a virtual environment and install the package:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ./magnetic_field_inspired_navigation
```

Run the reference scenario:

```bash
python3 magnetic_field_inspired_navigation/scripts/run_reference.py
```

The script prints rollout metrics and writes a trajectory CSV to:

```text
magnetic_field_inspired_navigation/artifacts/reference_trajectory.csv
```

## Mapping Back To The ROS Repo

This reference implementation mirrors the high-level structure of:

- `nodes/multi_agent_listener.py` / `nodes/ataka_listener.py`
- `nodes/collision_avoidance_okt.py` with `field == 2`
- `nodes/system_motion.py`

It does not yet include:

- ROS topics / TF
- point-cloud obstacle extraction
- nonholonomic mobile robot adapter
- 3D obstacle surfaces
- manipulator or quadcopter adapters

## Change Log Requirement

Major algorithm or model changes must be documented in:

- [docs/algorithm_and_model_changes.tex](/Users/ahmadataka/Documents/Bitbucket%20-%20Ataka/double_integrator/magnetic_field_inspired_navigation/docs/algorithm_and_model_changes.tex:1)

Read the rendered PDF for equations.
