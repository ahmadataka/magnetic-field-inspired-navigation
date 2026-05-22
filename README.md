# Magnetic-Field-Inspired Navigation

This repository is a clean Python reimplementation of the magnetic-field-inspired
navigation line of work developed by Ahmad Ataka and collaborators. The goal is
to turn the earlier research codebase into a reusable, readable, well-documented
package that other people can study, run, and extend.

The current focus is a 2D double-integrator benchmark platform with local
sensing, closest-point averaging, polygon and circle obstacles, and comparison
against a standard artificial potential field baseline.

## Algorithm In Brief

The method treats obstacle avoidance as a local field-generation problem inspired
by electromagnetism.

- The robot is always attracted toward the goal with a goal term `F_g`.
- When an obstacle is sensed within a boundary-following range `r_l`, the robot
  generates a tangential avoidance field `F_b` that steers motion along the
  obstacle boundary instead of directly into it.
- When the robot gets even closer, inside a stricter range `r_la`, an additional
  collision-avoidance term `F_a` pushes the robot away from the obstacle.
- The final command is the sum of the goal and obstacle terms:

```text
u = F_g + F_b + F_a
```

In the paper lineage, the key ideas are:

- no prior map is required
- only spatially and temporally local obstacle information is used
- the robot is guided around obstacles without the usual local-minima behavior
  associated with standard artificial potential fields
- the same idea can be adapted across multiple robot models, including mobile
  robots, 3D point-like robots, quadcopters, manipulators, and soft robots

For the math and implementation deltas in this repo, see:

- `docs/algorithm_and_model_changes.tex`
- `docs/algorithm_and_model_changes.pdf`

## Related Papers

This repository is based on the following magnetic-field-inspired navigation
papers.

1. Ahmad Ataka, Hak-Keung Lam, Kaspar Althoefer. *Reactive Magnetic-field-inspired Navigation for Non-holonomic Mobile Robots in Unknown Environments* (ICRA 2018).
   Link: https://kclpure.kcl.ac.uk/portal/en/publications/reactive-magnetic-field-inspired-navigation-for-non-holonomic-mob
   PDF: https://qmro.qmul.ac.uk/xmlui/bitstream/handle/123456789/57804/Rizqi%20Reactive%20Magnetic-field%202018%20Accepted.pdf?isAllowed=y&sequence=2
   DOI: https://doi.org/10.1109/ICRA.2018.8463203

2. Ahmad Ataka, Hak-Keung Lam, Kaspar Althoefer. *Reactive Magnetic-field-inspired Navigation Method for Robots in Unknown Convex 3D Environments* (IEEE Robotics and Automation Letters, 2018).
   Link: https://kclpure.kcl.ac.uk/portal/en/publications/reactive-magnetic-field-inspired-navigation-method-for-robots-in--2
   DOI: https://doi.org/10.1109/LRA.2018.2853801

3. Ahmad Ataka, Ali Shiva, Hak-Keung Lam, Kaspar Althoefer. *Magnetic-field-inspired Navigation for Soft Continuum Manipulator* (IROS 2018).
   PDF: https://kclpure.kcl.ac.uk/portal/files/99543280/iros2018final.pdf
   DOI: https://doi.org/10.1109/IROS.2018.8593666

4. Ahmad Ataka, Hak-Keung Lam, Kaspar Althoefer. *Magnetic-field-inspired Navigation for Quadcopter Robot in Unknown Environments* (ICRA 2019).
   PDF: https://qmro.qmul.ac.uk/xmlui/bitstream/handle/123456789/70112/Ypsilanti%20Magnetic-field-inspired%202019%20Accepted.pdf?isAllowed=y&sequence=2
   DOI: https://doi.org/10.1109/ICRA.2019.8793682

5. Ahmad Ataka, Hak-Keung Lam, Kaspar Althoefer. *Magnetic-Field-Inspired Navigation for Robots in Complex and Unknown Environments* (Frontiers in Robotics and AI, 2022).
   Article: https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.834177/full
   DOI: https://doi.org/10.3389/frobt.2022.834177

## Current Scope

- 2D double-integrator dynamics
- local sensing with closest-point averaging
- circle and polygon obstacle models
- benchmark scenarios for convex and non-convex environments
- artificial potential field baseline for comparison
- documented algorithm/model changes with equations

## Current Status

This is an active cleanroom rebuild, not a finished reproduction of every paper.

- The benchmark and evaluation tooling are now much cleaner and more honest than
  the original ROS-era research code.
- Polygonal obstacles and signed-clearance evaluation are supported.
- The method avoids hard penetration more reliably than before.
- The harder polygon benchmark cases are still being tuned so that the
  magnetic-field-inspired controller both avoids obstacles and finishes at the
  goal consistently.

## Quickstart

Create a virtual environment and install the package:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e .
```

Run the reference scenario:

```bash
python3 scripts/run_reference.py
```

Run the benchmark suite:

```bash
python3 scripts/run_benchmarks.py
```

Artifacts are written to:

```text
artifacts/
```

## Mapping Back To The Legacy ROS Repo

This repository started as a clean extraction from the older `double_integrator`
ROS implementation. The main legacy references were:

- `nodes/ataka_listener.py`
- `nodes/multi_agent_listener.py`
- `nodes/collision_avoidance_okt.py`
- `nodes/system_motion.py`

This repo intentionally does not depend on ROS and is being reorganized as a
general Python package first, with platform-specific adapters coming later.
