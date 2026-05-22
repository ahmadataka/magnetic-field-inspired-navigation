# 2022 Paper Implementation Checklist

This checklist compares the current cleanroom implementation against the 2022
paper:

- Ataka, Lam, Althoefer (2022), "Magnetic-Field-Inspired Navigation for Robots
  in Complex and Unknown Environments"
- Frontiers in Robotics and AI
- URL: https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2022.834177/full

Purpose:

- identify what the current code already matches
- identify what needs refactoring to match the paper more faithfully
- identify what is still missing

Reference code:

- [src/mfinav/double_integrator.py](/Users/ahmadataka/Documents/Bitbucket%20-%20Ataka/magnetic-field-inspired-navigation/src/mfinav/double_integrator.py:1)

## Key Paper Requirements

From the 2022 paper:

- The robot is modeled as a point-mass / double-integrator style system in the
  implementation section.
- The control law consists of goal attraction plus obstacle avoidance.
- Obstacle avoidance is composed of two fields:
  - boundary-following field `F_b`
  - collision-avoidance field `F_a`
- `F_b` is activated when the robot is within threshold `r_l`.
- `F_a` is activated when the robot gets even closer, within threshold `r_la`.
- The algorithm relies on local sensor information:
  - closest obstacle point `r_o`
  - local surface normal assumption from `r_o`
- The paper also includes an extension for non-unique closest points using
  averaging over sensed points.
- In the implementation section, PD goal attraction is used, and a geometric
  control term is introduced as an alternative to avoid undesirable speed
  changes.

Source anchors:

- Problem formulation and local sensing assumptions: lines 367-390 of the
  Frontiers page.
- Obstacle field decomposition and local obstacle-current construction: lines
  394-414.
- Goal convergence section and PD controller role: lines 471-479.
- Implementation details and threshold activation: lines 497-501.
- Special-case extension for non-unique closest points: lines 482-492.

## Already Matches The 2022 Paper

- `DoubleIntegratorState` and the simulation update follow a point-mass style
  dynamic model.
- The total control is split into goal plus obstacle terms in
  `ReferenceNavigator.command`.
- The obstacle term uses a local closest-obstacle vector rather than a global
  preplanned path.
- The obstacle current direction is built from projection of motion direction
  onto a local obstacle tangent surrogate, which is conceptually aligned with
  the paper.
- The code includes a speed-preserving / geometric-style steering component in
  the goal controller, which is directionally consistent with the implementation
  discussion in the paper.

## Needs Refactoring To Match The Paper More Faithfully

- The paper explicitly separates obstacle avoidance into `F_b` and `F_a`.
  The current code merges this behavior into one magnetic term plus one extra
  repulsive correction.
- The paper uses two activation thresholds, `r_l` and `r_la`.
  The current code uses `bound` and `bound_add`, but not in the exact paper
  structure.
- The paper’s implementation section describes PD goal attraction and says the
  geometric control term is introduced as an alternative.
  The current code uses a hybrid switching controller that is inspired by the
  ROS implementation rather than reconstructed directly from the paper.
- The current `GoalRelaxationController` is inherited from the legacy ROS code.
  It is not clearly part of the 2022 paper’s core implementation description.
- The current obstacle model is a 2D analytic circle. The paper assumes local
  sensed obstacle data and local surface geometry rather than a hard-coded
  geometric primitive.
- The current magnetic term is a compact 2D analogue. It is structurally
  related to the paper, but not yet a direct implementation of the paper’s
  `F_b` and `F_a` equations.

## Missing From The Current Implementation

- Explicit `F_b` implementation as its own named field.
- Explicit `F_a` implementation as its own named field.
- Separate threshold logic for `r_l` and `r_la` using the paper’s semantics.
- Local sensing abstraction that returns closest obstacle point data rather than
  querying a known obstacle object directly.
- The 2022 paper’s extension for non-unique closest points via averaging over
  nearby sensed points.
- Support for non-smooth and concave obstacle cases beyond a single circle.
- A paper-oriented parameter set matching the scenarios described in the 2022
  article.
- Benchmark scenarios that mirror the paper’s comparative tests.

## Current Verdict

The current implementation is:

- consistent with the paper at a high architectural level
- consistent with the old ROS code that inspired the paper implementation
- not yet a faithful implementation of the 2022 paper itself

In short:

- it is a useful MFI-inspired cleanroom prototype
- it is not yet a paper-faithful 2022 reference implementation

## Recommended Next Steps

1. Split the obstacle term into explicit `BoundaryFollowingField` and
   `CollisionAvoidanceField` classes.
2. Replace `GoalRelaxationController` with a paper-faithful goal module, or
   clearly mark goal relaxation as an optional legacy extension.
3. Introduce a `LocalSensingObservation` data structure:
   - closest obstacle point
   - estimated surface normal
   - optional averaged closest-point surrogate
4. Implement the non-unique closest-point extension from Section 4.4.
5. Add one benchmark scenario that is deliberately designed to distinguish:
   - current simplified implementation
   - paper-faithful implementation
6. Update
   [docs/algorithm_and_model_changes.tex](/Users/ahmadataka/Documents/Bitbucket%20-%20Ataka/magnetic-field-inspired-navigation/docs/algorithm_and_model_changes.tex:1)
   when the implementation changes materially.
