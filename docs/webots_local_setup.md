# Webots Local Setup

This repo now includes a first local Webots integration for quick controller development on macOS:

- World: `webots/worlds/mfi_epuck_arena.wbt`
- Controller: `webots/controllers/mfi_epuck_python/mfi_epuck_python.py`

## What It Does

- uses a built-in `E-puck` robot
- places a few box obstacles and a goal marker in a small arena
- runs a Python controller that imports `mfinav`
- uses the current `paper_geometric` differential-drive guidance path

## Current Assumption

This first bridge is a **development bridge**, not the final realistic sensing setup.

The e-puck is run as a `Supervisor`, so the controller reads:

- its own pose from Webots ground truth
- obstacle poses from Webots ground truth

That lets us validate:

- Webots project structure
- controller import path
- wheel command mapping
- basic closed-loop behavior inside a real simulator

The next realism step would be to replace the ground-truth obstacle geometry with:

- e-puck proximity sensors
- wheel odometry
- optionally a state estimator

## How To Open

1. Open Webots.
2. Open `webots/worlds/mfi_epuck_arena.wbt`.
3. Run the simulation.

If the controller cannot import the Webots Python module automatically, check:

- `WEBOTS_HOME=/Applications/Webots.app/Contents`

The controller already appends:

- `/Applications/Webots.app/Contents/lib/controller/python`
- this repo's `src/`

to `sys.path`.

## Notes

- This is the cleanest first local path on Mac.
- It is especially useful for mobile-robot algorithm iteration before moving to ROS or a heavier Linux-only stack.
