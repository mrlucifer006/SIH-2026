# MATLAB/Simulink integration

`sih_autonomy_step.m` is the first working closed-loop autonomy block for the
car model. Add a MATLAB Function block and call it once per simulation step:

```matlab
[command, tracks, path, debug] = sih_autonomy_step( ...
    egoState, cameraDetections, lidarDetections, radarDetections, ...
    staticObstacles, goal, struct('maxSpeed', 6));
```

Inputs use the following compact formats:

- `egoState`: `[x y yaw speed]`
- camera rows: `[x y confidence]`
- LiDAR rows: `[x y radius confidence]`
- radar rows: `[x y vx vy confidence]`
- static obstacles: `[x y radius]`
- goal: `[x y]`

The output `command` is `[steering; throttle; brake]`. The controller predicts
moving detections for 0.8 seconds, fuses detections by proximity, inflates them
by the vehicle safety envelope, plans with local 8-connected A*, and follows
the path using pure pursuit. If the predicted clearance is below the emergency
threshold it commands braking.

For a raw camera image, insert your camera detector before this block. The
block is intentionally detector-agnostic so it works immediately with the
ground-truth/object-detection outputs from Automated Driving Toolbox and can
later accept an IDD-trained detector without changing planning or control.

## Run the actual Python pipeline from Simulink

Use `sih_python_step.m` when you want the same Python components used by the
project (`CudaPerception`, `MotionPredictor`, `AStarPlanner`, and
`BicycleController`). Add an **Interpreted MATLAB Function** block and set its
function to:

```matlab
sih_python_step(u1,u2,u3,u4,u5,u6)
```

Connect the six inputs in the same order as `sih_autonomy_step`. In MATLAB,
configure the Python environment once before starting the simulation:

```matlab
pyenv(Version='F:\Personal Projects\SIH 2026\venv\Scripts\python.exe')
addpath('F:\Personal Projects\SIH 2026\matlab')
```

This mode requires an Interpreted MATLAB Function block because regular
code-generating MATLAB Function blocks cannot call arbitrary Python objects.
