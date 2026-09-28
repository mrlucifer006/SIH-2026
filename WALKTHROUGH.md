# SIH 2026 — Python + CUDA Walkthrough

## What is included

This is a runnable, software-only prototype of the proposed adaptive path-planning system. It keeps the five Indian-road scenarios in the project brief, while replacing proprietary simulation dependencies with a portable Python implementation.

`sih_sim` runs the closed loop below every 100 ms:

1. Mixed-traffic actors move (cars, rickshaws, carts, pedestrians, cattle, and trucks).
2. A short-horizon probabilistic predictor samples their likely positions.
3. PyTorch creates and dilates an occupancy grid on CUDA when a GPU is available.
4. A* replans a collision-free route over that grid.
5. A pure-pursuit controller drives a kinematic bicycle model along the new route.

The simulation is intentionally dataset-free so the demo runs immediately. A production perception model trained on IDD can replace the idealised actor detections at the `CudaPerception` boundary.

## Files

| Path | Purpose |
|---|---|
| `sih_sim/scenarios.py` | Five specified road environments and mixed traffic actors |
| `sih_sim/perception.py` | CUDA occupancy-grid rasterisation and safety dilation |
| `sih_sim/prediction.py` | CUDA-batched stochastic motion forecasts |
| `sih_sim/planner.py` | Collision-free 8-connected A* planner |
| `sih_sim/controller.py` | Pure-pursuit bicycle-model controller |
| `sih_sim/engine.py` | Perception → prediction → planning → control loop |
| `sih_sim/cli.py` | Command-line runner, metrics report, and route figures |
| `tests/test_simulation.py` | Fast structural/regression checks |

## Setup

Use Python 3.10–3.13 in a virtual environment. Python 3.14 may not yet have compatible PyTorch wheels.

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
```

### CUDA GPU setup (recommended)

Install the PyTorch build matching your NVIDIA driver/CUDA support from [pytorch.org](https://pytorch.org/get-started/locally/), then install the remaining packages:

```powershell
pip install torch --index-url https://download.pytorch.org/whl/cu128
pip install numpy matplotlib pytest
pip install -e . --no-deps
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

If the final command prints `True`, CUDA occupancy-grid construction and prediction sampling will run on the GPU. Use the exact index URL recommended by PyTorch if your platform uses another CUDA release.

### CPU fallback

The prototype also works without NVIDIA CUDA:

```powershell
pip install -r requirements.txt
pip install -e . --no-deps
```

## Run the demonstration

Run all five scenarios, save dashboard metrics and a final-route PNG for each:

```powershell
python -m sih_sim.cli --scenario all --device auto --plot --output outputs
```

Force CUDA and fail early if it cannot be used:

```powershell
python -m sih_sim.cli --scenario market --device cuda --plot
```

Force CPU for comparison:

```powershell
python -m sih_sim.cli --scenario village --device cpu
```

The command prints one JSON record per scenario and writes `outputs/metrics.json`. Each record reports planning latency, minimum clearance, collisions, goal completion, number of replans, and compute device. Generated PNGs show the ego trajectory, latest safe route, moving actors, and goal.

## Run checks

```powershell
pytest -q
```

## Interpreting the results

`mean_replan_ms` is the end-to-end prediction, occupancy-grid and A* planning time. The design target is below 200 ms. `min_clearance_m` stays positive when the ego vehicle’s safety envelope has not overlapped an actor or static obstacle. `collision: false` is required for a passing episode.

The simulator makes no claim of validated real-world safety or IDD detection accuracy; those require trained perception models, a calibrated sensor model, and scenario validation. Its role is a fast, inspectable baseline for the SIH software demo.
