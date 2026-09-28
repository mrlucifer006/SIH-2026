import argparse
import json
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.animation as animation
import numpy as np
import imageio_ffmpeg

mpl.rcParams['animation.ffmpeg_path'] = imageio_ffmpeg.get_ffmpeg_exe()

from .config import SimulationConfig
from .engine import SimulationEngine
from .scenarios import build_scenarios


def _render(scenario, frames, destination: Path) -> None:
    final = frames[-1]
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_facecolor("#f5f0df")
    for x, y, radius in scenario.static_obstacles:
        ax.add_patch(plt.Circle((x, y), radius, color="#6c5b4c", label="static obstacle"))
    for actor, position in zip(scenario.agents, final["agents"]):
        color = {"pedestrian": "#f28e2b", "cattle": "#8c564b", "rickshaw": "#59a14f"}.get(actor.kind, "#4e79a7")
        ax.add_patch(plt.Circle(position, actor.radius, color=color, alpha=.85))
        ax.text(*position, actor.kind[0].upper(), ha="center", va="center", fontsize=7)
    if len(final["path"]):
        ax.plot(final["path"][:, 0], final["path"][:, 1], "--", color="#e15759", linewidth=1.5, label="latest planned path")
    trace = [frame["ego"] for frame in frames]
    ax.plot([p[0] for p in trace], [p[1] for p in trace], color="#111111", linewidth=2, label="ego trajectory")
    ax.scatter(*scenario.start, marker="o", color="#2ca02c", s=70, label="start")
    ax.scatter(*scenario.goal, marker="*", color="#d62728", s=130, label="goal")
    ax.set(xlim=(0, 60), ylim=(0, 36), aspect="equal", title=f"{scenario.name.title()} — adaptive route")
    ax.legend(loc="upper left", ncol=2, fontsize=8)
    if final.get("occupancy") is not None:
        ax.imshow(final["occupancy"], extent=(0, 60, 0, 36), origin='lower', cmap='Reds', alpha=0.3, vmin=0, vmax=1)
    fig.tight_layout()
    fig.savefig(destination, dpi=150)
    plt.close(fig)


def _render_video(scenario, frames, destination: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_facecolor("#f5f0df")
    for x, y, radius in scenario.static_obstacles:
        ax.add_patch(plt.Circle((x, y), radius, color="#6c5b4c", label="static obstacle"))
    ax.scatter(*scenario.start, marker="o", color="#2ca02c", s=70, label="start")
    ax.scatter(*scenario.goal, marker="*", color="#d62728", s=130, label="goal")
    ax.set(xlim=(0, 60), ylim=(0, 36), aspect="equal", title=f"{scenario.name.title()} — adaptive route")
    ax.legend(loc="upper left", ncol=2, fontsize=8)
    
    occupancy_img = ax.imshow(np.zeros((72, 120)), extent=(0, 60, 0, 36), origin='lower', cmap='Reds', alpha=0.3, vmin=0, vmax=1)
    
    fig.tight_layout()

    actor_patches = []
    actor_texts = []
    for actor in scenario.agents:
        color = {"pedestrian": "#f28e2b", "cattle": "#8c564b", "rickshaw": "#59a14f"}.get(actor.kind, "#4e79a7")
        patch = plt.Circle((0, 0), actor.radius, color=color, alpha=.85)
        ax.add_patch(patch)
        actor_patches.append(patch)
        text = ax.text(0, 0, actor.kind[0].upper(), ha="center", va="center", fontsize=7)
        actor_texts.append(text)
        
    path_line, = ax.plot([], [], "--", color="#e15759", linewidth=1.5, label="latest planned path")
    trace_line, = ax.plot([], [], color="#111111", linewidth=2, label="ego trajectory")
    
    trace_x, trace_y = [], []
    
    sampled_frames = frames[::10]
    if frames and frames[-1] is not sampled_frames[-1]:
        sampled_frames.append(frames[-1])

    def init():
        path_line.set_data([], [])
        trace_line.set_data([], [])
        for p, t in zip(actor_patches, actor_texts):
            p.center = (-10, -10)
            t.set_position((-10, -10))
        occupancy_img.set_data(np.zeros((72, 120)))
        return [path_line, trace_line, occupancy_img] + actor_patches + actor_texts

    def update(frame):
        trace_x.append(frame["ego"][0])
        trace_y.append(frame["ego"][1])
        trace_line.set_data(trace_x, trace_y)
        
        if len(frame["path"]):
            path_line.set_data(frame["path"][:, 0], frame["path"][:, 1])
        else:
            path_line.set_data([], [])
            
        for patch, text, pos in zip(actor_patches, actor_texts, frame["agents"]):
            patch.center = tuple(pos)
            text.set_position(tuple(pos))
            
        if frame.get("occupancy") is not None:
            occupancy_img.set_data(frame["occupancy"])
            
        return [path_line, trace_line, occupancy_img] + actor_patches + actor_texts

    ani = animation.FuncAnimation(fig, update, frames=sampled_frames, init_func=init, blit=True, repeat=False)
    writer = animation.FFMpegWriter(fps=2)
    ani.save(destination, writer=writer)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="CUDA-aware autonomous-road simulation")
    parser.add_argument("--scenario", choices=[*build_scenarios(), "all"], default="all")
    parser.add_argument("--device", choices=["auto", "cuda", "cpu"], default="auto")
    parser.add_argument("--output", default="outputs", help="Folder for dashboard JSON and optional PNGs")
    parser.add_argument("--plot", action="store_true", help="Save final route image(s)")
    parser.add_argument("--video", action="store_true", help="Save animation of the route as MP4(s)")
    args = parser.parse_args()
    scenarios = build_scenarios()
    chosen = scenarios.values() if args.scenario == "all" else [scenarios[args.scenario]]
    engine = SimulationEngine(SimulationConfig(device=args.device))
    results, output = [], Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    for scenario in chosen:
        metrics, frames = engine.run(scenario, capture_frames=(args.plot or args.video))
        result = metrics.as_dict()
        result["compute_device"] = engine.device_name
        results.append(result)
        print(json.dumps(result))
        if args.plot and frames:
            _render(scenario, frames, output / f"{scenario.name}.png")
        if args.video and frames:
            _render_video(scenario, frames, output / f"{scenario.name}.mp4")
    (output / "metrics.json").write_text(json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
