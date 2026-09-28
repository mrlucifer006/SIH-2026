from sih_sim.config import SimulationConfig
from sih_sim.engine import SimulationEngine
from sih_sim.scenarios import build_scenarios


def test_village_episode_replans_and_reports_metrics():
    engine = SimulationEngine(SimulationConfig(device="cpu", max_steps=20), seed=2)
    metrics, _ = engine.run(build_scenarios()["village"])
    assert metrics.steps > 0
    assert metrics.replans > 0
    assert metrics.planner_ms


def test_scenarios_match_design_brief():
    scenarios = build_scenarios()
    assert set(scenarios) == {"village", "market", "intersection", "merge", "school"}
