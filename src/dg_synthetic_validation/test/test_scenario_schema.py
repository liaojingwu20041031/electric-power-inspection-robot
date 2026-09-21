from pathlib import Path

from dg_synthetic_validation.scenario_schema import load_scenario


CONFIG_DIR = Path(__file__).parents[1] / "config"


def test_all_scenarios_cover_duration_and_are_ordered():
    for scenario_id in ("S01", "S02", "S03", "S04"):
        scenario = load_scenario(CONFIG_DIR / f"{scenario_id}.yaml")
        assert scenario.scenario_id == scenario_id
        assert scenario.phases[0].start_sec == 0.0
        assert scenario.phases[-1].end_sec >= scenario.duration_sec
        assert all(left.end_sec <= right.start_sec for left, right in zip(scenario.phases, scenario.phases[1:]))


def test_phase_at_boundary_uses_next_phase():
    scenario = load_scenario(CONFIG_DIR / "S02.yaml")
    assert scenario.phase_at(3.99).phase_id == "GOOD"
    assert scenario.phase_at(4.0).phase_id == "DEGRADED"
    assert scenario.phase_at(8.0).phase_id == "REJECTED"
