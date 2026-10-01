import json
from pathlib import Path
from shapely.geometry import Polygon

ROOT = Path(__file__).parents[1]


def test_checked_in_lidar_plan_is_valid():
    plan = json.loads((ROOT / "runs/lidar/plan.json").read_text())
    assert plan["capture"]["tier"] == "lidar"
    assert len(plan["rooms"]) >= 3
    for room in plan["rooms"]:
        poly = Polygon(room["polygon_m"])
        assert poly.buffer(0).area > 0.5
        for field in (room["floor_area"], room["ceiling_height"]):
            assert field["ci95"][0] <= field["value"] <= field["ci95"][1]


def test_measurement_intervals_bracket_values():
    plan = json.loads((ROOT / "runs/lidar/plan.json").read_text())
    found = 0
    def walk(value):
        nonlocal found
        if isinstance(value, dict):
            if {"value", "ci95", "unit", "method"} <= value.keys():
                found += 1
                assert value["ci95"][0] <= value["value"] <= value["ci95"][1]
            for child in value.values(): walk(child)
        elif isinstance(value, list):
            for child in value: walk(child)
    walk(plan)
    assert found > 20
