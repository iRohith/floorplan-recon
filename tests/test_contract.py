import json
from pathlib import Path
import pytest
from shapely.geometry import Polygon

ROOT = Path(__file__).parents[1]
SCHEMA = json.loads((ROOT / "schema/plan.schema.json").read_text())

try:
    from jsonschema import Draft202012Validator as _Validator
except ImportError:  # pragma: no cover
    from jsonschema import Draft7Validator as _Validator


def _plans():
    return {t: json.loads((ROOT / f"runs/{t}/plan.json").read_text())
            for t in ("lidar", "photo", "video")}


@pytest.mark.parametrize("tier", ["lidar", "photo", "video"])
def test_plan_conforms_to_published_schema(tier):
    errors = sorted(_Validator(SCHEMA).iter_errors(_plans()[tier]),
                    key=lambda e: list(e.path))
    assert not errors, "\n".join(
        f"{'/'.join(str(p) for p in e.path) or '<root>'}: {e.message}" for e in errors)


@pytest.mark.parametrize("tier", ["lidar", "photo", "video"])
def test_measurements_bracket_their_value(tier):
    found = 0

    def walk(node):
        nonlocal found
        if isinstance(node, dict):
            if {"value", "ci95", "unit", "method"} <= node.keys():
                found += 1
                lo, hi = node["ci95"]
                assert lo <= node["value"] <= hi, node
            for child in node.values():
                walk(child)
        elif isinstance(node, list):
            for child in node:
                walk(child)

    walk(_plans()[tier])
    assert found >= 8


def test_lidar_rooms_are_valid_and_disjoint():
    plan = _plans()["lidar"]
    polys = [Polygon(r["polygon_m"]) for r in plan["rooms"]]
    assert len(polys) >= 3
    assert all(p.is_valid for p in polys)
    for i, a in enumerate(polys):
        for b in polys[i + 1:]:
            assert a.intersection(b).area == 0
