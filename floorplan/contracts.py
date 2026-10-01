"""Measurement contract: every delivered number is a Measure.

value + interval + method + evidence grade. Nothing in the pipeline is allowed
to emit a bare float where a measurement is meant — the schema enforces it and
the renderer draws it. Confidence intervals widen as evidence thins; a tight
number from weak evidence is the failure mode this contract exists to prevent.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any


class Evidence(str, Enum):
    """Where a number came from.

    measured — direct sensor/model evidence (depth points, instance mask extent)
    inferred — derived from measured evidence by geometry (e.g. wall length from
               corner intersections of measured wall planes)
    prior    — no scene-specific evidence; a hardware/population prior only
    """

    MEASURED = "measured"
    INFERRED = "inferred"
    PRIOR = "prior"


@dataclass(frozen=True)
class Measure:
    """A metric number with an honest interval."""

    value: float
    ci_half: float          # half-width of the interval, same unit as value
    unit: str
    method: str             # one line: how this number was obtained
    evidence: Evidence

    def __post_init__(self) -> None:
        if self.ci_half < 0:
            raise ValueError(f"negative interval: {self}")
        if self.evidence is Evidence.PRIOR and self.ci_half <= 0:
            raise ValueError(f"prior evidence must carry a nonzero interval: {self}")

    def to_json(self) -> dict[str, Any]:
        return {
            "value": round(float(self.value), 4),
            "ci_half": round(float(self.ci_half), 4),
            "unit": self.unit,
            "method": self.method,
            "evidence": self.evidence.value,
        }

    @staticmethod
    def measured(value: float, ci_half: float, unit: str, method: str) -> "Measure":
        return Measure(value, ci_half, unit, method, Evidence.MEASURED)

    @staticmethod
    def inferred(value: float, ci_half: float, unit: str, method: str) -> "Measure":
        return Measure(value, ci_half, unit, method, Evidence.INFERRED)

    @staticmethod
    def prior(value: float, ci_half: float, unit: str, method: str) -> "Measure":
        return Measure(value, ci_half, unit, method, Evidence.PRIOR)


@dataclass
class Warning_:
    code: str
    message: str
    context: dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> dict[str, Any]:
        return {"code": self.code, "message": self.message, "context": self.context}


@dataclass
class Assumption:
    code: str
    message: str

    def to_json(self) -> dict[str, Any]:
        return {"code": self.code, "message": self.message}


def measure_to_dict(m: Measure | None) -> dict[str, Any] | None:
    return None if m is None else m.to_json()
