"""Compares baseline predictions against a measured main_v01 run.

Deliberately does not claim generalization -- see PilotEvaluation.summary_note.
Success criteria for this pilot (2026-07-14):
    1. Scout/Main both run under the same SCM conditions.
    2. Scout metrics and Main ground truth are saved to CSV/JSON.
    3. Baseline predictions are compared against Main ground truth.
    4. slip/sinkage/torque/distance/completion errors are produced.
    5. "Not yet a validated generalization" is stated explicitly in the summary.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from .metrics import RunSummary
from .predictor import PredictionResult

COMPARED_METRICS = ["mean_slip", "mean_sinkage_m", "mean_wheel_torque_nm", "energy_j", "mean_speed_mps", "distance_m"]


@dataclass(frozen=True)
class PredictorEvaluation:
    predictor_name: str
    status: str
    errors: dict[str, float] | None
    notes: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PilotEvaluation:
    main_ground_truth: dict[str, Any]
    predictors: list[PredictorEvaluation]
    generalization_validated: bool
    summary_note: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "main_ground_truth": self.main_ground_truth,
            "predictors": [p.to_dict() for p in self.predictors],
            "generalization_validated": self.generalization_validated,
            "summary_note": self.summary_note,
        }


def _abs_error(predicted: float | None, actual: float | None) -> float | None:
    if predicted is None or actual is None:
        return None
    return abs(predicted - actual)


def evaluate(main_ground_truth: RunSummary, predictions: list[PredictionResult]) -> PilotEvaluation:
    actual = main_ground_truth.to_dict()
    evaluations: list[PredictorEvaluation] = []
    for prediction in predictions:
        if prediction.status != "OK" or prediction.metrics is None:
            evaluations.append(
                PredictorEvaluation(
                    predictor_name=prediction.predictor_name,
                    status=prediction.status,
                    errors=None,
                    notes=prediction.notes,
                )
            )
            continue
        errors = {
            metric: _abs_error(prediction.metrics.get(metric), actual.get(metric)) for metric in COMPARED_METRICS
        }
        evaluations.append(
            PredictorEvaluation(
                predictor_name=prediction.predictor_name,
                status="OK",
                errors={k: v for k, v in errors.items() if v is not None},
                notes=prediction.notes,
            )
        )

    return PilotEvaluation(
        main_ground_truth=actual,
        predictors=evaluations,
        generalization_validated=False,
        summary_note=(
            "Single (rover, slope, soil) datapoint only. This compares baseline predictions against one "
            "measured main_v01 run, not a validated generalizable transfer model -- do not report these "
            "errors as proof the scout-to-main transfer principle works broadly. See PDF section 06 "
            "(Experimental Campaign) for what a real calibration/hold-out campaign requires."
        ),
    )
