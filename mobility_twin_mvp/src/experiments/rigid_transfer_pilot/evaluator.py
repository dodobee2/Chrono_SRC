"""Aggregates predictions vs Main ground truth across all pilot conditions.

Unlike src/experiments/scm_pilot/evaluator.py (single condition, abs-error
only), this pilot runs 9 conditions, so aggregate error statistics (MAE) and
a rank correlation (does the predictor at least rank conditions in the right
order, even if the magnitude is off?) are meaningful here. Still explicitly
states this is not a validated generalization -- see
PilotEvaluation.summary_note.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from .metrics import RunSummary
from .predictor import COMPARED_METRICS, PredictionResult


@dataclass(frozen=True)
class ConditionComparison:
    condition_id: str
    predictor_name: str
    status: str
    errors: dict[str, float] | None
    predicted_completed: bool | None
    actual_completed: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PredictorAggregate:
    predictor_name: str
    condition_count: int
    configured_condition_count: int
    mae: dict[str, float]
    rank_correlation: dict[str, float | None]
    completion_accuracy: float | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PilotEvaluation:
    comparisons: list[ConditionComparison]
    aggregates: list[PredictorAggregate]
    generalization_validated: bool
    summary_note: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "comparisons": [c.to_dict() for c in self.comparisons],
            "aggregates": [a.to_dict() for a in self.aggregates],
            "generalization_validated": self.generalization_validated,
            "summary_note": self.summary_note,
        }


def _abs_error(predicted: float | None, actual: float | None) -> float | None:
    if predicted is None or actual is None:
        return None
    return abs(predicted - actual)


def _rank(values: list[float]) -> list[float]:
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    for rank, index in enumerate(order):
        ranks[index] = float(rank)
    return ranks


def _spearman(xs: list[float], ys: list[float]) -> float | None:
    """Pure-python Spearman rank correlation (no scipy dependency)."""
    if len(xs) < 2 or len(xs) != len(ys):
        return None
    rx, ry = _rank(xs), _rank(ys)
    mean_x, mean_y = sum(rx) / len(rx), sum(ry) / len(ry)
    cov = sum((a - mean_x) * (b - mean_y) for a, b in zip(rx, ry))
    var_x = sum((a - mean_x) ** 2 for a in rx)
    var_y = sum((b - mean_y) ** 2 for b in ry)
    if var_x <= 0 or var_y <= 0:
        return None
    return cov / (var_x**0.5 * var_y**0.5)


def evaluate(
    condition_ids: list[str],
    main_ground_truth: dict[str, RunSummary],
    predictions: dict[str, dict[str, PredictionResult]],
) -> PilotEvaluation:
    """predictions: {condition_id: {predictor_name: PredictionResult}}"""
    comparisons: list[ConditionComparison] = []
    predictor_names = sorted({name for per_condition in predictions.values() for name in per_condition})

    for condition_id in condition_ids:
        actual = main_ground_truth[condition_id].to_dict()
        for predictor_name in predictor_names:
            prediction = predictions[condition_id].get(predictor_name)
            if prediction is None or prediction.status != "OK" or prediction.metrics is None:
                comparisons.append(
                    ConditionComparison(
                        condition_id=condition_id,
                        predictor_name=predictor_name,
                        status=prediction.status if prediction else "MISSING",
                        errors=None,
                        predicted_completed=None,
                        actual_completed=actual["completed"],
                    )
                )
                continue
            errors = {m: _abs_error(prediction.metrics.get(m), actual.get(m)) for m in COMPARED_METRICS}
            predicted_completed = prediction.metrics.get("distance_m", 0.0) >= 0.5
            comparisons.append(
                ConditionComparison(
                    condition_id=condition_id,
                    predictor_name=predictor_name,
                    status="OK",
                    errors={k: v for k, v in errors.items() if v is not None},
                    predicted_completed=predicted_completed,
                    actual_completed=actual["completed"],
                )
            )

    aggregates: list[PredictorAggregate] = []
    for predictor_name in predictor_names:
        rows = [c for c in comparisons if c.predictor_name == predictor_name]
        configured = [c for c in rows if c.status == "OK"]
        mae: dict[str, float] = {}
        rank_correlation: dict[str, float | None] = {}
        for metric in COMPARED_METRICS:
            errs = [c.errors[metric] for c in configured if c.errors and metric in c.errors]
            mae[metric] = (sum(errs) / len(errs)) if errs else float("nan")

            predicted_vals: list[float] = []
            actual_vals: list[float] = []
            for condition_id in condition_ids:
                prediction = predictions[condition_id].get(predictor_name)
                if prediction is None or prediction.status != "OK" or prediction.metrics is None:
                    continue
                if metric not in prediction.metrics:
                    continue
                predicted_vals.append(prediction.metrics[metric])
                actual_vals.append(main_ground_truth[condition_id].to_dict()[metric])
            rank_correlation[metric] = _spearman(predicted_vals, actual_vals)

        completion_matches = [c.predicted_completed == c.actual_completed for c in configured if c.predicted_completed is not None]
        aggregates.append(
            PredictorAggregate(
                predictor_name=predictor_name,
                condition_count=len(rows),
                configured_condition_count=len(configured),
                mae=mae,
                rank_correlation=rank_correlation,
                completion_accuracy=(sum(completion_matches) / len(completion_matches)) if completion_matches else None,
            )
        )

    return PilotEvaluation(
        comparisons=comparisons,
        aggregates=aggregates,
        generalization_validated=False,
        summary_note=(
            f"{len(condition_ids)} rigid-terrain conditions only (one variable perturbed at a time from a "
            "flat/mid-friction baseline, not a full factorial sweep, single run each -- no repeats/seeds). "
            "This is a first-pass signal on whether the scout-to-main transfer idea is worth pursuing "
            "further, not a validated generalizable model. See PDF section 06 (Experimental Campaign) for "
            "what a real calibration/hold-out campaign requires."
        ),
    )
