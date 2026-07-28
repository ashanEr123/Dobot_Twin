"""NSGA-II over the four surrogate models. Writes Pareto front and candidates."""

import numpy as np
import pandas as pd
import joblib
import matplotlib.pyplot as plt

from pymoo.core.problem import Problem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.optimize import minimize
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM

import config

FEATURES = ["velocity_ratio", "acceleration_ratio", "blend_ratio"]
OBJ_NAMES = ["time_s", "position_error_mm", "energy_proxy", "smoothness_index"]


def load_models():
    return {name: joblib.load(f"{config.MODEL_DIR}/model_{name}.joblib")
            for name in OBJ_NAMES}


class TrajectoryProblem(Problem):
    """Minimize time, error, energy; maximize smoothness (as -smoothness)."""

    def __init__(self, models):
        xl = np.array([config.VELOCITY_RATIO_RANGE[0],
                        config.ACCELERATION_RATIO_RANGE[0],
                        config.BLEND_RATIO_RANGE[0]])
        xu = np.array([config.VELOCITY_RATIO_RANGE[1],
                        config.ACCELERATION_RATIO_RANGE[1],
                        config.BLEND_RATIO_RANGE[1]])
        super().__init__(n_var=3, n_obj=4, n_constr=0, xl=xl, xu=xu)
        self.models = models

    def _evaluate(self, X, out, *args, **kwargs):
        f_time = self.models["time_s"].predict(X)
        f_error = self.models["position_error_mm"].predict(X)
        f_energy = self.models["energy_proxy"].predict(X)
        f_smooth = self.models["smoothness_index"].predict(X)
        out["F"] = np.column_stack([f_time, f_error, f_energy, -f_smooth])


def select_representative_candidates(X, F):
    """Knee point plus best-per-objective extremes."""
    F_norm = (F - F.min(axis=0)) / (F.max(axis=0) - F.min(axis=0) + 1e-9)
    dist_to_ideal = np.linalg.norm(F_norm, axis=1)
    knee_idx = int(np.argmin(dist_to_ideal))

    extreme_idxs = [int(np.argmin(F[:, j])) for j in range(F.shape[1])]
    idxs = list(dict.fromkeys([knee_idx] + extreme_idxs))
    labels = ["knee_point"] + [f"best_{OBJ_NAMES[j]}" for j in range(F.shape[1])]
    labels = list(dict.fromkeys(labels))[:len(idxs)]

    rows = []
    for label, idx in zip(labels, idxs):
        row = dict(label=label)
        row.update(dict(zip(FEATURES, X[idx])))
        row.update(dict(
            time_s=F[idx, 0], position_error_mm=F[idx, 1],
            energy_proxy=F[idx, 2], smoothness_index=-F[idx, 3],
        ))
        rows.append(row)

    return pd.DataFrame(rows)


def main():
    models = load_models()
    problem = TrajectoryProblem(models)

    algorithm = NSGA2(
        pop_size=80,
        sampling=FloatRandomSampling(),
        crossover=SBX(prob=0.9, eta=15),
        mutation=PM(eta=20),
        eliminate_duplicates=True,
    )

    res = minimize(problem, algorithm, ("n_gen", 120),
                    seed=config.RANDOM_SEED, verbose=True)

    X = res.X
    F = res.F
    X_int = np.round(X).astype(int)

    pareto_df = pd.DataFrame(X_int, columns=FEATURES)
    pareto_df["time_s"] = F[:, 0]
    pareto_df["position_error_mm"] = F[:, 1]
    pareto_df["energy_proxy"] = F[:, 2]
    pareto_df["smoothness_index"] = -F[:, 3]
    pareto_df.to_csv(config.PARETO_CSV, index=False)
    print(f"\nSaved {len(pareto_df)} solutions -> {config.PARETO_CSV}")

    candidates = select_representative_candidates(X_int, F)
    candidates.to_csv(config.CANDIDATES_CSV, index=False)
    print(f"Saved {len(candidates)} candidates -> {config.CANDIDATES_CSV}")
    print(candidates.to_string(index=False))

    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
    labels = ["time_s", "position_error_mm", "energy_proxy", "-smoothness_index"]
    for ax, (i, j) in zip(axes.flat, pairs):
        ax.scatter(F[:, i], F[:, j], c="tab:blue", alpha=0.6, s=20)
        ax.set_xlabel(labels[i])
        ax.set_ylabel(labels[j])
    plt.tight_layout()
    plt.savefig("pareto_front.png", dpi=150)
    print("Saved -> pareto_front.png")


if __name__ == "__main__":
    main()
