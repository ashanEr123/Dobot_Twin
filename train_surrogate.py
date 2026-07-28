"""Train one Random Forest per objective from experiment_data.csv."""

import os
import numpy as np
import pandas as pd
import joblib
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import KFold, cross_val_predict
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

import config

FEATURES = ["velocity_ratio", "acceleration_ratio", "blend_ratio"]
TARGETS = ["time_s", "position_error_mm", "energy_proxy", "smoothness_index"]


def load_data():
    df = pd.read_csv(config.DATA_CSV)
    before = len(df)
    df = df.dropna(subset=TARGETS)
    after = len(df)
    if after < before:
        print(f"Dropped {before - after} rows with missing values.")
    return df


def train_and_evaluate(df, target, n_estimators=300, random_state=42):
    X = df[FEATURES].values
    y = df[target].values

    model = RandomForestRegressor(
        n_estimators=n_estimators,
        random_state=random_state,
        min_samples_leaf=3,
    )

    kf = KFold(n_splits=5, shuffle=True, random_state=random_state)
    y_pred_cv = cross_val_predict(model, X, y, cv=kf)

    mae = mean_absolute_error(y, y_pred_cv)
    rmse = np.sqrt(mean_squared_error(y, y_pred_cv))
    r2 = r2_score(y, y_pred_cv)

    print(f"\n=== {target} ===")
    print(f"  5-fold CV  MAE={mae:.4f}  RMSE={rmse:.4f}  R2={r2:.4f}")

    model.fit(X, y)
    importances = dict(zip(FEATURES, model.feature_importances_))
    print(f"  Feature importances: {importances}")

    return model, dict(mae=mae, rmse=rmse, r2=r2)


def main():
    os.makedirs(config.MODEL_DIR, exist_ok=True)
    df = load_data()
    print(f"Loaded {len(df)} trials from {config.DATA_CSV}")

    summary = {}
    for target in TARGETS:
        model, metrics = train_and_evaluate(df, target)
        path = os.path.join(config.MODEL_DIR, f"model_{target}.joblib")
        joblib.dump(model, path)
        summary[target] = metrics
        print(f"  Saved -> {path}")

    print("\n=== Summary ===")
    for target, m in summary.items():
        print(f"{target:22s}  MAE={m['mae']:.4f}  RMSE={m['rmse']:.4f}  R2={m['r2']:.4f}")


if __name__ == "__main__":
    main()
