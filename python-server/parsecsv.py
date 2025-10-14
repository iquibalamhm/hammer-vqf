from __future__ import annotations

import io
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


TRACE_COLUMNS = [
    "tag",
    "stride_idx",
    "t_rel",
    "dt",
    "v_raw_x",
    "v_raw_y",
    "v_raw_z",
    "v_df_x",
    "v_df_y",
    "v_df_z",
    "pos_x",
    "pos_y",
    "pos_z",
]


def load_trace_dataframe(trace_path: Path) -> pd.DataFrame:
    with trace_path.open("r", encoding="utf-8", errors="ignore") as fh:
        trace_lines = [line for line in fh if line.startswith("TRACE,")]

    if not trace_lines:
        raise ValueError(f"No TRACE lines found in {trace_path}")

    trace_buffer = io.StringIO("".join(trace_lines))
    df = pd.read_csv(trace_buffer, header=None, names=TRACE_COLUMNS)
    df["stride_idx"] = df["stride_idx"].astype(int)
    return df


def plot_trace(df: pd.DataFrame) -> None:
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

    axes[0].plot(df["t_rel"], df[["v_raw_x", "v_df_x", "v_df_y", "v_df_z"]])
    axes[0].set_ylabel("Velocity (m/s)")
    axes[0].legend(["v_raw_x", "v_df_x", "v_df_y", "v_df_z"], loc="upper left")

    axes[1].plot(df["t_rel"], df[["pos_x", "pos_y", "pos_z"]])
    axes[1].set_ylabel("Position (m)")
    axes[1].set_xlabel("Stride time (s)")
    axes[1].legend(["pos_x", "pos_y", "pos_z"], loc="upper left")

    fig.tight_layout()
    plt.show()


def main() -> None:
    trace_path = Path("trace_log.csv")
    df = load_trace_dataframe(trace_path)
    print(f"Loaded {len(df)} TRACE samples across {df['stride_idx'].nunique()} stride(s)")
    plot_trace(df)


if __name__ == "__main__":
    main()