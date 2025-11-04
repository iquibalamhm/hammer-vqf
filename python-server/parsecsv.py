from __future__ import annotations

import argparse
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


def plot_trace(df: pd.DataFrame, stride_idx: int | None = None) -> None:
    # Filter to specific stride if requested
    if stride_idx is not None:
        df = df[df["stride_idx"] == stride_idx].copy()
        if df.empty:
            raise ValueError(f"No data found for stride {stride_idx}")
        title_suffix = f" (Stride {stride_idx})"
    else:
        title_suffix = " (All Strides)"
    
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

    # Plot raw velocities with dashed lines and corrected with solid lines (same colors)
    axes[0].plot(df["t_rel"], df["v_raw_x"], "--", color="C0", label="v_raw_x")
    axes[0].plot(df["t_rel"], df["v_df_x"], "-", color="C0", label="v_df_x")
    axes[0].plot(df["t_rel"], df["v_raw_y"], "--", color="C1", label="v_raw_y")
    axes[0].plot(df["t_rel"], df["v_df_y"], "-", color="C1", label="v_df_y")
    axes[0].plot(df["t_rel"], df["v_raw_z"], "--", color="C2", label="v_raw_z")
    axes[0].plot(df["t_rel"], df["v_df_z"], "-", color="C2", label="v_df_z")
    axes[0].set_ylabel("Velocity (m/s)")
    axes[0].set_title(f"Velocity{title_suffix}")
    axes[0].legend(loc="upper left")

    axes[1].plot(df["t_rel"], df[["pos_x", "pos_y", "pos_z"]])
    axes[1].set_ylabel("Position (m)")
    axes[1].set_xlabel("Stride time (s)")
    axes[1].set_title(f"Position{title_suffix}")
    axes[1].legend(["pos_x", "pos_y", "pos_z"], loc="upper left")

    fig.tight_layout()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse and visualize FPA TRACE logs")
    parser.add_argument("--file", type=Path, default=Path("trace_log.csv"),
                        help="Path to trace log CSV file (default: trace_log.csv)")
    parser.add_argument("--stride", type=int, default=None,
                        help="Stride index to visualize (default: all strides)")
    args = parser.parse_args()
    
    trace_path = args.file
    df = load_trace_dataframe(trace_path)
    
    available_strides = sorted(df['stride_idx'].unique())
    print(f"Loaded {len(df)} TRACE samples across {df['stride_idx'].nunique()} stride(s)")
    print(f"Available strides: {available_strides}")
    
    if args.stride is not None:
        if args.stride not in available_strides:
            print(f"Warning: Stride {args.stride} not found. Available: {available_strides}")
            return
        print(f"Visualizing stride {args.stride}")
    else:
        print("Visualizing all strides")
    
    plot_trace(df, stride_idx=args.stride)


if __name__ == "__main__":
    main()