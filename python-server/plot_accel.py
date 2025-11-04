from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def load_csv_dataframe(csv_path: Path) -> pd.DataFrame:
    """Load CSV with acc, gyr, and other sensor data."""
    df = pd.read_csv(csv_path)
    return df


def plot_accel_gyr(df: pd.DataFrame, max_samples: int = 1000) -> None:
    """Plot raw accelerometer and gyroscope data."""
    # Limit to first N samples to see detail
    df = df.head(max_samples)
    
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    
    # Plot accelerometer
    if 'acc_x' in df.columns:
        axes[0].plot(df.index, df['acc_x'], label='acc_x')
        axes[0].plot(df.index, df['acc_y'], label='acc_y')
        axes[0].plot(df.index, df['acc_z'], label='acc_z')
        axes[0].axhline(y=9.81, color='k', linestyle='--', alpha=0.3, label='g=9.81')
        axes[0].axhline(y=-9.81, color='k', linestyle='--', alpha=0.3)
        axes[0].set_ylabel('Acceleration (m/s²)')
        axes[0].set_title('Raw Accelerometer Data')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
    
    # Plot gyroscope
    if 'gyr_x' in df.columns:
        axes[1].plot(df.index, df['gyr_x'], label='gyr_x')
        axes[1].plot(df.index, df['gyr_y'], label='gyr_y')
        axes[1].plot(df.index, df['gyr_z'], label='gyr_z')
        axes[1].set_ylabel('Angular velocity (rad/s)')
        axes[1].set_xlabel('Sample index')
        axes[1].set_title('Raw Gyroscope Data')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
    
    fig.tight_layout()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot raw sensor data from CSV")
    parser.add_argument("--file", type=Path, default=Path("trace_log.csv"),
                        help="Path to CSV file (default: trace_log.csv)")
    parser.add_argument("--samples", type=int, default=1000,
                        help="Number of samples to plot (default: 1000)")
    args = parser.parse_args()
    
    df = load_csv_dataframe(args.file)
    print(f"Loaded {len(df)} samples")
    print(f"Columns: {list(df.columns)}")
    plot_accel_gyr(df, max_samples=args.samples)


if __name__ == "__main__":
    main()
