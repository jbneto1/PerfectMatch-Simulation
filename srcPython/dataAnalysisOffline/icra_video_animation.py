#!/usr/bin/env python3
"""
ICRA 2027 synchronized localization-animation generator.

This script generates plot-only MP4 files to be placed beside the
corresponding front-camera footage in OpenShot.

AUDITED SYNCHRONIZATION
-----------------------
Baseline
    Camera: real_baseline_study.mp4
    Use source-video interval: 2.781 s -> 20.000 s
    Plot rows: 0 -> 742 (complete saved baseline trajectory)
    The mapping is anchored by:
        - onset of sustained robot/camera motion:
          video ~= 5.333 s <-> log row ~= 110
        - end of the recorded traversal:
          video ~= 20.000 s <-> log row 742

Case Study I
    Camera: case_study_I_edited.mp4
    Use complete edited clip: 0.000 s -> 15.000 s
    Plot rows: 0 -> 375
    Mapping: 25 log rows/s.
    The paper's quantitative interval is rows 0 -> 194 inclusive.
    It is retained only as a console verification; it is NOT drawn on
    the animation because the video should show the complete traversal.

Case Study II
    Camera: real_case_study_II.mp4
    Use source-video interval: 45.000 s -> 80.000 s
    Plot approximately rows 881 -> 1740.
    Camera/log alignment is anchored using the visible YOLO person
    detections in the front-camera recording and the RoPM rejected-beam
    activity:
        video 49.467 s <-> row 991
        video 73.933 s <-> row 1591
    The exact paper metric interval is rows 993 -> 1550 inclusive.
    Semantic rejection continues beyond row 1550, until the last nonzero
    rejected-beam sample at row 1597. To avoid conflating those notions,
    the animation uses:
        - dashed gray curve: actual rejected-beam activity
        - light band       : quantitative paper-comparison interval
    No separate "semantic-gating activity" band is drawn.

The script automatically tries to locate the repository root by walking
upward from this script and looking for:
    docs/logs/logs_offlineAnalysis/thesis

You can still override paths explicitly with --data-dir and --output-dir.

Examples
--------
    python3 icra_video_animation.py --case all

    python3 icra_video_animation.py --case baseline

    python3 icra_video_animation.py --case case2 \
        --data-dir "../../docs/logs/logs_offlineAnalysis/thesis"

Requirements
------------
    pip install numpy pandas matplotlib

FFmpeg must be installed and available on PATH:
    ffmpeg -version
"""

from __future__ import annotations

import argparse
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import matplotlib

matplotlib.use("Agg")

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# =============================================================================
# Paths
# =============================================================================

SCRIPT_DIR = Path(__file__).resolve().parent


def find_repo_root(start: Path) -> Optional[Path]:
    """Walk upward until the expected experiment-data directory is found."""
    for candidate in [start, *start.parents]:
        test = candidate / "docs" / "logs" / "logs_offlineAnalysis" / "thesis"
        if test.is_dir():
            return candidate
    return None


REPO_ROOT = find_repo_root(SCRIPT_DIR)

if REPO_ROOT is not None:
    DEFAULT_DATA_DIR = REPO_ROOT / "docs" / "logs" / "logs_offlineAnalysis" / "thesis"
else:
    # Safe fallback if the script is copied outside the repository.
    DEFAULT_DATA_DIR = Path(".")

DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "icra_video_plots"


# =============================================================================
# Visual configuration
# =============================================================================

FPS_DEFAULT = 30
DPI_DEFAULT = 100
BITRATE_DEFAULT = 5000  # kb/s, intermediate editing asset

PM_COLOR = "tab:blue"
ROPM_COLOR = "tab:orange"
GT_COLOR = "black"
COUNTER_COLOR = "0.45"
ACTIVITY_COLOR = "0.87"

FIG_LANDSCAPE = (9.6, 7.2)  # 960 x 720 at 100 dpi
FIG_STACKED = (9.6, 10.8)  # 960 x 1080 at 100 dpi


# =============================================================================
# Synchronization definitions
# =============================================================================


@dataclass(frozen=True)
class CaseConfig:
    key: str
    title: str
    pm_file: str
    ropm_file: Optional[str]
    video_trim_start: float
    video_trim_end: float
    video_time_to_index: Callable[[float], int]
    metric_start_idx: Optional[int]
    metric_end_idx: Optional[int]
    output_name: str


# -----------------------------------------------------------------------------
# Baseline
# -----------------------------------------------------------------------------
#
# Video audit:
#   sustained visible motion starts at about 5.333 s
#   visible traversal ends at about 20.000 s
#
# Log audit:
#   rows ~0..110 are essentially the initial stationary pose/jitter
#   sustained trajectory begins around row ~110
#   final saved traversal sample is row 742
#
# Solving row = a*t + b from:
#   (t=5.333333, row=110)
#   (t=20.000000, row=742)
#
BASE_MOTION_VIDEO_START = 5.333333333333333
BASE_MOTION_VIDEO_END = 20.0
BASE_MOTION_ROW_START = 110
BASE_ROW_END = 742

BASE_A = (BASE_ROW_END - BASE_MOTION_ROW_START) / (
    BASE_MOTION_VIDEO_END - BASE_MOTION_VIDEO_START
)
BASE_B = BASE_MOTION_ROW_START - BASE_A * BASE_MOTION_VIDEO_START

# Source-video time corresponding to row 0.  This retains the complete saved
# baseline trajectory, including the short initial stationary portion, while
# removing unrelated camera pre-roll.
BASE_VIDEO_TRIM_START = (0.0 - BASE_B) / BASE_A
BASE_VIDEO_TRIM_END = BASE_MOTION_VIDEO_END


def baseline_time_to_index(t: float) -> int:
    return int(round(BASE_A * t + BASE_B))


# -----------------------------------------------------------------------------
# Case Study I
# -----------------------------------------------------------------------------
# 376 rows over the complete 15 s edited clip.
def case1_time_to_index(t: float) -> int:
    return int(round(25.0 * t))


# -----------------------------------------------------------------------------
# Case Study II
# -----------------------------------------------------------------------------
# Alignment anchors recovered from the raw front-camera video and RoPM log:
#
#   first sustained visible person detections: ~49.4667 s
#       <-> rejected-beam activity row 991
#
#   last main visible person detections: ~73.9333 s
#       <-> rejected-beam activity row 1591
#
CASE2_A = 24.5231607629
CASE2_B = -222.079019074


def case2_time_to_index(t: float) -> int:
    return int(round(CASE2_A * t + CASE2_B))


def case2_index_to_video_time(idx):
    return (np.asarray(idx, dtype=float) - CASE2_B) / CASE2_A


CASES = {
    "baseline": CaseConfig(
        key="baseline",
        title="Baseline: complete factory-floor traversal",
        pm_file="pm_without_outliers_thesis.txt",
        ropm_file=None,
        video_trim_start=BASE_VIDEO_TRIM_START,
        video_trim_end=BASE_VIDEO_TRIM_END,
        video_time_to_index=baseline_time_to_index,
        metric_start_idx=0,
        metric_end_idx=742,
        output_name="baseline_plot.mp4",
    ),
    "case1": CaseConfig(
        key="case1",
        title="Case Study I: robot motion",
        pm_file="test_ppl_entrance_y_corr_thesis.txt",
        ropm_file="test_ppl_entrance_y_corr_thesis_semantics.txt",
        video_trim_start=0.0,
        video_trim_end=15.0,
        video_time_to_index=case1_time_to_index,
        metric_start_idx=0,
        metric_end_idx=194,
        output_name="case1_plot.mp4",
    ),
    "case2": CaseConfig(
        key="case2",
        title="Case Study II: stationary robot",
        pm_file="dynamic_ppl_moving_ST_30.txt",
        ropm_file="dynamic_ppl_moving_ST_30_semantics.txt",
        video_trim_start=45.0,
        video_trim_end=80.0,
        video_time_to_index=case2_time_to_index,
        metric_start_idx=993,
        metric_end_idx=1550,
        output_name="case2_plot.mp4",
    ),
}


# =============================================================================
# Data utilities
# =============================================================================


def require_columns(df: pd.DataFrame, columns: list[str], filename: Path) -> None:
    missing = [c for c in columns if c not in df.columns]
    if missing:
        raise ValueError(
            f"{filename} is missing required columns: {', '.join(missing)}"
        )


def load_log(path: Path, require_counter: bool = False) -> pd.DataFrame:
    path = path.expanduser().resolve()

    if not path.exists():
        raise FileNotFoundError(f"Log file not found: {path}")

    df = pd.read_csv(path)

    required = [
        "EKF_x",
        "EKF_y",
        "errorEKF_x",
        "errorEKF_y",
        "errorEKF_theta",
        "errorPM",
    ]
    if require_counter:
        required.append("counter")

    require_columns(df, required, path)
    return df


def add_mm_columns(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    out["EKF_x_mm"] = out["EKF_x"] * 1000.0
    out["EKF_y_mm"] = out["EKF_y"] * 1000.0
    out["errorEKF_x_mm"] = out["errorEKF_x"] * 1000.0
    out["errorEKF_y_mm"] = out["errorEKF_y"] * 1000.0
    return out


def reconstruct_ground_truth(df: pd.DataFrame) -> tuple[pd.Series, pd.Series]:
    # Saved convention: errorEKF = EKF - ground truth.
    gt_x_mm = df["EKF_x_mm"] - df["errorEKF_x_mm"]
    gt_y_mm = df["EKF_y_mm"] - df["errorEKF_y_mm"]
    return gt_x_mm, gt_y_mm


def clip_index(idx: int, n: int) -> int:
    return max(0, min(int(idx), n - 1))


def compute_metrics(df: pd.DataFrame, start: int, end: int) -> dict[str, float]:
    """End index is inclusive."""
    sub = df.iloc[start : end + 1]
    return {
        "x_mae_mm": float(sub["errorEKF_x"].abs().mean() * 1000.0),
        "y_mae_mm": float(sub["errorEKF_y"].abs().mean() * 1000.0),
        "theta_mae_rad": float(sub["errorEKF_theta"].abs().mean()),
        "pm_loss_mae_mm": float(sub["errorPM"].abs().mean()),
        "y_maxae_mm": float(sub["errorEKF_y"].abs().max() * 1000.0),
    }


def first_last_nonzero(series: pd.Series) -> tuple[int, int]:
    idx = np.flatnonzero(series.to_numpy() > 0)
    if len(idx) == 0:
        raise ValueError("Expected nonzero rejected-beam activity, but none was found.")
    return int(idx[0]), int(idx[-1])


def source_time_for_frame(
    frame: int,
    total_frames: int,
    start_time: float,
    end_time: float,
) -> float:
    """
    Map animation frames onto the CLOSED source interval [start_time, end_time].

    Using total_frames-1 ensures that the final animation frame reaches the
    exact final log/video sample while still producing the requested duration.
    """
    if total_frames <= 1:
        return start_time

    ratio = frame / (total_frames - 1)
    return start_time + ratio * (end_time - start_time)


# =============================================================================
# Plot helpers
# =============================================================================


def configure_rcparams() -> None:
    plt.rcParams.update(
        {
            "font.size": 15,
            "axes.titlesize": 19,
            "axes.labelsize": 16,
            "xtick.labelsize": 13,
            "ytick.labelsize": 13,
            "legend.fontsize": 14,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def padded_limits(
    values: list[np.ndarray],
    fraction: float = 0.08,
    min_padding: float = 5.0,
) -> tuple[float, float]:
    valid = [np.asarray(v, dtype=float) for v in values if len(v) > 0]

    low = min(np.nanmin(v) for v in valid)
    high = max(np.nanmax(v) for v in valid)

    span = max(high - low, min_padding)
    pad = max(span * fraction, min_padding)

    return low - pad, high + pad


def style_xy_axis(ax: plt.Axes, title: str) -> None:
    ax.set_title(title, pad=10)
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")


def make_current_marker(ax: plt.Axes, color: str):
    (marker,) = ax.plot(
        [],
        [],
        marker="o",
        linestyle="None",
        markersize=9,
        color=color,
        zorder=10,
    )
    return marker


def save_animation(
    anim: animation.FuncAnimation,
    fig: plt.Figure,
    output: Path,
    fps: int,
    dpi: int,
    bitrate: int,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    if shutil.which("ffmpeg") is None:
        raise RuntimeError(
            "FFmpeg was not found on PATH. Verify with 'ffmpeg -version'."
        )

    print(f"Rendering {output.name} ...")

    writer = animation.FFMpegWriter(
        fps=fps,
        codec="libx264",
        bitrate=bitrate,
        extra_args=[
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
        ],
        metadata={"artist": "Robot Localization"},
    )

    anim.save(str(output), writer=writer, dpi=dpi)
    print(f"Saved: {output}")


# =============================================================================
# Baseline animation
# =============================================================================


def render_baseline(
    cfg: CaseConfig,
    data_dir: Path,
    output_dir: Path,
    fps: int,
    dpi: int,
    bitrate: int,
) -> None:
    pm = add_mm_columns(load_log(data_dir / cfg.pm_file))
    gt_x, gt_y = reconstruct_ground_truth(pm)

    start_idx = clip_index(cfg.video_time_to_index(cfg.video_trim_start), len(pm))
    end_idx = clip_index(cfg.video_time_to_index(cfg.video_trim_end), len(pm))

    # This should be the complete stored baseline trajectory.
    if start_idx != 0 or end_idx != len(pm) - 1:
        print(
            "WARNING: baseline mapping does not span the complete log: "
            f"{start_idx}--{end_idx} of 0--{len(pm)-1}"
        )

    fig, ax = plt.subplots(figsize=FIG_LANDSCAPE)
    style_xy_axis(ax, "Baseline: complete EKF trajectory")

    xs = [
        pm["EKF_x_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        gt_x.iloc[start_idx : end_idx + 1].to_numpy(),
    ]
    ys = [
        pm["EKF_y_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        gt_y.iloc[start_idx : end_idx + 1].to_numpy(),
    ]

    ax.set_xlim(*padded_limits(xs, fraction=0.06, min_padding=25.0))
    ax.set_ylim(*padded_limits(ys, fraction=0.06, min_padding=25.0))

    (line_pm,) = ax.plot([], [], linewidth=3.0, color=PM_COLOR, label="PM-based EKF")
    (line_gt,) = ax.plot(
        [],
        [],
        linewidth=2.5,
        linestyle="--",
        color=GT_COLOR,
        label="Ground truth",
    )

    point_pm = make_current_marker(ax, PM_COLOR)
    point_gt = make_current_marker(ax, GT_COLOR)

    ax.legend(loc="best", frameon=True)

    duration = cfg.video_trim_end - cfg.video_trim_start
    total_frames = int(round(duration * fps))

    def update(frame: int):
        source_time = source_time_for_frame(
            frame,
            total_frames,
            cfg.video_trim_start,
            cfg.video_trim_end,
        )

        idx = clip_index(cfg.video_time_to_index(source_time), len(pm))
        idx = min(max(idx, start_idx), end_idx)

        sl = slice(start_idx, idx + 1)

        line_pm.set_data(
            pm["EKF_x_mm"].iloc[sl],
            pm["EKF_y_mm"].iloc[sl],
        )
        line_gt.set_data(
            gt_x.iloc[sl],
            gt_y.iloc[sl],
        )

        point_pm.set_data(
            [pm["EKF_x_mm"].iloc[idx]],
            [pm["EKF_y_mm"].iloc[idx]],
        )
        point_gt.set_data(
            [gt_x.iloc[idx]],
            [gt_y.iloc[idx]],
        )

        return line_pm, line_gt, point_pm, point_gt

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=total_frames,
        interval=1000.0 / fps,
        blit=False,
    )

    fig.tight_layout(pad=1.2)
    save_animation(
        anim,
        fig,
        output_dir / cfg.output_name,
        fps,
        dpi,
        bitrate,
    )
    plt.close(fig)

    print(
        f"Baseline source-video trim: "
        f"{cfg.video_trim_start:.3f}--{cfg.video_trim_end:.3f} s"
    )
    print(f"Baseline plotted rows: {start_idx}--{end_idx}")
    print(
        "Baseline full-log metrics:",
        compute_metrics(pm, 0, len(pm) - 1),
    )


# =============================================================================
# Case Study I animation
# =============================================================================


def render_case1(
    cfg: CaseConfig,
    data_dir: Path,
    output_dir: Path,
    fps: int,
    dpi: int,
    bitrate: int,
) -> None:
    pm = add_mm_columns(load_log(data_dir / cfg.pm_file))
    ropm = add_mm_columns(load_log(data_dir / cfg.ropm_file, require_counter=True))

    if len(pm) != len(ropm):
        raise ValueError("Case I PM and RoPM logs do not have the same number of rows.")

    gt_x, gt_y = reconstruct_ground_truth(pm)

    start_idx = clip_index(cfg.video_time_to_index(cfg.video_trim_start), len(pm))
    end_idx = clip_index(cfg.video_time_to_index(cfg.video_trim_end), len(pm))

    fig, ax = plt.subplots(figsize=FIG_LANDSCAPE)
    style_xy_axis(ax, "Case Study I: EKF position")

    xs = [
        pm["EKF_x_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        ropm["EKF_x_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        gt_x.iloc[start_idx : end_idx + 1].to_numpy(),
    ]
    ys = [
        pm["EKF_y_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        ropm["EKF_y_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        gt_y.iloc[start_idx : end_idx + 1].to_numpy(),
    ]

    ax.set_xlim(*padded_limits(xs, fraction=0.08, min_padding=30.0))
    ax.set_ylim(*padded_limits(ys, fraction=0.12, min_padding=30.0))

    (line_pm,) = ax.plot([], [], linewidth=3.0, color=PM_COLOR, label="PM")
    (line_ropm,) = ax.plot([], [], linewidth=3.0, color=ROPM_COLOR, label="RoPM")
    (line_gt,) = ax.plot(
        [],
        [],
        linewidth=2.5,
        linestyle="--",
        color=GT_COLOR,
        label="Ground truth",
    )

    point_pm = make_current_marker(ax, PM_COLOR)
    point_ropm = make_current_marker(ax, ROPM_COLOR)
    point_gt = make_current_marker(ax, GT_COLOR)

    ax.legend(loc="best", frameon=True)

    duration = cfg.video_trim_end - cfg.video_trim_start
    total_frames = int(round(duration * fps))

    def update(frame: int):
        source_time = source_time_for_frame(
            frame,
            total_frames,
            cfg.video_trim_start,
            cfg.video_trim_end,
        )

        idx = clip_index(cfg.video_time_to_index(source_time), len(pm))
        idx = min(max(idx, start_idx), end_idx)

        sl = slice(start_idx, idx + 1)

        line_pm.set_data(
            pm["EKF_x_mm"].iloc[sl],
            pm["EKF_y_mm"].iloc[sl],
        )
        line_ropm.set_data(
            ropm["EKF_x_mm"].iloc[sl],
            ropm["EKF_y_mm"].iloc[sl],
        )
        line_gt.set_data(
            gt_x.iloc[sl],
            gt_y.iloc[sl],
        )

        point_pm.set_data(
            [pm["EKF_x_mm"].iloc[idx]],
            [pm["EKF_y_mm"].iloc[idx]],
        )
        point_ropm.set_data(
            [ropm["EKF_x_mm"].iloc[idx]],
            [ropm["EKF_y_mm"].iloc[idx]],
        )
        point_gt.set_data(
            [gt_x.iloc[idx]],
            [gt_y.iloc[idx]],
        )

        return (
            line_pm,
            line_ropm,
            line_gt,
            point_pm,
            point_ropm,
            point_gt,
        )

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=total_frames,
        interval=1000.0 / fps,
        blit=False,
    )

    fig.tight_layout(pad=1.2)
    save_animation(
        anim,
        fig,
        output_dir / cfg.output_name,
        fps,
        dpi,
        bitrate,
    )
    plt.close(fig)

    print(
        f"Case I source-video trim: "
        f"{cfg.video_trim_start:.3f}--{cfg.video_trim_end:.3f} s"
    )
    print(f"Case I plotted rows: {start_idx}--{end_idx}")
    print(f"Case I paper metric rows: " f"{cfg.metric_start_idx}--{cfg.metric_end_idx}")
    print(
        "Case I PM paper-interval metrics:",
        compute_metrics(pm, cfg.metric_start_idx, cfg.metric_end_idx),
    )
    print(
        "Case I RoPM paper-interval metrics:",
        compute_metrics(ropm, cfg.metric_start_idx, cfg.metric_end_idx),
    )


# =============================================================================
# Case Study II animation
# =============================================================================


def render_case2(
    cfg: CaseConfig,
    data_dir: Path,
    output_dir: Path,
    fps: int,
    dpi: int,
    bitrate: int,
) -> None:
    pm = add_mm_columns(load_log(data_dir / cfg.pm_file))
    ropm = add_mm_columns(load_log(data_dir / cfg.ropm_file, require_counter=True))

    if len(pm) != len(ropm):
        raise ValueError(
            "Case II PM and RoPM logs do not have the same number of rows."
        )

    gt_x, gt_y = reconstruct_ground_truth(pm)

    start_idx = clip_index(cfg.video_time_to_index(cfg.video_trim_start), len(pm))
    end_idx = clip_index(cfg.video_time_to_index(cfg.video_trim_end), len(pm))

    # Audit the actual semantic-gating activity directly from the saved log.
    first_reject_idx, last_reject_idx = first_last_nonzero(ropm["counter"])

    # Convert activity interval into raw source-video time and then into the
    # local time axis of the 45--80 s OpenShot clip.
    activity_start_source = float(case2_index_to_video_time(first_reject_idx))
    activity_end_source = float(case2_index_to_video_time(last_reject_idx))

    activity_start_clip = activity_start_source - cfg.video_trim_start
    activity_end_clip = activity_end_source - cfg.video_trim_start

    fig, (ax_xy, ax_err) = plt.subplots(
        2,
        1,
        figsize=FIG_STACKED,
        gridspec_kw={
            "height_ratios": [1.15, 1.0],
            "hspace": 0.36,
        },
    )

    style_xy_axis(
        ax_xy,
        "Case Study II: position estimate (zoomed)",
    )

    xs = [
        pm["EKF_x_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        ropm["EKF_x_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        gt_x.iloc[start_idx : end_idx + 1].to_numpy(),
    ]
    ys = [
        pm["EKF_y_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        ropm["EKF_y_mm"].iloc[start_idx : end_idx + 1].to_numpy(),
        gt_y.iloc[start_idx : end_idx + 1].to_numpy(),
    ]

    ax_xy.set_xlim(*padded_limits(xs, fraction=0.10, min_padding=4.0))
    ax_xy.set_ylim(*padded_limits(ys, fraction=0.08, min_padding=8.0))

    (line_pm,) = ax_xy.plot([], [], linewidth=3.0, color=PM_COLOR, label="PM")
    (line_ropm,) = ax_xy.plot([], [], linewidth=3.0, color=ROPM_COLOR, label="RoPM")
    (line_gt,) = ax_xy.plot(
        [],
        [],
        linewidth=2.5,
        linestyle="--",
        color=GT_COLOR,
        label="Ground truth",
    )

    point_pm = make_current_marker(ax_xy, PM_COLOR)
    point_ropm = make_current_marker(ax_xy, ROPM_COLOR)
    point_gt = make_current_marker(ax_xy, GT_COLOR)

    ax_xy.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.16),
        ncol=3,
        frameon=True,
        fontsize=12,
    )

    # -------------------------------------------------------------------------
    # Time-series panel
    # -------------------------------------------------------------------------

    idx_all = np.arange(start_idx, end_idx + 1)

    raw_video_time = case2_index_to_video_time(idx_all)
    clip_time = raw_video_time - cfg.video_trim_start

    yerr_pm = pm["errorEKF_y_mm"].abs().iloc[start_idx : end_idx + 1].to_numpy()
    yerr_ropm = ropm["errorEKF_y_mm"].abs().iloc[start_idx : end_idx + 1].to_numpy()
    counter = ropm["counter"].iloc[start_idx : end_idx + 1].to_numpy()

    ax_err.set_title(
        "Localization error and rejected LiDAR measurements",
        pad=9,
    )
    ax_err.set_xlabel("Clip time [s]")
    ax_err.set_ylabel(r"$|y|$ error [mm]")
    ax_err.grid(True, alpha=0.25)

    clip_duration = cfg.video_trim_end - cfg.video_trim_start
    ax_err.set_xlim(0.0, clip_duration)

    ymax = max(
        float(np.max(yerr_pm)),
        float(np.max(yerr_ropm)),
    )
    ax_err.set_ylim(0.0, max(10.0, ymax * 1.10))

    ax_counter = ax_err.twinx()
    ax_counter.set_ylabel("Rejected beams")
    ax_counter.set_ylim(
        0.0,
        max(5.0, float(np.max(counter)) * 1.15),
    )

    # The rejected-beam curve itself is the direct semantic-gating activity
    # signal.  Shade only the quantitative comparison interval used for the
    # paper statistics so the two notions are not conflated.
    metric_start_clip = (
        float(case2_index_to_video_time(cfg.metric_start_idx)) - cfg.video_trim_start
    )
    metric_end_clip = (
        float(case2_index_to_video_time(cfg.metric_end_idx)) - cfg.video_trim_start
    )

    ax_err.axvspan(
        metric_start_clip,
        metric_end_clip,
        facecolor="0.92",
        alpha=0.65,
        zorder=0,
    )

    ax_err.axvline(
        metric_start_clip,
        color="0.40",
        linestyle=":",
        linewidth=1.5,
        alpha=0.95,
    )
    ax_err.axvline(
        metric_end_clip,
        color="0.40",
        linestyle=":",
        linewidth=1.5,
        alpha=0.95,
    )

    x_offset = 1.0

    ax_err.text(
        (metric_start_clip + metric_end_clip) / 2.0 + x_offset,
        0.965,
        "quantitative comparison interval",
        transform=ax_err.get_xaxis_transform(),
        ha="center",
        va="top",
        fontsize=10.5,
        color="0.28",
        bbox={
            "facecolor": "white",
            "edgecolor": "none",
            "alpha": 0.72,
            "pad": 1.5,
        },
    )

    (err_pm_line,) = ax_err.plot(
        [],
        [],
        linewidth=2.6,
        color=PM_COLOR,
        label=r"PM $|y|$ error",
    )
    (err_ropm_line,) = ax_err.plot(
        [],
        [],
        linewidth=2.6,
        color=ROPM_COLOR,
        label=r"RoPM $|y|$ error",
    )
    (counter_line,) = ax_counter.plot(
        [],
        [],
        linewidth=2.0,
        linestyle="--",
        color=COUNTER_COLOR,
        label="Rejected beams",
    )

    cursor = ax_err.axvline(
        0.0,
        color="black",
        linewidth=1.6,
        alpha=0.75,
    )

    handles = [
        err_pm_line,
        err_ropm_line,
        counter_line,
    ]
    labels = [h.get_label() for h in handles]

    ax_err.legend(
        handles,
        labels,
        loc="upper left",
        frameon=True,
        fontsize=12,
    )

    duration = cfg.video_trim_end - cfg.video_trim_start
    total_frames = int(round(duration * fps))

    def update(frame: int):
        source_time = source_time_for_frame(
            frame,
            total_frames,
            cfg.video_trim_start,
            cfg.video_trim_end,
        )

        idx = clip_index(
            cfg.video_time_to_index(source_time),
            len(pm),
        )
        idx = min(max(idx, start_idx), end_idx)

        sl = slice(start_idx, idx + 1)

        # XY
        line_pm.set_data(
            pm["EKF_x_mm"].iloc[sl],
            pm["EKF_y_mm"].iloc[sl],
        )
        line_ropm.set_data(
            ropm["EKF_x_mm"].iloc[sl],
            ropm["EKF_y_mm"].iloc[sl],
        )
        line_gt.set_data(
            gt_x.iloc[sl],
            gt_y.iloc[sl],
        )

        point_pm.set_data(
            [pm["EKF_x_mm"].iloc[idx]],
            [pm["EKF_y_mm"].iloc[idx]],
        )
        point_ropm.set_data(
            [ropm["EKF_x_mm"].iloc[idx]],
            [ropm["EKF_y_mm"].iloc[idx]],
        )
        point_gt.set_data(
            [gt_x.iloc[idx]],
            [gt_y.iloc[idx]],
        )

        # Time series
        local_end = idx - start_idx + 1
        t_now = source_time - cfg.video_trim_start

        err_pm_line.set_data(
            clip_time[:local_end],
            yerr_pm[:local_end],
        )
        err_ropm_line.set_data(
            clip_time[:local_end],
            yerr_ropm[:local_end],
        )
        counter_line.set_data(
            clip_time[:local_end],
            counter[:local_end],
        )

        cursor.set_xdata([t_now, t_now])

        return (
            line_pm,
            line_ropm,
            line_gt,
            point_pm,
            point_ropm,
            point_gt,
            err_pm_line,
            err_ropm_line,
            counter_line,
            cursor,
        )

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=total_frames,
        interval=1000.0 / fps,
        blit=False,
    )

    fig.subplots_adjust(
        left=0.12,
        right=0.86,
        top=0.95,
        bottom=0.08,
        hspace=0.40,
    )

    save_animation(
        anim,
        fig,
        output_dir / cfg.output_name,
        fps,
        dpi,
        bitrate,
    )
    plt.close(fig)

    print(
        f"Case II raw source-video trim: "
        f"{cfg.video_trim_start:.3f}--{cfg.video_trim_end:.3f} s"
    )
    print(f"Case II plotted rows: {start_idx}--{end_idx}")

    print(
        f"Case II rejected-beam activity rows: "
        f"{first_reject_idx}--{last_reject_idx}"
    )
    print(
        f"Case II rejected-beam activity in trimmed clip: "
        f"{activity_start_clip:.2f}--{activity_end_clip:.2f} s"
    )

    # Quantify estimator recovery after the final rejected-beam sample.
    pm_y_error_mm = pm["errorEKF_y"].abs().to_numpy() * 1000.0
    after_activity = np.arange(len(pm)) > last_reject_idx
    recovered = np.flatnonzero(after_activity & (pm_y_error_mm < 20.0))
    if len(recovered) > 0:
        recovery_idx = int(recovered[0])
        recovery_clip_time = (
            float(case2_index_to_video_time(recovery_idx)) - cfg.video_trim_start
        )
        print(
            "Case II PM |y| error first returns below 20 mm at "
            f"row {recovery_idx} / clip {recovery_clip_time:.2f} s "
            f"(about {recovery_clip_time - activity_end_clip:.2f} s "
            "after the final rejected-beam sample)."
        )

    print(
        f"Case II paper metric rows: " f"{cfg.metric_start_idx}--{cfg.metric_end_idx}"
    )

    print(
        f"Case II paper metric interval in trimmed clip: "
        f"{metric_start_clip:.2f}--{metric_end_clip:.2f} s"
    )

    print(
        "Case II PM paper-interval metrics:",
        compute_metrics(
            pm,
            cfg.metric_start_idx,
            cfg.metric_end_idx,
        ),
    )
    print(
        "Case II RoPM paper-interval metrics:",
        compute_metrics(
            ropm,
            cfg.metric_start_idx,
            cfg.metric_end_idx,
        ),
    )


# =============================================================================
# Manifest
# =============================================================================


def write_sync_manifest(output_dir: Path) -> None:
    base_duration = (
        CASES["baseline"].video_trim_end - CASES["baseline"].video_trim_start
    )

    # Compute Case II activity interval directly from the data if available.
    c2_path = DEFAULT_DATA_DIR / CASES["case2"].ropm_file

    activity_text = (
        "  Semantic-gating activity: computed from the saved counter at render time.\n"
    )

    if c2_path.exists():
        c2 = pd.read_csv(c2_path)
        if "counter" in c2.columns:
            s, e = first_last_nonzero(c2["counter"])
            ts = float(case2_index_to_video_time(s) - CASES["case2"].video_trim_start)
            te = float(case2_index_to_video_time(e) - CASES["case2"].video_trim_start)
            activity_text = (
                f"  Rejected-beam activity: rows {s} -> {e}, "
                f"clip time {ts:.2f} -> {te:.2f} s.\n"
            )

    text = f"""ICRA 2027 VIDEO SYNCHRONIZATION NOTES
======================================

BASELINE
  Camera file: real_baseline_study.mp4
  Source trim for OpenShot:
      {CASES["baseline"].video_trim_start:.3f} s
      ->
      {CASES["baseline"].video_trim_end:.3f} s
  Generated plot duration: {base_duration:.3f} s
  Plotted rows: 0 -> 742 (complete saved baseline trajectory)
  The initial part of this trim is stationary, matching the initial
  stationary samples in the saved log; the subsequent interval contains
  the full robot traversal.

CASE STUDY I
  Camera file: case_study_I_edited.mp4
  Source trim: 0.000 s -> 15.000 s
  Plotted rows: 0 -> 375 (complete sequence)
  Paper metric interval: rows 0 -> 194 inclusive.
  The metric interval is NOT drawn on the video plot.

CASE STUDY II
  Camera file: real_case_study_II.mp4
  Source trim: 45.000 s -> 80.000 s
  Approximate plotted rows: 881 -> 1740
{activity_text}  Paper metric interval: rows 993 -> 1550 inclusive.
  IMPORTANT: the metric interval is shorter than the full rejected-beam
  activity. The animation shades ONLY the quantitative paper-comparison
  interval; the dashed rejected-beam curve shows actual semantic-gating
  activity directly.

OPENSHOT
  Place each camera segment and matching generated plot at exactly the
  same timeline start. Do not independently stretch either clip.
"""

    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "sync_manifest.txt"
    path.write_text(text, encoding="utf-8")
    print(f"Saved: {path}")


# =============================================================================
# CLI
# =============================================================================


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate synchronized localization animations "
            "for the ICRA 2027 supplementary video."
        )
    )

    parser.add_argument(
        "--case",
        choices=["baseline", "case1", "case2", "all"],
        default="all",
        help="Which animation to generate (default: all).",
    )

    parser.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help=(
            "Directory containing the experiment .txt files. "
            f"Default: {DEFAULT_DATA_DIR}"
        ),
    )

    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=("Directory for generated MP4 files. " f"Default: {DEFAULT_OUTPUT_DIR}"),
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=FPS_DEFAULT,
    )

    parser.add_argument(
        "--dpi",
        type=int,
        default=DPI_DEFAULT,
    )

    parser.add_argument(
        "--bitrate",
        type=int,
        default=BITRATE_DEFAULT,
        help="Intermediate H.264 bitrate in kb/s (default: 5000).",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    configure_rcparams()

    data_dir = args.data_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()

    print(f"Script directory: {SCRIPT_DIR}")
    print(f"Repository root: {REPO_ROOT}")
    print(f"Data directory: {data_dir}")
    print(f"Output directory: {output_dir}")

    output_dir.mkdir(parents=True, exist_ok=True)

    selected = list(CASES.keys()) if args.case == "all" else [args.case]

    for key in selected:
        cfg = CASES[key]

        print("\n" + "=" * 72)
        print(cfg.title)
        print("=" * 72)

        if key == "baseline":
            render_baseline(
                cfg,
                data_dir,
                output_dir,
                args.fps,
                args.dpi,
                args.bitrate,
            )

        elif key == "case1":
            render_case1(
                cfg,
                data_dir,
                output_dir,
                args.fps,
                args.dpi,
                args.bitrate,
            )

        elif key == "case2":
            render_case2(
                cfg,
                data_dir,
                output_dir,
                args.fps,
                args.dpi,
                args.bitrate,
            )

    write_sync_manifest(output_dir)
    print("\nDone.")


if __name__ == "__main__":
    main()
