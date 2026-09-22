import os
import time
import sys

# Dataset configuration mapping
DATASET_CONFIG = {
    "case_baseline_test": {
        "original": "docs/logs/logs_offlineAnalysis/thesis/pm_without_outliers_thesis.txt",
        "ropm": "docs/logs/logs_offlineAnalysis/thesis/pm_without_outliers_thesis.txt",
    },
    "case_study_1": {
        "original": "docs/logs/logs_offlineAnalysis/thesis/test_ppl_entrance_y_corr_thesis.txt",
        "ropm": "docs/logs/logs_offlineAnalysis/thesis/test_ppl_entrance_y_corr_thesis_semantics.txt",
    },
    "case_study_2": {
        "original": "docs/logs/logs_offlineAnalysis/thesis/dynamic_ppl_moving_ST_30.txt",
        "ropm": "docs/logs/logs_offlineAnalysis/thesis/dynamic_ppl_moving_ST_30_semantics.txt",
    },
}

if os.environ.get("QUICK_TEST", "False").lower() == "true":
    total_frames = 10  # Test with just 10 frames
    print("QUICK TEST MODE: Only rendering 10 frames")

# Read environment variables for configuration
plot_both_systems = os.environ.get("PLOT_BOTH_SYSTEMS", "False").lower() == "true"
enable_zoom = os.environ.get("ENABLE_ZOOM", "False").lower() == "true"
case_name = os.environ.get("CASE_NAME", "case_baseline_test")

# Get file paths based on case name with fallback to environment variables
if case_name in DATASET_CONFIG:
    file_path_original = DATASET_CONFIG[case_name]["original"]
    file_path_ropm = DATASET_CONFIG[case_name]["ropm"]
    print(f"Using predefined dataset configuration for: {case_name}")
else:
    # Fallback to environment variables or defaults
    file_path_original = os.environ.get(
        "FILE_PATH_ORIGINAL",
        "docs/logs/logs_offlineAnalysis/thesis/pm_without_outliers_thesis.txt",
    )
    file_path_ropm = os.environ.get(
        "FILE_PATH_ROPM",
        "docs/logs/logs_offlineAnalysis/thesis/pm_without_outliers_thesis.txt",
    )
    print(f"Using fallback configuration for: {case_name}")

print("=" * 60)
print("ROBOT LOCALIZATION ANIMATION")
print("=" * 60)
print(f"Configuration:")
print(f"   - Case: {case_name}")
print(f"   - Both systems: {plot_both_systems}")
print(f"   - Zoom enabled: {enable_zoom}")
print(f"   - Original data: {file_path_original}")
print(f"   - RoPM data: {file_path_ropm}")
print(f"   - Display: {os.environ.get('DISPLAY', 'Not set')}")
print("=" * 60)

# Validate file paths exist
if not os.path.exists(file_path_original):
    print(f"ERROR: Original data file not found: {file_path_original}")
    print(f"Available dataset configurations: {list(DATASET_CONFIG.keys())}")
    sys.exit(1)

if plot_both_systems and not os.path.exists(file_path_ropm):
    print(f"ERROR: RoPM data file not found: {file_path_ropm}")
    print(f"Available dataset configurations: {list(DATASET_CONFIG.keys())}")
    sys.exit(1)

# name of the files
str = case_name

# ZOOM FUNCTIONALITY
zoom_x_min = -850  # mm
zoom_x_max = -300  # mm
zoom_y_min = -600  # mm
zoom_y_max = -150  # mm

# ANIMATION PARAMETERS
animation_duration_seconds = int(os.environ.get("ANIMATION_DURATION", 50))
fps = int(os.environ.get("ANIMATION_FPS", 30))
save_animation = True
show_animation = os.environ.get("MPLBACKEND", "TkAgg") != "Agg"
show_animation = False

print(f"Animation: {animation_duration_seconds}s at {fps} FPS")
print(f"Save: {save_animation}, Show: {show_animation}")

# TRAJECTORY MARKERS
start_marker_color = "green"
end_marker_color = "blue"
marker_size = 20

# Set font sizes
tick_fontsize = 14
legend_fontsize = 14
title_fontsize = 16
label_fontsize = 14

color_df_wo_semantics = "blue"
color_df_w_semantics = "darkorange"

print("\nLoading libraries...")
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Rectangle
from matplotlib.ticker import MultipleLocator
import matplotlib.lines as mlines
import matplotlib.animation as animation

# Set matplotlib backend
backend = os.environ.get("MPLBACKEND", "TkAgg")
matplotlib.use(backend)
print(f"Matplotlib backend: {backend}")

plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["ps.fonttype"] = 42
plt.rcParams["axes.titlesize"] = 16
plt.rcParams["axes.labelsize"] = 14
plt.rcParams["xtick.labelsize"] = 14
plt.rcParams["ytick.labelsize"] = 14
plt.rcParams["legend.fontsize"] = 14

# Load data conditionally
print("\nLoading data files...")
try:
    df = pd.read_csv(file_path_original)
    print(f"Original data loaded: {len(df)} rows")

    if plot_both_systems:
        df_semantics = pd.read_csv(file_path_ropm)
        print(f"Semantics data loaded: {len(df_semantics)} rows")
except Exception as e:
    print(f"Error loading data: {e}")
    sys.exit(1)

def get_obstacles_mm():
    """Extract obstacles from Code A and convert to millimeters"""
    cell_x = 0.15
    cell_y = 0.08
    wall_thickness = 0.02

    obstacles_m = [
        {
            "name": "incoming_warehouse",
            "pos": (-0.47, 0.580),
            "size": (4 * cell_x + wall_thickness, wall_thickness),
            "rotation": 0,
        },
        {
            "name": "outgoing_warehouse",
            "pos": (0.47, -0.58),
            "size": (4 * cell_x + wall_thickness, wall_thickness),
            "rotation": 180,
        },
        {
            "name": "machine_A",
            "pos": (-0.347, -0.08),
            "size": (2 * cell_x + wall_thickness, wall_thickness),
            "rotation": -90,
        },
        {
            "name": "machine_B",
            "pos": (0.347, 0.07),
            "size": (2 * cell_x + wall_thickness, wall_thickness),
            "rotation": -90,
        },
    ]

    obstacles_mm = []
    for obs in obstacles_m:
        size_x_mm = obs["size"][0] * 1000
        size_y_mm = obs["size"][1] * 1000

        rotation = obs.get("rotation", 0)
        if rotation == 90 or rotation == -90:
            size_x_mm, size_y_mm = size_y_mm, size_x_mm

        obstacles_mm.append(
            {
                "name": obs["name"],
                "pos": (obs["pos"][0] * 1000, obs["pos"][1] * 1000),
                "size": (size_x_mm, size_y_mm),
                "rotation": rotation,
            }
        )

    return obstacles_mm


def is_obstacle_in_zoom_area(obstacle, x_min, x_max, y_min, y_max):
    obs_x, obs_y = obstacle["pos"]
    size_x, size_y = obstacle["size"]

    obs_left = obs_x - size_x / 2
    obs_right = obs_x + size_x / 2
    obs_bottom = obs_y - size_y / 2
    obs_top = obs_y + size_y / 2

    return (
        obs_right >= x_min
        and obs_left <= x_max
        and obs_top >= y_min
        and obs_bottom <= y_max
    )


def draw_obstacle_rectangle(ax, obstacle, add_label=False):
    pos_x, pos_y = obstacle["pos"]
    size_x, size_y = obstacle["size"]

    rect_x = pos_x - size_x / 2
    rect_y = pos_y - size_y / 2

    rect = Rectangle(
        (rect_x, rect_y),
        size_x,
        size_y,
        facecolor="black",
        edgecolor="black",
        alpha=0.8,
        zorder=1,
    )

    if add_label:
        rect.set_label("Obstacles")

    ax.add_patch(rect)
    return rect


def draw_cov_ellipse(cov, pos, nstd=1, ax=None, **kwargs):
    if ax is None:
        ax = plt.gca()

    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1]))
    width, height = 2 * nstd * np.sqrt(vals)
    ellipse = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_patch(ellipse)
    return ellipse


def transform_dataframe_to_mm(df):
    columns_to_convert = ["EKF_x", "EKF_y", "PM_x", "PM_y", "errorEKF_x", "errorEKF_y"]
    cov_linear = ["EKFCovXX", "EKFCovXY", "EKFCovYX", "EKFCovYY"]
    cov_with_theta = [
        "EKFCovXTheta",
        "EKFCovYTheta",
        "EKFCovThetaX",
        "EKFCovThetaY",
        "EKFCovThetaTheta",
    ]

    df[columns_to_convert] = df[columns_to_convert] * 1000
    df[cov_linear] = df[cov_linear] * (1000**2)
    df[cov_with_theta] = df[cov_with_theta] * 1000

    return df


def draw_orientation_arrow(ax, x, y, theta, length=80, color="r"):
    end_x = x + length * np.cos(theta)
    end_y = y + length * np.sin(theta)

    ax.arrow(
        x,
        y,
        end_x - x,
        end_y - y,
        head_width=length * 0.15,
        head_length=length * 0.15,
        length_includes_head=True,
        fc=color,
        ec=color,
        zorder=3,
    )


print("\nProcessing data...")
# Transform data
df_original = df.copy()
df_mm = transform_dataframe_to_mm(df.copy())
df = df_mm
print("Original data transformed to mm")

if plot_both_systems:
    df_original_semantics = df_semantics.copy()
    df_semantics_mm = transform_dataframe_to_mm(df_semantics.copy())
    df_semantics = df_semantics_mm
    print("Semantics data transformed to mm")

# Get obstacles
obstacles = get_obstacles_mm()
print(f"Generated {len(obstacles)} obstacles")

if enable_zoom:
    obstacles = [
        obs
        for obs in obstacles
        if is_obstacle_in_zoom_area(obs, zoom_x_min, zoom_x_max, zoom_y_min, zoom_y_max)
    ]
    print(f"Zoom mode: filtered to {len(obstacles)} visible obstacles")

print("\nObstacle dimensions (mm):")
for obs in obstacles:
    print(
        f"   {obs['name']}: {obs['size'][0]:.0f} × {obs['size'][1]:.0f} mm at {obs['pos']}"
    )

print(f"\nSetting up animation...")
# Setup figure
if plot_both_systems:
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    print("Created dual-system plot layout")
else:
    fig, axs = plt.subplots(figsize=(8, 8))
    axs = [axs]
    print("Created single-system plot layout")

# Animation parameters
decimation_factor = 5
decimation_factor_arrows = 15
alpha = 0.5
total_frames = int(animation_duration_seconds * fps)

# Calculate data indices for animation
data_length = len(df)
if plot_both_systems:
    data_length = min(len(df), len(df_semantics))

print(f"Animation parameters:")
print(f"   - Total frames: {total_frames}")
print(f"   - Data points: {data_length}")
print(f"   - Frame rate: {fps} FPS")

# Initialize plot elements storage
plot_elements = {
    "trajectories": [[], []],
    "ellipses": [[], []],
    "arrows": [[], []],
    "gt_points": [[], []],
    "legends_set": [False, False],
}


# Progress tracking for animation
class AnimationProgress:
    def __init__(self, total_frames):
        self.total_frames = total_frames
        self.start_time = time.time()
        self.last_update = 0

    def update(self, frame):
        if frame - self.last_update >= 30:  # Update every 30 frames
            elapsed = time.time() - self.start_time
            progress = (frame / self.total_frames) * 100
            remaining = (elapsed / max(frame, 1)) * (self.total_frames - frame)
            print(
                f"Frame {frame}/{self.total_frames} ({progress:.1f}%) - ETA: {remaining:.1f}s"
            )
            self.last_update = frame


progress_tracker = AnimationProgress(total_frames)


def setup_plot():
    """Setup static elements of the plot"""
    print("Setting up plot layout...")

    if enable_zoom:
        field_x_min = zoom_x_min
        field_x_max = zoom_x_max
        field_y_min = zoom_y_min
        field_y_max = zoom_y_max
    else:
        field_x_limit = 875
        field_y_limit = 600
        field_x_min = -field_x_limit
        field_x_max = field_x_limit
        field_y_min = -field_y_limit
        field_y_max = field_y_limit

    for i, ax in enumerate(axs):
        if ax is not None:
            ax.clear()

            # Draw obstacles (static)
            obstacle_labeled = False
            for obstacle in obstacles:
                draw_obstacle_rectangle(ax, obstacle, add_label=(not obstacle_labeled))
                if not obstacle_labeled:
                    obstacle_labeled = True

            ax.grid(True, linestyle="-", alpha=0.3, color="gray", linewidth=0.5)
            ax.set_axisbelow(True)
            ax.set_xlim(field_x_min, field_x_max)
            ax.set_ylim(field_y_min, field_y_max)
            ax.set_autoscale_on(False)
            ax.autoscale(False)
            ax.set_aspect("equal", adjustable="box", anchor="C")

            # Set titles
            if i == 0:
                title = (
                    "Original system" if not plot_both_systems else "Original system"
                )
                ax.set_title(title + (" (Zoomed)" if enable_zoom else ""))
            elif i == 1 and plot_both_systems:
                ax.set_title("Robust PM" + (" (Zoomed)" if enable_zoom else ""))

            ax.set_xlabel("X [mm]")
            ax.set_ylabel("Y [mm]")

            # Set tick spacing
            if enable_zoom:
                x_range = field_x_max - field_x_min
                y_range = field_y_max - field_y_min

                if x_range <= 200:
                    x_tick_spacing = 50
                elif x_range <= 600:
                    x_tick_spacing = 100
                else:
                    x_tick_spacing = 200

                if y_range <= 200:
                    y_tick_spacing = 50
                elif y_range <= 600:
                    y_tick_spacing = 100
                else:
                    y_tick_spacing = 200

                ax.xaxis.set_major_locator(MultipleLocator(x_tick_spacing))
                ax.yaxis.set_major_locator(MultipleLocator(y_tick_spacing))
            else:
                ax.xaxis.set_major_locator(MultipleLocator(400))
                ax.yaxis.set_major_locator(MultipleLocator(200))
                ax.set_xticks([-800, -400, 0, 400, 800])
                ax.set_yticks([-600, -400, -200, 0, 200, 400, 600])


def animate(frame):
    """Animation function called for each frame"""
    progress_tracker.update(frame)

    # Calculate current data index based on frame
    current_idx = int((frame / total_frames) * (data_length - 1))

    # System 1 (Original)
    animate_system(0, df, current_idx, "C0")

    # System 2 (RoPM) if enabled
    if plot_both_systems:
        animate_system(1, df_semantics, current_idx, "darkorange")

    return []


def animate_system(system_idx, data, current_idx, color):
    """Animate a single system"""
    ax = axs[system_idx]

    if current_idx == 0:
        # Clear previous dynamic elements
        for elem_list in plot_elements["trajectories"][system_idx]:
            elem_list.remove()
        for elem_list in plot_elements["ellipses"][system_idx]:
            elem_list.remove()
        for elem_list in plot_elements["arrows"][system_idx]:
            elem_list.remove()
        for elem_list in plot_elements["gt_points"][system_idx]:
            elem_list.remove()

        plot_elements["trajectories"][system_idx] = []
        plot_elements["ellipses"][system_idx] = []
        plot_elements["arrows"][system_idx] = []
        plot_elements["gt_points"][system_idx] = []

    # Plot trajectory up to current point
    if current_idx > 0:
        x_data = data["EKF_x"].iloc[: current_idx + 1]
        y_data = data["EKF_y"].iloc[: current_idx + 1]

        (line,) = ax.plot(x_data, y_data, color=color, zorder=3, linewidth=2)
        plot_elements["trajectories"][system_idx].append(line)

        # Plot ground truth up to current point
        gt_x = (
            data["EKF_x"].iloc[: current_idx + 1]
            - data["errorEKF_x"].iloc[: current_idx + 1]
        )
        gt_y = (
            data["EKF_y"].iloc[: current_idx + 1]
            - data["errorEKF_y"].iloc[: current_idx + 1]
        )

        gt_scatter = ax.scatter(
            gt_x, gt_y, color="red", marker="x", zorder=4, s=7, alpha=0.1
        )
        plot_elements["gt_points"][system_idx].append(gt_scatter)

    # Add ellipses up to current point (decimated)
    for i in range(0, current_idx + 1, decimation_factor):
        cov_matrix = [
            [data.iloc[i]["EKFCovXX"], data.iloc[i]["EKFCovXY"]],
            [data.iloc[i]["EKFCovYX"], data.iloc[i]["EKFCovYY"]],
        ]
        ellipse = draw_cov_ellipse(
            cov_matrix,
            (data.iloc[i]["EKF_x"], data.iloc[i]["EKF_y"]),
            nstd=1,
            ax=ax,
            alpha=alpha,
            color="gray",
            zorder=2,
        )
        plot_elements["ellipses"][system_idx].append(ellipse)

    # Add orientation arrows (decimated)
    for i in range(0, current_idx + 1, decimation_factor_arrows):
        draw_orientation_arrow(
            ax,
            data.iloc[i]["EKF_x"],
            data.iloc[i]["EKF_y"],
            data.iloc[i]["EKF_theta"],
            color=color,
        )

    # Add start marker
    if current_idx >= 0:
        start_x, start_y = data.iloc[0]["EKF_x"], data.iloc[0]["EKF_y"]
        start_scatter = ax.scatter(
            start_x,
            start_y,
            color=start_marker_color,
            marker="o",
            s=marker_size,
            zorder=5,
            linewidth=2,
            alpha=0.5,
        )

    # Add end marker when we reach the end
    if current_idx >= len(data) - 1:
        end_x, end_y = data.iloc[-1]["EKF_x"], data.iloc[-1]["EKF_y"]
        end_scatter = ax.scatter(
            end_x,
            end_y,
            color=end_marker_color,
            marker="o",
            s=marker_size,
            zorder=5,
            linewidth=2,
            alpha=0.5,
        )

    # Add legend (only once)
    if not plot_elements["legends_set"][system_idx]:
        add_legend(ax, system_idx, color)
        plot_elements["legends_set"][system_idx] = True


def add_legend(ax, system_idx, color):
    """Add legend to the plot"""
    trajectory_legend_marker = mlines.Line2D(
        [],
        [],
        color=color,
        linestyle="-",
        linewidth=2,
        marker=">",
        markersize=8,
        label="EKF pose trajectory",
    )

    ellipse_legend_marker = mlines.Line2D(
        [],
        [],
        color="gray",
        marker="o",
        linestyle="None",
        markersize=6,
        alpha=alpha,
        label="1 STD confidence ellipse",
    )

    gt_legend_marker = mlines.Line2D(
        [],
        [],
        color="red",
        marker="x",
        linestyle="None",
        markersize=6,
        label="Ground truth trajectory",
    )

    start_legend_marker = mlines.Line2D(
        [],
        [],
        color=start_marker_color,
        marker="o",
        linestyle="None",
        markersize=6,
        label="Start position",
    )

    end_legend_marker = mlines.Line2D(
        [],
        [],
        color=end_marker_color,
        marker="o",
        linestyle="None",
        markersize=6,
        label="End position",
    )

    handles = [
        trajectory_legend_marker,
        ellipse_legend_marker,
        gt_legend_marker,
        start_legend_marker,
        end_legend_marker,
    ]

    if obstacles:
        obstacle_legend_marker = mlines.Line2D(
            [],
            [],
            color="black",
            marker="s",
            linestyle="None",
            markersize=6,
            label="Obstacles",
        )
        handles.append(obstacle_legend_marker)

    if plot_both_systems and case_name == "case_study_2":
        # For dual system, move legends further down and make them more compact
        ax.legend(
            handles=handles,
            loc="lower center",
            bbox_to_anchor=(0.5, -0.33),  # Further down for dual system
            fontsize=9,  # Smaller font
            markerscale=0.6,  # Smaller markers
            handlelength=1.2,
            handletextpad=0.3,
            columnspacing=0.8,
            frameon=True,
            ncol=2,  # Use 2 columns instead of 3 for better fit
        )
    elif plot_both_systems:
        ax.legend(
            handles=handles,
            loc="lower center",
            bbox_to_anchor=(0.45, -0.34),
            fontsize=10,
            markerscale=0.7,
            handlelength=1.5,
            handletextpad=0.5,
            columnspacing=1.0,
            frameon=True,
            ncol=3,
        )
    else:
        # For single system, use the existing positioning
        ax.legend(
            handles=handles,
            loc="lower center",
            bbox_to_anchor=(0.5, -0.24),
            fontsize=10,
            markerscale=0.7,
            handlelength=1.5,
            handletextpad=0.5,
            columnspacing=1.0,
            frameon=True,
            ncol=3,
        )


# Setup initial plot
setup_plot()
print("Plot setup complete")

print("\nCreating animation object...")
# Create animation
anim = animation.FuncAnimation(
    fig, animate, frames=total_frames, interval=1000 / fps, blit=False, repeat=False
)

if plot_both_systems:
    plt.subplots_adjust(
        bottom=0.25,  # Much larger bottom margin for dual system legends
        top=0.95,
        left=0.08,
        right=0.95,
        wspace=0.3,
    )
else:
    plt.subplots_adjust(bottom=0.18, top=0.95)  # Smaller margin for single system

plt.tight_layout()
print("Animation object created")

# Save animation
if save_animation:
    zoom_suffix = "_zoomed" if enable_zoom else ""
    if plot_both_systems:
        filename = str + "_dual_system_animated" + zoom_suffix + ".mp4"
    else:
        filename = str + "_original_system_animated" + zoom_suffix + ".mp4"

    print(f"\nStarting save process...")
    print(f"Filename: {filename}")
    print(f"Total frames to render: {total_frames}")
    print(f"This will take several minutes - please be patient!")

    start_time = time.time()

    try:
        Writer = animation.writers["ffmpeg"]
        writer = Writer(
            fps=fps, metadata=dict(artist="Robot Localization"), bitrate=1800
        )

        print("Rendering animation frames...")
        anim.save(filename, writer=writer)

        elapsed = time.time() - start_time
        print(f"Animation saved successfully in {elapsed:.1f} seconds!")
        print(f"File: {filename}")

    except Exception as e:
        print(f"MP4 save failed: {e}")

        # Try GIF fallback
        gif_filename = filename.replace(".mp4", ".gif")
        print(f"Trying GIF instead: {gif_filename}")

        try:
            anim.save(gif_filename, writer="pillow", fps=10)
            elapsed = time.time() - start_time
            print(f"GIF saved successfully in {elapsed:.1f} seconds!")
        except Exception as gif_e:
            print(f"GIF save also failed: {gif_e}")


# Show animation
if show_animation:
    print(f"\nDisplaying animation window...")
    try:
        plt.show()
        print("Animation window closed")
    except Exception as e:
        print(f"Could not display animation: {e}")

print(f"\nAnimation completed!")
print(f"Summary: {animation_duration_seconds}s animation at {fps} FPS")
print("=" * 60)
