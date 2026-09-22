# Define file paths for both systems
file_path_original = (
    "docs/logs/logs_offlineAnalysis/thesis/pm_without_outliers_thesis.txt"
)
file_path_ropm = "docs/logs/logs_offlineAnalysis/thesis/pm_without_outliers_thesis.txt"

# name of the files
str = "case_baseline_test"
# plot both systems? Set to False for single system plotting
plot_both_systems = False

# ZOOM FUNCTIONALITY
enable_zoom = False  # Set to True to enable zoom mode
zoom_x_min = -850  # mm
zoom_x_max = -300  # mm
zoom_y_min = -600  # mm
zoom_y_max = -150  # mm

# TRAJECTORY MARKERS
start_marker_color = "green"
end_marker_color = "blue"
marker_size = 20  # Size of start/end markers

# Set font sizes
tick_fontsize = 14
legend_fontsize = 14
title_fontsize = 16
label_fontsize = 14

color_df_wo_semantics = "blue"
color_df_w_semantics = "darkorange"

# %%
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Rectangle
from matplotlib.ticker import MultipleLocator
import matplotlib.lines as mlines

plt.rcParams["pdf.fonttype"] = 42
plt.rcParams["ps.fonttype"] = 42
plt.rcParams["axes.titlesize"] = 16
plt.rcParams["axes.labelsize"] = 14
plt.rcParams["xtick.labelsize"] = 14
plt.rcParams["ytick.labelsize"] = 14
plt.rcParams["legend.fontsize"] = 14

# Load data conditionally
df = pd.read_csv(file_path_original)
if plot_both_systems:
    df_semantics = pd.read_csv(file_path_ropm)

# %%


# Extract obstacle information from Code A (converted to mm)
def get_obstacles_mm():
    """
    Extract obstacles from Code A and convert to millimeters
    EXACT replication of Code A logic
    """
    # Constants from Code A (meters)
    cell_x = 0.15
    cell_y = 0.08
    wall_thickness = 0.02

    # Define obstacles exactly as in Code A (meters)
    obstacles_m = [
        {
            "name": "incoming_warehouse",
            "pos": (-0.47, 0.580),
            "size": (4 * cell_x + wall_thickness, wall_thickness),  # (0.62, 0.02) m
            "rotation": 0,
        },
        {
            "name": "outgoing_warehouse",
            "pos": (0.47, -0.58),
            "size": (4 * cell_x + wall_thickness, wall_thickness),  # (0.62, 0.02) m
            "rotation": 180,
        },
        {
            "name": "machine_A",
            "pos": (-0.347, -0.08),
            "size": (2 * cell_x + wall_thickness, wall_thickness),  # (0.32, 0.02) m
            "rotation": -90,
        },
        {
            "name": "machine_B",
            "pos": (0.347, 0.07),
            "size": (2 * cell_x + wall_thickness, wall_thickness),  # (0.32, 0.02) m
            "rotation": -90,
        },
    ]

    # Convert to mm and apply rotation logic from Code A
    obstacles_mm = []
    for obs in obstacles_m:
        size_x_mm = obs["size"][0] * 1000  # Convert to mm
        size_y_mm = obs["size"][1] * 1000  # Convert to mm

        # Apply Code A rotation logic: swap sizes for ±90°
        rotation = obs.get("rotation", 0)
        if rotation == 90 or rotation == -90:
            size_x_mm, size_y_mm = size_y_mm, size_x_mm

        obstacles_mm.append(
            {
                "name": obs["name"],
                "pos": (obs["pos"][0] * 1000, obs["pos"][1] * 1000),  # Convert to mm
                "size": (size_x_mm, size_y_mm),
                "rotation": rotation,
            }
        )

    return obstacles_mm


def is_obstacle_in_zoom_area(obstacle, x_min, x_max, y_min, y_max):
    """
    Check if obstacle overlaps with zoom area
    """
    obs_x, obs_y = obstacle["pos"]
    size_x, size_y = obstacle["size"]

    # Calculate obstacle bounds
    obs_left = obs_x - size_x / 2
    obs_right = obs_x + size_x / 2
    obs_bottom = obs_y - size_y / 2
    obs_top = obs_y + size_y / 2

    # Check if obstacle overlaps with zoom area
    return (
        obs_right >= x_min
        and obs_left <= x_max
        and obs_top >= y_min
        and obs_bottom <= y_max
    )


def draw_obstacle_rectangle(ax, obstacle, add_label=False):
    """
    Draw obstacle rectangle on the given axis
    """
    pos_x, pos_y = obstacle["pos"]
    size_x, size_y = obstacle["size"]

    # Calculate rectangle bottom-left corner
    rect_x = pos_x - size_x / 2
    rect_y = pos_y - size_y / 2

    # Create rectangle
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
    """
    Draw a covariance ellipse based on a covariance matrix (cov) and a position (pos),
    and include the ellipse in the legend with its number of standard deviations and color.
    """
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
    # Columns to be converted from meters to millimeters (excluding angles and errorPM which is already in mm)
    columns_to_convert = ["EKF_x", "EKF_y", "PM_x", "PM_y", "errorEKF_x", "errorEKF_y"]

    # Covariance columns involving linear measurements to be scaled by 1000^2
    cov_linear = ["EKFCovXX", "EKFCovXY", "EKFCovYX", "EKFCovYY"]

    # Covariance columns involving angles, to be scaled by 1000
    cov_with_theta = [
        "EKFCovXTheta",
        "EKFCovYTheta",
        "EKFCovThetaX",
        "EKFCovThetaY",
        "EKFCovThetaTheta",
    ]

    # Convert specified columns from meters to millimeters by multiplying by 1000
    df[columns_to_convert] = df[columns_to_convert] * 1000

    # Scale linear covariances by 1000^2
    df[cov_linear] = df[cov_linear] * (1000**2)

    # Scale covariances involving theta by 1000
    df[cov_with_theta] = df[cov_with_theta] * 1000

    return df


def draw_orientation_arrow(ax, x, y, theta, length=80, color="r"):
    """
    Draws an arrow representing the robot's orientation.

    Parameters:
    - ax: The Axes object on which to draw.
    - x, y: The starting point of the arrow (robot's position).
    - theta: The orientation angle in radians.
    - length: The length of the arrow.
    - color: The color of the arrow.
    """
    # Calculate the end point of the arrow
    end_x = x + length * np.cos(theta)
    end_y = y + length * np.sin(theta)

    # Draw the arrow
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


# Transform data conditionally
df_original = df.copy()
df_mm = transform_dataframe_to_mm(df.copy())
df = df_mm

if plot_both_systems:
    df_original_semantics = df_semantics.copy()
    df_semantics_mm = transform_dataframe_to_mm(df_semantics.copy())
    df_semantics = df_semantics_mm

# Get obstacles
obstacles = get_obstacles_mm()

# Filter obstacles if zoom is enabled
if enable_zoom:
    obstacles = [
        obs
        for obs in obstacles
        if is_obstacle_in_zoom_area(obs, zoom_x_min, zoom_x_max, zoom_y_min, zoom_y_max)
    ]
    print(
        f"Zoom mode enabled: showing area X=[{zoom_x_min}, {zoom_x_max}], Y=[{zoom_y_min}, {zoom_y_max}]"
    )
    print(f"Visible obstacles: {[obs['name'] for obs in obstacles]}")

# Print obstacle dimensions for verification
print("Obstacle dimensions (mm):")
for obs in obstacles:
    print(
        f"  {obs['name']}: {obs['size'][0]:.0f} × {obs['size'][1]:.0f} mm at {obs['pos']}"
    )

# Decide the subplot configuration based on the number of systems to plot
if plot_both_systems:
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))  # Better proportions
else:
    fig, axs = plt.subplots(figsize=(8, 8))  # Only one subplot
    axs = [axs]  # Wrap it in a list to use the same indexing approach

decimation_factor = 5
decimation_factor_arrows = 15
alpha = 0.5  # INCREASED from 0.3 to 0.5

# ----------------------------------------------- Plotting for df without semantics (Original System) ----------------------------------------------#

# Draw obstacles first (background) for original system
obstacle_labeled = False
for obstacle in obstacles:
    draw_obstacle_rectangle(axs[0], obstacle, add_label=(not obstacle_labeled))
    if not obstacle_labeled:
        obstacle_labeled = True

# Plot trajectory data for original system
for i in range(0, len(df), decimation_factor):
    cov_matrix = [
        [df.iloc[i]["EKFCovXX"], df.iloc[i]["EKFCovXY"]],
        [df.iloc[i]["EKFCovYX"], df.iloc[i]["EKFCovYY"]],
    ]
    ellipse = draw_cov_ellipse(
        cov_matrix,
        (df.iloc[i]["EKF_x"], df.iloc[i]["EKF_y"]),
        nstd=1,
        ax=axs[0],
        alpha=alpha,  # Using increased alpha
        color="gray",
        zorder=2,
        label="1 STD confidence ellipse" if i == 0 else "",
    )

for i in range(0, len(df), decimation_factor_arrows):
    draw_orientation_arrow(
        axs[0],
        df.iloc[i]["EKF_x"],
        df.iloc[i]["EKF_y"],
        df.iloc[i]["EKF_theta"],
        color="C0",  # Explicitly use blue instead of C0
    )

# FIXED: Explicitly use blue color for original system
axs[0].plot(df["EKF_x"], df["EKF_y"], color="C0", zorder=3, linewidth=2)

gt_pose_x = df.iloc[:]["EKF_x"] - df.iloc[:]["errorEKF_x"]
gt_pose_y = df.iloc[:]["EKF_y"] - df.iloc[:]["errorEKF_y"]

# Plot ground truth with low alpha - DON'T add label here
axs[0].scatter(
    gt_pose_x,
    gt_pose_y,
    color="red",
    marker="x",
    zorder=4,
    s=7,  # Small size for plot
    alpha=0.1,  # Low alpha for plot
)

# Add start and end markers for EKF trajectory
start_x, start_y = df.iloc[0]["EKF_x"], df.iloc[0]["EKF_y"]
end_x, end_y = df.iloc[-1]["EKF_x"], df.iloc[-1]["EKF_y"]

axs[0].scatter(
    start_x,
    start_y,
    color=start_marker_color,
    marker="o",
    s=marker_size,
    zorder=5,
    linewidth=2,
    alpha=0.5,
)
axs[0].scatter(
    end_x,
    end_y,
    color=end_marker_color,
    marker="o",
    s=marker_size,
    zorder=5,
    linewidth=2,
    alpha=0.5,
)

# Create custom legend entries with CORRECT COLORS
trajectory_legend_marker = mlines.Line2D(
    [],
    [],
    color="C0",
    linestyle="-",
    linewidth=2,
    marker=">",  # Arrow marker pointing right
    markersize=8,
    markevery=1,
    label="EKF pose trajectory",
)

gt_legend_marker = mlines.Line2D(
    [],
    [],
    color="red",
    marker="x",
    linestyle="None",
    markersize=6,  # Reduced size
    alpha=1.0,
    label="Ground truth trajectory",
)

start_legend_marker = mlines.Line2D(
    [],
    [],
    color=start_marker_color,
    marker="o",
    linestyle="None",
    markersize=6,  # Reduced size
    markeredgewidth=1,
    label="Start position",
)

end_legend_marker = mlines.Line2D(
    [],
    [],
    color=end_marker_color,
    marker="o",
    linestyle="None",
    markersize=6,  # Reduced size
    markeredgewidth=1,
    label="End position",
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

obstacle_legend_marker = mlines.Line2D(
    [],
    [],
    color="black",
    marker="s",
    linestyle="None",
    markersize=6,
    label="Obstacles",
)

axs[0].set_xlabel("X [mm]")
axs[0].set_ylabel("Y [mm]")

# Create custom legend with EXPLICIT handles
handles = [
    trajectory_legend_marker,
    ellipse_legend_marker,
    gt_legend_marker,
    start_legend_marker,
    end_legend_marker,
]
labels = [
    "EKF pose trajectory",
    "1 STD confidence ellipse",
    "Ground truth trajectory",
    "Start position",
    "End position",
]

# Add obstacles to legend if any exist
if obstacles:
    handles.append(obstacle_legend_marker)
    labels.append("Obstacles")

axs[0].set_title("Original system" + (" (Zoomed)" if enable_zoom else ""))

# ------------------------------------------ Plotting for df with semantics (RoPM System) ----------------------------------------------#

if plot_both_systems:

    # Draw obstacles first (background) for RoPM system
    obstacle_labeled = False
    for obstacle in obstacles:
        draw_obstacle_rectangle(axs[1], obstacle, add_label=(not obstacle_labeled))
        if not obstacle_labeled:
            obstacle_labeled = True

    for i in range(0, len(df_semantics), decimation_factor):
        cov_matrix = [
            [df_semantics.iloc[i]["EKFCovXX"], df_semantics.iloc[i]["EKFCovXY"]],
            [df_semantics.iloc[i]["EKFCovYX"], df_semantics.iloc[i]["EKFCovYY"]],
        ]
        ellipse = draw_cov_ellipse(
            cov_matrix,
            (df_semantics.iloc[i]["EKF_x"], df_semantics.iloc[i]["EKF_y"]),
            nstd=1,
            ax=axs[1],
            alpha=alpha,  # Using increased alpha
            color="gray",
            zorder=2,
            label="1 STD confidence ellipse" if i == 0 else "",
        )

    for i in range(0, len(df_semantics), decimation_factor_arrows):
        draw_orientation_arrow(
            axs[1],
            df_semantics.iloc[i]["EKF_x"],
            df_semantics.iloc[i]["EKF_y"],
            df_semantics.iloc[i]["EKF_theta"],
            color="darkorange",  # Orange for RoPM
        )

    axs[1].plot(
        df_semantics["EKF_x"],
        df_semantics["EKF_y"],
        color="darkorange",  # Orange for RoPM
        zorder=3,
        linewidth=2,
    )

    gt_pose_x = df_semantics["EKF_x"] - df_semantics["errorEKF_x"]
    gt_pose_y = df_semantics["EKF_y"] - df_semantics["errorEKF_y"]

    # Plot ground truth with low alpha - DON'T add label here
    axs[1].scatter(
        gt_pose_x,
        gt_pose_y,
        color="red",
        marker="x",
        zorder=4,
        s=7,  # Small size for plot
        alpha=0.1,  # Low alpha for plot
    )

    # Add start and end markers for RoPM trajectory
    start_x, start_y = df_semantics.iloc[0]["EKF_x"], df_semantics.iloc[0]["EKF_y"]
    end_x, end_y = df_semantics.iloc[-1]["EKF_x"], df_semantics.iloc[-1]["EKF_y"]

    axs[1].scatter(
        start_x,
        start_y,
        color=start_marker_color,
        marker="o",
        s=marker_size,
        zorder=5,
        linewidth=2,
        alpha=0.5,
    )
    axs[1].scatter(
        end_x,
        end_y,
        color=end_marker_color,
        marker="o",
        s=marker_size,
        zorder=5,
        linewidth=2,
        alpha=0.5,
    )

    # Create custom legend entries for RoPM system
    trajectory_legend_marker2 = mlines.Line2D(
        [],
        [],
        color="darkorange",
        linestyle="-",
        linewidth=2,
        marker=">",  # Arrow marker pointing right
        markersize=8,
        markevery=1,
        label="EKF pose trajectory",
    )

    # Use same other markers
    handles2 = [
        trajectory_legend_marker2,
        ellipse_legend_marker,
        gt_legend_marker,
        start_legend_marker,
        end_legend_marker,
    ]
    labels2 = [
        "EKF pose trajectory",
        "1 STD confidence ellipse",
        "Ground truth trajectory",
        "Start position",
        "End position",
    ]

    if obstacles:
        handles2.append(obstacle_legend_marker)
        labels2.append("Obstacles")

    axs[1].set_xlabel("X [mm]")
    axs[1].set_ylabel("Y [mm]")
    axs[1].set_title("Robust PM" + (" (Zoomed)" if enable_zoom else ""))

## Setting plot scale and range - WITH ZOOM FUNCTIONALITY

if enable_zoom:
    # Use zoom parameters
    field_x_min = zoom_x_min
    field_x_max = zoom_x_max
    field_y_min = zoom_y_min
    field_y_max = zoom_y_max

    print(
        f"Zoom limits: X=[{field_x_min}, {field_x_max}], Y=[{field_y_min}, {field_y_max}]"
    )

    # Calculate zoom area dimensions
    zoom_width = field_x_max - field_x_min
    zoom_height = field_y_max - field_y_min
    data_aspect_ratio = zoom_width / zoom_height

else:
    # Use full field dimensions
    field_x_limit = 875  # mm (plot limits)
    field_y_limit = 600  # mm
    field_x_min = -field_x_limit
    field_x_max = field_x_limit
    field_y_min = -field_y_limit
    field_y_max = field_y_limit

    print(f"Full field limits: X: ±{field_x_limit}, Y: ±{field_y_limit}")
    data_aspect_ratio = (2 * field_x_limit) / (2 * field_y_limit)

# Apply limits to all subplots
for ax in axs:
    if ax is not None:
        ax.grid(True, linestyle="-", alpha=0.3, color="gray", linewidth=0.5)
        ax.set_axisbelow(True)
        # Set limits
        ax.set_xlim(field_x_min, field_x_max)
        ax.set_ylim(field_y_min, field_y_max)

        # Disable all automatic adjustments
        ax.set_autoscale_on(False)
        ax.autoscale(False)

        # Set aspect equal but don't let it change limits
        ax.set_aspect("equal", adjustable="box", anchor="C")

        if enable_zoom:
            # For zoom mode, use dynamic tick spacing
            x_range = field_x_max - field_x_min
            y_range = field_y_max - field_y_min

            # Choose tick spacing based on range
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
            # Full field mode - use existing tick setup
            ax.xaxis.set_major_locator(MultipleLocator(400))
            ax.yaxis.set_major_locator(MultipleLocator(200))
            ax.set_xticks([-800, -400, 0, 400, 800])
            ax.set_yticks([-600, -400, -200, 0, 200, 400, 600])

# ADAPTIVE LEGEND POSITIONING - INTEGRATED
# Dynamic legend positioning based on zoom mode
if enable_zoom:
    legend_y_pos = -0.15  # Closer to plot for zoom mode
else:
    legend_y_pos = -0.15  # Slightly further for full view

# Apply legends to both subplots with adaptive positioning and smaller size
axs[0].legend(
    handles=handles,
    labels=labels,
    loc="upper right",
    bbox_to_anchor=(1, legend_y_pos),
    fontsize=10,  # Smaller legend
    markerscale=0.7,  # Smaller markers
    handlelength=1.5,  # Shorter lines
    handletextpad=0.5,  # Less space
    columnspacing=1.0,
    frameon=True,
)

if plot_both_systems:
    axs[1].legend(
        handles=handles2,
        labels=labels2,
        loc="upper right",
        bbox_to_anchor=(1, legend_y_pos),
        fontsize=10,  # Smaller legend
        markerscale=0.7,  # Smaller markers
        handlelength=1.5,  # Shorter lines
        handletextpad=0.5,  # Less space
        columnspacing=1.0,
        frameon=True,
    )

# Adjust subplot spacing accordingly
subplot_bottom = 0.12 if enable_zoom else 0.15

# IMPROVED SUBPLOT SPACING CONTROL
if plot_both_systems:
    # Control spacing between subplots
    plt.subplots_adjust(
        bottom=subplot_bottom,  # Space for legend
        left=0.08,  # Left margin
        right=0.92,  # Right margin
        wspace=0.25,  # Space between subplots (closer together)
    )
else:
    plt.subplots_adjust(bottom=subplot_bottom)

plt.tight_layout()

# Dynamic filename based on plotting mode
zoom_suffix = "_zoomed" if enable_zoom else ""
if plot_both_systems:
    filename = str + "_dual_system_with_obstacles" + zoom_suffix + ".png"
else:
    filename = str + "_original_system_with_obstacles" + zoom_suffix + ".png"

plt.savefig(filename, dpi=300, bbox_inches="tight")
plt.show()

# Verify final limits
for i, ax in enumerate(axs):
    if ax is not None:
        xlims = ax.get_xlim()
        ylims = ax.get_ylim()
        print(f"Subplot {i} final limits: X: {xlims}, Y: {ylims}")
