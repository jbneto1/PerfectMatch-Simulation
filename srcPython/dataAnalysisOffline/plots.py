# %%
# CASE STUDY I
file_path = "docs/logs/logs_offlineAnalysis/offlineAnalysis_2024-02-27_16-08-05.txt"
file_path_semantics = (
    "docs/logs/logs_offlineAnalysis/offlineAnalysis_w_semantics2024-02-27_16-08-05.txt"
)
# %%
# Case STUDY II
file_path = "docs/logs/logs_offlineAnalysis/offlineAnalysis_2024-02-29_12-19-56.txt"
file_path_semantics = (
    "docs/logs/logs_offlineAnalysis/offlineAnalysis_w_semantics2024-02-29_12-19-56.txt"
)
# %%
# Case STUDY CONTAINER REFACTOR
file_path = "docs/logs/logs_offlineAnalysis/first_test_working_no_ppl.txt"
file_path_semantics = (
    "docs/logs/logs_offlineAnalysis/first_test_working_no_ppl_semantics.txt"
)
print(file_path)
print(file_path_semantics)


# %%
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

color_df_wo_semantics = "blue"
color_df_w_semantics = "darkorange"


# Function to draw an ellipse based on the covariance matrix
def draw_cov_ellipse(cov, pos, nstd=1, ax=None, **kwargs):
    """
    Draw a covariance ellipse based on a covariance matrix (cov) and a position (pos),
    and include the ellipse in the legend with its number of standard deviations and color.

    Parameters:
    - cov: 2x2 covariance matrix.
    - pos: The (x, y) position of the ellipse center.
    - nstd: Number of standard deviations. The default is 1.
    - ax: Matplotlib axis where to draw the ellipse.
    - **kwargs: Additional keyword arguments passed to the Ellipse patch.
    """
    from matplotlib.patches import Ellipse
    import numpy as np

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


# %%

df = pd.read_csv(file_path)
df_semantics = pd.read_csv(file_path_semantics)

print("DF w/o semantics")
print(df.head())
print(df.shape, df.ndim)

print("DF with semantics")
print(df_semantics.head())
print(df_semantics.shape, df_semantics.ndim)

# %%


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


# Apply the transformation to your dataframes
# df_transformed = transform_dataframe_to_mm(df.copy())  # Use .copy() to avoid modifying the original dataframe
# df_semantics_transformed = transform_dataframe_to_mm(df_semantics.copy())  # Assuming df_semantics is your second dataframe
df_mm = transform_dataframe_to_mm(df.copy())
df_semantics_mm = transform_dataframe_to_mm(df_semantics.copy())
# %%


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
        fc=color,
        ec=color,
    )


df = df_mm
df_semantics = df_semantics_mm

# Create a figure with two subplots (1 row, 2 columns)
fig, axs = plt.subplots(1, 2, figsize=(14, 7))
decimation_factor = 5
decimation_factor_arrows = 15
alpha = 0.3
# Plotting for df without semantics

for i in range(0, len(df), decimation_factor):
    cov_matrix = [
        [df.iloc[i]["EKFCovXX"], df.iloc[i]["EKFCovXY"]],
        [df.iloc[i]["EKFCovYX"], df.iloc[i]["EKFCovYY"]],
    ]
    gt_pose_x = df.iloc[i]["EKF_x"] - df.iloc[i]["errorEKF_x"]
    gt_pose_y = df.iloc[i]["EKF_y"] - df.iloc[i]["errorEKF_y"]
    ellipse = draw_cov_ellipse(
        cov_matrix,
        (df.iloc[i]["EKF_x"], df.iloc[i]["EKF_y"]),
        nstd=1,
        ax=axs[0],
        alpha=alpha,
        color="purple",
        label="1 STD confidence ellipse" if i == 0 else "",
    )
    axs[0].scatter(
        gt_pose_x,
        gt_pose_y,
        color="black",
        marker="x",
        label="Ground truth trajectory" if i == 0 else "",
    )

for i in range(0, len(df), decimation_factor_arrows):
    draw_orientation_arrow(
        axs[0],
        df.iloc[i]["EKF_x"],
        df.iloc[i]["EKF_y"],
        df.iloc[i]["EKF_theta"],
        color="green",
    )

axs[0].plot(
    df["EKF_x"], df["EKF_y"], label="EKF pose trajectory", color=color_df_wo_semantics
)
axs[0].set_xlabel("X [mm]")
axs[0].set_ylabel("Y [mm]")
axs[0].legend()
axs[0].set_title("Original system")

# Plotting for df with semantics
for i in range(0, len(df_semantics), decimation_factor):
    cov_matrix = [
        [df_semantics.iloc[i]["EKFCovXX"], df_semantics.iloc[i]["EKFCovXY"]],
        [df_semantics.iloc[i]["EKFCovYX"], df_semantics.iloc[i]["EKFCovYY"]],
    ]
    gt_pose_x = df_semantics.iloc[i]["EKF_x"] - df_semantics.iloc[i]["errorEKF_x"]
    gt_pose_y = df_semantics.iloc[i]["EKF_y"] - df_semantics.iloc[i]["errorEKF_y"]
    ellipse = draw_cov_ellipse(
        cov_matrix,
        (df_semantics.iloc[i]["EKF_x"], df_semantics.iloc[i]["EKF_y"]),
        nstd=1,
        ax=axs[1],
        alpha=alpha,
        color="blue",
        label="1 STD confidence ellipse" if i == 0 else "",
    )
    axs[1].scatter(
        gt_pose_x,
        gt_pose_y,
        color="black",
        marker="x",
        label="Ground truth trajectory" if i == 0 else "",
    )

for i in range(0, len(df_semantics), decimation_factor_arrows):
    draw_orientation_arrow(
        axs[1],
        df_semantics.iloc[i]["EKF_x"],
        df_semantics.iloc[i]["EKF_y"],
        df_semantics.iloc[i]["EKF_theta"],
        color=color_df_w_semantics,
    )

axs[1].plot(
    df_semantics["EKF_x"],
    df_semantics["EKF_y"],
    label="EKF pose trajectory",
    color=color_df_w_semantics,
)
axs[1].set_xlabel("X [mm]")
axs[1].set_ylabel("Y [mm]")
axs[1].legend()
axs[1].set_title("Robust PM")

plt.tight_layout()
plt.savefig("case_study_2_trajectory_dynamic.pdf", dpi=300)
plt.show()
# %% Error comparison of both systems

fig, axs = plt.subplots(2, 2, figsize=(15, 10))

# Calculate absolute errors without altering the original dataframes
abs_errorEKF_x_original = df["errorEKF_x"].abs()
abs_errorEKF_x_semantics = df_semantics["errorEKF_x"].abs()

abs_errorEKF_y_original = df["errorEKF_y"].abs()
abs_errorEKF_y_semantics = df_semantics["errorEKF_y"].abs()

abs_errorEKF_theta_original = df["errorEKF_theta"].abs()
abs_errorEKF_theta_semantics = df_semantics["errorEKF_theta"].abs()

abs_errorPM_original = df["errorPM"].abs() if "errorPM" in df else pd.Series()
abs_errorPM_semantics = (
    df_semantics["errorPM"].abs() if "errorPM" in df_semantics else pd.Series()
)

# Set font sizes
tick_fontsize = 12
legend_fontsize = 12
title_fontsize = 14
label_fontsize = 12


# Function to plot and set y-limits
def plot_with_y_zero(ax, data1, data2, label1, label2, title):
    ax.plot(data1, label=label1)
    ax.plot(data2, label=label2)
    ax.set_title(title, fontsize=title_fontsize)
    ax.legend(fontsize=legend_fontsize)
    ax.set_ylim(bottom=0)  # Set the minimum y value to 0
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontsize(tick_fontsize)


# Plot each graph with specified font sizes
plot_with_y_zero(
    axs[0, 0],
    abs_errorEKF_x_original,
    abs_errorEKF_x_semantics,
    "Original system",
    "Robust PM",
    "Absolute Error in X [mm]",
)
plot_with_y_zero(
    axs[0, 1],
    abs_errorEKF_y_original,
    abs_errorEKF_y_semantics,
    "Original system",
    "Robust PM",
    "Absolute Error in Y [mm]",
)
plot_with_y_zero(
    axs[1, 0],
    abs_errorEKF_theta_original,
    abs_errorEKF_theta_semantics,
    "Original system",
    "Robust PM",
    "Absolute Error in θ [rad]",
)

if not abs_errorPM_original.empty and not abs_errorPM_semantics.empty:
    plot_with_y_zero(
        axs[1, 1],
        abs_errorPM_original,
        abs_errorPM_semantics,
        "Original system",
        "Robust PM",
        "Absolute Error in PM [mm]",
    )
else:
    axs[1, 1].clear()
    axs[1, 1].set_visible(False)

# Set common x-axis and y-axis labels with specified font sizes
for ax in axs[-1, :]:
    ax.set_xlabel("Samples", fontsize=label_fontsize)
for ax in axs[:, 0]:
    ax.set_ylabel("Error", fontsize=label_fontsize)

plt.tight_layout()
plt.savefig("case_study_2_absolute_errors_dynamic.pdf", dpi=300)
plt.show()


# %% EKF POSE comparison
plt.figure(figsize=(10, 8))

# Calculate ground truth poses for both systems
gt_x = df["EKF_x"] - df["errorEKF_x"]
gt_y = df["EKF_y"] - df["errorEKF_y"]

# Plot EKF paths for both systems
plt.plot(df["EKF_x"], df["EKF_y"], label="Original system", linestyle="-", color="blue")
plt.plot(
    df_semantics["EKF_x"],
    df_semantics["EKF_y"],
    label="Robust PM",
    linestyle="-",
    color="green",
)

# Plot ground truth paths for comparison
plt.plot(gt_x, gt_y, label="Ground Truth", linestyle="--", color="darkorange")


plt.xlabel("X [mm]")
plt.ylabel("Y [mm]")
plt.title("EKF trajectory comparison")
plt.legend()
plt.axis("equal")  # Ensure equal scaling for x and y axes
plt.grid(True)
plt.tight_layout()
plt.savefig("case_study_2_trajectory_comparison.pdf", dpi=300)
plt.show()


# %%


def compute_metrics(df):
    error_types = ["errorEKF_x", "errorEKF_y", "errorEKF_theta", "errorPM"]

    metrics = {error: {} for error in error_types}
    for error_type in error_types:
        tmp = np.absolute(df[error_type])
        # metrics[error_type]['MEAN'] = tmp.mean()
        metrics[error_type]["MAE"] = tmp.mean()
        metrics[error_type]["STD"] = tmp.std()
        metrics[error_type]["MAX"] = tmp.max()
        metrics[error_type]["MIN"] = tmp.min()
        metrics[error_type]["RMSE"] = np.sqrt((tmp**2).mean())

    return pd.DataFrame(metrics).T


def compute_error_percentage(df1_metrics, df2_metrics):
    error_percentage = (df2_metrics - df1_metrics) / df1_metrics * 100
    return error_percentage


# Compute metrics for df1 and df2
df1_metrics = compute_metrics(df)
df2_metrics = compute_metrics(df_semantics)

# Compute error percentage between df1 and df2
error_percentage = compute_error_percentage(df1_metrics, df2_metrics)

# Print tables for inspection
print("DF1 Metrics:")
print(df1_metrics)
print("\nDF2 Metrics:")
print(df2_metrics)
print("\nError Percentage (DF2 relative to DF1):")
print(error_percentage)

# Export to LaTeX
print("DF1 Metrics LaTeX:")
print(df1_metrics.to_latex(index=True))

print("DF2 Metrics LaTeX:")
print(df2_metrics.to_latex(index=True))

print("Error Percentage LaTeX:")
print(error_percentage.to_latex(index=True))


# %%


def to_latex_custom(df, decimals, num_format_dec, caption, label, per=False):
    """
    Generates a LaTeX table from a DataFrame with values rounded and formatted to a given number of decimal places,
    and renames the row labels according to specific mappings for better readability in the LaTeX output.

    Parameters:
    - df: Pandas DataFrame with the data.
    - decimals: Number of decimal places for rounding and formatting the values.
    - num_format_dec: Number of decimal places in the formatting string.
    - caption: Table caption.
    - label: Table label.

    Returns:
    - A string containing the LaTeX table code.
    """
    # Map old index values to new labels
    if per:
        index_map = {
            "errorEKF_x": "X [\%]",
            "errorEKF_y": "Y [\%]",
            "errorEKF_theta": "$\\theta$ [\%]",
            "errorPM": "PM [\%]",
        }
    else:
        index_map = {
            "errorEKF_x": "X [mm]",
            "errorEKF_y": "Y [mm]",
            "errorEKF_theta": "$\\theta$ [rad]",
            "errorPM": "PM [mm]",
        }

    df = df.rename(index=index_map)

    # Define the number format based on the specified decimals
    num_format = f"{{:.{num_format_dec}f}}"

    # Apply rounding and formatting to each element
    formatted_df = df.apply(
        lambda x: x.round(decimals).apply(lambda y: num_format.format(y))
    )

    # Generate the LaTeX code for the table
    latex_str = formatted_df.to_latex(
        index=True, column_format="l" + "r" * (len(df.columns)), escape=False
    )

    # Wrap the generated LaTeX code with the table* environment, including caption and label
    latex_table = f"""
\\begin{{table}}[htb!]
    \\centering
    {latex_str}
    \\caption{{{caption}}}
    \\label{{{label}}}
\\end{{table}}
"""
    return latex_table


decimals = 4  # Number of decimal places to round to
num_format = 2
# Example usage of the function
caption1 = "Original."
label1 = "tab:original"
latex_table1 = to_latex_custom(df1_metrics, decimals, num_format, caption1, label1)

caption2 = "Localization system with object rejection."
label2 = "tab:with_detection"
latex_table2 = to_latex_custom(df2_metrics, decimals, num_format, caption2, label2)

# Assuming error_percentage_df is the DataFrame containing error percentages
caption3 = "Error percentage of the localization system with outlier rejection relative to the original."
label3 = "tab:error_percentage"
latex_table3 = to_latex_custom(
    error_percentage, decimals, num_format, caption3, label3, True
)
print(latex_table1)
print(latex_table2)
print(latex_table3)

# %%
