# # IROS

# # %%
# # CASE STUDY I
# file_path = "docs/logs/logs_offlineAnalysis/offlineAnalysis_2024-02-27_16-08-05.txt"
# file_path_semantics = (
#     "docs/logs/logs_offlineAnalysis/offlineAnalysis_w_semantics2024-02-27_16-08-05.txt"
# )
# # %%
# # Case STUDY II
# file_path = "docs/logs/logs_offlineAnalysis/offlineAnalysis_2024-02-29_12-19-56.txt"
# file_path_semantics = (
#     "docs/logs/logs_offlineAnalysis/offlineAnalysis_w_semantics2024-02-29_12-19-56.txt"
# )

# %%


# def modify_path(original_path, insert_text):
#     index = original_path.find("Analysis_2024")
#     insert_position = index + len("Analysis_")
#     modified_path = (
#         original_path[:insert_position] + insert_text + original_path[insert_position:]
#     )

#     return modified_path


# # Example usage
# file_path = "docs/logs/logs_offlineAnalysis/offlineAnalysis_2024-02-27_16-08-05.txt"
# file_path_semantics = modify_path(file_path, "w_semantics")

# print(file_path)
# print(file_path_semantics)

# %%

file_path = "docs/logs/logs_offlineAnalysis/thesis/test_ppl_entrance_y_corr_thesis.txt"
file_path_semantics = "docs/logs/logs_offlineAnalysis/thesis/test_ppl_entrance_y_corr_thesis_semantics.txt"

str = "test_ppl_entrance"

plot_both_systems = True
with_outliers = True


start_index = None
end_index = None


# Set font sizes
tick_fontsize = 12
legend_fontsize = 12
title_fontsize = 14
label_fontsize = 12

color_df_wo_semantics = "blue"
color_df_w_semantics = "darkorange"


# %%
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


plt.rcParams["axes.titlesize"] = 14  # Title font size
plt.rcParams["axes.labelsize"] = 12  # Label font size
plt.rcParams["xtick.labelsize"] = 12  # Tick font size for x-axis
plt.rcParams["ytick.labelsize"] = 12  # Tick font size for y-axis
plt.rcParams["legend.fontsize"] = 12  # Legend font size


df = pd.read_csv(file_path)
df_semantics = pd.read_csv(file_path_semantics)

# %%


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


df_mm = transform_dataframe_to_mm(df.copy())
df_semantics_mm = transform_dataframe_to_mm(df_semantics.copy())


# %% --------------------------------------- SCATTER PLOTS WITH ARROWS AND CONFIDENCE ELLIPSES ----------------------------------------


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
    )


df_original = df.copy()
df_original_semantics = df_semantics.copy()

df = df_mm
df_semantics = df_semantics_mm


# Decide the subplot configuration based on the number of systems to plot
if plot_both_systems:
    fig, axs = plt.subplots(1, 2, figsize=(14, 7))  # Two subplots
else:
    fig, axs = plt.subplots(figsize=(7, 7))  # Only one subplot
    axs = [axs]  # Wrap it in a list to use the same indexing approach

decimation_factor = 5
decimation_factor_arrows = 15
alpha = 0.3

# ----------------------------------------------- Plotting for df without semantics ----------------------------------------------#

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
        alpha=alpha,
        color="purple",
        label="1 STD confidence ellipse" if i == 0 else "",
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

gt_pose_x = df.iloc[:]["EKF_x"] - df.iloc[:]["errorEKF_x"]
gt_pose_y = df.iloc[:]["EKF_y"] - df.iloc[:]["errorEKF_y"]
axs[0].scatter(
    gt_pose_x, gt_pose_y, color="black", marker="x", label="Ground truth trajectory"
)

axs[0].set_xlabel("X [mm]")
axs[0].set_ylabel("Y [mm]")
axs[0].legend(
    loc="upper right",
    bbox_to_anchor=(1, -0.1),
)
axs[0].set_title("Original system")


# ------------------------------------------ Plotting for df with semantics ----------------------------------------------#

if plot_both_systems:

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
            alpha=alpha,
            color="blue",
            label="1 STD confidence ellipse" if i == 0 else "",
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

    gt_pose_x = df_semantics["EKF_x"] - df_semantics["errorEKF_x"]
    gt_pose_y = df_semantics["EKF_y"] - df_semantics["errorEKF_y"]

    axs[1].scatter(
        gt_pose_x, gt_pose_y, color="black", marker="x", label="Ground truth trajectory"
    )

    axs[1].set_xlabel("X [mm]")
    axs[1].set_ylabel("Y [mm]")
    axs[1].legend(
        loc="upper right",
        bbox_to_anchor=(1, -0.1),
    )
    axs[1].set_title("Robust PM")

plt.tight_layout()
plt.savefig(str + "_test_arrow_scatter.pdf", dpi=300)
plt.show()


# %% -------------------------------------------- Error metrics comparison of both systems ---------------------------------------------------


# Create the first figure with three subplots
fig1, axs1 = plt.subplots(1, 3, figsize=(18, 6))  # 1 row, 3 columns


# Calculate absolute errors without altering the original dataframes
abs_errorEKF_x_original = df["errorEKF_x"].abs()
abs_errorEKF_y_original = df["errorEKF_y"].abs()
abs_errorEKF_theta_original = df["errorEKF_theta"].abs()
abs_errorPM_original = df["errorPM"].abs()

abs_errorEKF_x_semantics = df_semantics["errorEKF_x"].abs()
abs_errorEKF_y_semantics = df_semantics["errorEKF_y"].abs()
abs_errorEKF_theta_semantics = df_semantics["errorEKF_theta"].abs()
abs_errorPM_semantics = df_semantics["errorPM"].abs()


def plot_with_y_zero_and_counter(
    ax,
    data1,
    data2,
    counter_data,
    label1,
    label2,
    title,
    plot_counter=False,
    ylabel_primary=None,
    plot_two_approaches=True,
):
    """
    Plot function with customized y-axis labels for both primary and secondary axes.

    Parameters:
    - ax: The matplotlib axis object.
    - data1: Data series for the first plot.
    - data2: Data series for the second plot.
    - counter_data: Data series for the counter data.
    - label1: Label for the first data series.
    - label2: Label for the second data series.
    - title: Title of the plot.
    - plot_counter: Boolean flag to plot the counter data.
    - ylabel_primary: Label for the primary y-axis.
    """
    # Plot the primary data directly without assigning to variables
    (line1,) = ax.plot(data1, label=label1)
    if plot_two_approaches:
        (line2,) = ax.plot(data2, label=label2)
    ax.set_title(title, fontsize=title_fontsize)
    ax.set_ylabel(ylabel_primary, fontsize=label_fontsize)  # Set primary y-axis label
    ax.set_ylim(bottom=0)  # Set the minimum y value to 0

    if plot_two_approaches:
        handles, labels = [line1, line2], [
            label1,
            label2,
        ]  # Collect primary axis labels and handles
    else:
        handles, labels = [line1], [label1]

    if plot_counter:
        # Create a secondary y-axis if flag is True
        ax_counter = ax.twinx()
        (line3,) = ax_counter.plot(
            counter_data, color="gray", linestyle="--", label="Rejected beams"
        )
        ax_counter.set_ylabel("Rejected beams [samples]", fontsize=label_fontsize)
        ax_counter.tick_params(axis="y", labelsize=tick_fontsize)

        handles.append(line3)
        labels.append("Rejected beams")

    ax.legend(
        handles,
        labels,
        loc="upper right",
        fontsize=legend_fontsize,
        bbox_to_anchor=(1, -0.1),
        # borderaxespad=0.0,
    )

    # Set font size for the primary axis labels
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontsize(tick_fontsize)


# Call this function for your subplots

# Plotting each graph with specified font sizes and optional counter data
plot_with_y_zero_and_counter(
    axs1[0],
    abs_errorEKF_x_original,
    abs_errorEKF_x_semantics,
    df_semantics["counter"],
    "Original system",
    "Robust PM",
    "Absolute Error in X State",
    plot_counter=with_outliers,  # Change to False if you do not want to plot the counter
    ylabel_primary="Error [mm]",
    plot_two_approaches=with_outliers,
)

plot_with_y_zero_and_counter(
    axs1[1],
    abs_errorEKF_y_original,
    abs_errorEKF_y_semantics,
    df_semantics["counter"],
    "Original system",
    "Robust PM",
    "Absolute Error in Y State",
    plot_counter=with_outliers,
    ylabel_primary="Error [mm]",
    plot_two_approaches=with_outliers,
)

plot_with_y_zero_and_counter(
    axs1[2],
    abs_errorEKF_theta_original,
    abs_errorEKF_theta_semantics,
    df_semantics["counter"],
    "Original system",
    "Robust PM",
    "Absolute Error in θ State",
    plot_counter=with_outliers,
    ylabel_primary="Error [rad]",
    plot_two_approaches=with_outliers,
)

# Set common x-axis and y-axis labels with specified font sizes
for ax in axs1[:]:
    ax.set_xlabel("Samples", fontsize=label_fontsize)

    # Display the first figure
fig1.tight_layout()
fig1.savefig(str + "_test_error_metrics_xytheta.pdf", dpi=300)
plt.show()

# Create the second figure with one plot
fig2, axs2 = plt.subplots(figsize=(6, 6))


plot_with_y_zero_and_counter(
    axs2,
    abs_errorPM_original,
    abs_errorPM_semantics,
    df_semantics["counter"],
    "Original system",
    "Robust PM",
    "Error of Perfect Match",
    plot_counter=with_outliers,
    ylabel_primary="Error [mm]",
    plot_two_approaches=with_outliers,
)

axs2.set_xlabel("Samples", fontsize=label_fontsize)

# Display the second figure
fig2.tight_layout()
fig2.savefig(str + "_test_error_PM.pdf", dpi=300)
plt.show()

# %% ------------------------------------------ EKF POSE comparison without clutter ---------------------------------------------
plt.figure(figsize=(10, 8))


gt_x = df["EKF_x"] - df["errorEKF_x"]
gt_y = df["EKF_y"] - df["errorEKF_y"]


plt.plot(df["EKF_x"], df["EKF_y"], label="Original system", linestyle="-", color="blue")


if plot_both_systems:
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
plt.savefig(str + "_test_pose_comparison.pdf", dpi=300)
plt.show()


# %% ------------------------------------------------ ERROR METRICS FOR TABLE ---------------------------------------------------


# TODO: add a method that selects the important part for the metrics computations to avoid the metrics being
# driven by unimportant data


def compute_metrics(df, start_idx=None, end_idx=None):

    if start_idx != None and end_idx != None:
        df = df.loc[start_idx:end_idx].copy()

    error_types = ["errorEKF_x", "errorEKF_y", "errorEKF_theta", "errorPM"]

    metrics = {error: {} for error in error_types}
    for error_type in error_types:
        tmp = np.absolute(df[error_type])
        metrics[error_type]["MAE"] = tmp.mean()
        metrics[error_type]["STD"] = tmp.std()
        metrics[error_type]["MAX"] = tmp.max()
        metrics[error_type]["MIN"] = tmp.min()
        metrics[error_type]["RMSE"] = np.sqrt((tmp**2).mean())

    return pd.DataFrame(metrics).T


def compute_percentage_difference(df1_metrics, df2_metrics):
    epsilon = 1e-8  # A small number close to zero
    adjusted_df1_metrics = np.where(
        df1_metrics == 0, epsilon, df1_metrics
    )  # Use epsilon in case an attribute is 0

    # Compute the percentage difference
    diff_percentage = abs(
        ((df2_metrics - adjusted_df1_metrics) / adjusted_df1_metrics) * 100
    )

    return diff_percentage


# Compute metrics for df1 and df2
df1_metrics = compute_metrics(df, start_index, end_index)

if plot_both_systems:

    df2_metrics = compute_metrics(df_semantics, start_index, end_index)

    # Compute error percentage between df1 and df2
    percentage_difference = compute_percentage_difference(df1_metrics, df2_metrics)

# %% -------------------------------------------------------- FORMAT TABLE FOR LATEX ------------------------------------------------


def to_latex_custom(
    df,
    decimals,
    num_format_dec,
    caption,
    label,
    per=False,
    file_name=None,
):
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

    latex_str = formatted_df.to_latex(
        index=True,
        column_format="l" + "r" * (len(df.columns)),
        escape=False,
        header=True,
        longtable=False,
    )

    latex_table = f"""
\\begin{{table}}[h!]
    \\centering
    {latex_str}
    \\caption{{{caption}}}
    \\label{{{label}}}
\\end{{table}}
"""

    if file_name:
        with open(file_name, "w") as file:
            file.write(latex_table)

    return latex_table


decimals = 4  # Number of decimal places to round to
num_format = 2
# Example usage of the function
caption1 = "Original."
label1 = "tab:original"
latex_table1 = to_latex_custom(
    df1_metrics,
    decimals,
    num_format,
    caption1,
    label1,
    per=False,
    file_name=str + "_ycorr_thesis_table_original.txt",
)

print(latex_table1)

if plot_both_systems:
    caption2 = "Robust PM."
    label2 = "tab:with_detection"
    latex_table2 = to_latex_custom(
        df2_metrics,
        decimals,
        num_format,
        caption2,
        label2,
        per=False,
        file_name=str + "_ycorr_thesis_table_ropm.txt",
    )

    # Assuming error_percentage_df is the DataFrame containing error percentages
    caption3 = "Percentage difference of the localization system with outlier rejection relative to the original."
    label3 = "tab:error_percentage"
    latex_table3 = to_latex_custom(
        percentage_difference,
        decimals,
        num_format,
        caption3,
        label3,
        True,
        file_name=str + "_ycorr_thesis_table_percentage_comparison.txt",
    )

    print(latex_table2)
    print(latex_table3)

# %%
