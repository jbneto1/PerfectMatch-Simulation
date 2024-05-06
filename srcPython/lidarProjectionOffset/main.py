# %%

import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.spatial import procrustes


def parse_line(line):
    tokens = line.strip().split(",")
    if len(tokens) < 9:  # Minimum tokens check
        print("Insufficient tokens in the log line.")
        return None

    try:
        # Extracting pose and encoders
        pose = (float(tokens[0]), float(tokens[1]), float(tokens[2]))
        encoders = [int(tokens[i]) for i in range(3, 7)]
        lidar_start_index = 7

        # Parsing LIDAR points
        lidar_points = []
        while lidar_start_index < len(tokens) - 1 and tokens[lidar_start_index] not in [
            "N",
            "NoDetections",
        ]:
            lidar_points.append(float(tokens[lidar_start_index]))
            lidar_start_index += 1

        # Timestamp processing
        timestamp = int(tokens[-1])
        return {
            "pose": pose,
            "encoders": encoders,
            "lidar_points": lidar_points,
            "timestamp": timestamp,
        }
    except Exception as e:
        print(f"Error parsing line: {str(e)}")
        return None


def process_log_file(file_path):
    rows = []
    with open(file_path, "r") as file:
        for line in file:
            row = parse_line(line)
            if row:
                rows.append(row)
    return pd.DataFrame(rows)


def project_lidar_points(df):
    projected_points = []

    for index, row in df.iterrows():
        gt_x, gt_y, gt_theta = row["pose"]
        lidar_data = row["lidar_points"]
        lidar_angles = np.linspace(
            0, 2 * np.pi, len(lidar_data), endpoint=False
        )  # Assuming 360-degree coverage

        # Adjusting for the LIDAR rotation relative to the robot's frame
        adjusted_theta = gt_theta + np.pi  # Add π to account for 180 degrees rotation

        # Calculate global coordinates
        x_global = gt_x + lidar_data * np.cos(lidar_angles + adjusted_theta)
        y_global = gt_y + lidar_data * np.sin(lidar_angles + adjusted_theta)

        for x, y in zip(x_global, y_global):
            projected_points.append({"x": x, "y": y})

    return pd.DataFrame(projected_points)


def plot_lidar_points_with_boundary(
    df, x_range, y_range, index_range=None, title="LIDAR Points with Boundary"
):
    """
    Plots LIDAR points and colors them based on whether they are inside or outside a specified boundary.
    Points inside the boundary are blue, outside are orange.

    Args:
    df (DataFrame): DataFrame containing 'x' and 'y' columns of LIDAR points.
    x_range (tuple): Tuple (min_x, max_x) specifying the X boundary.
    y_range (tuple): Tuple (min_y, max_y) specifying the Y boundary.
    index_range (tuple, optional): Tuple (start_index, end_index) to specify the range of DataFrame to plot. Plots all if None.
    title (str): Title of the plot.
    """
    # If index_range is specified, filter the DataFrame to that index range
    if index_range:
        df = df.iloc[index_range[0] : index_range[1]]

    # Determine which points are inside the boundary
    inside_mask = (
        (df["x"] >= x_range[0])
        & (df["x"] <= x_range[1])
        & (df["y"] >= y_range[0])
        & (df["y"] <= y_range[1])
    )
    outside_mask = ~inside_mask

    # Plotting
    plt.figure(figsize=(10, 10))
    # Plot points inside the boundary in blue
    plt.scatter(
        df.loc[inside_mask, "x"],
        df.loc[inside_mask, "y"],
        color="blue",
        s=1,
        label="Inside Boundary",
    )
    # Plot points outside the boundary in orange
    plt.scatter(
        df.loc[outside_mask, "x"],
        df.loc[outside_mask, "y"],
        color="orange",
        s=1,
        label="Outside Boundary",
    )

    plt.title(title)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    plt.grid(True)
    plt.show()


def custom_procrustes(A, B):
    """
    Aligns dataset A to dataset B using translation, uniform scaling, and rotation,
    and calculates the RMSE similarity metric between aligned A and B.

    Parameters:
    A, B : numpy arrays of points (each row corresponds to a point)

    Returns:
    A_aligned : numpy array, aligned A to B
    R : numpy array, rotation matrix
    scaling : float, uniform scaling factor
    translation : numpy array, translation vector
    RMSE : float, root mean square error between A_aligned and B
    """
    # Calculate the centroids of both datasets
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Center the datasets
    A_centered = A - centroid_A
    B_centered = B - centroid_B

    # Normalize the datasets to unit size
    norm_A = np.linalg.norm(A_centered, "fro")
    norm_B = np.linalg.norm(B_centered, "fro")
    A_normalized = A_centered / norm_A if norm_A > 0 else A_centered
    B_normalized = B_centered / norm_B if norm_B > 0 else B_centered

    # Singular Value Decomposition (SVD) to find the best rotation
    U, S, Vt = np.linalg.svd(np.dot(B_normalized.T, A_normalized))
    R = np.dot(U, Vt)

    # Check if we need to correct the reflection issue
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(U, Vt)

    # Apply the rotation to the normalized dataset A
    A_rotated = np.dot(A_normalized, R.T)

    # Scale back to the size of B
    scaling = norm_B / norm_A if norm_A > 0 else 1
    A_scaled = A_rotated * norm_A * scaling

    # Calculate the translation
    translation = centroid_B

    # Translate to align with dataset B
    A_aligned = A_scaled + translation

    # Calculate the RMSE between A_aligned and B
    RMSE = np.sqrt(np.mean(np.linalg.norm(A_aligned - B, axis=1) ** 2))

    return A_aligned, R, scaling, translation, RMSE


def read_processed_log(file_path):
    """Read the CSV file and extract PM pose data."""
    df = pd.read_csv(file_path)
    pm_positions = df[["PM_x", "PM_y"]].values
    return pm_positions


def plot_results(
    gt_positions, pm_positions, transformed_pm_positions, title="Aligned frame"
):
    plt.figure(figsize=(10, 8))
    plt.scatter(
        gt_positions[:, 0], gt_positions[:, 1], color="red", label="Ground Truth"
    )
    plt.scatter(
        pm_positions[:, 0],
        pm_positions[:, 1],
        color="blue",
        alpha=0.6,
        label="Original PM Positions",
    )
    plt.scatter(
        transformed_pm_positions[:, 0],
        transformed_pm_positions[:, 1],
        color="green",
        alpha=0.8,
        label=title,
    )
    plt.title("Alignment using Procrustes Analysis")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.legend()
    plt.grid(True)
    plt.show()


def verify_data(df):
    print("DataFrame head:", df.head())  # Print the first few rows
    print(
        "DataFrame info:", df.info()
    )  # Summary info to check data types and non-null counts


# %%

# Define the paths to your log filesa
raw_log_path = "docs/logs/test_translated_table_max_1_7m_no_ppl.txt"
processed_log_path = "docs/logs/logs_offlineAnalysis/test_translated_table_max_1_7m_no_ppl_wo_semantics.txt"
# %%

df_raw = process_log_file(raw_log_path)

projected_df = project_lidar_points(df_raw)
verify_data(projected_df)
verify_data(df_raw)
plot_lidar_points_with_boundary(projected_df, (-0.85, 0.85), (-0.6, 0.6))


# %%


# Example usage with your data
gt_positions = np.array([row["pose"][:2] for index, row in df_raw.iterrows()])
pm_positions = read_processed_log(processed_log_path)

A_aligned, R, scale, translation, RMSE = custom_procrustes(gt_positions, pm_positions)

# print("Aligned A:", A_aligned)

print("Rotation Matrix:\n", R)
print("Scale factor:\n", scale)
print("Translation Vector:", translation)
print("RMSE:", RMSE)


plot_results(gt_positions, pm_positions, A_aligned, "GT aligned to PM")


# %%
