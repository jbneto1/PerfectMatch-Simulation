#%%
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


# Function to draw an ellipse based on the covariance matrix
def draw_cov_ellipse(cov, pos, nstd=1, **kwargs):
    """
    Draw a covariance ellipse based on a covariance matrix (cov) and a position (pos).
    
    Parameters:
    - cov: 2x2 covariance matrix.
    - pos: The (x, y) position of the ellipse center.
    - nstd: Number of standard deviations. The default is 1.
    - **kwargs: Additional keyword arguments passed to the Ellipse patch.
    """
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    theta = np.degrees(np.arctan2(*vecs[:, 0][::-1])) #FIXME: check atan2 (x,y or y,x)
    width, height = 2 * nstd * np.sqrt(vals)
    ellipse = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)
    
    return ellipse
#%%
# Assuming your log file is named 'log_file.csv' and is located in the current directory
file_path = 'docs/logs/logs_offlineAnalysis/offlineAnalysis_2024-02-13_20-25-52.txt'
file_path_semantics = "docs/logs/logs_offlineAnalysis/offlineAnalysis_w_semantics2024-02-13_20-25-52.txt"

df = pd.read_csv(file_path)
df_semantics = pd.read_csv(file_path_semantics)

print("DF w/o semantics")
print(df.head())
print(df.shape, df.ndim)

print("DF with semantics")
print(df_semantics.head())
print(df_semantics.shape, df_semantics.ndim)
#%%

# Create a figure with two subplots (1 row, 2 columns)
fig, axs = plt.subplots(1, 2, figsize=(14, 7))
decimation_factor = 15
alpha = 0.05

# First subplot for df
axs[0].scatter(df['EKF_x'], df['EKF_y'], s=10, label='EKF Points w/o Semantics')
for i in range(0, len(df), decimation_factor):
    cov_matrix = [[df.iloc[i]['EKFCovXX'], df.iloc[i]['EKFCovXY']], 
                  [df.iloc[i]['EKFCovYX'], df.iloc[i]['EKFCovYY']]]
    ellipse = draw_cov_ellipse(cov_matrix, (df.iloc[i]['EKF_x'], df.iloc[i]['EKF_y']), alpha=alpha, color='red')
    axs[0].add_patch(ellipse)
axs[0].set_xlabel('EKF_x')
axs[0].set_ylabel('EKF_y')
axs[0].legend()
axs[0].set_title('Without Semantics')

# Second subplot for df_semantics
axs[1].scatter(df_semantics['EKF_x'], df_semantics['EKF_y'], s=10, label='EKF Points w/ Semantics')
for i in range(0, len(df_semantics), decimation_factor):
    cov_matrix = [[df_semantics.iloc[i]['EKFCovXX'], df_semantics.iloc[i]['EKFCovXY']], 
                  [df_semantics.iloc[i]['EKFCovYX'], df_semantics.iloc[i]['EKFCovYY']]]
    ellipse = draw_cov_ellipse(cov_matrix, (df_semantics.iloc[i]['EKF_x'], df_semantics.iloc[i]['EKF_y']), alpha=alpha, color='blue')
    axs[1].add_patch(ellipse)
axs[1].set_xlabel('EKF_x')
axs[1].set_ylabel('EKF_y')
axs[1].legend()
axs[1].set_title('With Semantics')

plt.tight_layout()  # Adjust subplots to fit into the figure area.
plt.show()
# %% Error comparison of both systems

fig, axs = plt.subplots(2, 2, figsize=(15, 10))

# Error in X
axs[0, 0].plot(df['errorEKF_x'], label='System w/o semantics Error X')
axs[0, 0].plot(df_semantics['errorEKF_x'], label='System with semantics Error X')
axs[0, 0].set_title('Error in X')
axs[0, 0].legend()

# Error in Y
axs[0, 1].plot(df['errorEKF_y'], label='System w/o semantics Error Y')
axs[0, 1].plot(df_semantics['errorEKF_y'], label='System with semantics Error Y')
axs[0, 1].set_title('Error in Y')
axs[0, 1].legend()

# Error in Theta
axs[1, 0].plot(df['errorEKF_theta'], label='System w/o semantics Error Theta')
axs[1, 0].plot(df_semantics['errorEKF_theta'], label='System with semantics Error Theta')
axs[1, 0].set_title('Error in Theta')
axs[1, 0].legend()

# Error PM (if it exists in both)
axs[1, 1].plot(df['errorPM'], label='System w/o semantics Error PM')
axs[1, 1].plot(df_semantics['errorPM'], label='System with semantics Error PM')
axs[1, 1].set_title('Error PM')
axs[1, 1].legend()

plt.tight_layout()
plt.show()


# %% EKF POSE comparison
plt.figure(figsize=(10, 8))

# Plot EKF paths for both systems
plt.plot(df['EKF_x'], df['EKF_y'], label='System w/o semantics Path')
plt.plot(df_semantics['EKF_x'], df_semantics['EKF_y'], label='System with semantics Path')
plt.xlabel('EKF_x')
plt.ylabel('EKF_y')
plt.title('EKF Pose Comparison')
plt.legend()
plt.axis('equal')  # Ensure equal scaling for x and y axes
plt.show()


# %%
def error_metrics_statistics(df1, df2):
    error_types = ['errorEKF_x', 'errorEKF_y', 'errorEKF_theta', 'errorPM']
    stats = ['mean', 'std', 'max', 'min']

    metrics = {f'{stat}_{error}': [] for stat in stats for error in error_types}

    for error_type in error_types:
        metrics[f'mean_{error_type}'].append(df1[error_type].mean())
        metrics[f'std_{error_type}'].append(df1[error_type].std())
        metrics[f'max_{error_type}'].append(df1[error_type].max())
        metrics[f'min_{error_type}'].append(df1[error_type].min())

        metrics[f'mean_{error_type}'].append(df2[error_type].mean())
        metrics[f'std_{error_type}'].append(df2[error_type].std())
        metrics[f'max_{error_type}'].append(df2[error_type].max())
        metrics[f'min_{error_type}'].append(df2[error_type].min())

    error_metrics_df = pd.DataFrame(metrics, index=['System w/o semantics', 'System with semantics'])

    return error_metrics_df

error_stats = error_metrics_statistics(df, df_semantics)
print(error_stats)


# %%
