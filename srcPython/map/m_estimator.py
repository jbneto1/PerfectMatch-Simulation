import numpy as np
import matplotlib.pyplot as plt


plt.rcParams["axes.titlesize"] = 16  # Title font size
plt.rcParams["axes.labelsize"] = 14  # Label font size
plt.rcParams["xtick.labelsize"] = 14  # Tick font size for x-axis
plt.rcParams["ytick.labelsize"] = 14  # Tick font size for y-axis
plt.rcParams["legend.fontsize"] = 14  # Legend font size


def d_err(d, c_err):
    d = np.float64(d)
    c2 = c_err * c_err
    return 1 - c2 / (c2 + d * d)


# Define a symmetric range of d values
d_values = np.linspace(-701, 701, 1000)  # You can adjust the range as needed

# Define a list of c_err values to visualize
c_err_values = [70]  # This is just an example list; adjust as needed

# Plot d_err for each c_err value
plt.figure(figsize=(10, 6))
for c_err in c_err_values:
    plt.plot(d_values, d_err(d_values, c_err), label=f"c_err = {c_err}")

plt.title("M-Estimator for c_err value of 70 mm")
plt.xlabel("Error [mm]")
plt.ylabel("M-Estimator (Error)")
plt.legend()
plt.grid(True)
plt.xticks(np.arange(-700, 751, step=100))  # Extending the range to include 700
plt.tight_layout()
plt.savefig("./srcPython/map/m_estimator.pdf", dpi=300)
