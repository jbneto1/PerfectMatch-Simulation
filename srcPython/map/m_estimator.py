import numpy as np
import matplotlib.pyplot as plt

def d_err(d, c_err):
    d = np.float64(d)
    c2 = c_err * c_err
    return 1 - c2 / (c2 + d * d)

# Define a symmetric range of d values
d_values = np.linspace(-701, 701, 1000)  # You can adjust the range as needed

# Define a list of c_err values to visualize
c_err_values = [40]  # This is just an example list; adjust as needed

# Plot d_err for each c_err value
plt.figure(figsize=(10, 6))
for c_err in c_err_values:
    plt.plot(d_values, d_err(d_values, c_err), label=f"c_err = {c_err}")

plt.title("Behavior of d_err for different c_err values")
plt.xlabel("d")
plt.ylabel("d_err(d)")
plt.legend()
plt.grid(True)
plt.xticks(np.arange(-700, 700, step=100))
plt.show()
