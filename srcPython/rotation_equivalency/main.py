import numpy as np


def rotation_matrix_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rotation_matrix_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotation_matrix_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_rotation_equivalence(R1, R2, vectors):
    transformed1 = [np.dot(R1, v) for v in vectors]
    transformed2 = [np.dot(R2, v) for v in vectors]
    return all(np.allclose(v1, v2) for v1, v2 in zip(transformed1, transformed2))


# Define angles
alpha = np.deg2rad(90)  # Example angle for Rx
beta = np.deg2rad(0)  # Example angle for Ry'
gamma = np.deg2rad(-90)  # Example angle for Rz''

# Compute individual rotation matrices
Rx = rotation_matrix_x(alpha)
Ry = rotation_matrix_y(beta)
Rz = rotation_matrix_z(gamma)

# Compute total rotation matrix
R_total = Rx @ Rz @ Ry
R_CL = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])

print(R_total)
# print(R_CL)

t_cl = np.array([0, -0.055, -0.155 / 2])

T_CL_map = np.array(
    [[0, 1, 0, 0], [0, 0, -1, -0.055], [-1, 0, 0, -0.155 / 2], [0, 0, 0, 1]]
)

T_CL_trait_bryan = np.zeros((4, 4))
T_CL_trait_bryan[:3, :3] = R_total
T_CL_trait_bryan[:3, 3] = t_cl
T_CL_trait_bryan[3, 3] = 1


p_L = np.array([1, 0, 0, 1])

# print(np.dot(T_CL_map, p_L))

# print(np.dot(T_CL_trait_bryan, p_L))


TH_LC = np.eye(4)
TH_LC[:3, :3] = R_total
TH_LC[:3, 3] = t_cl

# print(TH_LC)
