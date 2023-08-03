import numpy as np

def R_x(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def R_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def R_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

# roll pitch yaw = ZYX in euler
def roll_pitch_yaw_to_R(roll, pitch, yaw):
    return R_z(yaw) @ R_y(pitch) @ R_x(roll)


def R_to_roll_pitch_yaw(R):
    # roll pitch yaw = ZYX in euler
    pitch = np.arcsin(-R[2, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw

roll, pitch, yaw = 1.19730672, 1.07478656, 1.15162329

R_be = roll_pitch_yaw_to_R(roll, pitch, yaw)

R_eo = R_z(-38/180*np.pi)

R_bo = R_be @ R_eo

new_roll, new_pitch, new_yaw = R_to_roll_pitch_yaw(R_bo)

print(f"{new_roll:.4f} {new_pitch:.4f} {new_yaw:.4f}")
