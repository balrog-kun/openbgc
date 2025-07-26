#! /usr/bin/python3
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import tkinter as tk
from tkinter import ttk

# Initialize state
state = {
    'q0': R.from_quat([0, 0, 0, 1]),  # Handle orientation (world frame)
    'a': np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]),  # Joint axes: [Z (yaw), X (roll), Y (pitch)]
    'angles': np.array([0.0, 0.0, 0.0]),  # Joint angles (radians)
    'q1': R.from_quat([0, 0, 0, 1])      # Camera orientation (world frame)
}

# Create figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_zlim(-1.5, 1.5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Draw gimbal and camera
handle = ax.plot([], [], [], color='brown', linewidth=10)[0]
camera = ax.plot([], [], [], color='black', linewidth=5)[0]

def normalize(v): return v / np.linalg.norm(v)

def skew(v):
    x, y, z = v
    return np.array([[0, -z, y],
                     [z, 0, -x],
                     [-y, x, 0]])

def project_angle_from_matrix(R, axis):
    # Get angle from rotation matrix projected onto axis
    v = axis
    w = R @ v
    cross = np.cross(v, w)
    sin_angle = np.linalg.norm(cross)
    cos_angle = np.dot(v, w)
    angle = np.arctan2(sin_angle, cos_angle)
    return angle

def rotation_matrix_from_axis_angle(axis, angle):
    axis = normalize(axis)
    K = skew(axis)
    return np.eye(3) + np.sin(angle)*K + (1 - np.cos(angle))*(K @ K)

def decompose_rotation_recursive(R_rel, axes, iters=2):
    theta = np.zeros(3)
    for _ in range(iters):
        # R = R1 * R2 * R3
        R1 = rotation_matrix_from_axis_angle(axes[0], theta[0])
        R2 = rotation_matrix_from_axis_angle(axes[1], theta[1])
        R12 = R1 @ R2
        R3_est = np.linalg.inv(R12) @ R_rel
        theta[2] = project_angle_from_matrix(R3_est, axes[2])

        R3 = rotation_matrix_from_axis_angle(axes[2], theta[2])
        R1_est = R_rel @ np.linalg.inv(R2 @ R3)
        theta[0] = project_angle_from_matrix(R1_est, axes[0])

        R1 = rotation_matrix_from_axis_angle(axes[0], theta[0])
        R3 = rotation_matrix_from_axis_angle(axes[2], theta[2])
        R2_est = np.linalg.inv(R1) @ R_rel @ np.linalg.inv(R3)
        theta[1] = project_angle_from_matrix(R2_est, axes[1])
    return theta

def compute_angles(q0, q1, a):
    q_rel = q1 * q0.inv()
    #q_rel = q0.inv() * q1

    theta = decompose_rotation_recursive(q_rel.as_matrix(), a)
    return theta

def update_state():
    """Compute q1 from q0 and joint angles, and log state."""
    q_total = state['q0']
    for i in range(3):
        q_total = q_total * R.from_rotvec(state['angles'][i] * state['a'][i])
    state['q1'] = q_total

    # Compute angles using the iterative algorithm
    computed_angles = compute_angles(state['q0'], state['q1'], state['a'])

    print(
        f"Current Angles (deg): Yaw={np.degrees(state['angles'][0]):.1f}, "
        f"Roll={np.degrees(state['angles'][1]):.1f}, "
        f"Pitch={np.degrees(state['angles'][2]):.1f}\n"
        f"Computed Angles (deg): Yaw={np.degrees(computed_angles[0]):.1f}, "
        f"Roll={np.degrees(computed_angles[1]):.1f}, "
        f"Pitch={np.degrees(computed_angles[2]):.1f}\n"
        f"q0 (handle): {state['q0'].as_quat()}\n"
        f"q1 (camera): {state['q1'].as_quat()}\n"
        "---"
    )

def draw():
    """Update 3D plot."""
    # Handle (cylinder along local Z-axis)
    handle_z = np.array([[0, 0, -1], [0, 0, 1]])
    handle_rotated = state['q0'].apply(handle_z)
    handle.set_data(handle_rotated[:, 0], handle_rotated[:, 1])
    handle.set_3d_properties(handle_rotated[:, 2])

    # Camera (simplified: a small box)
    camera_points = np.array([
        [0.2, 0.2, 0], [0.2, -0.2, 0], [-0.2, -0.2, 0], [-0.2, 0.2, 0],
        [0.2, 0.2, 0.4], [0.2, -0.2, 0.4], [-0.2, -0.2, 0.4], [-0.2, 0.2, 0.4]
    ])
    camera_rotated = state['q1'].apply(camera_points)
    camera.set_data(camera_rotated[:, 0], camera_rotated[:, 1])
    camera.set_3d_properties(camera_rotated[:, 2])
    plt.draw()

def on_button_click(joint_idx, delta_deg):
    """Update joint angles."""
    state['angles'][joint_idx] += np.radians(delta_deg)
    update_state()
    draw()

def on_handle_rotate(axis_idx, delta_deg):
    """Rotate gimbal handle."""
    state['q0'] = state['q0'] * R.from_rotvec(np.radians(delta_deg) * state['a'][axis_idx])
    update_state()
    draw()

# Add buttons (using Tkinter)
root = tk.Tk()
root.title("Gimbal Simulator Controls")

for i in range(3):
    ttk.Button(root, text=f"Joint {i} +10°", command=lambda i=i: on_button_click(i, 10)).pack()
    ttk.Button(root, text=f"Joint {i} -10°", command=lambda i=i: on_button_click(i, -10)).pack()

for i in range(3):
    ttk.Button(root, text=f"Rotate Handle +10° (a{i})", command=lambda i=i: on_handle_rotate(i, 10)).pack()
    ttk.Button(root, text=f"Rotate Handle -10° (a{i})", command=lambda i=i: on_handle_rotate(i, -10)).pack()

# Initialize
update_state()
draw()
plt.show(block=False)  # Non-blocking to allow Tkinter to run
root.mainloop()
