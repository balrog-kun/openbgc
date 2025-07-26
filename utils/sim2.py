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

def compute_angles(q0, q1, a):
    """Constant-time angle calculation for arbitrary axes"""
    q_rel = q1 * q0.inv()
    R0 = q0.as_matrix()

    # Get axis directions in world frame
    world_axes = [R0 @ axis for axis in a] ### shuold be just a???
    rotated_axes = [q_rel.apply(axis) for axis in a]

    angles = np.zeros(3)

    # Solve for θ2 (last axis)
    v2 = rotated_axes[2]
    w2 = world_axes[2]
    angles[2] = np.arccos(np.clip(np.dot(v2, w2), -1, 1)) ##... so just the angle between a and q_rel @ a

    # Solve for θ1 (second axis)
    v1 = rotated_axes[1] - np.dot(rotated_axes[1], v2) * v2
    w1 = world_axes[1] - np.dot(world_axes[1], w2) * w2
    v1_norm = np.linalg.norm(v1)
    w1_norm = np.linalg.norm(w1)
    if v1_norm > 1e-6 and w1_norm > 1e-6:
        v1 /= v1_norm
        w1 /= w1_norm
        angles[1] = np.arccos(np.clip(np.dot(v1, w1), -1, 1))

    # Solve for θ0 (first axis)
    v0 = rotated_axes[0] - np.dot(rotated_axes[0], v1) * v1 - np.dot(rotated_axes[0], v2) * v2
    w0 = world_axes[0] - np.dot(world_axes[0], w1) * w1 - np.dot(world_axes[0], w2) * w2
    v0_norm = np.linalg.norm(v0)
    w0_norm = np.linalg.norm(w0)
    if v0_norm > 1e-6 and w0_norm > 1e-6:
        v0 /= v0_norm
        w0 /= w0_norm
        angles[0] = np.arccos(np.clip(np.dot(v0, w0), -1, 1))

    return angles

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
