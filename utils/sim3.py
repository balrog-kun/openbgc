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

def exp_so3(w):
    """Exponential map from axis-angle vector w (3D) to SO(3) matrix"""
    theta = np.linalg.norm(w)
    if theta < 1e-6:
        return np.eye(3)
    k = w / theta
    K = skew(k)
    return np.eye(3) + np.sin(theta)*K + (1 - np.cos(theta))*(K @ K)

def log_SO3(R):
    """Log map from rotation matrix to axis-angle vector"""
    cos_theta = (np.trace(R) - 1) / 2
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-6:
        return np.zeros(3)
    lnR = (theta / (2 * np.sin(theta))) * (R - R.T)
    return np.array([lnR[2, 1], lnR[0, 2], lnR[1, 0]])

def forward_kinematics(theta, axes):
    R = np.eye(3)
    for i in range(3):
        R = R @ exp_so3(theta[i] * axes[i])
    return R

def compute_jacobian(axes):
    """Numerically compute the Jacobian of FK at theta=0"""
    eps = 1e-6
    J = np.zeros((3,3))
    base_R = forward_kinematics([0, 0, 0], axes)
    for i in range(3):
        dtheta = np.zeros(3)
        dtheta[i] = eps
        R_eps = forward_kinematics(dtheta, axes)
        delta = log_SO3(R_eps @ base_R.T)  # R_eps * R⁻¹ ≈ delta_R
        J[:,i] = delta / eps
    return J

def compute_jacobian_at(theta, axes, eps=1e-6):
    J = np.zeros((3,3))
    base_R = forward_kinematics(theta, axes)
    for i in range(3):
        dtheta = theta.copy()
        dtheta[i] += eps
        R_eps = forward_kinematics(dtheta, axes)
        delta = log_SO3(R_eps @ base_R.T)
        J[:,i] = delta / eps
    return J

J = compute_jacobian(state['a'])

def inverse_kinematics(R_target, axes, max_iter=10, tol=1e-6, theta_init=None):
    theta = np.zeros(3) if theta_init is None else theta_init.copy()
    for _ in range(max_iter):
        R_theta = forward_kinematics(theta, axes)
        delta_R = R_theta.T @ R_target
        log_err = -log_SO3(delta_R)
        if np.linalg.norm(log_err) < tol:
            break
        J2 = compute_jacobian_at(theta, axes)
        # Use damped least squares solve (stable even when J is nearly singular)
        JTJ = J2.T @ J2
        lambda_ = 1e-4  # damping factor
        delta_theta = np.linalg.solve(JTJ + lambda_ * np.eye(3), J2.T @ log_err)
        theta += delta_theta
    return theta

axes = [np.array([0,0,1]), np.array([1,0,0]), np.array([0,1,0])]
angles_true = np.radians([0, 10, 15])
R_target = forward_kinematics(angles_true, axes)

# Try linear init
J0 = compute_jacobian(axes)
theta0 = np.linalg.solve(J0, log_SO3(R_target))
print("True (deg):", np.degrees(angles_true))
print("Recovered (deg):", np.degrees(theta0))

# Then run Newton
theta_est = inverse_kinematics(R_target, axes, theta_init=theta0)
print("Recovered (deg):", np.degrees(theta_est))

def compute_angles(q0, q1, a):
    q_rel = q1 * q0.inv()
    #q_rel = q0.inv() * q1
    R_rel = q_rel.as_matrix()

    #log_r = log_SO3(R_rel)
    #theta = np.linalg.solve(J, log_r)
    theta = inverse_kinematics(R_rel, a)
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
