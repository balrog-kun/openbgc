#!/usr/bin/python3
import numpy as np

def normalize(v): return v / np.linalg.norm(v)

def skew(v):
    x, y, z = v
    return np.array([
        [0, -z, y],
        [z,  0, -x],
        [-y, x, 0]
    ])

def exp_so3(w):
    theta = np.linalg.norm(w)
    if theta < 1e-8:
        return np.eye(3)
    k = w / theta
    K = skew(k)
    return np.eye(3) + np.sin(theta)*K + (1 - np.cos(theta))*(K @ K)

def log_SO3(R):
    trace = np.trace(R)
    cos_theta = (trace - 1) / 2
    cos_theta = np.clip(cos_theta, -1, 1)
    theta = np.arccos(cos_theta)
    if theta < 1e-8:
        return np.zeros(3)
    lnR = (theta / (2*np.sin(theta))) * (R - R.T)
    return np.array([lnR[2,1], lnR[0,2], lnR[1,0]])

def forward_kinematics(theta, axes):
    R = np.eye(3)
    for i in range(3):
        R = R @ exp_so3(theta[i] * axes[i])
    return R

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

def inverse_kinematics(R_target, axes, max_iter=10, tol=1e-6, theta_init=None):
    theta = np.zeros(3) if theta_init is None else theta_init.copy()
    for _ in range(max_iter):
        R_theta = forward_kinematics(theta, axes)
        delta_R = R_theta.T @ R_target
        log_err = -log_SO3(delta_R)
        if np.linalg.norm(log_err) < tol:
            break
        J = compute_jacobian_at(theta, axes)
        try:
            delta_theta = np.linalg.solve(J, log_err)
        except np.linalg.LinAlgError:
            delta_theta = np.linalg.lstsq(J + 1e-4 * np.eye(3), log_err, rcond=None)[0]
        theta += delta_theta
    return theta

# ----------------- TEST -----------------

# Axes: Z, X, Y
axes = [normalize(np.array([0,0,1])),
        normalize(np.array([1,0,0])),
        normalize(np.array([0,1,0]))]

# True angles: 10 deg around Z, 0 elsewhere
angles_true = np.radians([0, 10, 15])
R_target = forward_kinematics(angles_true, axes)

# Initial guess: linear estimate
J0 = compute_jacobian_at(np.zeros(3), axes)
theta0 = np.linalg.solve(J0, log_SO3(R_target))

# Refined via Newton
theta_est = inverse_kinematics(R_target, axes, theta_init=theta0)

# Results
print("True angles (deg):     ", np.degrees(angles_true))
print("Estimated angles (deg):", np.degrees(theta_est))
print("Error (deg):           ", np.degrees(theta_est - angles_true))
