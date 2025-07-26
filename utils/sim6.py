#! /usr/bin/python3
import numpy as np
import tkinter as tk
from tkinter import ttk
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.spatial.transform import Rotation as R

# ----------------- Utility Functions -----------------
def normalize(v): return v / np.linalg.norm(v)

def joint_axes_global(a, theta):
    q1 = R.from_rotvec(theta[0] * a[0])
    q2 = q1 * R.from_rotvec(theta[1] * a[1])

    A0 = a[0]
    A1 = q1.apply(a[1])
    A2 = q2.apply(a[2])
    return A0, A1, A2

def compute_joint_velocities(q_current, q_target, a, theta, gain=1.0):
    q_err = R.from_quat(q_target) * R.from_quat(q_current).inv()
    omega = q_err.as_rotvec()
    A0, A1, A2 = joint_axes_global(a, theta)
    J = np.column_stack([A0, A1, A2])
    # V1 was:
    #dtheta = gain * J.T @ omega
    # V2 with damping:
    #JTJ = J.T @ J + 0.1 * np.eye(3)
    #dtheta = gain * np.linalg.solve(JTJ, J.T @ omega)
    dtheta = gain * J.T @ omega - 0.001 * theta
    return dtheta

def compute_joint_velocities_p(q_current, q_target, a, theta, gain=1.0):
    q_err = R.from_quat(q_target) * R.from_quat(q_current).inv()
    omega = q_err.as_rotvec()
    A0, A1, A2 = joint_axes_global(a, theta)
    dtheta = gain * np.array([np.dot(omega, A0), np.dot(omega, A1), np.dot(omega, A2)])
    return dtheta

def angles_to_quat(a, theta):
    q0 = R.from_rotvec(theta[0] * a[0])
    q1 = R.from_rotvec(theta[1] * a[1])
    q2 = R.from_rotvec(theta[2] * a[2])
    return (q0 * q1 * q2).as_quat()

# ----------------- GUI Setup -----------------
class GimbalSim:
    def __init__(self, master):
        self.master = master
        master.title("3-Axis Gimbal Simulator")

        # Gimbal axes
        self.a = [
            normalize(np.array([0, 0, 1])),
            normalize(np.array([0, 1, 0])),
            normalize(np.array([1, 0, 0]))
        ]

        self.theta = np.array([0.0, 0.0, 0.0])
        self.q_target = R.from_euler('zyx', [0, 0, 0], degrees=True).as_quat()
        self.theta = np.array([0.77952747, -0.60897021, -1.65113853])     ### example
        self.q_target = np.array([-0.3831899, 0.1103326, -0.46967863, -0.78765107]) ### example
        # This one shows a very unobvious path with 2 or 3 turns:
        self.theta = np.array([0.88731989, -0.56308082, 0.63097139])     ### example
        self.q_target = np.array([0.21515867, 0.73729213, -0.5666915, -0.2982747]) ### example

        self.create_widgets()
        self.update_plot()

    def create_widgets(self):
        frame = ttk.Frame(self.master)
        frame.pack()

        for i, axis in enumerate(['X', 'Y', 'Z']):
            ttk.Button(frame, text=f"{axis} +10°", command=lambda i=i: self.rotate_target(i, +10)).grid(row=0, column=i*2)
            ttk.Button(frame, text=f"{axis} -10°", command=lambda i=i: self.rotate_target(i, -10)).grid(row=0, column=i*2+1)

        self.start_j = ttk.Button(frame, text="Movement Jacobian", command=self.start_movement_j)
        self.start_j.grid(row=1, column=0, columnspan=3, pady=10)
        self.start_p = ttk.Button(frame, text="Movement Projection", command=self.start_movement_p)
        self.start_p.grid(row=1, column=3, columnspan=3, pady=10)

        # Plot
        self.fig = plt.Figure(figsize=(4, 4))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().pack(expand=True, fill='both')

    def rotate_target(self, axis_index, deg):
        r = R.from_rotvec(np.radians(deg) * np.eye(3)[axis_index])
        self.q_target = (r * R.from_quat(self.q_target)).as_quat()
        print('Target q is ' + str(self.q_target))
        self.update_plot()

    def start_movement_j(self):
        self.method_j = True
        threading.Thread(target=self.animate_to_target, daemon=True).start()

    def start_movement_p(self):
        self.method_j = False
        threading.Thread(target=self.animate_to_target, daemon=True).start()

    def animate_to_target(self):
        duration = 10.0
        dt = 0.1
        steps = int(duration / dt)
        self.start_j.config(state="disabled")
        self.start_p.config(state="disabled")
        for _ in range(steps):
            q_current = angles_to_quat(self.a, self.theta)
            if self.method_j:
                dtheta = compute_joint_velocities(q_current, self.q_target, self.a, self.theta, gain=0.8)
            else:
                dtheta = compute_joint_velocities_p(q_current, self.q_target, self.a, self.theta, gain=0.8)
            self.theta += dtheta * dt
            self.update_plot()
            print('Current theta is ' + str(self.theta))
            time.sleep(dt)
        self.start_j.config(state="normal")
        self.start_p.config(state="normal")

    def draw_camera(self, Rmat, target=False):
        origin = np.array([0, 0, 0])
        axes = Rmat
        linestyle = '--' if target else '-'
        for i, c in enumerate(['r', 'g', 'b']):
            self.ax.quiver(*origin, *axes[:, i], color=c, length=0.9, normalize=True, linestyle=linestyle)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])

        R_current = R.from_quat(angles_to_quat(self.a, self.theta)).as_matrix()
        R_target = R.from_quat(self.q_target).as_matrix()

        self.draw_camera(R_current)
        self.draw_camera(R_target, target=True)

        self.ax.set_title("Camera Frame")
        self.canvas.draw()

# ----------------- Run -----------------
if __name__ == '__main__':
    root = tk.Tk()
    sim = GimbalSim(root)
    root.mainloop()
