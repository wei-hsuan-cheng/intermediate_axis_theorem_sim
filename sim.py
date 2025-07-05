import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

# Moments of inertia
I1, I2, I3 = 1.0, 2.0, 3.0

# Euler's equations
def euler_eq(t, omega):
    w1, w2, w3 = omega
    return [
        ((I2 - I3) / I1) * w2 * w3,
        ((I3 - I1) / I2) * w3 * w1,
        ((I1 - I2) / I3) * w1 * w2
    ]

# Time and initial omega
omega_y = 4.0  # [Hz] Mainly intermediate axis
omega0 = np.array([omega_y * 0.0, omega_y, omega_y * 0.001]) * 2 * np.pi  # [rad/s] 
t_span = (0, 20)
t_eval = np.linspace(*t_span, 1000)

# Integrate Euler's equations
sol = solve_ivp(euler_eq, t_span, omega0, t_eval=t_eval)
omega_t = sol.y.T
dt = t_eval[1] - t_eval[0]

# Integrate quaternion over time
def integrate_quaternion(omega_t, t_eval):
    q = np.array([1, 0, 0, 0])  # Identity quaternion
    qs = [q.copy()]

    for i in range(1, len(t_eval)):
        w = omega_t[i]  # Angular velocity in body frame
        w_quat = np.concatenate([[0], w])
        dq = 0.5 * quat_mult(q, w_quat)
        q = q + dq * dt
        q = q / np.linalg.norm(q)
        qs.append(q.copy())

    return np.array(qs)

# Quaternion multiplication
def quat_mult(q, r):
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ])

# Get body orientation over time
quats = integrate_quaternion(omega_t, t_eval)

# Setup 3D plot
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_box_aspect([1,1,1])
ax.set_title("Tennis Racket Theorem – 3D Rotation")
ax.view_init(elev=30, azim=45)

# Initial body axes
x_line, = ax.plot([], [], [], 'r', lw=3, label='x-body')
y_line, = ax.plot([], [], [], 'g', lw=3, label='y-body')
z_line, = ax.plot([], [], [], 'b', lw=3, label='z-body')

ax.legend()

# Animation update
def update(i):
    rot = R.from_quat([quats[i,1], quats[i,2], quats[i,3], quats[i,0]])  # scipy uses (x, y, z, w)
    R_mat = rot.as_matrix()

    origin = np.array([0, 0, 0])
    x_axis = R_mat[:, 0]
    y_axis = R_mat[:, 1]
    z_axis = R_mat[:, 2]

    x_line.set_data([origin[0], x_axis[0]], [origin[1], x_axis[1]])
    x_line.set_3d_properties([origin[2], x_axis[2]])

    y_line.set_data([origin[0], y_axis[0]], [origin[1], y_axis[1]])
    y_line.set_3d_properties([origin[2], y_axis[2]])

    z_line.set_data([origin[0], z_axis[0]], [origin[1], z_axis[1]])
    z_line.set_3d_properties([origin[2], z_axis[2]])

    return x_line, y_line, z_line

ani = FuncAnimation(fig, update, frames=len(t_eval), interval=20, blit=True)
plt.show()




# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.integrate import solve_ivp

# # Moments of inertia (I1 < I2 < I3)
# I1, I2, I3 = 1.0, 2.0, 3.0

# # Euler's equations of motion
# def euler_equations(t, omega):
#     w1, w2, w3 = omega
#     dw1 = ((I2 - I3) / I1) * w2 * w3
#     dw2 = ((I3 - I1) / I2) * w3 * w1
#     dw3 = ((I1 - I2) / I3) * w1 * w2
#     return [dw1, dw2, dw3]

# # Initial angular velocity (perturb middle axis slightly)
# omega0 = [0.0, 1.0, 0.01]  # Rotate mainly around I2

# # Time span
# t_span = (0, 50)
# t_eval = np.linspace(*t_span, 2000)

# # Solve the system
# sol = solve_ivp(euler_equations, t_span, omega0, t_eval=t_eval)

# # Plot the angular velocity components over time
# plt.figure(figsize=(10, 5))
# plt.plot(sol.t, sol.y[0], label=r'$\omega_1$')
# plt.plot(sol.t, sol.y[1], label=r'$\omega_2$')
# plt.plot(sol.t, sol.y[2], label=r'$\omega_3$')
# plt.xlabel('Time')
# plt.ylabel('Angular Velocity')
# plt.title('Rigid Body Rotation – Tennis Racket Theorem')
# plt.legend()
# plt.grid()
# plt.tight_layout()
# plt.show()
