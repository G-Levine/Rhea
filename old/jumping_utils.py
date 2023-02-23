import numpy as np
from scipy.integrate import solve_ivp
from matplotlib import pyplot as plt

from linkage_utils import forward_kinematics, calc_limiting_angles, plot_linkage

num_legs = 2
robot_mass = 7.5
mass = robot_mass / 2
g = 9.81
torque = 35 #50
gear_ratio = 9 #7
rotor_inertia = 1200 / (1000 * 100 ** 2) #5675 / (1000 * 100 ** 2)
rotor_refl_inertia = rotor_inertia * (gear_ratio * 2) ** 2
max_power = 80#500
eps = 0.00001
max_speed = 120 * 2 * np.pi / 60 # 120 RPM

"""
Calculates the derivative of the z position of the robot's body with respect to the angle of the actuator output shaft.
"""
def dzdtheta(forward_kinematics_fn, link_lengths, coupler_eef_offset, frame_rotation, theta):
    delta = 0.001
    # Multiply by -1 because negative z-axis motion of the end effector corresponds to positive z-axis motion of the robot's body.
    return -1 * (forward_kinematics_fn(link_lengths, coupler_eef_offset, theta + delta, frame_rotation)[0][1] - forward_kinematics_fn(link_lengths, coupler_eef_offset, theta, frame_rotation)[0][1]) / delta

"""
Simplified model of the jumping robot.
Treats the robot as a point mass being accelerated in the vertical direction, with the reflected inertia of the actuator additionally modeled.
Starts at min_angle and ends at max_angle.
The actuator is torque- and power-limited -- always applies the maximum possible torque given its current angular velocity.

State variables:
    y[0] = actuator output shaft angle
    y[1] = actuator output shaft angular velocity
"""
def create_jumping_model(forward_kinematics_fn, link_lengths, coupler_eef_offset, frame_rotation):
    def jumping_model(t,y):
        # if any(np.isnan(y)):
            # assert False
        dzdtheta_t = dzdtheta(forward_kinematics_fn, link_lengths, coupler_eef_offset, frame_rotation, y[0])

        """
        Reflected inertia of the robot's body.
        dzdtheta_t = v / w
        1/2 * m * v ** 2 = 1/2 * I * w ** 2
        I = m * v ** 2 / w ** 2
        = m * dzdtheta_t ** 2
        """
        body_refl_inertia = mass * dzdtheta_t ** 2

        total_inertia_at_output_shaft = rotor_refl_inertia + body_refl_inertia

        """
        Torque at the output shaft due to gravity.
        F = m * g
        tau = dzdtheta_t * F
        = dzdtheta_t * m * g
        """
        gravity_torque = mass * g * dzdtheta_t

        output_torque = min(torque, max_power / (np.abs(y[1]) + eps))
        # print(y[0], dzdtheta_t, output_torque - gravity_torque, end_event(t, y))

        dydt = np.zeros(2)
        dydt[0] = y[1]
        dydt[1] = (output_torque - gravity_torque) / total_inertia_at_output_shaft
        return dydt
    return jumping_model

def get_apex_height(forward_kinematics_fn, link_lengths, coupler_eef_offset, frame_rotation, theta_min, theta_max):
    end_event_pos = lambda t, y: theta_max - y[0]
    end_event_vel = lambda t, y: max_speed - np.abs(y[1])
    end_event_pos.terminal = True
    end_event_vel.terminal = True

    jumping_model = create_jumping_model(forward_kinematics_fn, link_lengths, coupler_eef_offset, frame_rotation)

    sol = solve_ivp(jumping_model, t_span=[0, 1.0], y0=[theta_min, 0], events=[end_event_pos, end_event_vel], max_step=0.01)
    w_final = sol.y[1,-1]
    v_z_final = sol.y[1,-1] * dzdtheta(forward_kinematics_fn, link_lengths, coupler_eef_offset, frame_rotation, sol.y[0,-1])
    if v_z_final < 0:
        return 0
    z_apex = 0.5 * v_z_final ** 2 / g
    print("w_final:", w_final, "v_z_final:", v_z_final, "z_apex:", z_apex)
    return z_apex

if __name__ == "__main__":
    # Define the link lengths.
    link_lengths = np.array([0.1, 0.25, 0.05, 0.2])

    # Define the coupler end effector offset.
    coupler_eef_offset = np.array([-0.22, -0.05])

    # Define the frame rotation.
    frame_rotation = np.pi * 0.9

    # Calculate the limiting angles.
    theta_min, theta_max = calc_limiting_angles(link_lengths)
    theta_tolerance = 0.2
    theta_min += theta_tolerance
    theta_max -= theta_tolerance

    end_event_pos = lambda t, y: theta_max - y[0]
    end_event_vel = lambda t, y: max_speed - np.abs(y[1])
    end_event_pos.terminal = True
    end_event_vel.terminal = True

    jumping_model = create_jumping_model(forward_kinematics, link_lengths, coupler_eef_offset, frame_rotation)

    sol = solve_ivp(jumping_model, t_span=[0, 1.0], y0=[theta_min, 0], events=[end_event_pos, end_event_vel], max_step=0.01)#, t_eval=np.linspace(0, 1.0, 50), max_step=0.001)
    w_final = sol.y[1,-1]
    v_z_final = sol.y[1,-1] * dzdtheta(forward_kinematics, link_lengths, coupler_eef_offset, frame_rotation, sol.y[0,-1])
    print("w_final:", w_final, "v_z_final:", v_z_final, "z_apex:", get_apex_height(forward_kinematics, link_lengths, coupler_eef_offset, frame_rotation, theta_min, theta_max))
    # plt.plot(sol.t, sol.y[0,:])
    # plt.plot(sol.t, sol.y[1,:])
    # plt.show()

    # plot the linkage state over time
    for i in range(len(sol.t)):
        if i % 2 != 0:
            continue
        fk_results = forward_kinematics(link_lengths, coupler_eef_offset, sol.y[0,i], frame_rotation)
        plot_linkage(fk_results)
        plt.show()