import numpy as np
from matplotlib import pyplot as plt

"""
Definitions:
    link_lengths[0] = ground link length
    link_lengths[1] = driven link length
    link_lengths[2] = coupler link length
    link_lengths[3] = rocker link length
"""

"""
Calculate the limiting angles for the four-bar linkage.
"""
def calc_limiting_angles(link_lengths):
    theta_min = np.arccos((link_lengths[0] ** 2 + link_lengths[1] ** 2 - (link_lengths[3] - link_lengths[2]) ** 2) / (2 * link_lengths[0] * link_lengths[1]))
    theta_max = np.arccos((link_lengths[0] ** 2 + link_lengths[1] ** 2 - (link_lengths[3] + link_lengths[2]) ** 2) / (2 * link_lengths[0] * link_lengths[1]))
    return theta_min, theta_max

"""
Forward kinematics for the four-bar linkage.
"""
def forward_kinematics(link_lengths, coupler_eef_offset, theta, frame_rotation):
    # Calculate the positions of the start of the driven and rocker links.
    x_driven = np.array([0, 0])
    x_rocker = np.array([link_lengths[0], 0])

    # Calculate the position of the end of the driven link / start of the coupler link.
    x_coupler = np.array([link_lengths[1] * np.cos(theta), link_lengths[1] * np.sin(theta)]) + x_driven

    # Use the law of cosines to calculate the length of the diagonal of the triangle formed by the driven link and the ground link.
    diagonal_length = np.sqrt(link_lengths[0] ** 2 + link_lengths[1] ** 2 - 2 * link_lengths[0] * link_lengths[1] * np.cos(theta))

    # Use the law of sines to calculate the angle between the driven link and the diagonal.
    driven_diagonal_angle = np.abs(np.arcsin(link_lengths[0] * np.sin(theta) / diagonal_length))

    # Use the law of cosines to calculate the angle between the diagonal and the coupler link.
    diagonal_coupler_angle = np.abs(np.arccos((diagonal_length ** 2 + link_lengths[2] ** 2 - link_lengths[3] ** 2) / (2 * diagonal_length * link_lengths[2])))

    # Calculate the angle between the driven link and the coupler link.
    driven_coupler_angle = driven_diagonal_angle + diagonal_coupler_angle

    # Calculate the angle between the ground link and the coupler link.
    ground_coupler_angle = theta - np.pi + driven_coupler_angle

    # Calculate the position of the end of the rocker link / end of the coupler link.
    x_rocker_coupler = np.array([link_lengths[2] * np.cos(ground_coupler_angle), link_lengths[2] * np.sin(ground_coupler_angle)]) + x_coupler

    # Calculate the position of the end effector
    x_eef = np.array([[np.cos(ground_coupler_angle), -np.sin(ground_coupler_angle)], [np.sin(ground_coupler_angle), np.cos(ground_coupler_angle)]]) @ coupler_eef_offset + x_coupler

    frame_rotation_matrix = np.array([[np.cos(frame_rotation), -np.sin(frame_rotation)], [np.sin(frame_rotation), np.cos(frame_rotation)]])

    return [frame_rotation_matrix @ x for x in [x_eef, x_driven, x_rocker, x_coupler, x_rocker_coupler]]

"""
Plot the linkage.
"""
def plot_linkage(fk_results):
    # Make the plot square.
    plt.axis('square')
    # Set the x and y limits.
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    plt.grid()
    plt.xlabel('x')
    plt.ylabel('z')
    plt.title('Four-bar linkage')
    x_eef, x_driven, x_rocker, x_coupler, x_rocker_coupler = fk_results
    # Plot the driven link.
    plt.plot([x_driven[0], x_coupler[0]], [x_driven[1], x_coupler[1]], 'b')
    # Plot the rocker link.
    plt.plot([x_rocker[0], x_rocker_coupler[0]], [x_rocker[1], x_rocker_coupler[1]], 'g')
    # Plot the coupler link.
    plt.plot([x_coupler[0], x_rocker_coupler[0]], [x_coupler[1], x_rocker_coupler[1]], 'y')
    # Plot the end effector.
    plt.plot([x_coupler[0], x_eef[0]], [x_coupler[1], x_eef[1]], 'r')
    plt.plot([x_eef[0]], [x_eef[1]], 'ro')

"""
Test the forward kinematics function by plotting the linkage.
"""
def test_forward_kinematics():
    # Define the link lengths.
    link_lengths = np.array([0.1, 0.25, 0.05, 0.2])

    # Define the coupler end effector offset.
    coupler_eef_offset = np.array([-0.22, -0.05])

    # Define the frame rotation.
    frame_rotation = np.pi * 3 / 4

    # Calculate the limiting angles.
    theta_min, theta_max = calc_limiting_angles(link_lengths)

    # Calculate the positions of the end effector.
    theta_tolerance = 0.1
    thetas = np.linspace(theta_min + theta_tolerance, theta_max - theta_tolerance, 10)
    # thetas = np.array([(theta_min + theta_max) / 2])
    x_eefs = np.zeros((2, thetas.size))

    for i in range(thetas.size):
        fk_results = forward_kinematics(link_lengths, coupler_eef_offset, thetas[i], frame_rotation)
        x_eefs[:, i] = fk_results[0]
        plot_linkage(fk_results)
    # Plot the end effector trajectory.
    plt.plot(x_eefs[0, :], x_eefs[1, :], 'r--')
    plt.show()

if __name__ == "__main__":
    test_forward_kinematics()