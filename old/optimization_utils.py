import numpy as np
from matplotlib import pyplot as plt
from linkage_utils import calc_limiting_angles, forward_kinematics, plot_linkage
from jumping_utils import get_apex_height

MIN_LINK_LENGTH = 0.025
MAX_LINK_LENGTH = 0.3
MIN_FRAME_ROTATION = np.pi / 2
MAX_FRAME_ROTATION = np.pi
MAX_EEF_HEIGHT = 0.0

"""
Perform rejection sampling to generate a random set of link lengths.
"""
def generate_random_link_lengths():
    link_lengths = np.zeros(4)
    while True:
        link_lengths = np.random.uniform(MIN_LINK_LENGTH, MAX_LINK_LENGTH, 4)
        L_max = np.max(link_lengths)
        L_min = np.min(link_lengths)

        if link_lengths[2] == L_min and L_max + L_min < np.sum(link_lengths) - (L_max + L_min) and link_lengths[1] + link_lengths[2] > link_lengths[3] and link_lengths[3] + link_lengths[2] > link_lengths[1] and link_lengths[1] > link_lengths[0] and link_lengths[3] > link_lengths[0]:
            break
    return link_lengths

"""
Generate a random frame rotation.
"""
def generate_random_frame_rotation():
    return np.random.uniform(MIN_FRAME_ROTATION, MAX_FRAME_ROTATION)

"""
Generate a random coupler end effector offset such that its norm plus the length of the coupler link is less than the maximum link length.
"""
def generate_random_coupler_eef_offset(coupler_length):
    while True:
        coupler_eef_offset = np.random.uniform([-MAX_LINK_LENGTH, -MAX_LINK_LENGTH], [0, MAX_LINK_LENGTH], 2)
        if np.linalg.norm(coupler_eef_offset - np.array([coupler_length, 0])) < MAX_LINK_LENGTH:
            break
    return coupler_eef_offset

"""
Sample a random set of parameters and calculate the apex jumping height of the linkage.
"""
def sample_random_parameters():
    # Generate a random set of link lengths.
    link_lengths = generate_random_link_lengths()

    # Calculate the limiting angles for the four-bar linkage.
    theta_min, theta_max = calc_limiting_angles(link_lengths)
    theta_tolerance = 0.01
    theta_min += theta_tolerance
    theta_max -= theta_tolerance

    coupler_eef_offset = generate_random_coupler_eef_offset(link_lengths[2])
    # frame_rotation = generate_random_frame_rotation()

    # Choose the frame rotation such that the average horizontal deviation from vertical of the end effector positions is minimized.
    frame_rotation = 0
    num_points = 100
    theta = np.linspace(theta_min, theta_max, num_points)
    
    x_eefs = np.zeros((2, num_points))
    for i in range(num_points):
        fk_results = forward_kinematics(link_lengths, coupler_eef_offset, theta[i], frame_rotation)
        x_eefs[:, i] = fk_results[0]
    avg_eef_pos = np.mean(x_eefs, axis=1)
    avg_eef_deviation = x_eefs - avg_eef_pos.reshape(2, 1)
    avg_eef_rotation = np.arctan(avg_eef_deviation[1, :] / avg_eef_deviation[0, :])
    frame_rotation = np.pi/2 - np.mean(avg_eef_rotation)

    # Limit theta_min to ensure that the end effector z-position is no greater than MAX_EEF_HEIGHT.
    for i in range(num_points):
        fk_results = forward_kinematics(link_lengths, coupler_eef_offset, theta[i], frame_rotation)
        x_eefs[:, i] = fk_results[0]
    # Find the first theta value for which the end effector z-position is greater than MAX_EEF_HEIGHT.
    for i in range(num_points):
        if x_eefs[1, i] < MAX_EEF_HEIGHT:
            theta_min = theta[i]
            x_eefs = x_eefs[:, i:]
            break

    # avg_eef_pos = np.mean(x_eefs, axis=1)
    # avg_eef_deviation = x_eefs - avg_eef_pos.reshape(2, 1)
    # avg_eef_rotation = np.arctan(avg_eef_deviation[1, :] / avg_eef_deviation[0, :])
    # frame_rotation = np.pi/2 - np.mean(avg_eef_rotation) - frame_rotation

    frame_rotation = 0
    num_points = 100
    theta = np.linspace(theta_min, theta_max, num_points)
    
    x_eefs = np.zeros((2, num_points))
    for i in range(num_points):
        fk_results = forward_kinematics(link_lengths, coupler_eef_offset, theta[i], frame_rotation)
        x_eefs[:, i] = fk_results[0]
    avg_eef_pos = np.mean(x_eefs, axis=1)
    avg_eef_deviation = x_eefs - avg_eef_pos.reshape(2, 1)
    avg_eef_rotation = np.arctan(avg_eef_deviation[1, :] / avg_eef_deviation[0, :])
    frame_rotation = np.pi/2 - np.mean(avg_eef_rotation)


    parameters = [link_lengths, coupler_eef_offset, frame_rotation, theta_min, theta_max]

    # Calculate the apex jumping height of the linkage.
    apex_height = get_apex_height(forward_kinematics, link_lengths, coupler_eef_offset, frame_rotation, theta_min, theta_max)
    print("sampled parameters: {}".format(parameters))
    return apex_height, parameters

if __name__ == "__main__":
    # Sample N random sets of parameters.
    N = 100
    apex_heights = np.zeros(N)
    parameters = []
    for i in range(N):
        apex_heights[i], parameters_i = sample_random_parameters()
        parameters.append(parameters_i)
    
    # Find the set of parameters that maximizes the apex jumping height.
    max_index = np.argmax(apex_heights)
    link_lengths = parameters[max_index][0]
    coupler_eef_offset = parameters[max_index][1]
    frame_rotation = parameters[max_index][2]
    theta_min = parameters[max_index][3]
    theta_max = parameters[max_index][4]

    # Print the optimal parameters.
    print("Optimal parameters:")
    print("Link lengths: {}".format(link_lengths))
    print("Coupler end effector offset: {}".format(coupler_eef_offset))
    print("Frame rotation: {}".format(frame_rotation))
    print("Limiting angles: [{}, {}]".format(theta_min, theta_max))

    # Print the optimal apex jumping height.
    print("Optimal apex jumping height: {}".format(apex_heights[max_index]))

    # Plot the linkage over its range of motion.
    num_points = 10
    theta = np.linspace(theta_min, theta_max, num_points)
    x_eefs = np.zeros((2, num_points))
    for i in range(num_points):
        fk_results = forward_kinematics(link_lengths, coupler_eef_offset, theta[i], frame_rotation)
        x_eefs[:, i] = fk_results[0]
        plot_linkage(fk_results)
    
    # Show the plot.
    plt.show()