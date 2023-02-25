from scipy.linalg import solve_continuous_are
import numpy as np
import yaml

config = yaml.safe_load(open("config.yaml"))

l_min = config["sitting_leg_length_m"]
l_max = config["standing_leg_length_m"]
g = 9.81
m_c = 1.0
m_p = 3.0
Q = np.diag([10, 1, 2, 0.1])
R = np.array([[1]])

def calculate_A_B(l):
    H = np.array([[m_c + m_p, m_p * l],
                    [m_p * l, m_p * l ** 2]])
    B_ = np.array([1, 0])
    H_inv = np.linalg.inv(H)
    dGdq = np.array([[0, 0], [0, m_p * g * l]])
    A = np.block([[np.zeros((2, 2)), np.eye(2)],
                    [-H_inv @ dGdq, np.zeros((2, 2))]])
    B = np.block([np.zeros(2), H_inv @ B_])[:, np.newaxis]
    return A, B

num_steps = 10
for i in range(num_steps):
    l = i / num_steps * (l_max - l_min) + l_min
    A, B = calculate_A_B(l)

    P = solve_continuous_are(A, B, Q, R)

    K = -np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
    print(f"l = {l:.5f}, K = {K}")