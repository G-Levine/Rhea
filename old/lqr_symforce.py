import symforce
import symforce.symbolic as sf
from symforce.notebook_util import display
from symforce.values import Values

# symforce.set_symbolic_api("symengine")
# symforce.set_epsilon_to_symbol()


# Constants
m_c = sf.Symbol('m_c')  # mass of cart
m_p = sf.Symbol('m_p')  # mass of pendulum
l = sf.Symbol('l')  # length of pendulum
g = sf.Symbol('g')  # gravity


# State variables
theta = sf.Symbol('theta')  # angle of pendulum
theta_dot = sf.Symbol('theta_dot')  # angular velocity of pendulum
x = sf.Symbol('x')  # position of cart
x_dot = sf.Symbol('x_dot')  # velocity of cart

q = sf.Matrix([theta, x])
q_dot = sf.Matrix([theta_dot, x_dot])


# Weights
Q = sf.Matrix([[500, 0, 0, 0],
               [0, 100, 0, 0],
               [0, 0, 0, 0],
               [0, 0, 0, 0]])
R = sf.Matrix([[1]])


# Dynamics
H = sf.Matrix([[m_c + m_p, m_p * l * sf.cos(theta)],
              [m_p * l * sf.cos(theta), m_p * l ** 2]])
C = sf.Matrix([[0, -m_p * l * theta_dot * sf.sin(theta)], [0, 0]])
G = sf.Matrix([[0], [m_p * g * l * sf.sin(theta)]])
B_ = sf.Matrix([1, 0])
H_inv = H.inv()
dGdq = G.jacobian(q)

A = sf.Matrix.block_matrix(
    [[sf.Matrix.zeros(2, 2), sf.Matrix.eye(2)],
     [-H_inv * G.jacobian(q), -H_inv * C]])
B = sf.Matrix.block_matrix([[sf.Matrix.zeros(2, 1)],
                            [H_inv * B_]])


# LQR solution
def K_new(A: sf.M44, B: sf.M14, R: sf.M11, P: sf.M44) -> sf.M14:
    """Perform one iteration of the LQR algorithm to compute the optimal gain matrix K"""
    K_new = -(R + B.T * P * B).inv() * B.T * P * A
    return K_new

def P_new(A: sf.M44, B: sf.M14, Q: sf.M44, R: sf.M11, P: sf.M44, K: sf.M14, K_new: sf.M14) -> sf.M44:
    """Perform one iteration of the LQR algorithm to compute the optimal cost-to-go matrix P"""
    P_new = Q + K_new.T * R * K_new + (A + B * K).T * P * (A + B * K)
    return P_new


from scipy.linalg import solve_continuous_are
import numpy as np

l_min = 0.07814
l_max = 0.3182
g_ = 9.81
m_c_ = 1.0
m_p_ = 3.0
num_steps = 10
Q_ = np.array(Q)
R_ = np.array(R)

for i in range(num_steps):
    l_ = i / num_steps * (l_max - l_min) + l_min
    A_ = np.array(A.subs({l: l_, g: g_, m_c: m_c_, m_p: m_p_, theta: 0, theta_dot: 0}))
    B_ = np.array(B.subs({l: l_, g: g_, m_c: m_c_, m_p: m_p_, theta: 0, theta_dot: 0}))

    P = solve_continuous_are(A_, B_, Q_, R_)

    K = K_new(A_, B_, R, P)
    print(f"l = {l_:.5f}, K = {K}")