"""
reach_check.py - Reachability check for the two-xArm6 pick-and-place cell
(ETH Zurich technical assignment, drawing AMZ-SA-26-001).

Purpose
    Check that every pick and place point of the chosen layout can be reached
    with the gripper pointing straight down, and how stretched the arm is there
    (a nearly straight arm is close to the elbow singularity).

Data sources (official UFACTORY documentation)
    - Standard DH parameters of the xArm 6:
      https://docs.ufactory.cc/support_articles/developer/kinematic-and-dynamic-parameters/xarm-series/xarm-6
    - Joint ranges of the xArm 6 (Hardware Manual V2.6.0, model 1305, p. "Joint Range"):
      https://www.ufactory.cc/wp-content/uploads/2025/05/xArm_Hardware_Manual1305_V2.6.0-1.pdf

Model check
    The DH model was cross-checked against the official ROS kinematics file
    (github.com/xArm-Developer/xarm_ros2, xarm_description/config/kinematics/default/
    xarm6_default_kinematics.yaml): zero pose flange at (207, 0, 112) mm in both,
    max difference 0.003 mm over 1000 random poses (rounding in the file).

Limits of this check
    Geometry only: no collision check (arms, fixture), no speeds or torques.

Assumptions (ours, to be confirmed on the real cell)
    - Tool length (flange to TCP) = 200 mm: xArm Gripper + custom fingers.
    - Robots and fixture stand on the same table; fixture plate 20 mm thick.
    - Layout (a): fixture centre at (0, 0); R1 at (0, -400); R2 at (0, +400).

Run:  python reach_check.py     (needs numpy and scipy)
"""

import numpy as np                                   # vectors and matrices
from scipy.optimize import least_squares             # numerical solver used for inverse kinematics

# ---------------------------------------------------------------- robot model
A2 = np.hypot(284.5, 53.5)                           # link length a2 = sqrt(284.5^2 + 53.5^2) mm (UFACTORY page)
T2_OFF = -np.arctan(284.5 / 53.5)                    # joint-2 offset from mathematical to mechanical zero (rad)
T3_OFF = -T2_OFF                                     # joint-3 offset is the opposite of joint-2 offset

# Standard DH table, one row per joint: (d [mm], alpha [rad], a [mm], theta offset [rad])
DH = [(267.0, -np.pi / 2, 0.0, 0.0),
      (0.0,    0.0,       A2,   T2_OFF),
      (0.0,   -np.pi / 2, 77.5, T3_OFF),
      (342.5,  np.pi / 2, 0.0,  0.0),
      (0.0,   -np.pi / 2, 76.0, 0.0),
      (97.0,   0.0,       0.0,  0.0)]

# Joint ranges in degrees (Hardware Manual V2.6.0), converted to radians
LIMITS = np.radians([[-360, 360], [-117, 116], [-219, 10],
                     [-360, 360], [-97, 180], [-360, 360]])

ARM_MAX = A2 + np.hypot(342.5, 77.5)                 # shoulder-to-wrist distance with a straight elbow (mm)
TOOL = 200.0                                         # assumed tool length, flange to TCP (mm)
R_DOWN = np.diag([1.0, -1.0, -1.0])                  # orientation "gripper pointing straight down"


def dh_matrix(theta, d, alpha, a):
    """Homogeneous 4x4 transform of one joint (standard DH convention)."""
    ct, st = np.cos(theta), np.sin(theta)            # cosine and sine of the joint angle
    ca, sa = np.cos(alpha), np.sin(alpha)            # cosine and sine of the link twist
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,      sa,       ca,      d],
                     [0.0,     0.0,      0.0,    1.0]])


def forward(q):
    """Forward kinematics: joint angles -> list of the 6 joint frames (base coordinates)."""
    T = np.eye(4)                                    # start at the robot base
    frames = []                                      # will hold the frame after each joint
    for qi, (d, alpha, a, off) in zip(q, DH):        # walk along the chain, joint by joint
        T = T @ dh_matrix(qi + off, d, alpha, a)     # add the transform of this joint
        frames.append(T.copy())                      # store a copy of the current frame
    return frames


def tcp_pose(q):
    """Position and orientation of the tool centre point for joint angles q."""
    flange = forward(q)[-1]                          # frame of the tool flange (last joint)
    pos = flange[:3, 3] + flange[:3, 2] * TOOL       # move TOOL mm along the flange z-axis
    return pos, flange[:3, :3]                       # TCP position and rotation matrix


def solve_ik(target):
    """Inverse kinematics for a target point with the gripper pointing down.
    Returns joint angles, or None if the point cannot be reached within the joint ranges."""
    def error(q):                                    # what the solver tries to make zero
        pos, rot = tcp_pose(q)
        return np.r_[pos - target,                   # position error (mm)
                     100.0 * (rot - R_DOWN).ravel()] # orientation error, weighted to be comparable
    rng = np.random.default_rng(0)                   # fixed seed: same result at every run
    for _ in range(20):                              # several random starting guesses
        q0 = rng.uniform(LIMITS[:, 0], LIMITS[:, 1]) * 0.6
        q0[0] = np.arctan2(target[1], target[0])     # point joint 1 towards the target
        sol = least_squares(error, q0, bounds=(LIMITS[:, 0], LIMITS[:, 1]))
        if np.linalg.norm(error(sol.x)) < 0.05:      # accept only a near-exact solution
            return sol.x
    return None                                      # no solution found


def check(name, robot_xy, point_xyz):
    """Print distance, reachability and arm extension for one point of the cell."""
    local = np.array(point_xyz, float) - np.r_[robot_xy, 0.0]   # point in robot coordinates
    dist = np.hypot(local[0], local[1])              # horizontal distance from the robot axis
    q = solve_ik(local)                              # try to reach it
    if q is None:
        print(f"{name:30s} r = {dist:5.0f} mm   NOT reachable")
        return
    frames = forward(q)
    shoulder, wrist = frames[0][:3, 3], frames[3][:3, 3]         # shoulder and wrist positions
    ext = np.linalg.norm(wrist - shoulder) / ARM_MAX * 100       # how stretched the arm is (%)
    flag = "ok" if 30 <= round(ext) <= 75 else "CHECK"   # our design band: 30-75 % extension
    print(f"{name:30s} r = {dist:5.0f} mm   extension = {ext:3.0f} %   {flag}")


if __name__ == "__main__":
    # Sanity check of the model: TCP position with all joints at zero and no tool
    TOOL, saved = 0.0, TOOL                          # temporarily remove the tool
    print("Zero pose TCP (no tool):", np.round(tcp_pose(np.zeros(6))[0], 1), "mm")
    TOOL = saved                                     # put the tool back

    # Radial scan: how stretched is the arm at increasing distance (TCP 60 mm above the base plane)
    print("\nRadial scan, gripper down, TCP height 60 mm:")
    for r in range(150, 701, 50):
        check(f"  point at {r} mm", (0, 0), (r, 0, 60))

    # Layout (a): heights above the table = fixture plate 20 mm + profile height
    Z_POS1 = 20 + 20                                 # TCP on the top of Pos 1 (web top)
    Z_POS2 = 20 + 20 + 15                            # TCP on the top of Pos 2, lying on Pos 1
    R1, R2 = (0, -400), (0, 400)                     # robot base positions (mm)
    print("\nLayout (a), place points:")
    check("R1 place Pos 1 (near)", R1, (0, -60, Z_POS1))
    check("R1 place Pos 1 (far)",  R1, (0,  60, Z_POS1))
    check("R2 place Pos 2 (left)", R2, (-60, 0, Z_POS2))
    check("R2 place Pos 2 (right)", R2, (60, 0, Z_POS2))
    print("Layout (a), pick points in the trays (east side):")
    check("R1 pick from tray", R1, (400, -400, 40))
    check("R2 pick from tray", R2, (400,  400, 35))
