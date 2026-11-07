#!/usr/bin/env python3
"""Publish the joint angles of one scene of the cell and the fixture/parts as RViz markers.

The joint angles come from reach_check.py (numerical inverse kinematics with the
official xArm6 DH parameters, gripper pointing down, tool length 200 mm assumed).
Units: radians for joints, metres for markers.
"""
import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

# Joint angles [J1..J6] in rad for each robot pose (from reach_check.py).
# For the Pos 2 place poses J6 is turned by 90 deg, so the fingers close across
# the profile width (Pos 2 lies along Y); this does not move the TCP.
POSE = {
    "R1_home":        [0.0, -0.8298, -0.6570, 0.0, 1.4868, 0.0],
    "R1_pick":        [0.0, -0.0233, -0.6637, 0.0, 0.6869, 0.0],
    "R1_place_near":  [1.5708, -0.2213, -0.4524, 0.0, 0.6738, 1.5708],
    "R1_place_far":   [1.5708, 0.1566, -0.8897, 0.0, 0.7330, 1.5708],
    "R2_home":        [0.0, -0.8298, -0.6570, 0.0, 1.4868, 0.0],
    "R2_pick":        [0.0, -0.0110, -0.6599, 0.0, 0.6709, 0.0],
    "R2_wait":        [-0.4636, -0.4976, -0.7206, 0.0, 1.2182, -0.4636],
    "R2_place_left":  [-1.7197, -0.0440, -0.6928, 0.0, 0.7368, -0.1489],
    "R2_place_right": [-1.4219, -0.0440, -0.6928, 0.0, 0.7368, 0.1489],
}

# Scene = (pose of R1, pose of R2, parts already located in the fixture)
SCENE = {
    "home":           ("R1_home", "R2_home", []),
    "pick":           ("R1_pick", "R2_pick", []),
    "r1_place_near":  ("R1_place_near", "R2_wait", []),
    "r1_place_far":   ("R1_place_far", "R2_wait", ["P1a"]),
    "r2_place_left":  ("R1_home", "R2_place_left", ["P1a", "P1b"]),
    "r2_place_right": ("R1_home", "R2_place_right", ["P1a", "P1b", "P2a"]),
}

# Boxes: name -> (centre x, y, z), (size x, y, z), colour (r, g, b)
BOXES = {
    "plate":   ((0.0, 0.0, 0.010), (0.42, 0.35, 0.020), (0.80, 0.80, 0.80)),
    "tray_R1": ((0.40, -0.40, 0.010), (0.34, 0.15, 0.020), (1.00, 0.90, 0.55)),
    "tray_R2": ((0.40, 0.40, 0.010), (0.30, 0.15, 0.020), (1.00, 0.90, 0.55)),
    "P1a":     ((0.0, -0.06, 0.030), (0.32, 0.04, 0.020), (0.45, 0.60, 0.85)),
    "P1b":     ((0.0, 0.06, 0.030), (0.32, 0.04, 0.020), (0.45, 0.60, 0.85)),
    "P2a":     ((-0.06, 0.0, 0.0475), (0.03, 0.28, 0.015), (0.95, 0.65, 0.40)),
    "P2b":     ((0.06, 0.0, 0.0475), (0.03, 0.28, 0.015), (0.95, 0.65, 0.40)),
}


def box_marker(i, name):
    (x, y, z), (sx, sy, sz), (r, g, b) = BOXES[name]
    m = Marker()
    m.header.frame_id = "world"
    m.ns, m.id, m.type, m.action = "cell", i, Marker.CUBE, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
    return m


def main():
    rospy.init_node("cell_poses")
    scene = rospy.get_param("~scene", "r1_place_far")
    pose1, pose2, parts = SCENE[scene]
    rospy.loginfo("Scene %s: R1 %s, R2 %s", scene, pose1, pose2)

    js = JointState()
    js.name = ["R1_joint%d" % k for k in range(1, 7)] + ["R2_joint%d" % k for k in range(1, 7)]
    js.position = POSE[pose1] + POSE[pose2]

    markers = MarkerArray()
    for i, name in enumerate(["plate", "tray_R1", "tray_R2"] + parts):
        markers.markers.append(box_marker(i, name))

    pub_js = rospy.Publisher("/cell_joint_states", JointState, queue_size=1)
    pub_mk = rospy.Publisher("/cell_markers", MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        js.header.stamp = rospy.Time.now()
        pub_js.publish(js)
        pub_mk.publish(markers)
        rate.sleep()


if __name__ == "__main__":
    main()
