#
# Transform example
#

import math
import numpy as np

# Converts roll (X), pitch (Y), yaw (Z) to 3x3 rotation matrix
def eulerToRot3x3(roll, pitch, yaw):
    # roll: rotation about x axis
    rot_x = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(roll), -math.sin(roll)],
        [0.0, math.sin(roll), math.cos(roll)]], np.float64)
    # pitch: rotation about y axis
    rot_y = np.array([
        [math.cos(pitch), 0.0, math.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-math.sin(pitch), 0.0, math.cos(pitch)]], np.float64)
    # yaw: rotation about z axis
    rot_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0]], np.float64)
    # 3x3 rotation matrix
    rot_3x3 = rot_z @ rot_y @ rot_x
    return rot_3x3;

 # Converts roll (X), pitch (Y), yaw (Z) to quaternion (x, y, z, w)
def eulerToQuaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5);
    sy = math.sin(yaw * 0.5);
    cp = math.cos(pitch * 0.5);
    sp = math.sin(pitch * 0.5);
    cr = math.cos(roll * 0.5);
    sr = math.sin(roll * 0.5);
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
    w = cr * cp * cy + sr * sp * sy;
    return x, y, z, w

# Example: rotation by roll = 5, pitch = -10, yaw = 15 degree
roll, pitch, yaw = 5, -10, 15
rot3x3 = eulerToRot3x3(roll * math.pi / 180, pitch * math.pi / 180, yaw * math.pi / 180)
quat = eulerToQuaternion(roll * math.pi / 180, pitch * math.pi / 180, yaw * math.pi / 180)
print("roll = {}, pitch = {}, yaw = {}:\nrot3x3 = {}\nquaternion = {}".format(roll, pitch, yaw, rot3x3, quat))
