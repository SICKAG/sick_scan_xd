#
# Transform example
#
# This python script demonstrates how a transform by a 6D pose (x,y,z,roll,pitch,yaw) is computed.
# See coordinate_transforms.md for more details.
#

import math
import numpy as np

# Converts roll (rotation about X-axis), pitch (rotation about Y-axis), yaw (rotation about Z-axis)
# to a 3x3 rotation matrix. Note: This function uses the "ZYX" notation, i.e. it compute the rotation
# by rotation = rot_z * rot_y * rot_x
# Input angles roll, pitch, yaw in radians.
def eulerZYXToRot3x3(roll, pitch, yaw):
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

# Converts roll (rotation about X-axis), pitch (rotation about Y-axis), yaw (rotation about Z-axis)
# to quaternion (x, y, z, w). Note: This function uses the "ZYX" notation, i.e. it compute the rotation
# by rotation = rot_z * rot_y * rot_x
# Input angles roll, pitch, yaw in radians.
def eulerZYXToQuaternion(roll, pitch, yaw):
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
rot3x3 = eulerZYXToRot3x3(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
quat = eulerZYXToQuaternion(np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw))
print("roll = {}, pitch = {}, yaw = {}:\nrot3x3 = {}\nquaternion = {}".format(roll, pitch, yaw, rot3x3, quat))
# Example output:
# roll = 5, pitch = -10, yaw = 15:
# rot3x3 = [[ 0.95125124 -0.2724529  -0.14453543]
#  [ 0.254887    0.95833311 -0.12895841]
#  [ 0.17364818  0.08583165  0.98106026]]
# quaternion = (0.05444693224342944, -0.08065606284759969, 0.1336748975829986, 0.9862358505202384)
