"""
Command line tool to for IMU delay Test
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from statsmodels.graphics.tsaplots import plot_acf

import argparse
from argparse import RawTextHelpFormatter  # to preserve line breaks in description
# see https://stackoverflow.com/questions/3853722/how-to-insert-newlines-on-argparse-help-text

def binning_decay(timestamp_way_offset, decay_microsec):
    """
    binning of azimuth angles in 1, 2 or 3 lidar segments with decay time ca. 10 milliseconds
    """
    timestamp_azimuth_pairs = np.zeros([0,2])
    last_timestamp = 0
    for src_row in range(len(timestamp_way_offset)):
        timestamp = timestamp_way_offset[src_row, 0]
        azimuth = timestamp_way_offset[src_row, 1]
        if timestamp - last_timestamp > decay_microsec: # append new measurement
            timestamp_azimuth_pairs = np.vstack((timestamp_azimuth_pairs,np.array(([timestamp, azimuth]))))
            last_timestamp = timestamp
        else: # add azimuth to last measurement
            dst_row = len(timestamp_azimuth_pairs) - 1
            timestamp_azimuth_pairs[dst_row, 1] = timestamp_azimuth_pairs[dst_row, 1] + azimuth
    return timestamp_azimuth_pairs

def filter_azimuth(timestamp_way_offset, min_azimuth):
    """
    remove azimuth angles less than e.g. 16 degree (other segments or outliers)
    """
    timestamp_azimuth_pairs = np.zeros([0,2])
    for row in range(len(timestamp_way_offset)):
        timestamp = timestamp_way_offset[row, 0]
        azimuth = timestamp_way_offset[row, 1]
        if azimuth > min_azimuth:
            timestamp_azimuth_pairs = np.vstack((timestamp_azimuth_pairs,np.array(([timestamp, azimuth]))))
    return timestamp_azimuth_pairs

def calc_azimuth_gradient(timestamp_way_offset):
    """
    calculates the gradients of azimuth angles
    """
    timestamp_azi_gradient = np.zeros([0,2])
    for row in range(1, len(timestamp_way_offset)):
        time1 = timestamp_way_offset[row - 1, 0]
        azi1 = timestamp_way_offset[row - 1, 1]
        time2 = timestamp_way_offset[row, 0]
        azi2 = timestamp_way_offset[row, 1]
        timestamp = 0.5 * (time1 + time2)
        gradient = (azi2 - azi1) / (time2 - time1)
        timestamp_azi_gradient = np.vstack((timestamp_azi_gradient,np.array(([timestamp, gradient]))))
    return timestamp_azi_gradient

def filter_local_minima(timestamp_way_offset):
    """
    returns the array of (timestamp, value) of all local minima
    """
    timestamp_minima = np.zeros([0,2])
    for row in range(1, len(timestamp_way_offset) - 1):
        timestamp = timestamp_way_offset[row, 0]
        value = timestamp_way_offset[row, 1]
        if value < timestamp_way_offset[row - 1, 1] and value < timestamp_way_offset[row + 1, 1]:
            timestamp_minima = np.vstack((timestamp_minima,np.array(([timestamp, value]))))
    return timestamp_minima

def filter_local_maxima(timestamp_way_offset):
    """
    returns the array of (timestamp, value) of all local maxim
    """
    timestamp_minima = np.zeros([0,2])
    for row in range(1, len(timestamp_way_offset) - 1):
        timestamp = timestamp_way_offset[row, 0]
        value = timestamp_way_offset[row, 1]
        if value > timestamp_way_offset[row - 1, 1] and value > timestamp_way_offset[row + 1, 1]:
            timestamp_minima = np.vstack((timestamp_minima,np.array(([timestamp, value]))))
    return timestamp_minima

def get_closest_timestamp(timestamp_velocity, imu_timestamp):
    best_lidar_timestamp = timestamp_velocity[0, 0]
    best_dt = abs(timestamp_velocity[0, 0] - imu_timestamp)
    for row in range(1, len(timestamp_velocity)):
        lidar_timestamp = timestamp_velocity[row, 0]
        if abs(lidar_timestamp - imu_timestamp) < best_dt:
            best_lidar_timestamp = lidar_timestamp
            best_dt = abs(lidar_timestamp - imu_timestamp)
    return best_lidar_timestamp, best_dt

def calc_2nd_derivate_from_xy(xaxis, yaxis):
    """
    Idea taken from https://mathformeremortals.wordpress.com/2013/01/12/a-numerical-second-derivative-from-three-points/
    """
    num_pairs = len(xaxis)
    y_2nd_derivate = np.zeros(num_pairs)

    for idx in range(1,num_pairs-1):
        y_part = yaxis[idx-1:idx+2]
        x_part = xaxis[idx-1:idx+2]

        x1 = x_part[0]
        x2 = x_part[1]
        x3 = x_part[2]

        x_diff = np.array([2.0/((x2-x1)*(x3-x1)), -2.0/((x3 - x2)*(x2 - x1)), 2.0/((x3 - x2)*(x3 - x1))])
        y_2nd_derivate[idx] = np.dot(x_diff, y_part)

    return y_2nd_derivate


"""
main
"""

parser = argparse.ArgumentParser(
                    prog='IMU Delay Tester',
                    formatter_class=RawTextHelpFormatter,
                    description="Calculates delay between LIDAR and IMU\n"
                               )

csv_filename = '../test/20231024_multiscan_timestamp_azimuth_imuacceleration.csv'
parser.add_argument('--csv_filename', help='CSV data file with timestamp', default=csv_filename, type=str)
args = parser.parse_args()

csv_filename = args.csv_filename
# read csv file
with open(csv_filename) as file:
    lines = [line.rstrip() for line in file]

timestamp_way_offset = np.zeros([0,2])
timestamp_imu_z = np.zeros([0,2])
timestamp_start = -1.0
for line in lines:
    line = line.replace(',', '.')  # replace german "," with "."
    token_list = line.split(";")
    if len(token_list) == 3:  # valid data line holding <timestamp>, <way offset>, <imu Z-data>
        for idx, token in enumerate(token_list):
            if token == "":
                continue
            if idx == 0:
                timestamp = float(token)
                # convert timestamp to milliseconds from start
                if timestamp_start < 0:
                    timestamp_start = timestamp
                timestamp = 0.001 * (timestamp - timestamp_start)
            if idx == 1:
                way_offset = float(token)
                timestamp_way_offset = np.vstack((timestamp_way_offset,np.array(([timestamp, way_offset]))))
            if idx == 2:
                imu_value = float(token)
                timestamp_imu_z = np.vstack((timestamp_imu_z,np.array(([timestamp, imu_value]))))

# binning of azimuth angles in 1, 2 or 3 lidar segments with decay time ca. 10 milliseconds
# timestamp_way_offset = binning_decay(timestamp_way_offset, 10000)

# remove azimuth angles less than 16 degree (other lidar segments)
timestamp_way_offset = filter_azimuth(timestamp_way_offset, 16)

# X axis parameter:
xaxis_azi_org = np.array(timestamp_way_offset[:, 0])
# Y axis parameter:
yaxis_azi_org = np.array(timestamp_way_offset[:, 1])

xaxis_imu_org = np.array(timestamp_imu_z[:,0])
yaxis_imu_org = np.array(timestamp_imu_z[:,1])

timestamp_azi_gradient = calc_azimuth_gradient(timestamp_way_offset)
xaxis_azi_grad = np.array(timestamp_azi_gradient[:,0])
yaxis_azi_grad = -np.array(timestamp_azi_gradient[:,1])

compute_derivates = False
if compute_derivates:

    # Min imu acceleration at point of free fall => max azimuth gradient, max velocity
    timestamp_min_imu_accel = filter_local_minima(timestamp_imu_z)
    timestamp_max_velocity = filter_local_maxima(timestamp_azi_gradient)

    # Sort timestamp_min_imu_accel by ascending values and get 9 minima for the 9 periods
    timestamp_min_imu_accel_sorted = timestamp_min_imu_accel[timestamp_min_imu_accel[:, 1].argsort()] # sort by acceleration
    timestamp_min_imu_accel = timestamp_min_imu_accel_sorted[0:8,:] # 9 minima for 9 periods
    timestamp_min_imu_accel = timestamp_min_imu_accel[timestamp_min_imu_accel[:, 0].argsort()] # resort by timestamp
    xaxis_min_imu_accel = np.array(timestamp_min_imu_accel[:,0])
    yaxis_min_imu_accel = np.array(timestamp_min_imu_accel[:,1])
    xaxis_max_velocity = np.array(timestamp_max_velocity[:,0])
    yaxis_max_velocity = np.array(timestamp_max_velocity[:,1])

    # Find max velocity closest to min acceleration
    min_imu_latency_sec = 1.0e6
    for row in range(len(timestamp_min_imu_accel)):
        imu_timestamp = timestamp_min_imu_accel[row, 0]
        lidar_timestamp, dt = get_closest_timestamp(timestamp_max_velocity, imu_timestamp)
        imu_latency_sec = dt * 1.0e-6 # delta timestamp in micro seconds to latency in second
        min_imu_latency_sec = min(min_imu_latency_sec, imu_latency_sec)
        print(f"imu latency: {imu_latency_sec:.6} sec")

    print(f"\nimu_delay_tester ({csv_filename}): min imu latency = {min_imu_latency_sec:.6} sec")

    # Compute lidar acceleration, i.e. 2.nd derivate
    y_2nd_derivate = calc_2nd_derivate_from_xy(xaxis_azi_org, yaxis_azi_org)

show_plot = True
if show_plot:

    fig, (ax1, ax2, ax3) = plt.subplots(3)
    # fig, (ax1, ax2) = plt.subplots(2)

    ax1.set_title('azimuth over time')
    ax1.scatter(xaxis_azi_org, yaxis_azi_org)
    ax1.plot(xaxis_azi_org, yaxis_azi_org)

    ax2.set_title('imu acceleration over time')
    ax2.scatter(xaxis_imu_org, yaxis_imu_org)
    ax2.plot(xaxis_imu_org, yaxis_imu_org)

    ax3.set_title('azimuth gradient over time')
    ax3.scatter(xaxis_azi_grad, yaxis_azi_grad)
    ax3.plot(xaxis_azi_grad, yaxis_azi_grad)

    fig.set_figheight(10) # height = 1000 px
    fig.set_figwidth(20)  # width = 2000 px
    fig.show()
    plt.savefig(fname=csv_filename[:-4]+'.png')

    # fig, (ax1, ax2) = plt.subplots(2)
    # ax1.set_title('filtered maxima velocity over time')
    # ax1.scatter(xaxis_max_velocity, yaxis_max_velocity)
    # ax1.plot(xaxis_max_velocity, yaxis_max_velocity)
    # ax2.set_title('filtered minima imu acceleration over time')
    # ax2.scatter(xaxis_min_imu_accel, yaxis_min_imu_accel)
    # ax2.plot(xaxis_min_imu_accel, yaxis_min_imu_accel)
    # fig.set_figheight(10) # height = 1000 px
    # fig.set_figwidth(20)  # width = 2000 px
    # fig.show()

    # show_acceleration = True
    # f = plt.figure(1)
    # plt.scatter(xaxis_azi_org, y_2nd_derivate)
    # plt.plot(xaxis_azi_org, y_2nd_derivate)
    # f.show()
    # g = plt.figure(2)
    # plt.scatter(xaxis_azi_org, yaxis_azi_org)
    # plt.plot(xaxis_azi_org, yaxis_azi_org)
    # g.show()
    # h = plt.figure(3)
    # plt.scatter(xaxis_imu_org, yaxis_imu_org)
    # plt.plot(xaxis_imu_org, yaxis_imu_org)
    # h.show()

    plt.show()
    plt.close()

pass

