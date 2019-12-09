#!/usr/bin/env python2

# Calculate and visualize timing accuracy of the VersaVIS 2.0. Used to calibrate
# the time offset between end of integration time and time stamping by driver.
#
# Author: Andreas Pfrunder, Florian Tschopp
#
# Version: 2.0 Switched to more informative plots and added calculation
#          1.0 Visualization script from Andreas

import argparse

import rosbag
from std_msgs.msg import Time
from sensor_msgs.msg import Image
from tqdm import tqdm

from decimal import *

import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(
        description='Plot all timestamps of the bag topics.')
    parser.add_argument('--input_rosbag', default= '',
        help='Path to the rosbag file from which data is read.')
    parser.add_argument('--cam0', default= '',
        help='ROS topic name of cam0.')
    parser.add_argument('--cam1', default= '',
        help='ROS topic name of cam1.')
    parsed_args = parser.parse_args()

    input_path = parsed_args.input_rosbag

    input_bag = rosbag.Bag(input_path, 'r')
    
    #################
    # Parse ROS bag #
    #################
    img0_arrival_time_nsec = [t.nsecs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam0])]
    img0_arrival_time_sec = [t.secs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam0])]
    img0_timestamp_nsec = [msg.header.stamp.nsecs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam0])]
    img0_timestamp_sec = [msg.header.stamp.secs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam0])]
    img1_arrival_time_nsec = [t.nsecs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam1])]
    img1_arrival_time_sec = [t.secs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam1])]
    img1_timestamp_nsec = [msg.header.stamp.nsecs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam1])]
    img1_timestamp_sec = [msg.header.stamp.secs for (topic, msg, t) in input_bag.read_messages(topics=[parsed_args.cam1])]
 
    ard_img0_arrival_time_nsec = [t.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam0'])]
    ard_img0_arrival_time_sec = [t.secs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam0'])]
    ard_img0_timestamp_nsec = [msg.time.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam0'])]
    ard_img0_timestamp_sec = [msg.time.secs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam0'])]
    ard_img1_arrival_time_nsec = [t.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam1'])]
    ard_img1_arrival_time_sec = [t.secs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam1'])]
    ard_img1_timestamp_nsec = [msg.time.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam1'])]
    ard_img1_timestamp_sec = [msg.time.secs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/img_time_cam1'])]
    
    imu_arrival_time_nsec = [t.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/imu_micro'])]
    imu_arrival_time_sec = [t.secs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/imu_micro'])]
    imu_timestamp_nsec = [msg.time.data.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/imu_micro'])]
    imu_timestamp_sec = [msg.time.data.secs for (topic, msg, t) in input_bag.read_messages(topics=['/flex_vi/imu_micro'])]
    
    getcontext().prec = 5

    index_cam = min(len(img0_arrival_time_sec), len(img0_timestamp_sec), len(img1_arrival_time_sec), len(img1_timestamp_sec), len(img1_arrival_time_sec), len(ard_img0_timestamp_sec), len(ard_img1_timestamp_sec))
    index_imu = min(len(imu_arrival_time_sec), len(imu_timestamp_sec))
    
    img0_arrival_time = [0] * (index_cam -1)
    img0_timestamp = [0] * (index_cam -1)
    img1_arrival_time = [0] * (index_cam -1)
    img1_timestamp = [0] * (index_cam -1)
   
    ard_img0_arrival_time = [0] * (index_cam -1)
    ard_img0_timestamp = [0] * (index_cam -1)
    ard_img1_arrival_time = [0] * (index_cam -1)
    ard_img1_timestamp = [0] * (index_cam -1)

    imu_arrival_time = [0] * (index_imu -1)
    imu_timestamp = [0] * (index_imu -1)
    
    #######################
    # Compile time stamps #
    #######################
    it_img0 = 0
    if ard_img0_timestamp_sec[1] + ard_img0_timestamp_nsec[1] / 1e9 > img0_timestamp_sec[1] + img0_timestamp_nsec[1] / 1e9:
      it_img0 = 1
    it_img1 = 0
    if ard_img1_timestamp_sec[1] + ard_img1_timestamp_nsec[1] / 1e9 > img1_timestamp_sec[1] + img1_timestamp_nsec[1] / 1e9:
      it_img1 = 1
    
    for x in range (1,(index_cam -2)):
        img0_arrival_time[x] = img0_arrival_time_sec[x + it_img0] + img0_arrival_time_nsec[x + it_img0] / 1e9
        img0_timestamp[x] = img0_timestamp_sec[x + it_img0] + img0_timestamp_nsec[x + it_img0] / 1e9
        img1_arrival_time[x] = img1_arrival_time_sec[x + it_img1] + img1_arrival_time_nsec[x + it_img1] / 1e9
        img1_timestamp[x] = img1_timestamp_sec[x + it_img1] + img1_timestamp_nsec[x + it_img1] / 1e9
        ard_img0_arrival_time[x] = ard_img0_arrival_time_sec[x] + ard_img0_arrival_time_nsec[x] / 1e9
        ard_img0_timestamp[x] = ard_img0_timestamp_sec[x] + ard_img0_timestamp_nsec[x] / 1e9
        ard_img1_arrival_time[x] = ard_img1_arrival_time_sec[x] + ard_img1_arrival_time_nsec[x] / 1e9
        ard_img1_timestamp[x] = ard_img1_timestamp_sec[x] + ard_img1_timestamp_nsec[x] / 1e9
    
    for x in range(1,(index_imu -2)):
        imu_arrival_time[x] = imu_arrival_time_sec[x] + imu_arrival_time_nsec[x] / 1e9
        imu_timestamp[x] = imu_timestamp_sec[x] + imu_timestamp_nsec[x] / 1e9

    ###################
    # Calculate delay #
    ###################
    # This delay is the time between the end of integration time and the time
    # stamp given from the ROS driver. Often, this corresponds to the highest
    # framerate of the sensor (may also be affected by the bandwidth of the 
    # interface.
    integration_time_ms = 1
    time_dela_img0_ms = 1000 * np.mean(np.subtract(img0_timestamp, ard_img0_timestamp)) - integration_time_ms / 2
    time_dela_img1_ms = 1000 * np.mean(np.subtract(img1_timestamp, ard_img1_timestamp)) - integration_time_ms / 2
    time_delta = (time_dela_img0_ms + time_dela_img1_ms) / 2 # Weight both cameras equally.
    print("Time offset between end of integration and time stamping: " + str(time_delta) + " ms")

    ########
    # Plot #
    ########
    plt.figure(1)
    plt.plot(np.diff(img0_arrival_time[2:-1]), 'r.')
    plt.plot(np.diff(img0_timestamp[2:-1]), 'b.')
    plt.plot(np.diff(ard_img0_arrival_time[2:-1]), 'g.')
    plt.plot(np.diff(ard_img0_timestamp[2:-1]), 'm.')
    plt.xlabel('ROS message sequence sumber')
    plt.ylabel('Time difference [s]')
    plt.title('Time difference between consecutive messages.')    
    plt.legend(['img0_arrival_time','img0_timestamp','ard_img0_arrival_time','ard_img0_timestamp'])
    
    plt.figure(2)
    plt.plot(np.diff(img1_arrival_time[2:-1]), 'r.')
    plt.plot(np.diff(img1_timestamp[2:-1]), 'b.')
    plt.plot(np.diff(ard_img1_arrival_time[2:-1]), 'g.')
    plt.plot(np.diff(ard_img1_timestamp[2:-1]), 'm.')
    plt.xlabel('ROS message sequence sumber')
    plt.ylabel('Time difference [s]')
    plt.title('Time difference between consecutive messages.')    
    plt.legend(['img1_arrival_time','img1_timestamp','ard_img1_arrival_time','ard_img1_timestamp'])
    
    plt.figure(3)
    plt.plot(np.subtract(img0_arrival_time, img0_timestamp), 'r.')
    plt.plot(np.subtract(img1_arrival_time, img1_timestamp), 'b.')
    plt.plot(np.subtract(ard_img0_arrival_time, ard_img0_timestamp), '.g')
    plt.plot(np.subtract(ard_img1_arrival_time, ard_img1_timestamp), '.m')
    plt.xlabel('ROS message sequence number')
    plt.ylabel('Time difference [s]')
    plt.title('Time delay between timestamp and arrival time')
    plt.legend(['img_0','img1','ard_img0','ard_img1'])

    plt.figure(4)
    plt.plot(np.subtract(img0_timestamp, ard_img0_timestamp), '.r')
    plt.plot(np.subtract(img1_timestamp, ard_img1_timestamp), '.g')
    plt.xlabel('Message sequence number')
    plt.ylabel('Time difference [s]')
    plt.title('Delay between image stamp - arduino stamp.')
    plt.legend(['img0','img1'])

    plt.figure(5)
    plt.plot(np.diff(imu_arrival_time[2:-1]), 'r.')
    plt.plot(np.diff(imu_timestamp[2:-1]), 'm.')
    plt.xlabel('ROS message sequence sumber')
    plt.ylabel('Time difference [s]')
    plt.title('Time difference between consecutive messages (IMU).')    
    plt.legend(['imu_arrival_time','imu_timestamp'])

    plt.figure(6)
    plt.plot(np.subtract(imu_arrival_time, imu_timestamp), 'r.')
    plt.xlabel('ROS message sequence number')
    plt.ylabel('Time difference [s]')
    plt.title('Time delay between arrival time and timestamp (IMU)')
  
    plt.show()

if __name__ == '__main__':
    main()
