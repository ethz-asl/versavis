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
    parser.add_argument('--topic', default= '/VersaVIS/img_time',
        help='Topic to plot.')

    args = parser.parse_args()

    input_path = args.input_rosbag

    input_bag = rosbag.Bag(input_path, 'r')
    
    #################
    # Parse ROS bag #
    #################
    arduino_time_t_nsec = [t.nsecs for (topic, msg, t) in input_bag.read_messages(topics=[args.topic])]
    arduino_time_t_sec = [t.secs for (topic, msg, t) in input_bag.read_messages(topics=[args.topic])]
    arduino_time_nsec = [msg.time.data.nsecs for (topic, msg, t) in input_bag.read_messages(topics=[args.topic])]
    arduino_time_sec = [msg.time.data.secs for (topic, msg, t) in input_bag.read_messages(topics=[args.topic])]
    index = len(arduino_time_t_nsec)
    arduino_time_t = [0] * (index -1)
    arduino_time = [0] * (index -1)

    #######################
    # Compile time stamps #
    #######################
    for x in range (1,(index -2)):
        arduino_time_t[x] = arduino_time_t_sec[x] + arduino_time_t_nsec[x] / 1e9
        arduino_time[x] = arduino_time_sec[x] + arduino_time_nsec[x] / 1e9

    ########
    # Plot #
    ########
    plt.figure(1)
    plt.plot(np.diff(arduino_time_t[2:-1]), 'b.')
    plt.plot(np.diff(arduino_time[2:-1]), 'c.')
    plt.xlabel('ROS message sequence number')
    plt.ylabel('Time difference [s]')
    plt.title('Time difference between consecutive messages.')    
    plt.legend(['ard_t','ard'])

    plt.figure(2)
    plt.plot(np.subtract(arduino_time_t, arduino_time), '.b')
    plt.xlabel('ROS message sequence number')
    plt.ylabel('Time difference [s]')
    plt.title('Time delay between timestamp and arrival time')

    plt.figure(3)
    plt.plot(np.subtract(arduino_time_t, arduino_time_t[1]), '.r')
    plt.plot(np.subtract(arduino_time, arduino_time_t[1]), '.g')
    plt.xlabel('Message sequence number')
    plt.ylabel('Time [s]')
    plt.title('Time series.')
    plt.legend(['ard_t','ard'])

    plt.show()

if __name__ == '__main__':
    main()
