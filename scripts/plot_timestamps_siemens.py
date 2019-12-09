#!/usr/bin/env python2

# Simple script to analyze the timing issue related to the arduino vi setup.
# Author: Andreas Pfrunder

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
    parser.add_argument('input_rosbag', help='Path to the rosbag file from '
                                             'which data is read.')
    parsed_args = parser.parse_args()

    input_path = parsed_args.input_rosbag

    input_bag = rosbag.Bag(input_path, 'r')

    #img_time_nsec = [msg.header.stamp.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/chameleon3/image_raw'])]
    #img_time_sec = [msg.header.stamp.secs for (topic, msg, t) in input_bag.read_messages(topics=['/chameleon3/image_raw'])]
    img_time_nsec = [msg.header.stamp.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/pg_c3_usb_0/image_raw'])]
    img_time_sec = [msg.header.stamp.secs for (topic, msg, t) in input_bag.read_messages(topics=['/pg_c3_usb_0/image_raw'])]
    #arduino_time_nsec = [msg.data.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/versavis/img_time'])]
    #arduino_time_sec = [msg.data.secs for (topic, msg, t) in input_bag.read_messages(topics=['/versavis/img_time'])]
    arduino_time_nsec = [t.nsecs for (topic, msg, t) in input_bag.read_messages(topics=['/versavis/img_time'])]
    arduino_time_sec = [t.secs for (topic, msg, t) in input_bag.read_messages(topics=['/versavis/img_time'])]
    
    getcontext().prec = 5

    time = np.linspace(img_time_sec[0], img_time_sec[-1],len(img_time_nsec))

    if(len(img_time_sec) < len(arduino_time_sec)):
        index = len(img_time_sec)
    else:
        index = len(arduino_time_sec)


    img_time = [0] * (index -1)
    img_time_diff = [0] * (index -1)
    arduino_time = [0] * (index -1)
    arduino_time_diff = [0] * (index -1)    
    time_difference = [0] * (index -1)

    for x in range (1,(index -1)):
        img_time[x] = img_time_sec[x] + img_time_nsec[x] / 1e9 - (img_time_sec[0] + img_time_nsec[0] / 1e9)
        arduino_time[x] = arduino_time_sec[x] + arduino_time_nsec[x] / 1e9 - (arduino_time_sec[0] + arduino_time_nsec[0] / 1e9)
        time_difference[x] = img_time[x] - arduino_time[x]
        img_time_diff[x] = img_time[x] - img_time[x-1]
        arduino_time_diff[x] = arduino_time[x] - arduino_time[x-1]

    plt.figure(1)
    plt.plot(time_difference, 'k.')
    plt.xlabel('ROS message sequence sumber')
    plt.ylabel('Time difference in [s]')
    plt.title('Time difference between arduino and image time')    

    plt.figure(2)
    plt.plot(img_time, '.b')
    plt.xlabel('ROS message sequence number')
    plt.ylabel('Epoch time in [s]')
    plt.title('Image time')

    plt.figure(3)
    plt.plot(arduino_time, '.r')
    plt.xlabel('Message sequence number')
    plt.ylabel('Epoch time in [s]')
    plt.title('Arduino time')

    plt.figure(4)
    plt.plot(arduino_time_diff, '.r')
    plt.ylim(-0.1, 0.1)
    plt.xlabel('Message sequence number')
    plt.ylabel('Epoch time in [s]')
    plt.title('Arduino time diff')

    plt.figure(5)
    plt.plot(img_time_diff, '.r')
    plt.ylim(-0.1, 0.1)
    plt.xlabel('Message sequence number')
    plt.ylabel('Epoch time in [s]')
    plt.title('Image time diff')
    plt.show()

if __name__ == '__main__':
    main()
