# This is a super-minimized message to hold data from an IMU (Inertial Measurement Unit).
#
# Accelerations and rotational velocities are raw data and have to be scaled!
#
# To minimize the amount of data transfered by this message (e.g. form an Arduino through rosserial)
# the amount of data is truncated to its absolute minimum.

std_msgs/Time time

int16 ax
int16 ay
int16 az
int16 gx
int16 gy
int16 gz

