# This is a super-minimized message to hold data from a range sensor, such as Garmin LidarLite.
#
# Ranges and signal strengths are raw data and have to be scaled!
#
# To minimize the amount of data transfered by this message (e.g. form an Arduino through rosserial)
# the amount of data is truncated to its absolute minimum.

std_msgs/Time time

int16 range
uint8 signal_strength
