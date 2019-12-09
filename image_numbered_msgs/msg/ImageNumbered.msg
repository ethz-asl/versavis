# This is a image message including a sequence number for association with a timestamp.
#
Header header  # Had to add this for the TimeSynchronizer (see https://answers.ros.org/question/197811/time-synchronizer-message_filters-with-custom-message-type/)
sensor_msgs/Image image
uint64 number
float32 exposure
