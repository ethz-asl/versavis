#!/bin/bash
node_name=/ACE_usb_0/pylon_camera_node &
echo $node_name &
rosservice call --wait $node_name/set_trigger_source "value: 1" &
rosservice call --wait $node_name/set_trigger_mode "data: true" &
exec "$@"