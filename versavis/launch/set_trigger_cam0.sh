#!/bin/bash
rosservice call --wait /ACE_usb_0/pylon_camera_node/set_trigger_source "value: 1" &
exec "$@"