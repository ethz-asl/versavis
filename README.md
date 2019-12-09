# VersaVIS
An Open Versatile Multi-Camera Visual-Inertial Sensor Suite


## Supported camera drivers
* [Basler](https://github.com/ethz-asl/ros_basler_camera/tree/devel/versavis) tested with acA1920-155uc
* [MatrixVision](https://github.com/ethz-asl/bluefox2/tree/devel/versavis) tested with Bluefox 2 MLC200WG, needs adaption for new format
* [PointGrey/Flir](https://github.com/ethz-asl/flir_camera_driver/tree/devel/versavis) tested with Chameleon 3, Blackfly S
* [CamBoard](https://github.com/ethz-asl/pico_flexx_driver/tree/devel/versavis) tested with CamBoard pico monstar

## Preliminary documentation (WIP)
[Paper](https://arxiv.org/abs/1912.02469) and [Presentation](https://docs.google.com/presentation/d/1SZYs3M01k65-Rg9b8MOcZT-UuRliV7OSue2i0WA65Tc/edit?usp=sharing)

## Install

### Clone and build

```
cd ~/catkin_ws/src/
git clone -b devel/open-source git@github.com:ethz-asl/versavis_private.git --recursive
catkin build versavis
cd firmware
./setup.sh
```

### Setup udev rule
Add yourself to `dialout` group
```
sudo adduser <username> dialout
```

Copy udev rule file to your system:
```
cp firmware/98-versa-vis.rules /etc/udev/rules.d/98-versa-vis.rules
```
Afterwards, use the following commands to reload the rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
```
Note: You might have to reboot your computer for this to take effect. You can check by see whether a `/dev/versavis` is available and pointing to the correct device.

### Configure
Adapt the [configuration file](https://github.com/ethz-asl/versavis_private/blob/devel/open-source/firmware/libraries/versavis/src/versavis_configuration.h) to your setup needs. Also check the [datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776) for how to configure the hardware switches.

### Flash firmware on the VersaVIS board
* Install the arduino IDE from [here](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous). Use version 1.8.2!
* Open `firmware/versavis/versavis.ino` in the IDE
* Go to `File -> Preferences`
* Change Sketchbook location to `versavis/firmware/`
* Install board support:
    - For AVI 2.1 (the green one): Tools -> Boards -> Boards Manager -> Arduino SAMD Boards (32-bits ARM Cortex-M0+) -> Use version 1.6.20!
    - For VersaVIS 1.0 (the black one): [Check here](https://github.com/ethz-asl/versavis_hw/)
* Set `Tools -> Port -> tty/ACM0 (Arduino Zero)`, and `Tools -> Board -> VersaVIS`.
* Compile using the *Verify* menu option
* Flash using the *Upload* menu option

## Calibration
Typically, a VI Setup needs to be carefully calibrated for camera intrinsics and camera-camera extrinsics and camera-imu extrinsics.
Refer to [Kalibr](https://github.com/ethz-asl/kalibr) for a good calibration framework. Note: To enable a good calibration, a high-quality calibration target needs to be available. Furthermore, a good and uniform light source is needed in order to reduce motion blur, especially during camera-imu calibration.

## Usage
* Adapt `versavis/launch/run_versavis.launch` to your needs.
* Run with
```
rosrun versavis run_versavis.launch
```
* Wait for successfull initialization.


## Troubleshooting
### My sensor is stuck at initialization
Main symptoms are:
* No IMU message published.
* Cameras are not triggered or only very slowly (e.g. 1 Hz).
*
Troubleshooting steps:
* Check that your camera receives a triggering signal by checking the trigger LED (Note: As the trigger pulse is very short, look for a dim flicker.).
* Check that all topics are correctly set up and connected.
* If the USB3 blackfly is powered over the Hirose plug, there seems to be a longer delay (0.2s+) until the image arrives on the host computer. Initialization does not succeed because it only allows successful synchronization if the image message is not older than 0.1s compared to the time message coming from the triggering board. With power over USB things seem to work fine. One can increase the threshold [kMaxImageDelayThreshold](https://github.com/ethz-asl/versavis_private/blob/c9f2ffad4f3ed9db04004d2098abf0c3c83e4923/versavis/src/versavis_synchronizer.cpp#L19) but keep in mind that `kMaxImageDelayThreshold >> 1/f_init`. Decrease initialization frequency [`f_init`](https://github.com/ethz-asl/versavis_private/blob/c9f2ffad4f3ed9db04004d2098abf0c3c83e4923/firmware/versavis/versavis.ino#L83) if necessairy.

### The board is not doing what I expect / How can I enter debug mode
Easiest way to debug is to enable `DEBUG` mode in `firmware/libraries/versavis/versavis_configuration.h` and check the debug output using the Arduino Serial Monitor or `screen /dev/versavis`.
Note: In debug mode, ROS/rosserial communication is deactivated!
### I don't get any IMU messages on `/versavis/imu`
This is normal during initialization as no IMU messages are published. Check [Inintialization issues](https://github.com/ethz-asl/versavis_private/blob/devel/open-source/README.md#my-sensor-is-stuck-at-initialization) for further info.
### After uploading a new firmware, I am unable to communicate with the VersaVIS board
This is most likely due to an infinite loop in the code in an error case. Reset the board by double clicking the reset button and upload your code in `DEBUG` mode. Then check your debug output.
### IMU shows strange data or spikes
To decrease the bandwidth between VersaVIS and host but keep all information, only the IMu raw data is transferred and later scaled. If there is a scale offset, adapt the [scale/sensitivity parameters](https://github.com/ethz-asl/versavis_private/blob/1b09b18efa273b71206aabb8ab74e4c3903b7d63/versavis/launch/run_versavis.launch#L141) in your launch file.

Depending on the IMU, recursive data grabbing is implemented with a CRC or temperature check. If this fails multiple time, the latest message is used. Check your IMU if this persists.
### It looks like my exposure time is not correctly compensated
Check whether your board can correctly detect your exposure time in the [debug output](https://github.com/ethz-asl/versavis_private/blob/devel/open-source/README.md#the-board-is-not-doing-what-i-expect--how-can-i-enter-debug-mode).
Troubleshooting steps:
* Enable exposure/strobe output on your camera.
* Check that all dip switches are set according to the [datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776).
* Check that the exposure LED on the board flashes with exposure time.
