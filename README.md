# VersaVIS -- An Open Versatile Multi-Camera Visual-Inertial Sensor Suite
[![Build Test](https://github.com/ethz-asl/versavis/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/versavis/actions/workflows/build_test.yml)

<a href="https://youtu.be/bsvSZJq76Mc"> <img src="https://user-images.githubusercontent.com/17647950/133631034-f633df6b-7ce7-4851-8cd3-f67c0bbec67a.jpg" alt="https://youtu.be/bsvSZJq76Mc" width="600"> </a>

VersaVIS provides a complete, open-source hardware, firmware and software bundle to perform time synchronization of multiple cameras with an IMU featuring exposure compensation, host clock translation and independent and stereo camera triggering.

## News
* Enjoy our new [trailer video](https://youtu.be/bsvSZJq76Mc) explaining the main conecpt behind VersaVIS.
* VersaVIS runs on Ubuntu 20.04 focal fossa / ROS Noetic now, look at this [issue](https://github.com/ethz-asl/versavis/issues/21#issuecomment-853007617).

## Supported camera drivers
* [Basler (not open-source)](https://github.com/ethz-asl/ros_basler_camera/tree/devel/versavis) tested with acA1920-155uc
* [MatrixVision](https://github.com/ethz-asl/bluefox2/tree/devel/versavis) tested with Bluefox 2 MLC200WG, needs adaption for new format
* [PointGrey/Flir](https://github.com/ethz-asl/flir_camera_driver/tree/devel/versavis) tested with Chameleon 3, Blackfly S
* [CamBoard](https://github.com/ethz-asl/pico_flexx_driver/tree/devel/versavis) tested with CamBoard pico monstar

## Citing

Please cite the [following paper](https://www.mdpi.com/1424-8220/20/5/1439) when using VersaVIS for your research:

```bibtex
@article{Tschopp2020,
author = {Tschopp, Florian and Riner, Michael and Fehr, Marius and Bernreiter, Lukas and Furrer, Fadri and Novkovic, Tonci and Pfrunder, Andreas and Cadena, Cesar and Siegwart, Roland and Nieto, Juan},
doi = {10.3390/s20051439},
journal = {Sensors},
number = {5},
pages = {1439},
publisher = {Multidisciplinary Digital Publishing Institute},
title = {{VersaVIS—An Open Versatile Multi-Camera Visual-Inertial Sensor Suite}},
url = {https://www.mdpi.com/1424-8220/20/5/1439},
volume = {20},
year = {2020}
}
```

Additional information can be found [here](https://docs.google.com/presentation/d/1Yi71cYtIBGUP5bFDKDFcUF2MjNS_CV3LM7W1_3jNLEs/edit?usp=sharing).
## Install

### Clone and build

```
cd ~/catkin_ws/src/
git clone git@github.com:ethz-asl/versavis.git --recursive
catkin build versavis
cd versavis/firmware
./setup.sh
```

### Setup udev rule
Add yourself to `dialout` group
```
sudo adduser <username> dialout
```

Copy udev rule file to your system:
```
sudo cp firmware/98-versa-vis.rules /etc/udev/rules.d/98-versa-vis.rules
```
Afterwards, use the following commands to reload the rules
```
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo ldconfig
```
Note: You might have to reboot your computer for this to take effect. You can check by see whether a `/dev/versavis` is available and pointing to the correct device.

### Configure
Adapt the [configuration file](https://github.com/ethz-asl/versavis/blob/master/firmware/libraries/versavis/src/versavis_configuration.h) to your setup needs. Also check the [datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776) for how to configure the hardware switches.

### Flash firmware on the VersaVIS board
* Install the arduino IDE from [here](https://www.arduino.cc/en/Main/OldSoftwareReleases#previous). Use version 1.8.2!
    - Note that a small modification of the install script (`install.sh`) might be required. In particular you may need to change the line `RESOURCE_NAME=cc.arduino.arduinoide` to `RESOURCE_NAME=arduino-arduinoide` as per the issue [here](https://github.com/arduino/Arduino/issues/6116#issuecomment-290012812).
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
roslaunch versavis run_versavis.launch
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
* If the USB3 blackfly is powered over the Hirose plug, there seems to be a longer delay (0.2s+) until the image arrives on the host computer. Initialization does not succeed because it only allows successful synchronization if the image message is not older than 0.1s compared to the time message coming from the triggering board. With power over USB things seem to work fine. One can increase the threshold [kMaxImageDelayThreshold](https://github.com/ethz-asl/versavis/blob/af83f34d4471a7886a197f305dbe76603b92747a/versavis/src/versavis_synchronizer.cpp#L20) but keep in mind that `kMaxImageDelayThreshold >> 1/f_init`. Decrease initialization frequency [`f_init`](https://github.com/ethz-asl/versavis/blob/af83f34d4471a7886a197f305dbe76603b92747a/firmware/versavis/versavis.ino#L87) if necessairy.

### The board is not doing what I expect / How can I enter debug mode
Easiest way to debug is to enable `DEBUG` mode in `firmware/libraries/versavis/versavis_configuration.h` and check the debug output using the Arduino Serial Monitor or `screen /dev/versavis`.
Note: In debug mode, ROS/rosserial communication is deactivated!
### I don't get any IMU messages on `/versavis/imu`
This is normal during initialization as no IMU messages are published. Check [Inintialization issues](https://github.com/ethz-asl/versavis#my-sensor-is-stuck-at-initialization) for further info.
### After uploading a new firmware, I am unable to communicate with the VersaVIS board
This is most likely due to an infinite loop in the code in an error case. Reset the board by double clicking the reset button and upload your code in `DEBUG` mode. Then check your debug output.
### IMU shows strange data or spikes
To decrease the bandwidth between VersaVIS and host but keep all information, only the IMu raw data is transferred and later scaled. If there is a scale offset, adapt the [scale/sensitivity parameters](https://github.com/ethz-asl/versavis/blob/af83f34d4471a7886a197f305dbe76603b92747a/versavis/launch/run_versavis.launch#L140) in your launch file.

Depending on the IMU, recursive data grabbing is implemented with a CRC or temperature check. If this fails multiple time, the latest message is used. Check your IMU if this persists.
### It looks like my exposure time is not correctly compensated
Check whether your board can correctly detect your exposure time in the [debug output](https://github.com/ethz-asl/versavis#my-sensor-is-stuck-at-initialization).
Troubleshooting steps:
* Enable exposure/strobe output on your camera.
* Check that all dip switches are set according to the [datasheet](https://drive.google.com/file/d/11QCjc5PVuMU9bAr8Kjvqz2pqVIhoMbHA/view?ts=5dc98776).
* Check that the exposure LED on the board flashes with exposure time.
### I receive errors on the host computer
#### Time candidate overflow
```bash
[ WARN] [1619791310.834852014]: /versavis/camO/tmage_raw: Time candidates buffer overflow at 1025.
...
```
Means that the synchronizer receives more timestamps than images. Double check if the camera is actually triggering with every pulse it receives. A typical problem is when the exposure time is higher than the measurement period.

#### Image candidate overflow
```bash
[ WARN] [1619791310.834852014]: /versavis/camO/tmage_raw: Image candidates buffer overflow at 1025.
...
```
Means that the synchronizer receives more iamges than timestamps. Double check if the camera is actually triggering and not free running.
