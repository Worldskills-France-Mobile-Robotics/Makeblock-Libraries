# ruediger2_anduino Library v0.2

Arduino Library for Ruediger2 
This aim to use the Makeblock Hardware to use it with ROS2

# Copyright notice

In makeblock's library, some of the modules are derived from other open source projects, and also part of some code is inspired by the algorithms of other individuals or organizations. We will retain the copyright of the original open source code.

These modules is derived from other open source projects:

- MeRGBLed
- MeHumitureSensor
- Me7SegmentDisplay
- MeOneWire
- MeStepper

and these modules is inspired by some projects:

- MeUSBHost

As an open source library, we respect all contributors to the open source community and thank you very much for everyone's supervision.

If you have a discussion about licensing issues, please contact (myan@makeblock.com -- Mark Yan)

### How to use:

## Use to modify it
1. Download the source from github
```
export ROBOT_ROOT=${HOME}/workspace/robot
mkdir -p ${ROBOT_ROOT} && cd ${ROBOT_ROOT}

git clone git@github.com:Worldskills-France-Mobile-Robotics/Makeblock-Libraries.git ruediger2_arduino

ln -s ${ROBOT_ROOT}/ruediger2_arduino ${HOME}/Arduino/libraries/ruediger2_arduino

```
2. In the Arduino IDE: Click "File-> Examples". There are some test programs in "ruediger2_anduino->"

## Use "as is" 

1. Download the source from the git code-> "Download ZIP" 

2. In the Arduino IDE: "Sketch-> Include Library-> Add .ZIP Library-> select the downloaded file-> Open"

3. Click "File-> Examples". There are some test programs in "ruediger2_anduino->"

### Learn more from Makeblock official website: www.makeblock.com
