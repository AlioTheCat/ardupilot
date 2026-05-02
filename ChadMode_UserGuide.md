# CHAD MODE user guide for ArduSub
The Computer Held Aim Drive mode (CHAD) is used to maintain a ROV position using camera information.

It consists of two parts : a custom version of ardupilot that implements a new piloting mode (see https://www.github.com/AlioTheCat/ardupilot.git) and a program running on the Ground Control Station (GCS) called the CHAD Processing Unit (CPU) (see https://www.github.com/noeterrien/CHAD_Processing_Unit.git). 

The CPU is used to fetch images from the camera and compute the translation between two images. The measurements are then sent to ardupilot which computes forces to apply to the motors using a PID.

# CHAD Processing Unit
All informations concerning the CPU can be found in the README.md of the repo. Note that the CPU must be running if you want to arm a ROV while in CHAD mode. If connection between ardupilot and the CPU is lost, the ROV will automatically disarm after one second.

Currently, available http requests to interact with the CPU while running include : 
- http://localhost:8000/reset_reference_frame (POST, no parameters)
- http://localhost:8000/set (POST, parameters : parameter_name and parameter_value)
- http://localhost:8000/status/[param_name] (GET) : where param_name is one of the following, dx, dy, dz, kp_num (keypoints number), m_kp_num (matched keypoints number) 

# BlueOS

BlueOS is the operating system used by ROVs. Once connected to a ROV, an interface can be accessed by opening ```192.168.2.2``` in a navigator. There, you can : 

- upload your custom ardupilot firmware (see autopilot firmware)
- change parameters of ardupilot (see autopilot parameters)
- configure video stream endpoint, which you must make sure corresponds to the one indicated in the CPU ```config.yaml```
- configure video stream parameters. 800x600 is sufficient as the resolution and 15 fps is enough. This can solve issues of frames not being properly decoded with the H264 protocol.

If you are an advanced user, you can use the parameters interface to set the parameters for the CHAD mode, including its PID and the angle threshold above which the ROV should only try to adjust its attitude and not its position. 

*The PID parameters for Victor can be found in the file attached. However, do not expect it to work the first time you try it. You may need to change the parameters for it to work, depending on your context, and you will certainly need to do fine adjustments.*

*As the parameters that can be uploaded depend on your firmware, the firmware with which these parameters were saved is provided as well.*

# Cockpit

Cockpit is used on the GCS to pilote your ROV, it allows you to send information to ardupilot or receive information from it, send http requests to the CPU or display useful information. The version that was used during tests is a custom one available at : https://gitlab.com/istrate/cockpit-fork.

Two files are attached to help you configure cockpit : 
- a widget configuration file that includes the interface configuration (including display from the CPU)
- a node editor configuration file that includes input to reset the reference frame

Please make sure that the addresses used to send and retrieve information in cockpit correspond to the ones indicated in the CPU ```config.yaml``` file.

# Ardupilot

Ardupilot is the firmware that runs on BlueOS. It handles interactions with sensors, communications with the GCS and motors and servo controls. To use the CHAD mode, you will need a custom version of ardupilot.

## Clone the repo and configure ardupilot
First, clone the repo and configure ardupilot : 
```bash
git clone https://github.com/AlioTheCat/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

The close and reopen your terminal.

## Setup docker
The versions between the RasPi and ardupilot's compiler may differ wish may lead to some issues. To solve this, we're using a downgraded version of the compiler via docker :
In ardupilot's root folder, run this command to prepare the compiler environment.
```bash
docker build . -t ardupilot --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)
```

## Compile using docker
Each time you wish to start the docker environment, run this command in the ardupilot folder :
```bash
docker run --rm -it -v "$(pwd):/ardupilot" -u "$(id -u):$(id -g)" ardupilot:latest bash 
```

Then, if you are compiling this custom version of ardupilot for the first time, you will need to configure the build environment as well using

```bash
./waf configure --board navigator
```

Finally, to build ardupilot, use

```bash
./waf sub
```

If the compilation is successful, the firmware file `ardusub` will be in folder `ardupilot/build/navigator/bin`.

## Merging with your own custom version of ardupilot
If you want to add the CHAD mode to your own version of ardupilot, here are the files that were added / modified compared to the base version of ardupilot at the time (4.6.0) : 
```
MODIFIED :
ArduSub/ArduSub.cpp
ArduSub/GCS_MAVLink_Sub.cpp
ArduSub/Parameters.cpp
ArduSub/Parameters.h
ArduSub/Sub.h
ArduSub/mode.cpp
ArduSub/mode.h
ArduSub/wscript
ArduSub/system.cpp
libraries/AP_Arming/AP_Arming.cpp
libraries/AP_Arming/AP_Arming.h

ADDED :
ArduSub/mode_chad.cpp
libraries/AP_CHAD/AP_CHAD.cpp
libraries/AP_CHAD/AP_CHAD.h
libraries/AP_CHAD/AP_CHAD_Parameters.cpp
libraries/AP_CHAD/AP_CHAD_Parameters.h
libraries/AP_CHAD/AP_CHAD_config.h
```

If you want more information on what was modified, use ```git diff 3ba4f10``` from the root of the repository