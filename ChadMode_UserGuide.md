# Custom Ardupilot's diff recap
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
More detailled diff to be found on github.
# Installation

## Downloading resources
Clone the repositories for CHAD mode's custom ardupilot version and the CHAD Processing Unit (CPU for short)
```bash
mkdir CHAD_mode
cd CHAD_mode
git clone git@github.com:AlioTheCat/ardupilot.git
git clone git@github.com:noeterrien/CHAD_Processing_Unit.git
```

Or merge it with your custom ardupilot version if you want (and at your own risks)

## Requirements for the CPU
*Excerpt from CPU's* `Readme.md`

To compile the CHAD Processing Unit (CPU), you will need the following dependencies :
### OpenCV2
First install the required dependencies using the provided ```CV2_install_dependencies.sh``` script :
```bash
source CV2_install_dependencies.sh
```

Then install OpenCV2 minimal prerequisites, running ```sudo apt update && sudo apt install -y cmake g++ wget unzip```. You can then download and unpack sources (as found in OpenCV's documentation)
```bash
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
unzip opencv.zip
```
Finally configure, build and install OpenCV2 : 
```bash
cd opencv-4.x
mkdir build
cmake -D WITH_GSTREAMER=ON \
      -D WITH_FFMPEG=ON \  # optional if you want only GStreamer
      -D CMAKE_BUILD_TYPE=Release \
      -B build
cd build
make -j$(nproc)
sudo make install
```
**MAKE SURE OpenCV IS BUILT WITH GStreamer**. The provided script should install GStreamer dependencies but OpenCV might need additionnal configuration for the build. You can check if GStreamer is installed using a c++ script : Once executed, look for `Video I/O : GStreamer`. It should be set to `YES`
```c++
#include <opencv2/opencv.hpp>
#include <iostream>

std::cout << cv::getBuildInformation() << std::endl;
```

### cpp-mjpeg-streamer
nadjieb' `cpp-mjpeg-streamer` is used to display camera in a browser in real time.
To install, do :
```bash
git clone https://github.com/nadjieb/cpp-mjpeg-streamer.git;
cd cpp-mjpeg-streamer;
mkdir build && cd build;
cmake ../;
make;
sudo make install;
```

### cpp-httplib
This library is used to communicate with the CHAD Processing Unit via http requests (usually using cockpit).
To install, do :
```bash
git clone https://github.com/yhirose/cpp-httplib.git
cd cpp-httplib
mkdir build && cd build
cmake ../
make 
sudo make install
```

### yaml-cpp
This library is used to parse the configuration file `config.yaml`
```bash
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=on ..
make 
sudo make install
```

## Requirements for ardupilot
Ardupilot has a built-in script to install all dependencies. In ardupilot's root folder, run :
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y . ~/.profile
```

# Compiling ardupilot
## Configure and start docker
The versions between the RasPi and ardupilot's compiler may differ wish may lead to some issues. To solve this, we're using a downgraded version of the compiler via docker :
In ardupilot's root folder, run this command to prepare the compiler environment.
```bash
docker build . -t ardupilot --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g)
```

Then, each time you wish to start this environment, run this command in the ardupilot folder :
```bash
docker run --rm -it -v "$(pwd):/ardupilot" -u "$(id -u):$(id -g)" ardupilot:latest bash 
```
## Compiling in the docker environment
The first time you compile ardupilot (and every once in a while), run this command to specify the type of card plugged in the RaspPi (in our case a navigator) :
```bash
./waf configure --board navigator
```

Run this command to compile ardupilot (sub / ArduSub stand for submarine)
```bash
./waf sub
```

If the compilation is successful, the firmware file `ardusub` will be in folder `ardupilot/build/navigator/bin`.

# Compiling and running the CPU
