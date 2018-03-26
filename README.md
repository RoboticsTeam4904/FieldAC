# Field AC
Building a realtime model of the field and all its components [to eventually reduce entropy](http://www.wikiwand.com/en/Franchise_(short_story)).

# Table of Contents

* [FRC Field Model](#frc-field-model)
* [Table of Contents](#table-of-contents)
* [About](#about)
  * [Field Model](#field-model)
  * [Object Tracking](#object-tracking)
    * [Optical Flow](#optical-flow)
  * [Robot Localization](#robot-localization)
  * [Robot Tracking](#robot-tracking)
  * [Pathing](#pathing)
* [Getting Started](#getting-started)
  * [Project Structure](#project-structure)
  * [Dependencies](#dependencies)
    * [Hardware](#hardware)
    * [Software](#software)
  * [Installation](#installation)
    * [macOS](#macOS)
    * [Ubuntu 16.04 (Jetson TX1/TX2)](#ubuntu-1604-jetson-tx1tx2)
  * [Usage](#usage)

# About

Our goal this year was to build a comprehensive, real-time model of the field and each entity upon it, in a way that can be extensible for future years. This was broken down into four concrete pieces which ideally would function independently of one another just as well as in one bundled executable:
- Field Model
- Robot Localization
- Object Tracking
- Robot Tracking
- Pathing

## Field Model

The field model is an in-memory representation of the absolute poses of each entity discovered so far on the field. Each entity implements its own confidence degradation formula to account for real-time dynamic conditions on the field.

This degradation formula is formatted in a way that can be run simultaneously on the RoboRIO and the TX\*. In doing so, we minimize the number of updates the TX\* must send to the RoboRIO to cases where an entity's confidence drops below a cutoff threshold, or goes up.

When the TX\* must send an update, we send it over the robot LAN, formatted as JSON (?). This allows us to easily manipulate and track data, as well as performing full field reconstructions/simulations post-match.

## Object Tracking

We use [AlexeyAB's implementation of the Darknet clang neural network framework](https://github.com/AlexeyAB/darknet) detect and track the dynamic game pieces using [pjreddie's YOLO object detection and classification model](https://pjreddie.com/darknet/yolo/) which we retrain.

A link to our labeled training data will be available [here](#) at the end of build season.

The YOLO model outputs a bounding box and class, which we use in conjunction with optical flow to detect and track cubes. This method allows us to run a heavier, more robust model while still maintaining real-time accuracy. The main downside to this approach is objects are less likely to be properly detected and tracked while the robot is in motion, due to the decreased operating frame-rate of the model.

| Tables        | Are           | Cool  |
| ------------- |:-------------:| -----:|
| col 3 is      | right-aligned | $1600 |
| col 2 is      | centered      |   $12 |
| zebra stripes | are neat      |    $1 |

### Optical Flow

We use the OpenCV implementation of the Lucas-Kanade optical flow method in between frames, then do large recalculations on each "keyframe" (darknet frame). The center of each detected object in the keyframe becomes a single feature to simplify the tracking. 

## Robot Localization

We use a 2D slice of the field built from [AutoDesk's models](https://www.autodesk.com/education/competitions-and-events/first/recommended-software#Kit-of-parts), and localize ourselves within this model using Monte Carlo Localization (MCL) based upon LIDAR measurements.

To increase accuracy and confidence in our pose estimation, we weight the points based on encoder measurements for distance and velocity, and IMU measurements for acceleration. 

***WIP***: The code hereof will be abstracted out into a *point-cloud provider* which will be implemented based on specific LIDAR SDK limitations, and *point-cloud consumer* which implements the ray-tracing and the MCL therefrom, etc. 

## Robot Tracking

***NOTE***: We have only conducted preliminary research into the implementation of this project, and we will likely not work on it until the offseason of 2018. 

By using OCR on the team numbers located printed on the bumpers (See *Section 8.5: Bumper Rules* of the 2018 Game Manual), we can determine pose of the robot and assume padding upon the maximum robot size to ensure that our estimations err on the side of caution.

## Pathing

Our pathing library is maintained separately and written in Java. We have plans to rewrite the library in C++ with JNI bindings, however we will only be tackling this later. For now, they will remain independent.

Check out our Motion Control/Pathing library for details:   
https://github.com/RoboticsTeam4904/MotionControl

# Getting Started

## Project Structure

```ASCII
.
├── botlocale/          Robot Localization
├── bottracking/        Robot Tracking
├── build/
│   ├── darknet-src/    AlexeyAB/Darknet (Cloned when `cmake ../` is run)
│   └── rplidar-src/    RPLIDAR SDK (Downloaded when `cmake ../ is run`)
├── network/            Shared Darknet abstraction layer
├── objecttracking/     Object Tracking
├── CMakeLists.txt
└── main.cpp
```

*Note: Files or directories that were omitted are either unimportant to the project's structure, generated by CMake, or both.*

The Network directory allows for elegant sharing of a single Darknet Detector instance while maintaining a concrete separation between `Robot Tracking` and `Object Tracking`. This is especially important due to the drastically different needs of the two components during the optical flow phase.  

For more insight into the project structure, `CMakeLists.txt` and its companion files may help.

## Dependencies

### Hardware
- RPLIDAR A2M8
  - If you have another LIDAR, you must implement a point-cloud supplier yourself.
- Jetson TX1/TX2
  - These are the only boards officially tested and supported. Use others at your own peril.
- Camera (Optional)
  - You can also use pre-captured footage. See the [usage](#usage) section for details.

### Software
Minimum Dependencies:
- C++11
- OpenCV 3.x
- CMake 9.4

Advanced Dependencies:
- CUDA 7.5 or higher
- cuDNN 5 **and** cuDNN 6

## Installation
### macOS

```bash
brew install 
brew install opencv
brew install pkg-config
brew install cmake
mkdir build && cd build
cmake ../
make
```

### Ubuntu 16.04 (Jetson TX1/TX2)
**Note:** 
>We have experienced troubles with CMake not properly following 302 redirects when downloading the static files on the TX1. You may have to set these up manually. The links and hashes are in [CMakeLists.txt](/CMakeLists.txt)

```bash
# Install all the dependencies here
mkdir build && cd build
cmake ../
make
```

## Usage

```shell
Usage: field [params] 

	--dev (value:0)
		Capture Device
	--help (value:true)
		help
	--ldr_baud (value:115200)
		[LIDAR] Baudrate for serial communications
	--ldr_dev
		[LIDAR] Path to the *nix device (eg. /dev/ttyUSB0)
	--net_cfd (value:0.5)
		[Network] Minimum identification confidence threshold
	--net_cfg
		[Network] Model ".cfg" file
	--net_cls
		[Network] ".names" file for identifiable classes
	--net_mdl
		[Network] Model ".weights" file
	--net_save
		[Network] Detection output file. Shows what the network detects
	--src
		Source Video file. Overrides any specified capture device

```