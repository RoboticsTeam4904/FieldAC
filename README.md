# 2018-Field
Building a realtime model of the field and all its components.

Our goal this year was to build a comprehensive, real-time model of the field and each entity upon it, in a way that can be extensible for future years. This was broken down into four concrete pieces which ideally would function independently of one another just as well as in one bundled executable:
- Field Model
- Robot Localization
- Object Tracking
- Robot Tracking

## Field Model

The field model is an in-memory representation of the absolute poses of each entity discovered so far on the field. Each entity implements its own confidence degradation formula to account for real-time dynamic conditions on the field.

This degradation formula is formatted in a way that can be run simultaneously on the RoboRIO and the TX1. In doing so, we minimize the number of updates the TX1 must send to the RoboRIO to cases where an entity's confidence drops below a cutoff threshold, or goes up.

When the TX1 must send an update, we send it over USB serial connected directly to the RoboRIO, formatted as JSON (?). This allows us to easily manipulate and track data, as well as performing full field reconstructions post-match.

## Object Tracking

We use [AlexeyAB's implementation of the Darknet clang neural network framework](https://github.com/AlexeyAB/darknet) detect and track the dynamic game pieces using [pjreddie's YOLO object detection and classification model](https://pjreddie.com/darknet/yolo/) which we retrain.

The YOLO model outputs a bounding box and class, which we use in conjunction with optical flow to detect and track cubes. This method allows us to run a heavier, more robust model while still maintaining real-time accuracy. The main downside to this approach is objects are less likely to be properly detected and tracked while the robot is in motion, due to the decreased operating frame-rate of the model.

A link to our labeled training data will be available [here](#) at the end of build season.

## Robot Localization

Using pre-constructed 2D maps of the field, we are able to localize ourselves within the map using Monte Carlo Localization (MCL) based upon LIDAR measurements weighted by our encoders.

***-- WIP --***

## Robot Tracking

Due to various technical limitations of our robot configuration, most notably the brutal tradeoff between Camera FPS and resolution we must weigh, this project is on hiatus until the mid-season.

# Getting Started

## Dependencies
Minimum Dependencies:
- C++11
- OpenCV 3.x
- CMake 9.4

Advanced Dependencies:
- CUDA 7.5 or higher
- cuDNN 5 **and** cuDNN 6
- RPLIDAR A2M8 (Hardware)
- Jetson TX1 (Hardware)
- Camera (Hardware)

## Installation (macOS)

```bash
brew install 
brew install opencv
brew install pkg-config
brew install cmake
mkdir build && cd build
cmake ../
make
```

## Installation (Ubuntu 16.04/Jetson TX1)

```bash
# Install all the dependencies here
mkdir build && cd build
cmake ../
make
```


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