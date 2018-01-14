# 2018-Field
Building a realtime model of the field and all its components.

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


## Project Structure

### Robot Tracking (rt)
- Looking for bumpers?¿

### Robot Localization (rl)
- MCL using IMU data?


### Object Tracking (ot)
- Cube tracking

```
main.cpp
|-vision.cpp
|-field.cpp
 |-cubetrack.cpp

 ...

 assorted files from @leijurv
```
