# 2018-Vision
Messing around with OpenCV and C++

## Building instructions
```bash
brew install opencv
brew install pkg-config
pkg-config --list-all | grep opencv		# Make sure this says somethning
pkg-config --cflags --libs opencv 		# Make sure the first directory looks like "-I/usr/local/Cellar/opencv/3.4.0/include/opencv"
make
```

The binary will be in `bin/`

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
