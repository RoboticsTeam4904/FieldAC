FROM ubuntu:artful as build

# Install Basic Packages
RUN apt-get update
RUN apt-get install -y build-essential git pkg-config curl libcurl4-openssl-dev unzip make

# Install CMake
WORKDIR /build/cmake
RUN curl -sL https://cmake.org/files/v3.9/cmake-3.9.1-Linux-x86_64.sh > cmake-3.9.1.sh
RUN sh cmake-3.9.1.sh --prefix=/usr/local/ --exclude-subdir

# Install OpenCV
WORKDIR /build/opencv
RUN apt-get install -y libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev clang-4.0
RUN apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
RUN curl -sL https://github.com/opencv/opencv/archive/3.4.0.zip > opencv.zip
RUN unzip opencv.zip && cd opencv-3.4.0 && mkdir build
WORKDIR /build/opencv/opencv-3.4.0/build
RUN CXX="g++" CC="gcc" cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_JASPER=OFF -D WITH_TBB=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_OPENGL=ON -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF ../
RUN CXX="g++" CC="gcc" make -j4
RUN CXX="g++" CC="gcc" make install
RUN sh -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
RUN ldconfig

# Copy project over
COPY . /fieldac
WORKDIR /fieldac
RUN mkdir build && cd build

# Get the project dependencies ready
WORKDIR /fieldac/build
RUN CXX="g++" CC="gcc" cmake ..

# Get the project binary ready
WORKDIR /fieldac/build
RUN CXX="g++" CC="gcc" LD_DEBUG=all make -j4

FROM ubuntu:xenial
COPY --from=build /fieldac /fieldac
ENTRYPOINT ["/fieldac/build/field"]
CMD ["--help"]




