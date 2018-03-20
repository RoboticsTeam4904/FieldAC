#!/bin/sh
set -ex
curl -sL https://cmake.org/files/v3.9/cmake-3.9.1-Linux-x86_64.sh > cmake-3.9.1.sh
sh cmake-3.9.1.sh --prefix=/usr/local/ --exclude-subdir
