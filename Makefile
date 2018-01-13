CXX = g++
OPENCV = $(shell pkg-config --cflags --libs opencv)
CPPFLAGS = -std=c++11 $(OPENCV)

build:
	$(CXX) $(CPPFLAGS) main.cpp -o bin/field
