#!/bin/sh

qmake
make

mkdir -p obj/
mv *.o obj/

mkdir -p build/
mv rgbd_odometry build/