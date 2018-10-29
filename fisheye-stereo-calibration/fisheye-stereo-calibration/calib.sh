#!/usr/bin/env bash

./build/calibrate -w 11 -h 8 -s 0.03 -n 17 -d ../imgs/ -l left -r right -o cam_stereo.yml

cd ../imgs/

ll |grep lef | awk '{print "cp "$NF"  " $NF}' | sed s/left/fisheye_/2  | sh

cd -

rosrun camera_model Calibration -w 11 -h 8 -s 30 -p fisheye_ -e .jpg -i ../imgs/ --camera-model mei

