# RoVi-2021
Project in Robotics (kinematics, motion planning, grasping, etc.) and Computer Vision (point clouds, pose estimation, etc.) (Using OpenCV and RobWorkStudio)

build:
/usr/bin/cmake --build /home/thobias/Documents/MSc1/RoVi-2021/build --config Debug --target all -j 10 --

run:
/home/thobias/Documents/MSc1/RoVi-2021/build/rovi2021


BEFORE RUNNING
Check CMakeLists.txt and ensure that the path FIND_PACKAGE(RobWork REQUIRED HINTS /home/ulrich/RobWork) is set
correctly for your current setup.

Ensure that the main.cpp file is running the function you intend to test.
