# RoVi-2021
Project in Robotics (kinematics, motion planning, grasping, etc.) and Computer Vision (point clouds, pose estimation, etc.) (Using OpenCV and RobWorkStudio



BEFORE RUNNING
Check CMakeLists.txt and ensure that the path FIND_PACKAGE(RobWork REQUIRED HINTS <PATH>) and FIND_PACKAGE(RobWorkStudio REQUIRED HINTS <PATH>) is set correctly for your current setup.

Ensure that the main.cpp file is running the function you intend to test. That is by commenting out the functions you want to test inside the main function manually
There are comments in the main function guiding you how to remove uncomment code to run the individual tests. This is done to gain more
control over what is tested and what is not since testing everything at once can take an hour or more.
