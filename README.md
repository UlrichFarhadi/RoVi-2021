# RoVi-2021
Project in Robotics (kinematics, motion planning, grasping, etc.) and Computer Vision (point clouds, pose estimation, etc.) (Using OpenCV and RobWorkStudio

This project is developed on three seperate branches. For tests regarding Reachabillity analysis and Sparse stereo pose estimation method see the README.md file in the "Triangulation" branch. For tests regarding the dense stereo pose estimation see the README.md file in the "cv4dense" branch.

To run the test function for the combination of robotics motion planning and computer vision pose estimation simply compile and run the main.cpp file.

By running the test function, RobWorkStudio play files are generated that show the robot pick and place a ball object from the pick area to the place area. To view these play files load the Project_WorkCell_Obstacle/Scene_ball.wc.xml file in RobWorkStudio and the load the any of the play files located at experiment_data/combination/rwplays/with_obstacles/XXX.rwplay files. The motion planning is computed purely by use of pose estimation from the implemented sparse stereo pose estimation method.

Please ensure that the CMakeLists.txt file contains the correct paths to the used libraries.