// std includes
#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <fstream>

// Robworks includes
#include <rw/rw.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>

// rovi includes
#include "../includes/p3_2.hpp"


USE_ROBWORK_NAMESPACE
using namespace robwork;


// Function provided by Aljaz during the course in lecture 3
std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base"; 
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal); // "GraspTarget" is the coordinate system of the cylinder (in its center)
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp); // "GraspTCP" is the coordinate system of the robots TCP
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state); //Robot base frame to target object frame
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state); //GraspTCP frame to robotTCP frame

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp; //Put GraspTCP reference frame center at object frame center

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}


int reachabilityAnalysis_pick(double x_pos, double y_pos, bool side, bool replay, rw::models::WorkCell::Ptr wc)
{
    std::cout << "Running reachability analysis..." << std::endl;
    
        //load workcell
        // std::cout << "Loading workcell..." << std::endl;
        // rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../workcell/Scene_ball.wc.xml");
        // if(NULL == wc){
        //     RW_THROW("COULD NOT LOAD scene... check path!");
        // }

        // find relevant frames
        /*
        rw::kinematics::MovableFrame::Ptr cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
        if(NULL == cylinderFrame){
            RW_THROW("COULD not find movable frame Cylinder ... check model");
        }
        */

        rw::kinematics::MovableFrame::Ptr ballFrame = wc->findFrame<rw::kinematics::MovableFrame>("Ball");
        if(NULL == ballFrame){
            RW_THROW("COULD not find movable frame Ball ... check model");
        }

        rw::kinematics::MovableFrame::Ptr URReferenceFrame = wc->findFrame<rw::kinematics::MovableFrame>("URReference");
        if(NULL == URReferenceFrame){
            RW_THROW("COULD not find movable frame URReferenceFrame ... check model");
        }

        /*
        rw::kinematics::MovableFrame::Ptr squareFrame = wc->findFrame<rw::kinematics::MovableFrame>("Square");
        if(NULL == squareFrame){
            RW_THROW("COULD not find movable frame Square ... check model");
        }
        */
        // find UR device
        rw::models::SerialDevice::Ptr robotUR685A = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
        if(NULL == robotUR685A){
            RW_THROW("COULD not find device UR-6-85-5-A ... check model");
        }

        // find WSG device
        rw::models::TreeDevice::Ptr wsg50 = wc->findDevice<rw::models::TreeDevice>("WSG50");
        if(NULL == wsg50){
            RW_THROW("COULD not find device WSG50 ... check model");
        }
        
        
        // colision detector initialization
        rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


        // get the default state
        State state = wc->getDefaultState();
        std::vector<rw::math::Q> collisionFreeSolutions;

        // sample the solutions in a loop (360  deg around rollAngle) with resolution of 1 deg. 
        // Use the function MoveTo to move the manipulator, 
        // record the state

        // Moving robot base:
        rw::math::Vector3D<> robot_T = Vector3D<>(x_pos, y_pos, 0.11);
        rw::math::RPY<> robot_R(0, 0, 0);
        rw::math::Transform3D<> robot_transform(robot_T, robot_R);
        URReferenceFrame->moveTo(robot_transform, state);

        std::string toolTCP = wsg50->getName() + "." + "TCP";
        //std::cout << "toolTCP: " << toolTCP << std::endl;

        std::string target;
        if (side)
        {
            target = "Ball";
        }
        else
        {
            target = "GraspTargetTop";
        }

        for (double rollAngle = 0.0; rollAngle < 360.0; rollAngle += 1.0)
        {
            // First set the cylinderFrame
            rw::math::Vector3D<> t(ballFrame->getTransform(state).P());

            // Notes:
            // Euler ZYX Convention --> Rotation about z0 of angle alpha + Rotation about y1 of angle beta + Rotation about x2 of angle gamma
            // RPY Convention --> (All rotations are about fixed frame (x0, y0, z0) base vectors)
            //      - Rotation about x0 of angle gamma + Rotation about y0 of angle beta + Rotation about z0 of angle alpha
            //      - roll = Rotation around z, pitch = Rotation around y, yaw = Rotation around x
            // Summary --> Roll Pitch Yaw XYZ (g,b,a) <=> Euler ZYX (a,b,g)
            // In our example the Ball rotates around its y axis, so in RPY angles that is the pitch (aka rotation around y)
            //      BUT! It has also been rotated in respect to the world frame which needs to be accounted for:
            //          The Ball has been rotated 90 degrees about the x axis and -90 degrees about the z axis, in reference to the world frame
            //              The -90 can be omitted in the roll equation since it has to go 360 degrees around it anyways
            rw::math::RPY<> R(rollAngle*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
            rw::math::Transform3D<> newBallTransform(t, R);
            ballFrame->moveTo(newBallTransform, state);

            std::vector<rw::math::Q> solutions = getConfigurations(target, toolTCP, robotUR685A, wc, state);

            // loop and check the solutions if there is a colision 
            for (int i = 0; i < solutions.size(); i++)
            {
                // Set the robot in i'th configuration ("q") from the found solutions
                // and see if there is a collision
                robotUR685A->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) ){
                    collisionFreeSolutions.push_back(solutions[i]); // save it
                    break; // we only need one
                }
            }
        }

        if (replay)
        {
            // print the number of colision free solutions
        
            std:: cout << "Current position of the robot vs object to be grasped has: "
                    << collisionFreeSolutions.size()
                    << " collision-free inverse kinematics solutions!" << std::endl;

            // visualize them
            TimedStatePath tStatePath;
            double time=0;
            for(unsigned int i=0; i<collisionFreeSolutions.size(); i++){
                robotUR685A->setQ(collisionFreeSolutions[i], state);
                tStatePath.push_back(TimedState(time,state));
                time+=0.01;
            }

            rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../experiment_data/p3_2/pick.rwplay");
        }

        return (int)collisionFreeSolutions.size();

}

int reachabilityAnalysis_place(double x_pos, double y_pos, bool side, bool replay, rw::models::WorkCell::Ptr wc)
{
    
        //load workcell
        // rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../workcell/Scene_ball.wc.xml");
        // if(NULL == wc){
        //     RW_THROW("COULD NOT LOAD scene... check path!");
        // }

        // find relevant frames
        /*
        rw::kinematics::MovableFrame::Ptr cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
        if(NULL == cylinderFrame){
            RW_THROW("COULD not find movable frame Cylinder ... check model");
        }
        */

        rw::kinematics::MovableFrame::Ptr ballFrame = wc->findFrame<rw::kinematics::MovableFrame>("Ball");
        if(NULL == ballFrame){
            RW_THROW("COULD not find movable frame Ball ... check model");
        }

        rw::kinematics::MovableFrame::Ptr URReferenceFrame = wc->findFrame<rw::kinematics::MovableFrame>("URReference");
        if(NULL == URReferenceFrame){
            RW_THROW("COULD not find movable frame URReferenceFrame ... check model");
        }

        /*
        rw::kinematics::MovableFrame::Ptr squareFrame = wc->findFrame<rw::kinematics::MovableFrame>("Square");
        if(NULL == squareFrame){
            RW_THROW("COULD not find movable frame Square ... check model");
        }
        */
        // find UR device
        rw::models::SerialDevice::Ptr robotUR685A = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
        if(NULL == robotUR685A){
            RW_THROW("COULD not find device UR-6-85-5-A ... check model");
        }

        // find WSG device
        rw::models::TreeDevice::Ptr wsg50 = wc->findDevice<rw::models::TreeDevice>("WSG50");
        if(NULL == wsg50){
            RW_THROW("COULD not find device WSG50 ... check model");
        }
        
        
        // colision detector initialization
        rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));


        // get the default state
        State state = wc->getDefaultState();
        std::vector<rw::math::Q> collisionFreeSolutions;

        // sample the solutions in a loop (360  deg around rollAngle) with resolution of 1 deg. 
        // Use the function MoveTo to move the manipulator, 
        // record the state


        // Moving robot base:
        rw::math::Vector3D<> robot_T = Vector3D<>(x_pos, y_pos, 0.11);
        rw::math::RPY<> robot_R(0, 0, 0);
        rw::math::Transform3D<> robot_transform(robot_T, robot_R);
        URReferenceFrame->moveTo(robot_transform, state);
        

        std::string toolTCP = wsg50->getName() + "." + "TCP";
        //std::cout << "toolTCP: " << toolTCP << std::endl;

        rw::math::Vector3D<> T_place = Vector3D<>(0.285, -0.490, 0.151);

        std::string target;
        if (side)
        {
            target = "Ball";
        }
        else
        {
            target = "GraspTargetTop";
        }

        for (double rollAngle = 0.0; rollAngle < 360.0; rollAngle += 1.0)
        {
            // First set the cylinderFrame
            //rw::math::Vector3D<> t(ballFrame->getTransform(state).P());
            rw::math::RPY<> R(rollAngle*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
            rw::math::Transform3D<> newBallTransform(T_place, R);
            //std::cout << ballFrame->getTransform(state) << std::endl;
            ballFrame->moveTo(newBallTransform, state);

            
            std::vector<rw::math::Q> solutions = getConfigurations(target, toolTCP, robotUR685A, wc, state);

            // loop and check the solutions if there is a colision 
            for (int i = 0; i < solutions.size(); i++)
            {
                // Set the robot in i'th configuration ("q") from the found solutions
                // and see if there is a collision
                robotUR685A->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) ){
                    collisionFreeSolutions.push_back(solutions[i]); // save it
                    break; // we only need one
                }
            }
        }

        
        if (replay)
        {
            // print the number of colision free solutions
        
            std:: cout << "Current position of the robot vs object to be grasped has: "
                    << collisionFreeSolutions.size()
                    << " collision-free inverse kinematics solutions!" << std::endl;

            // visualize them
            TimedStatePath tStatePath;
            double time=0;
            for(unsigned int i=0; i<collisionFreeSolutions.size(); i++){
                robotUR685A->setQ(collisionFreeSolutions[i], state);
                tStatePath.push_back(TimedState(time,state));
                time+=0.01;
            }

            rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../experiment_data/p3_2/place.rwplay");
        }

        return (int)collisionFreeSolutions.size();

}

void simulateCollisionLandscape(int N, rw::models::WorkCell::Ptr wc)
{
    // Make N uniformly distributed base positions of the robot
    // Test both the Pick and Place position

    // Remember ONLY place the robot in the selected area NOT the PICK or PLACE area


    // Pick area:
    //      x = +- 0.320
    //      y = + 0.220 & -0.520

    // Remove points if they are inside the place square:
    //      Lower Left Corner --> x = 0.320 & y = -0.520
    //      Upper right Corner --> x = 0.100 & y = -0.300

    std::default_random_engine generator;
    std::uniform_real_distribution<double> dist_x(-0.320, 0.320);
    std::uniform_real_distribution<double> dist_y(-0.520, 0.220);

    double a_1 = -0.320, b_1 = 0.320;
    double a_2 = -0.520, b_2 = 0.220;

    std::vector<std::vector<double>> xy_positions;

    for (int i = 0; i <= N; i++)
    {
        //double x_pos = dist_x(generator);
        //double y_pos = dist_y(generator);
        double y_pos = a_2 + i * ((b_2 - a_2) / N);
        for (int j = 0; j <= N; j++)
        {
            double x_pos = a_1 + j * ((b_1 - a_1) / N);
            std::vector<double> xy_position = {x_pos, y_pos};
            xy_positions.push_back(xy_position);
        }

        /*
        if (((x_pos < 0.320) and (x_pos > 0.100)) and ((y_pos > -0.520) and (y_pos < -0.300)))    // Check if we are in the place area
        {
            //i--;
            continue;
        }
        */
    }

    // Pick and place with gripping from the SIDE
    std::cout << "Pick and Place with gripping from the SIDE" << std::endl;
    std::vector<int> pick_solutions_side;
    std::vector<int> place_solutions_side;
    // Calculating pick and place solutions
    std::cout << "Calculating pick and place solutions" << std::endl; 
    for (int i = 0; i < xy_positions.size(); i++)
    {
        std::cout << i << " out of " << xy_positions.size()-1 << std::endl;
        //std::cout << reachabilityAnalysis_pick(xy_positions[i][0], xy_positions[i][1]) << std::endl;
        pick_solutions_side.push_back(reachabilityAnalysis_pick(xy_positions[i][0], xy_positions[i][1], true, false, wc));
        place_solutions_side.push_back(reachabilityAnalysis_place(xy_positions[i][0], xy_positions[i][1], true, false, wc));
    }
    
    std::ofstream data_file_side_combined("../experiment_data/p3_2/combined_collision_landscape_SIDE.csv");
    std::ofstream data_file_side_pick("../experiment_data/p3_2/pick_collision_landscape_SIDE.csv");
    std::ofstream data_file_side_place("../experiment_data/p3_2/place_collision_landscape_SIDE.csv");
    std::cout << "Writing to file..." << std::endl;
    int indx = 0;
    for (int i = 0; i <= N; i++)
    {
        for (int j = 0; j <= N; j++)
        {
            data_file_side_combined << pick_solutions_side[indx] + place_solutions_side[indx] << ",";
            data_file_side_pick << pick_solutions_side[indx] << ",";
            data_file_side_place << place_solutions_side[indx] << ",";
            indx++;
        }
        data_file_side_combined << "\n";
        data_file_side_pick << "\n";
        data_file_side_place << "\n";
    }
    std::cout << "Collision Landscape Generated" << std::endl;


    // Pick and place with gripping from the TOP
    std::cout << "Pick and Place with gripping from the TOP" << std::endl;
    std::vector<int> pick_solutions_top;
    std::vector<int> place_solutions_top;
    // Calculating pick and place solutions
    std::cout << "Calculating pick and place solutions" << std::endl; 
    for (int i = 0; i < xy_positions.size(); i++)
    {
        std::cout << i << " out of " << xy_positions.size()-1 << std::endl;
        //std::cout << reachabilityAnalysis_pick(xy_positions[i][0], xy_positions[i][1]) << std::endl;
        pick_solutions_top.push_back(reachabilityAnalysis_pick(xy_positions[i][0], xy_positions[i][1], false, false, wc));
        place_solutions_top.push_back(reachabilityAnalysis_place(xy_positions[i][0], xy_positions[i][1], false, false, wc));
    }

    
    std::ofstream data_file_top_combined("../experiment_data/p3_2/combined_collision_landscape_TOP.csv");
    std::ofstream data_file_top_pick("../experiment_data/p3_2/pick_collision_landscape_TOP.csv");
    std::ofstream data_file_top_place("../experiment_data/p3_2/place_collision_landscape_TOP.csv");
    std::cout << "Writing to file..." << std::endl;
    indx = 0;
    for (int i = 0; i <= N; i++)
    {
        for (int j = 0; j <= N; j++)
        {
            data_file_top_combined << pick_solutions_top[indx] + place_solutions_top[indx] << ",";
            data_file_top_pick << pick_solutions_top[indx] << ",";
            data_file_top_place << place_solutions_top[indx] << ",";
            indx++;
        }
        data_file_top_combined << "\n";
        data_file_top_pick << "\n";
        data_file_top_place << "\n";
    }

    std::cout << "Collision Landscape Generated" << std::endl;
    



}