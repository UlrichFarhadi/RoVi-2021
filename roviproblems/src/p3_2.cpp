// std includes
#include <iostream>
#include <string>

// Robworks includes
#include <rw/rw.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>

// rovi includes
#include "../includes/p3_2.hpp"


USE_ROBWORK_NAMESPACE
using namespace robwork;

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


void reachabilityAnalysis()
{
    std::cout << "Running reachability analysis..." << std::endl;
    
        //load workcell
        std::cout << "Loading workcell..." << std::endl;
        rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../workcell/Scene.wc.xml");
        if(NULL == wc){
            RW_THROW("COULD NOT LOAD scene... check path!");
        }

        // find relevant frames
        rw::kinematics::MovableFrame::Ptr cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
        if(NULL == cylinderFrame){
            RW_THROW("COULD not find movable frame Cylinder ... check model");
        }

        rw::kinematics::MovableFrame::Ptr bottleFrame = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
        if(NULL == bottleFrame){
            RW_THROW("COULD not find movable frame Bottle ... check model");
        }
    
        rw::kinematics::MovableFrame::Ptr squareFrame = wc->findFrame<rw::kinematics::MovableFrame>("Square");
        if(NULL == squareFrame){
            RW_THROW("COULD not find movable frame Square ... check model");
        }

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

        std::string toolTCP = wsg50->getName() + "." + "TCP";
        std::cout << "toolTCP: " << toolTCP << std::endl;

        for (double rollAngle = 0.0; rollAngle <= 360.0; rollAngle += 1.0)
        {

            // First set the cylinderFrame
            rw::math::Vector3D<> t(bottleFrame->getTransform(state).P());
            rw::math::RPY<> R(rollAngle*rw::math::Deg2Rad,0,0);
            rw::math::Transform3D<> newBottleTransform(t, R);
            bottleFrame->moveTo(newBottleTransform, state);

            
            std::vector<rw::math::Q> solutions = getConfigurations("Bottle", toolTCP, robotUR685A, wc, state);

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

        rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../experiment_data/p3_2/visu.rwplay");

}
