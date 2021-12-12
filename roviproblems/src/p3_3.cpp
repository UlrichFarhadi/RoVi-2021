// std includes
#include <iostream>
#include <cstring>
#include <cassert>
#include <vector>
#include <math.h>
#include <algorithm>    // std::reverse
#include <fstream>

// Robworks includes
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/trajectory/Timed.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/trajectory/Path.hpp>

// rovi includes
#include "../includes/p3_3.hpp"


USE_ROBWORK_NAMESPACE
using namespace robwork;

std::vector<rw::math::Q> LIPConfigurations(std::vector<rw::math::Q> &Qs, double tf = 2)
{
    std::vector<rw::math::Q> Qs_LIPed;

    for(size_t i = 0; i < Qs.size() -1; ++i)
    {
        rw::trajectory::LinearInterpolator<Q> LIP(Qs[i], Qs[i+1], tf);
        
        for(double tk=0; tk<=tf; tk += 0.01){
            Qs_LIPed.push_back(LIP.x(tk));
        }

    }

    return Qs_LIPed;

}

std::vector<rw::math::Q> RRTConfigurations(std::vector<rw::math::Q> &Qs, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    CollisionDetector::Ptr collision_detector = common::ownedPtr(new CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    const rw::pathplanning::PlannerConstraint pathplanner_constraint = PlannerConstraint::make(collision_detector, robot, state);
    rw::pathplanning::QToQPlanner::Ptr RRTQQPathplanner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(pathplanner_constraint, robot, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    

    std::vector<rw::math::Q> Qs_RRTed;
    //std::cout << "Going through configs" << std::endl;
    for(size_t i = 0; i < Qs.size() -1; ++i)
    {
        std::cout << "Config " << i << std::endl;
        //std::cout << "From config " << Qs[i] << " to " << Qs[i+1] << std::endl;
        rw::trajectory::Path<Q> pick_path;
        RRTQQPathplanner->query(Qs[i], Qs[i+1], pick_path);
        Qs_RRTed.insert(Qs_RRTed.end(), pick_path.begin(), pick_path.end());
    }

    return Qs_RRTed;

}

std::vector<rw::math::Q> PBConfigurations(const std::vector<rw::trajectory::LinearInterpolator<Q>> &LIPS, double blend_time)
{
    std::vector<rw::math::Q> PBed_Path;
    double t = 0;
    for(double tk = 0; tk < LIPS[0].duration() - blend_time; tk += 0.01, t += 0.01)
    {

        PBed_Path.push_back(LIPS[0].x(tk));
    }

    for(int i = 1; i < LIPS.size(); ++i)
    {
        rw::trajectory::ParabolicBlend<Q> PB(LIPS[i-1], LIPS[i], blend_time);
        for(double tk = 0; tk <  2 * blend_time; tk += 0.01, t+= 0.01)
        {

            PBed_Path.push_back(PB.x(tk));
        }

        for(double tk = blend_time; tk < LIPS[i].duration() - blend_time; tk += 0.01, t+= 0.01)
        {

            PBed_Path.push_back(PB.x(tk));
        }


    }
    size_t n_LIPS = LIPS.size();
    for(double tk = LIPS[n_LIPS - 1].duration() - blend_time; tk <= LIPS[n_LIPS - 1].duration(); tk += 0.01,  t+= 0.01)
    {
        PBed_Path.push_back(LIPS[n_LIPS-1].x(tk));
    }


    return PBed_Path;
}

std::vector<rw::math::Q> mygetConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
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

std::vector<Q> q_collision_free(std::vector<Q> &configurations, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    //Initialisation of collision detector
    CollisionDetector::Ptr collision_detector = common::ownedPtr(new CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    std::vector<Q> collisionFreeconfigurations;
    // loop and check the solutions if there is a colision 
    for (size_t i = 0; i < configurations.size(); ++i)
    {
        // Set the robot in i'th configuration ("q") from the found configurations
        // and see if there is a collision
        robot->setQ(configurations[i], state);
        if( !collision_detector->inCollision(state,NULL,true) ){
            collisionFreeconfigurations.push_back(configurations[i]); // save it
            break; // we only need one
        }
    }

    return collisionFreeconfigurations;
}

std::vector<std::pair<Transform3D<>, Q>> compute_path_points(double R_init, double R_max,const std::vector<Vector3D<>> &ps, const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    rw::kinematics::MovableFrame::Ptr goalFrame = wc->findFrame<rw::kinematics::MovableFrame>(nameGoal);
    if(nullptr == goalFrame){
        Log::errorLog() << "Goal frame could not be found." << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Goal frame could not be found.");
    }

    std::vector<std::pair<Transform3D<>, Q>> path_points;
    Transform3D<> goalFrame_wTF = goalFrame->wTf(state);
    double R_MAX = R_max;
    for(double R = R_init; R <= R_MAX ; R+=10)
    {
        Log::infoLog() << R << std::endl;        
        Transform3D<> T_prev = goalFrame_wTF;
        for(auto p : ps)
        {
            
            Transform3D<> T(T_prev * p, Rotation3D<>(RPY<>(R * Deg2Rad, 0, 0)) * goalFrame_wTF.R());
            goalFrame->moveTo(T, state);
            std::vector<Q> possible_object_configurations = mygetConfigurations(nameGoal, nameTcp, robot, wc, state);
            if(possible_object_configurations.empty())
            {
                break;
            }
            //TEST
            /*
            std::pair<Transform3D<>, Q> found_pair = std::make_pair(T, possible_object_configurations[0]);
            path_points.push_back(found_pair);
            break;*/
            //
            std::vector<Q> collision_free_goal_config = q_collision_free(possible_object_configurations, robot, wc, state);

            if(!collision_free_goal_config.empty())
            {
                std::pair<Transform3D<>, Q> found_pair = std::make_pair(T, collision_free_goal_config[0]);
                path_points.push_back(found_pair);
            }
            else
            {
                break;
            }
            T_prev = T;
        }

        if(path_points.size() == ps.size())
        {
            break;
        }
        else
        {
            path_points = {};
            continue;
        }

    }
    
    return path_points;
}

std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Q>> get_path_points(const std::string &scene_path, const std::string& robot_device_name, Transform3D<> &objectInitFrame)
{
    Log::infoLog() << "Running motion planning..." << std::endl;

    //Loading workcell
    Log::infoLog() << "Loading workcell..." << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(scene_path);
    if(wc == nullptr)
    {
        Log::errorLog() << "Workcell could not be loaded from " << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Workcell could not be loaded from " + scene_path);
    }
    Log::infoLog() << "Workcell has been loaded succesfully" << std::endl;

    //Loading robot device
    Log::infoLog() << "Loading robot device from workcell..." << std::endl;
    SerialDevice::Ptr robot_device= wc->findDevice<SerialDevice>(robot_device_name);
    if(robot_device== nullptr)
    {
        Log::errorLog() << "Robot device named " << robot_device_name << 
            " could not be loaded from workcell at path." << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Robot device named " + robot_device_name  + " could not be loaded from workcell at path.");
    }
    Log::infoLog() << "Robot device found" << std::endl;

    Log::infoLog() << "Loading gripper..." << std::endl;
    const std::string gripper_device_name = "WSG50"; 
    TreeDevice::Ptr gripper_device = wc->findDevice<TreeDevice>(gripper_device_name);



    //Get relevant frame names
    const std::string gripper_tcp_name = gripper_device_name + "." + "TCP";
    const std::string object_name = "Bottle";

    //Get object and gripper frame
    MovableFrame* object_frame = wc->findFrame<MovableFrame>(object_name);

    rw::kinematics::Frame* frameTcp = wc->findFrame<Frame>(gripper_tcp_name);

    //Place target frame name
    const std::string place_target_name = "Place_Target";
    rw::kinematics::Frame* place_target_frame = wc->findFrame<Frame>(place_target_name);

    //Loading default state
    State state = wc->getDefaultState();    

    Q default_q = robot_device->getQ(state);

    //Move object to start frame
    object_frame->moveTo(objectInitFrame, state);
    
    Vector3D<> Pi_O1(0,0,0);
    Vector3D<> Pi_O2(0,0.05,0);
    Vector3D<> Pi_O3(0,0.0,-0.08);
    Vector3D<> Pi_O4(0,0.735,0);
    std::vector<Vector3D<>> ps_pick = {Pi_O1, Pi_O2, Pi_O3, Pi_O4};

    Log::infoLog() << "Computing pick path points..." << std::endl;
    std::vector<std::pair<Transform3D<>, Q>> pick_path_points = compute_path_points(180,360, ps_pick, object_name, gripper_tcp_name, robot_device, wc, state);
    Log::infoLog() << "Found " << pick_path_points.size() << " pick path points" << std::endl;
    std::vector<Q> pick_path_points_Q;
    for(auto path_point : pick_path_points)
    {
        pick_path_points_Q.push_back(path_point.second);
    }

    Vector3D<> Pl_O1(0,0,0);
    Vector3D<> Pl_O2(0,0.05,0);
    Vector3D<> Pl_O3(0,0.0,-0.08);
    Vector3D<> Pl_O4(0,0.735,0);
    std::vector<Vector3D<>> ps_place = {Pl_O1, Pl_O2, Pl_O3, Pl_O4};

    Log::infoLog() << "Computing place path points..." << std::endl;
    //std::vector<std::pair<Transform3D<>, Q>> path_points = compute_path_points(ps_pick, object_name, gripper_tcp_name, robot_device, wc, state);
    std::vector<std::pair<Transform3D<>, Q>> place_path_points = compute_path_points(28, 100, ps_place, place_target_name, gripper_tcp_name, robot_device, wc, state);
    Log::infoLog() << "Found " << place_path_points.size() << " place path points" << std::endl;
    std::vector<Q> place_path_points_Q;
    for(auto path_point : place_path_points)
    {
        place_path_points_Q.push_back(path_point.second);
    }

    std::pair<std::vector<rw::math::Q>, std::vector<rw::math::Q>> pick_n_place_Qs = std::make_pair(pick_path_points_Q, place_path_points_Q);        
    return pick_n_place_Qs;
}

void motion_planning_LIP(const std::string &scene_path, const std::string& robot_device_name, rw::math::Transform3D<> &objectInitFrame, std::vector<Q> &pick_path_points_Q, std::vector<Q> &place_path_points_Q, std::string map, int trial = 1)
{
    Log::infoLog() << "Running motion planning..." << std::endl;

    //Loading workcell
    Log::infoLog() << "Loading workcell..." << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(scene_path);
    if(wc == nullptr)
    {
        Log::errorLog() << "Workcell could not be loaded from " << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Workcell could not be loaded from " + scene_path);
    }
    Log::infoLog() << "Workcell has been loaded succesfully" << std::endl;

    //Loading robot device
    Log::infoLog() << "Loading robot device from workcell..." << std::endl;
    SerialDevice::Ptr robot_device= wc->findDevice<SerialDevice>(robot_device_name);
    if(robot_device== nullptr)
    {
        Log::errorLog() << "Robot device named " << robot_device_name << 
            " could not be loaded from workcell at path." << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Robot device named " + robot_device_name  + " could not be loaded from workcell at path.");
    }
    Log::infoLog() << "Robot device found" << std::endl;

    Log::infoLog() << "Loading gripper..." << std::endl;
    const std::string gripper_device_name = "WSG50"; 
    TreeDevice::Ptr gripper_device = wc->findDevice<TreeDevice>(gripper_device_name);



    //Get relevant frame names
    const std::string gripper_tcp_name = gripper_device_name + "." + "TCP";
    const std::string object_name = "Bottle";

    //Get object and gripper frame
    MovableFrame* object_frame = wc->findFrame<MovableFrame>(object_name);

    rw::kinematics::Frame* frameTcp = wc->findFrame<Frame>(gripper_tcp_name);

    //Place target frame name
    const std::string place_target_name = "Place_Target";
    rw::kinematics::Frame* place_target_frame = wc->findFrame<Frame>(place_target_name);

    //Loading default state
    State state = wc->getDefaultState();    

    Q default_q = robot_device->getQ(state);

    //Move object to start frame
    object_frame->moveTo(objectInitFrame, state);
    
    
    Log::infoLog() << "Linear Interpolation of picked motion. " << pick_path_points_Q.size()  << " path points..." << std::endl;
    std::vector<rw::math::Q> Qs_IPed_picked = LIPConfigurations(pick_path_points_Q, 2);
    Log::infoLog() << "Interpolating path points resulted in " << Qs_IPed_picked.size() << " total path points." << std::endl;

    std::reverse(pick_path_points_Q.begin(), pick_path_points_Q.end());
    pick_path_points_Q.insert(pick_path_points_Q.begin(),default_q);
    Log::infoLog() << "Linear Interpolation of pick motion. " << pick_path_points_Q.size()  << " path points..." << std::endl;
    std::vector<rw::math::Q> Qs_IPed_pick = LIPConfigurations(pick_path_points_Q, 2);
    Log::infoLog() << "Interpolating path points resulted in " << Qs_IPed_pick.size() << " total path points." << std::endl;
    pick_path_points_Q.erase(pick_path_points_Q.begin());

    std::vector<rw::math::Q> transition_path_points_Q = {pick_path_points_Q.front(), place_path_points_Q.back()};
    std::vector<rw::math::Q> Qs_IPed_transition = LIPConfigurations(transition_path_points_Q, 2);
    Log::infoLog() << "Interpolating transition path points resulted in " << Qs_IPed_transition.size() << " total path points." << std::endl;

    place_path_points_Q.push_back(default_q);
    Log::infoLog() << "Linear Interpolation of placed motion. " << place_path_points_Q.size()  << " path points..." << std::endl;
    std::vector<rw::math::Q> Qs_IPed_placed = LIPConfigurations(place_path_points_Q, 2);
    Log::infoLog() << "Interpolating path points resulted in " << Qs_IPed_placed.size() << " total path points." << std::endl;
    place_path_points_Q.pop_back();

    std::reverse(place_path_points_Q.begin(), place_path_points_Q.end());
    Log::infoLog() << "Linear Interpolation of place motion. " << place_path_points_Q.size()  << " path points..." << std::endl;
    std::vector<rw::math::Q> Qs_IPed_place = LIPConfigurations(place_path_points_Q, 2);
    Log::infoLog() << "Interpolating path points resulted in " << Qs_IPed_place.size() << " total path points." << std::endl;

    std::string trajectory_path = std::string("../experiment_data/p3_3/trajectories/") + map + std::string("/")
                             + std::string("LIP_trajectory") + std::string("_trial") + std::to_string(trial)
                             + std::string(".csv");
    std::ofstream f(trajectory_path);

    rw::trajectory::TimedStatePath tStatePath;
    double tk = 0;
    for(auto Q : Qs_IPed_pick){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }

    rw::kinematics::Kinematics::gripFrame(object_frame, frameTcp, state);
    tk += 0.01;

    for(auto Q : Qs_IPed_picked){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }

    for(auto Q : Qs_IPed_transition){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }

    for(auto Q : Qs_IPed_place){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }

    rw::kinematics::Kinematics::gripFrame(object_frame, place_target_frame, state);
    tk += 0.01;

    for(auto Q : Qs_IPed_placed){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }
    std::string rwplay_path = std::string("../experiment_data/p3_3/rwplays/") + map + std::string("/")
                             + std::string("LIP_visu_s") + std::string("_trial") + std::to_string(trial)
                             + std::string(".rwplay");
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplay_path);
    
    std::cout << "DONE" << std::endl;
    

}

void motion_planning_RRT(const std::string &scene_path, const std::string& robot_device_name, rw::math::Transform3D<> &objectInitFrame, std::string map, int trial=1)
{
    Log::infoLog() << "Running motion planning..." << std::endl;

    //Loading workcell
    Log::infoLog() << "Loading workcell..." << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(scene_path);
    if(wc == nullptr)
    {
        Log::errorLog() << "Workcell could not be loaded from " << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Workcell could not be loaded from " + scene_path);
    }
    Log::infoLog() << "Workcell has been loaded succesfully" << std::endl;

    //Loading robot device
    Log::infoLog() << "Loading robot device from workcell..." << std::endl;
    SerialDevice::Ptr robot_device= wc->findDevice<SerialDevice>(robot_device_name);
    if(robot_device== nullptr)
    {
        Log::errorLog() << "Robot device named " << robot_device_name << 
            " could not be loaded from workcell at path." << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Robot device named " + robot_device_name  + " could not be loaded from workcell at path.");
    }
    Log::infoLog() << "Robot device found" << std::endl;

    Log::infoLog() << "Loading gripper..." << std::endl;
    const std::string gripper_device_name = "WSG50"; 
    TreeDevice::Ptr gripper_device = wc->findDevice<TreeDevice>(gripper_device_name);



    //Get relevant frame names
    const std::string gripper_tcp_name = gripper_device_name + "." + "TCP";
    const std::string object_name = "Bottle";

    //Get object and gripper frame
    MovableFrame* object_frame = wc->findFrame<MovableFrame>(object_name);

    rw::kinematics::Frame* frameTcp = wc->findFrame<Frame>(gripper_tcp_name);

    //Place target frame name
    const std::string place_target_name = "Place_Target";
    rw::kinematics::Frame* place_target_frame = wc->findFrame<Frame>(place_target_name);

    //Loading default state
    State state = wc->getDefaultState();    

    Q default_q = robot_device->getQ(state);

    //Move object to start frame
    object_frame->moveTo(objectInitFrame, state);

    std::vector<Vector3D<>> goal_pos = {Vector3D<>(0,0,0)};
    std::vector<std::pair<Transform3D<>, Q>> pick_path_points_TQ = compute_path_points(180, 360, goal_pos, object_name, gripper_tcp_name, robot_device, wc, state);
    std::vector<std::pair<Transform3D<>, Q>> place_path_points_TQ = compute_path_points(180, 360, goal_pos, place_target_name, gripper_tcp_name, robot_device, wc, state);

    std::vector<Q> path_points;
    path_points.push_back(default_q);
    path_points.push_back(pick_path_points_TQ[0].second);
    
    Log::infoLog() << "Computing RRT configurations..." << std::endl;
    std::cout << path_points.size() << std::endl;
    std::vector<Q> path_pick = RRTConfigurations(path_points, robot_device, wc, state);
    Log::infoLog() << "RRT configurations computed." << std::endl;
    
    std::string trajectory_path = std::string("../experiment_data/p3_3/trajectories/") + map + std::string("/")
                             + std::string("RRT_trajectory") + std::string("_trial") + std::to_string(trial)
                             + std::string(".csv");
    std::ofstream f(trajectory_path);

    rw::trajectory::TimedStatePath tStatePath;
    
    double tk = 0;
    for(auto Q : path_pick){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }

    rw::kinematics::Kinematics::gripFrame(object_frame, frameTcp, state);
    tStatePath.push_back(TimedState(tk,state));
    tk += 0.01;

    path_points.erase(path_points.begin());
    path_points.push_back(place_path_points_TQ[0].second);
    std::vector<Q> path_pick_to_place = RRTConfigurations(path_points, robot_device, wc, state);

    for(auto Q : path_pick_to_place){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }

    rw::kinematics::Kinematics::gripFrame(object_frame, place_target_frame, state);
    tStatePath.push_back(TimedState(tk,state));
    tk += 0.01;

    path_points.erase(path_points.begin());
    path_points.push_back(default_q);
    std::vector<Q> path_place_to_default = RRTConfigurations(path_points, robot_device, wc, state);

    for(auto Q : path_place_to_default){
        robot_device->setQ(Q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk += 0.01;
    }



    std::string rwplay_path = std::string("../experiment_data/p3_3/rwplays/") + map + std::string("/")
                             + std::string("RRT_visu_s") + std::string("_trial") + std::to_string(trial)
                             + std::string(".rwplay");

    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplay_path);
    
    std::cout << "DONE" << std::endl;
    
}

void motion_planning_PB(const std::string &scene_path, const std::string& robot_device_name, rw::math::Transform3D<> &objectInitFrame, std::vector<Q> &pick_path_points_Q, std::vector<Q> &place_path_points_Q, std::string map, int trial = 1)
{
    Log::infoLog() << "Running motion planning..." << std::endl;

    //Loading workcell
    Log::infoLog() << "Loading workcell..." << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(scene_path);
    if(wc == nullptr)
    {
        Log::errorLog() << "Workcell could not be loaded from " << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Workcell could not be loaded from " + scene_path);
    }
    Log::infoLog() << "Workcell has been loaded succesfully" << std::endl;

    //Loading robot device
    Log::infoLog() << "Loading robot device from workcell..." << std::endl;
    SerialDevice::Ptr robot_device= wc->findDevice<SerialDevice>(robot_device_name);
    if(robot_device== nullptr)
    {
        Log::errorLog() << "Robot device named " << robot_device_name << 
            " could not be loaded from workcell at path." << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Robot device named " + robot_device_name  + " could not be loaded from workcell at path.");
    }
    Log::infoLog() << "Robot device found" << std::endl;

    Log::infoLog() << "Loading gripper..." << std::endl;
    const std::string gripper_device_name = "WSG50"; 
    TreeDevice::Ptr gripper_device = wc->findDevice<TreeDevice>(gripper_device_name);



    //Get relevant frame names
    const std::string gripper_tcp_name = gripper_device_name + "." + "TCP";
    const std::string object_name = "Bottle";

    //Get object and gripper frame
    MovableFrame* object_frame = wc->findFrame<MovableFrame>(object_name);

    rw::kinematics::Frame* frameTcp = wc->findFrame<Frame>(gripper_tcp_name);

    //Place target frame name
    const std::string place_target_name = "Place_Target";
    rw::kinematics::Frame* place_target_frame = wc->findFrame<Frame>(place_target_name);

    //Loading default state
    State state = wc->getDefaultState();    

    Q default_q = robot_device->getQ(state);

    //Move object to start frame
    object_frame->moveTo(objectInitFrame, state);

    double tf = 2;
    
    rw::trajectory::LinearInterpolator<Q> LIP_Pi_01(pick_path_points_Q[0], pick_path_points_Q[1], tf);
    rw::trajectory::LinearInterpolator<Q> LIP_Pi_12(pick_path_points_Q[1], pick_path_points_Q[2], tf);
    rw::trajectory::LinearInterpolator<Q> LIP_Pi_23(pick_path_points_Q[2], pick_path_points_Q[3], tf);


    rw::trajectory::LinearInterpolator<Q> LIP_Pi3_Pl3(pick_path_points_Q[3],  place_path_points_Q[3], tf);

    rw::trajectory::LinearInterpolator<Q> LIP_Pl_10(place_path_points_Q[1], place_path_points_Q[0], tf);
    rw::trajectory::LinearInterpolator<Q> LIP_Pl_21(place_path_points_Q[2], place_path_points_Q[1], tf);
    rw::trajectory::LinearInterpolator<Q> LIP_Pl_32(place_path_points_Q[3], place_path_points_Q[2], tf);

    rw::trajectory::LinearInterpolator<Q> LIP_Pi_3d(pick_path_points_Q[3], default_q, tf);
    rw::trajectory::LinearInterpolator<Q> LIP_Pl_d3(default_q, place_path_points_Q[3], tf);
    
    double blend_time = 0.5 * tf;
    std::vector<rw::trajectory::LinearInterpolator<Q>> reverse_pick_LIPs = {LIP_Pi_01, LIP_Pi_12, LIP_Pi_23, LIP_Pi_3d};
    std::vector<Q> pick = PBConfigurations(reverse_pick_LIPs, blend_time);
    std::reverse(pick.begin(), pick.end());

    std::vector<rw::trajectory::LinearInterpolator<Q>> pick_to_place_LIPs = {LIP_Pi_01, LIP_Pi_12, LIP_Pi_23, LIP_Pi3_Pl3, LIP_Pl_32,  LIP_Pl_21,  LIP_Pl_10};
    std::vector<Q> pick_to_place = PBConfigurations(pick_to_place_LIPs, blend_time);

    std::vector<rw::trajectory::LinearInterpolator<Q>> reverse_place_LIPs = {LIP_Pl_d3, LIP_Pl_32,  LIP_Pl_21,  LIP_Pl_10};
    std::vector<Q> place = PBConfigurations(reverse_place_LIPs, blend_time);
    std::reverse(place.begin(), place.end());

    std::vector<Q> path;
    path.insert(path.end(), pick.begin(), pick.end());
    path.insert(path.end(), pick_to_place.begin(), pick_to_place.end());
    path.insert(path.end(), place.begin(), place.end());    

    std::string trajectory_file_name = std::string("../experiment_data/p3_3/trajectories/") + map + std::string("/")
                             + std::string("PB_trajectory") + std::string("_trial") + std::to_string(trial)
                             + std::string(".csv");

    std::ofstream f(trajectory_file_name);
    //Visualize them and save them
    TimedStatePath tStatePath;
    double tk=0;
    for(auto q : pick)
    {
        robot_device->setQ(q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk+=0.01;
    }

    rw::kinematics::Kinematics::gripFrame(object_frame, frameTcp, state);
    tStatePath.push_back(TimedState(tk,state));
    tk += 0.01;

    for(auto q : pick_to_place)
    {
        robot_device->setQ(q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk+=0.01;
    }

    rw::kinematics::Kinematics::gripFrame(object_frame, place_target_frame, state);
    tStatePath.push_back(TimedState(tk,state));
    tk += 0.01;

    for(auto q : place)
    {
        robot_device->setQ(q, state);
        tStatePath.push_back(TimedState(tk,state));

        Transform3D<> wTf_grasp_tcp = frameTcp->wTf(state);
        Vector3D<> p = wTf_grasp_tcp.P();
        RPY<> rpy = RPY<>(wTf_grasp_tcp.R());
        for(int i = 0; i < p.size(); ++i)
        {
            f << p[i] << ",";
        }
        for(int i = 0; i < rpy.size(); ++i)
        {
            f << rpy[i] << ",";
        }
        f << tk << std::endl;

        tk+=0.01;
    }

    f.close();

    std::string rwplay_path = std::string("../experiment_data/p3_3/rwplays/") + map + std::string("/")
                             + std::string("PB_visu_s") + std::string("_trial") + std::to_string(trial)
                             + std::string(".rwplay");
    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplay_path);
   
    std::cout << "DONE" << std::endl;
    
}

void pick_n_place_analysis()
{
    const std::string project_path = "../";
    const std::string workcell_path = project_path + "workcell/";
    const std::string scene_path = workcell_path + "Scene.wc.xml";
    const std::string robot_device_name = "UR-6-85-5-A";

    const std::string workcell_path_w_obstacles = project_path + "Project_WorkCell_Obstacle/";
    const std::string scene_path_w_obstacles = workcell_path_w_obstacles + "Scene.wc.xml";

    
    rw::math::Transform3D<> bottleFrame1(rw::math::Vector3D<>(0, 0.474, 0.11), rw::math::RPY<>(-90*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad));
    rw::math::Transform3D<> bottleFrame2(rw::math::Vector3D<>(0.25, 0.474, 0.11), rw::math::RPY<>(-90*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad));
    rw::math::Transform3D<> bottleFrame3(rw::math::Vector3D<>(-0.25, 0.474, 0.11), rw::math::RPY<>(-90*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad));
    std::vector<rw::math::Transform3D<>> bottleFrames = {bottleFrame1, bottleFrame2, bottleFrame3};
    int bottlePos = 1;
    const int TRIAL_MAX = 50;
    const std::string map_no = "no_obstacles";
    const std::string map_wo = "with_obstacles";
    for(auto bottleFrame : bottleFrames)
    {
        
        std::cout << "---------------------------- Computing path points LIP and PB ----------------------------" << std::endl;
        std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Q>> path_points_no = get_path_points(scene_path, robot_device_name, bottleFrame);
        std::cout << "---------------------------- LIP bottle pos " << bottlePos << "----------------------------" << std::endl;
        motion_planning_LIP(scene_path, robot_device_name, bottleFrame, path_points_no.first, path_points_no.second, map_no, bottlePos);
        std::cout << "---------------------------- PB bottle pos " << bottlePos << "----------------------------" << std::endl;
        motion_planning_PB(scene_path, robot_device_name, bottleFrame, path_points_no.first, path_points_no.second, map_no, bottlePos);
        
        for(int i = 1; i <= TRIAL_MAX; ++i)
        {
            std::cout << "---------------------------- RRT bottle pos " << bottlePos << " trial " << i << "----------------------------" << std::endl;
            int trial = 100 * bottlePos + i;
            motion_planning_RRT(scene_path_w_obstacles, robot_device_name, bottleFrame, map_wo, trial);
        }
        
        
       std::cout << "---------------------------- Ending bottle ----------------------------" << std::endl;
       bottlePos += 1;
    }
}