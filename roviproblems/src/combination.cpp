// std includes
#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <fstream>
#include <numeric>
#include <cmath>

// Robworks includes
#include <rw/rw.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/trajectory/Timed.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/Path.hpp>

// OpenCV Includes
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// RoVi includes
#include "../includes/rw_image_handler.hpp"
#include "../includes/rw_camera.hpp"


USE_ROBWORK_NAMESPACE
using namespace robwork;

rw::math::Transform3D<> rwTriangulatePoints(cv::Mat &image_right, cv::Mat &image_left, int noise_iterations, int blur_or_saltpepper, double salt_and_pepper_percentage)
{
    cv::Mat right_image;
    cv::Mat left_image;
    image_right.copyTo(right_image);
    image_left.copyTo(left_image);

    // Add noise if nessesary
    if(blur_or_saltpepper == 1)
    {
        for (int j = 0; j < noise_iterations; j++)
        {
            cv::blur(right_image, right_image, cv::Size(3, 3));
            cv::blur(left_image, left_image, cv::Size(3, 3));
        }
    }
    if(blur_or_saltpepper == 2)
    {
        addSaltAndPepperNoise(right_image, salt_and_pepper_percentage);
        addSaltAndPepperNoise(left_image, salt_and_pepper_percentage);
    }

    std::vector<std::vector<int>> RL_features = feature2DRightLeftImage(right_image, left_image);

    if(true)
    {
        cv::circle(right_image, cv::Point(RL_features[0][1], RL_features[0][0]), 2, cv::Scalar(0, 255, 0), cv::FILLED);
        cv::circle(left_image, cv::Point(RL_features[1][1], RL_features[1][0]), 2, cv::Scalar(0, 255, 0), cv::FILLED);
        cv::imshow("Right_Image", right_image);
        cv::imshow("Left_Image", left_image);

        //cv::waitKey(200);
        cv::waitKey(0);
    }

    //cv::waitKey(0);

    //load workcell
    //std::cout << "Loading workcell..." << std::endl;
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../workcell/Scene.wc.xml");
    if(NULL == wc){
        RW_THROW("COULD NOT LOAD scene... check path!");
    }
    // get the default state
    State state = wc->getDefaultState();
    rw::kinematics::Frame* frame_world = wc->findFrame("Table");
    rw::kinematics::Frame* frame_camera_right = wc->findFrame("Camera_Right");
    rw::kinematics::Frame* frame_camera_left = wc->findFrame("Camera_Left");

    // Calculate the camera parameters (Formulas from RoViSamplePlugin)
    rw::math::Transform3D<> camPosOGL_r = frame_camera_right->wTf(state);
    rw::math::Transform3D<> openGLToVis_r = rw::math::Transform3D<>(rw::math::RPY<>(-1 * rw::math::Pi, 0, rw::math::Pi).toRotation3D());
    rw::math::Transform3D<> camera_right_extrinsics = rw::math::inverse(camPosOGL_r * rw::math::inverse(openGLToVis_r));
    rw::math::Transform3D<> camPosOGL_l = frame_camera_left->wTf(state);
    rw::math::Transform3D<> openGLToVis_l = rw::math::Transform3D<>(rw::math::RPY<>(-1 * rw::math::Pi, 0, rw::math::Pi).toRotation3D());
    rw::math::Transform3D<> camera_left_extrinsics = rw::math::inverse(camPosOGL_l * rw::math::inverse(openGLToVis_l));

    // Calculate the focal length
    rw::kinematics::Frame* const camera_right= wc->findFrame ("Camera_Right");
    if (camera_right == nullptr)
        RW_THROW ("Camera frame could not be found.");
    const PropertyMap& properties_right = camera_right->getPropertyMap ();
    if (!properties_right.has ("Camera"))
        RW_THROW ("Camera frame does not have Camera property.");
    const std::string parameters_right = properties_right.get< std::string > ("Camera");
    std::istringstream iss_right (parameters_right, std::istringstream::in);
    double fovy_right;
    int width_right;
    int height_right;
    iss_right >> fovy_right >> width_right >> height_right;
    double fovy_pixel = height_right / 2.0 / std::tan(fovy_right * (2*M_PI) / 360.0 / 2.0 );

    // Cal
    Eigen::Matrix<double, 3, 4> camera_intrinsics;
    camera_intrinsics << fovy_pixel, 0, width_right / 2.0, 0,
            0, fovy_pixel, height_right / 2.0, 0,
            0, 0, 1, 0;

    Eigen::Matrix<double, 3, 4> P_camera_right = camera_intrinsics * camera_right_extrinsics.e();
    Eigen::Matrix<double, 3, 4> P_camera_left = camera_intrinsics * camera_left_extrinsics.e();

    // Triangulate
    cv::Mat P_mat_R;
    cv::Mat P_mat_L;
    cv::eigen2cv(P_camera_right, P_mat_R);
    cv::eigen2cv(P_camera_left, P_mat_L);

    cv::Mat points4D(4, 1, CV_64F);
    cv::Mat points_right(2, 1, CV_64F);
    cv::Mat points_left(2, 1, CV_64F);
    points_right.at<double>(0,0) = RL_features[0][1];
    points_right.at<double>(1,0) = RL_features[0][0];
    points_left.at<double>(0,0) = RL_features[1][1];
    points_left.at<double>(1,0) = RL_features[1][0];

    cv::triangulatePoints(P_mat_R, P_mat_L, points_left, points_right, points4D);
    points4D = points4D / points4D.at<double>(3,0);
    //std::cout << points4D << std::endl;
    points4D.at<double>(0,0) *= -1; // Because the image is mirrored.
    //double d1 = points4D.at<double>(0,0) - ground_truth[i][0];
    //double d2 = points4D.at<double>(1,0) - ground_truth[i][1];
    //double d3 = points4D.at<double>(2,0) - ground_truth[i][2];
    //double error = std::sqrt(d1*d1 + d2*d2 + d3*d3);
    //std::cout << "error: " << error << " _ " << d1 << "," << d2 << "," << d3 << std::endl;

    Transform3D<> objectPose(Vector3D<>(points4D.at<double>(0,0), points4D.at<double>(1,0), points4D.at<double>(2,0)), RPY<>(0,0,0));
    return objectPose;
}




//////////////////////////////////////////////ROBWORKS///////////////////////////
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

            Transform3D<> T(T_prev * p, Rotation3D<>(RPY<>(0, 0, R * Deg2Rad)) * goalFrame_wTF.R());
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

void motion_planning_RRT(const std::string &scene_path, const std::string& robot_device_name, State state, rw::math::Transform3D<> &objectInitFrame, std::string map, int trial=1)
{
    Log::infoLog() << "Running motion planning..." << std::endl;
    std::cout << "Running motion planning cout" << std::endl;

    //Loading workcell
    Log::infoLog() << "Loading workcell..." << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(scene_path);
    std::cout << "Loading workcell" << std::endl;
    if(wc == nullptr)
    {
        Log::errorLog() << "Workcell could not be loaded from " << scene_path << std::endl;
        Log::infoLog() << "Motion planning closing..." << std::endl;
        throw("Workcell could not be loaded from " + scene_path);
    }
    Log::infoLog() << "Workcell has been loaded succesfully" << std::endl;
    std::cout << "Workcell loaded" << std::endl;

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
    const std::string object_name = "Ball";

    //Get object and gripper frame
    MovableFrame* object_frame = wc->findFrame<MovableFrame>(object_name);

    rw::kinematics::Frame* frameTcp = wc->findFrame<Frame>(gripper_tcp_name);

    //Place target frame name
    const std::string place_target_name = "Place_Target";
    rw::kinematics::Frame* place_target_frame = wc->findFrame<Frame>(place_target_name); 

    Q default_q = robot_device->getQ(state);

    //Move object to start frame
    object_frame->moveTo(objectInitFrame, state);

    std::vector<Vector3D<>> goal_pos = {Vector3D<>(0,0,0)};
    std::vector<std::pair<Transform3D<>, Q>> pick_path_points_TQ = compute_path_points(0, 360, goal_pos, object_name, gripper_tcp_name, robot_device, wc, state);
    std::vector<std::pair<Transform3D<>, Q>> place_path_points_TQ = compute_path_points(0, 360, goal_pos, place_target_name, gripper_tcp_name, robot_device, wc, state);
    std::cout << pick_path_points_TQ.size() << std::endl;
    std::cout << place_path_points_TQ.size() << std::endl;
    std::vector<Q> path_points;
    path_points.push_back(default_q);
    path_points.push_back(pick_path_points_TQ[0].second);
    
    Log::infoLog() << "Computing RRT configurations..." << std::endl;
    std::cout << path_points.size() << std::endl;
    std::vector<Q> path_pick = RRTConfigurations(path_points, robot_device, wc, state);
    Log::infoLog() << "RRT configurations computed." << std::endl;
    
    std::string trajectory_path = std::string("../experiment_data/combination/trajectories/") + map + std::string("/")
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



    std::string rwplay_path = std::string("../experiment_data/combination/rwplays/") + map + std::string("/")
                             + std::string("RRT_visu_s") + std::string("_trial") + std::to_string(trial)
                             + std::string(".rwplay");

    rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplay_path);
    
    std::cout << "DONE" << std::endl;
    
}

///////////////COMBINATION///////////////////

void testCombined()
{
    // Load Workcell, Robot and Map
    std::string scene_path = "../Project_WorkCell_Obstacle/Scene_ball.wc.xml";
    std::string robot_device_name = "UR-6-85-5-A";
    std::string map = "with_obstacles";
    int trial = 1;

    
    WorkCell::Ptr wc = WorkCellLoader::Factory::load(scene_path);
    State state = wc->getDefaultState();

    std::random_device rd;  // non-deterministic generator
    std::mt19937 gen(rd()); // to seed mersenne twister.

    std::default_random_engine generator(gen());
    std::uniform_real_distribution<double> dist_x(-0.35, 0.35);
    std::uniform_real_distribution<double> dist_y(0.36, 0.53);
    std::uniform_real_distribution<double> dist_z(0.151, 0.232);
    std::uniform_real_distribution<double> r_dist(0, 90);
    std::uniform_real_distribution<double> p_dist(0, 90);
    std::uniform_real_distribution<double> y_dist(0, 90);

    rw::kinematics::MovableFrame::Ptr ballFrame = wc->findFrame<rw::kinematics::MovableFrame>("Ball");
    if(NULL == ballFrame){
        RW_THROW("COULD not find movable frame Ball ... check model");
    }

    for(int i = 0; i < 5; i++)
    {
        trial += i;
        double pos_x = dist_x(generator);
        double pos_y = dist_y(generator);
        double pos_z = dist_z(generator);
        double r_pos = dist_x(generator);
        double p_pos = dist_y(generator);
        double y_pos = dist_z(generator);
        
        // Moving robot base:
        rw::math::Vector3D<> bottle_T = rw::math::Vector3D<>(pos_x, pos_y, pos_z);
        rw::math::RPY<> bottle_R(r_pos*rw::math::Deg2Rad, p_pos*rw::math::Deg2Rad, y_pos*rw::math::Deg2Rad);
        rw::math::Transform3D<> bottle_transform(bottle_T, bottle_R);
        ballFrame->moveTo(bottle_transform, state);
        
        std::cout << "Capturing images..." << std::endl;
        generateImages(wc, state);
        cv::waitKey(10000); // Wait 10 seconds to make sure the application closes
        std::cout << "Images captured" << std::endl;

        // Save the images as cv::Mat objects
        cv::Mat img_right = cv::imread("../experiment_data/combination/images/Camera_right.ppm");
        cv::Mat img_left = cv::imread("../experiment_data/combination/images/Camera_left.ppm");
        std::cout << "Images loaded to OpenCV structures" << std::endl;
        std::cout << "img_right dims: " << img_right.cols << "x" << img_right.rows << std::endl;
        std::cout << "img_left dims: " << img_left.cols << "x" << img_left.rows << std::endl;

        // Get estimated pose of the 3D object in world coordinates by triangulating features in the left and right images
        // Camera parameters are automatically calculated from the information in the workcell
        // Noise can be added here as Gaussian blur or Black and white Salt and Pepper.
        rw::math::Transform3D<> triangulatedObjectPose = rwTriangulatePoints(img_right, img_left, 0, 0, 0.0);
        std::cout << "Object pose computed" << std::endl;

        std::cout << triangulatedObjectPose << std::endl;

        // Do motion planning with RRT to create a path from the object to the place position.
        std::cout << "Starting motion planning" << std::endl;
        motion_planning_RRT(scene_path, robot_device_name, state, triangulatedObjectPose, map, trial);

    }
    
        
    // Get images from the left and right camera
    //generateImages(wc, state);
    


}
