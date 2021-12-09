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

// OpenCV Includes
#include <opencv4/opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// RoVi includes
#include "../includes/p4_1_M3.hpp"
#include "../includes/rw_image_handler.hpp"


int test()
{
    //getImageFromCameras();


    cv::Mat right_image;
    cv::Mat left_image;
    storeImages(right_image, left_image);
    /*
    cv::Mat right_image_blur;
    cv::Mat left_image_blur;

    cv::blur(right_image, right_image_blur, cv::Size(10, 10));
    cv::blur(left_image, left_image_blur, cv::Size(10, 10));
    std::vector<std::vector<int>> RL_features_blur = feature2DRightLeftImage(right_image_blur, left_image_blur);
    std::cout << "Right_blur x,y: " << RL_features_blur[0][1] << "," << RL_features_blur[0][0] << std::endl;
    std::cout << "Left_blur x,y: " << RL_features_blur[1][1] << "," << RL_features_blur[1][0] << std::endl;
    cv::circle(right_image_blur, cv::Point(RL_features_blur[0][1], RL_features_blur[0][0]), 1, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::circle(left_image_blur, cv::Point(RL_features_blur[1][1], RL_features_blur[1][0]), 1, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::imshow("Right Blur", right_image_blur);
    cv::imshow("Left Blur", left_image_blur);
    */

    std::vector<std::vector<int>> RL_features = feature2DRightLeftImage(right_image, left_image);
    std::cout << "Right x,y: " << RL_features[0][1] << "," << RL_features[0][0] << std::endl;
    std::cout << "Left x,y: " << RL_features[1][1] << "," << RL_features[1][0] << std::endl;
    cv::circle(right_image, cv::Point(RL_features[0][1], RL_features[0][0]), 1, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::circle(left_image, cv::Point(RL_features[1][1], RL_features[1][0]), 1, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::imshow("Right_Image", right_image);
    cv::imshow("Left_Image", left_image);

    cv::waitKey(0);

    //load workcell
    std::cout << "Loading workcell..." << std::endl;
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../workcell/Scene.wc.xml");
    if(NULL == wc){
        RW_THROW("COULD NOT LOAD scene... check path!");
    }
    // get the default state
    State state = wc->getDefaultState();
    rw::kinematics::Frame* frame_world = wc->findFrame("Table");
    rw::kinematics::Frame* frame_camera_right = wc->findFrame("Camera_Right");
    rw::kinematics::Frame* frame_camera_left = wc->findFrame("Camera_Left");

    //auto RW_bad_programming = rw::math::RPY<>(0.0, M_PI, M_PI + (rw::math::Deg2Rad *-25));
    auto RW_bad_programming = rw::math::RPY<>(0.0, 0.0, M_PI - (rw::math::Deg2Rad *-25));
    rw::math::Transform3D<> camera_right_extrinsics = rw::kinematics::Kinematics::frameTframe(frame_world, frame_camera_right, state);
    rw::math::Transform3D<> camera_left_extrinsics = rw::kinematics::Kinematics::frameTframe(frame_world, frame_camera_left, state);
    
    auto R_cams = rw::math::RPY<>(0.0, 0.0, M_PI - (rw::math::Deg2Rad *-25));
    rw::math::Vector3Dd d_right(-0.10, 0.82, 1.00);
    rw::math::Vector3Dd d_left(0.10, 0.82, 1.00);

    camera_right_extrinsics = rw::math::Transform3D<>(d_right, R_cams);
    camera_left_extrinsics = rw::math::Transform3D<>(d_left, R_cams);
    //camera_right_extrinsics = rw::math::Transform3D<>(camera_right_extrinsics.P(), RW_bad_programming);
    //camera_left_extrinsics = rw::math::Transform3D<>(camera_left_extrinsics.P(), RW_bad_programming);
    std::cout << camera_right_extrinsics << std::endl;
    std::cout << camera_left_extrinsics << std::endl;
    //std::cout << camera_right_extrinsics << std::endl;

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
    
    std::cout << "fov: " << fovy_pixel << std::endl;

    Eigen::Matrix<double, 3, 4> camera_intrinsics;
    camera_intrinsics << fovy_pixel, 0, width_right / 2.0, 0,
            0, fovy_pixel, height_right / 2.0, 0,
            0, 0, 1, 0;


    std::cout << camera_intrinsics << std::endl;
    std::cout << camera_right_extrinsics.e() << std::endl;
    std::cout << camera_left_extrinsics.e() << std::endl;

    Eigen::Matrix<double, 3, 4> P_camera_right = camera_intrinsics * camera_right_extrinsics.e();
    Eigen::Matrix<double, 3, 4> P_camera_left = camera_intrinsics * camera_left_extrinsics.e();

    std::cout << P_camera_right << std::endl;
    std::cout << P_camera_left << std::endl;

    // Triangulate
    cv::Mat P_mat_R;
    cv::Mat P_mat_L;
    cv::eigen2cv(P_camera_right, P_mat_R);
    cv::eigen2cv(P_camera_left, P_mat_L);
    std::cout << P_mat_R << std::endl;
    std::cout << P_mat_L << std::endl;

    cv::Mat points4D(4, 1, CV_64F);
    cv::Mat points_right(2, 1, CV_64F);
    cv::Mat points_left(2, 1, CV_64F);
    points_right.at<double>(0,0) = RL_features[0][1];
    points_right.at<double>(1,0) = RL_features[0][0];
    points_left.at<double>(0,0) = RL_features[1][1];
    points_left.at<double>(1,0) = RL_features[1][0];
    std::cout << points_right << std::endl;
    std::cout << points_left << std::endl;
    cv::triangulatePoints(P_mat_R, P_mat_L, points_right, points_left, points4D);
    points4D = points4D / points4D.at<double>(3,0);
    std::cout << points4D << std::endl;

    


    return 0;
}


