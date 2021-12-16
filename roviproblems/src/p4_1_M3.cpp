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


int triangulationTest(int noise_iterations, int blur_or_saltpepper, double salt_and_pepper_percentage, std::string filename) // Blur = 1, salt and pepper = 2
{
    
    std::vector<std::vector<double>> ground_truth;
    // Generate 20 images that are going to be used for testing (Only needs to be done 1 time)
    getImageFromCameras(ground_truth);

    //std::cout << "Ground Truth for: " << ground_truth[2][0] << " , " << ground_truth[2][1] << std::endl;

    std::string filename_combined = "../experiment_data/p4_1_M3/" + filename + ".csv";
    std::ofstream datafile(filename_combined);
    for(int i = 0; i < 20; i++)
    {
        std::cout << "Image: " << i << std::endl;
        cv::Mat right_image;
        cv::Mat left_image;
        std::string i_r_str = "ImageRight" + std::to_string(i) + ".ppm";
        std::string i_l_str = "ImageLeft" + std::to_string(i) + ".ppm";
        storeImages(right_image, left_image, i_r_str, i_l_str);

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
        //std::cout << "Right x,y: " << RL_features[0][1] << "," << RL_features[0][0] << std::endl;
        //std::cout << "Left x,y: " << RL_features[1][1] << "," << RL_features[1][0] << std::endl;
        //cv::circle(right_image, cv::Point(RL_features[0][1], RL_features[0][0]), 1, cv::Scalar(0, 255, 0), cv::FILLED);
        //cv::circle(left_image, cv::Point(RL_features[1][1], RL_features[1][0]), 1, cv::Scalar(0, 255, 0), cv::FILLED);
        //cv::imshow("Right_Image", right_image);
        //cv::imshow("Left_Image", left_image);
        if(true)
        {
            cv::circle(right_image, cv::Point(RL_features[0][1], RL_features[0][0]), 30, cv::Scalar(0, 255, 0), 2);
            cv::circle(left_image, cv::Point(RL_features[1][1], RL_features[1][0]), 30, cv::Scalar(0, 255, 0), 2);
            cv::circle(right_image, cv::Point(RL_features[0][1], RL_features[0][0]), 2, cv::Scalar(0, 255, 0), cv::FILLED);
            cv::circle(left_image, cv::Point(RL_features[1][1], RL_features[1][0]), 2, cv::Scalar(0, 255, 0), cv::FILLED);
            cv::imshow("Right_Image", right_image);
            cv::imshow("Left_Image", left_image);

            cv::waitKey(200);
            //cv::waitKey(0);
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

        //printProjectionMatrix("Camera_Right", wc, state);
        //printProjectionMatrix("Camera_Left", wc, state);

        rw::math::Transform3D<> camPosOGL_r = frame_camera_right->wTf(state);
        rw::math::Transform3D<> openGLToVis_r = rw::math::Transform3D<>(rw::math::RPY<>(-1 * rw::math::Pi, 0, rw::math::Pi).toRotation3D());
        rw::math::Transform3D<> camera_right_extrinsics = rw::math::inverse(camPosOGL_r * rw::math::inverse(openGLToVis_r));
        rw::math::Transform3D<> camPosOGL_l = frame_camera_left->wTf(state);
        rw::math::Transform3D<> openGLToVis_l = rw::math::Transform3D<>(rw::math::RPY<>(-1 * rw::math::Pi, 0, rw::math::Pi).toRotation3D());
        rw::math::Transform3D<> camera_left_extrinsics = rw::math::inverse(camPosOGL_l * rw::math::inverse(openGLToVis_l));

        //std::cout << camera_right_extrinsics << std::endl;
        //std::cout << camera_left_extrinsics << std::endl;


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

        //std::cout << "fov: " << fovy_pixel << std::endl;

        Eigen::Matrix<double, 3, 4> camera_intrinsics;
        camera_intrinsics << fovy_pixel, 0, width_right / 2.0, 0,
                0, fovy_pixel, height_right / 2.0, 0,
                0, 0, 1, 0;


        //std::cout << camera_intrinsics << std::endl;
        //std::cout << camera_right_extrinsics.e() << std::endl;
        //std::cout << camera_left_extrinsics.e() << std::endl;

        Eigen::Matrix<double, 3, 4> P_camera_right = camera_intrinsics * camera_right_extrinsics.e();
        Eigen::Matrix<double, 3, 4> P_camera_left = camera_intrinsics * camera_left_extrinsics.e();

        //std::cout << P_camera_right << std::endl;
        //std::cout << P_camera_left << std::endl;

        // Triangulate
        cv::Mat P_mat_R;
        cv::Mat P_mat_L;
        cv::eigen2cv(P_camera_right, P_mat_R);
        cv::eigen2cv(P_camera_left, P_mat_L);
        //std::cout << P_mat_R << std::endl;
        //std::cout << P_mat_L << std::endl;

        cv::Mat points4D(4, 1, CV_64F);
        cv::Mat points_right(2, 1, CV_64F);
        cv::Mat points_left(2, 1, CV_64F);
        points_right.at<double>(0,0) = RL_features[0][1];
        points_right.at<double>(1,0) = RL_features[0][0];
        points_left.at<double>(0,0) = RL_features[1][1];
        points_left.at<double>(1,0) = RL_features[1][0];
        //std::cout << points_right << std::endl;
        //std::cout << points_left << std::endl;
        cv::triangulatePoints(P_mat_R, P_mat_L, points_left, points_right, points4D);
        points4D = points4D / points4D.at<double>(3,0);
        //std::cout << points4D << std::endl;
        points4D.at<double>(0,0) *= -1; // Because the image is mirrored.
        double d1 = points4D.at<double>(0,0) - ground_truth[i][0];
        double d2 = points4D.at<double>(1,0) - ground_truth[i][1];
        double d3 = points4D.at<double>(2,0) - ground_truth[i][2];
        double error = std::sqrt(d1*d1 + d2*d2);
        //std::cout << "error: " << error << " --> GT:" << ground_truth[i][0] << "," << ground_truth[i][1] << " _ TRI: " << points4D.at<double>(0,0) << "," << points4D.at<double>(1,0) << std::endl;
        std::cout << error << std::endl;
        datafile << error << "\n";
    }

    return 0;
}


