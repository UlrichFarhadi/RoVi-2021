// std includes
#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <fstream>
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

using namespace rw::core;
using namespace rw::common;
using rw::graphics::SceneViewer;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;
using namespace rwlibs::simulation;
using namespace rws;

void addSaltAndPepperNoise(cv::Mat &image, double noise_percentage = 10)
{
    int image_rows = image.rows;
    int image_columns = image.cols;
    int image_channels = image.channels();
    int noise_points = (int) (((double) image_rows*
    image_columns*image_channels)*noise_percentage/100.0);
    for (int count = 0; count < noise_points; count++)
    {
        int row = rand() % image_rows;
        int column = rand() % image_columns;
        int bw = rand() % 2;
        cv::Vec3b & color = image.at<cv::Vec3b>(row,column);
        if (bw == 0)
        {
            color[0] = 255;
            color[1] = 255;
            color[2] = 255;
        }
        else
        {
            color[0] = 0;
            color[1] = 0;
            color[2] = 0;
        }
        
        //int channel = rand() % image_channels;
        //uchar* pixel = image.ptr<uchar>(row) + (column*image_channels) + channel;
        //*pixel = (rand()%2 == 1) ? 255 : 0;
    }
}


void printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr _wc, State _state) {
    Frame* cameraFrame = _wc->findFrame(frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width,height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0,
                  0, fovy_pixel, height / 2.0, 0,
                  0, 0, 1, 0;

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;


            rw::math::Transform3D<> camPosOGL = cameraFrame->wTf(_state);
            rw::math::Transform3D<> openGLToVis = rw::math::Transform3D<>(rw::math::RPY<>(-1 * rw::math::Pi, 0, rw::math::Pi).toRotation3D());
            rw::math::Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e() << std::endl;

        }
    }
}

// Function to find feature in both images (2D point to be used for triangulation)
std::vector<std::vector<int>> feature2DRightLeftImage(cv::Mat image_right, cv::Mat image_left)
{
    // Thresholding the colours to remove any shadows or light spots from the light source
    for (int row = 0; row < image_right.rows; row++)
    {
        for (int col = 0; col < image_right.cols; col++)
        {
            cv::Vec3b & color = image_right.at<cv::Vec3b>(row,col);
            if (color[0] >= 50 && color[1] <= 20 && color[2] <= 20)
            {
                color[0] = 255;
                color[1] = 0;
                color[2] = 0;
            }
        }
    }
    for (int row = 0; row < image_left.rows; row++)
    {
        for (int col = 0; col < image_left.cols; col++)
        {
            cv::Vec3b & color = image_left.at<cv::Vec3b>(row,col);
            if (color[0] >= 50 && color[1] <= 20 && color[2] <= 20)
            {
                color[0] = 255;
                color[1] = 0;
                color[2] = 0;
            }
        }
    }
    

    cv::Mat gray_right;
    cv::cvtColor(image_right, gray_right, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray_right, gray_right, 5);
    std::vector<cv::Vec3f> circles_right;
    cv::HoughCircles(gray_right, circles_right, cv::HOUGH_GRADIENT, 1,
                 50,  // change this value to detect circles with different distances to each other
                 40, 10, 1, 40 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );

    cv::Mat gray_left;
    cv::cvtColor(image_left, gray_left, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray_left, gray_left, 5);
    std::vector<cv::Vec3f> circles_left;
    cv::HoughCircles(gray_left, circles_left, cv::HOUGH_GRADIENT, 1,
                 50,  // change this value to detect circles with different distances to each other
                 40, 10, 1, 40 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    cv::Vec3i c_r = circles_right[0];
    cv::Vec3i c_l = circles_left[0];

    std::cout << c_r[2] << " " << c_l[2] << std::endl;
    std::vector<int> right_feature = {std::round(c_r[1]), std::round(c_r[0])};
    std::vector<int> left_feature = {std::round(c_l[1]), std::round(c_l[0])};


    std::vector<std::vector<int>> features = {right_feature, left_feature};
    return features;
}

void storeImages(cv::Mat &img_right, cv::Mat &img_left, std::string image_right, std::string image_left)
{
    //getImageFromCameras();
    cv::Mat img_right_file = cv::imread(image_right);
    cv::Mat img_left_file = cv::imread(image_left);
    img_right_file.cv::Mat::copyTo(img_right);
    img_left_file.cv::Mat::copyTo(img_left);
}

void getImageFromCameras(std::vector<std::vector<double>> &ground_truth)
{
    {
    static const std::string WC_FILE = "../workcell/Scene.wc.xml";
    const WorkCell::Ptr wc = WorkCellLoader::Factory::load (WC_FILE);
    if (wc.isNull ())
        RW_THROW ("WorkCell could not be loaded.");
    Frame* const camera_right= wc->findFrame ("Camera_Right");
    Frame* const camera_left = wc->findFrame ("Camera_Left");
    //if (camera == nullptr)
    //   RW_THROW ("Camera frame could not be found.");
    const PropertyMap& properties_right = camera_right->getPropertyMap ();
    const PropertyMap& properties_left = camera_left->getPropertyMap ();
    //if (!properties.has ("Camera"))
    //    RW_THROW ("Camera frame does not have Camera property.");
    const std::string parameters_right = properties_right.get< std::string > ("Camera");
    const std::string parameters_left = properties_left.get< std::string > ("Camera");
    std::istringstream iss_right (parameters_right, std::istringstream::in);
    std::istringstream iss_left (parameters_left, std::istringstream::in);
    double fovy_right, fovy_left;
    int width_right, width_left;
    int height_right, height_left;
    iss_right >> fovy_right >> width_right >> height_right;
    iss_left >> fovy_left >> width_left >> height_left;
    //std::cout << "Camera properties: fov " << fovy << " width " << width << " height " << height
    //          << std::endl;

    rw::kinematics::MovableFrame::Ptr bottleFrame = wc->findFrame<rw::kinematics::MovableFrame>("Ball");
    if(NULL == bottleFrame){
        RW_THROW("COULD not find movable frame Ball ... check model");
    }

    // Ditribution for different bottle spawning points
    // Upper Left corner: x = 0.35, y = 0.36
    // Upper right corner: x = -0.35, y = 0.53
    std::default_random_engine generator(40);
    std::uniform_real_distribution<double> dist_x(-0.35, 0.35);
    std::uniform_real_distribution<double> dist_y(0.36, 0.53);
    std::uniform_real_distribution<double> dist_z(0.151, 0.232);

    State state = wc->getDefaultState();
    
    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        //rwstudio->postOpenWorkCell (WC_FILE);
        rwstudio->postWorkCell(wc);
        //rwstudio->
        rwstudio->setState(state);
        TimerUtil::sleepMs (5000);

        for (int i = 0; i < 20; i++)
        //while(1)
        {
            double bottlepos_x = dist_x(generator);
            double bottlepos_y = dist_y(generator);
            double bottlepos_z = dist_z(generator);
            std::vector<double> gt_pos = {bottlepos_x, bottlepos_y, bottlepos_z};
            ground_truth.push_back(gt_pos);
            //std::cout << i << " Current Bottle Pos" << bottlepos_x << " " << bottlepos_y << std::endl;
            // Moving robot base:
            rw::math::Vector3D<> bottle_T = rw::math::Vector3D<>(bottlepos_x, bottlepos_y, bottlepos_z);
            rw::math::RPY<> bottle_R(-90*rw::math::Deg2Rad, 0, 90*rw::math::Deg2Rad);
            rw::math::Transform3D<> bottle_transform(bottle_T, bottle_R);
            bottleFrame->moveTo(bottle_transform, state);
            rwstudio->setState(state);
            {
            const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
            const GLFrameGrabber::Ptr framegrabber =
                ownedPtr (new GLFrameGrabber (width_right, height_right, fovy_right));
            framegrabber->init (gldrawer);
            SimulatedCamera::Ptr simcam =
                ownedPtr (new SimulatedCamera ("SimulatedCamera", fovy_right, camera_right, framegrabber));
            simcam->setFrameRate (100);
            simcam->initialize ();
            simcam->start ();
            simcam->acquire ();

            static const double DT = 0.001;
            const Simulator::UpdateInfo info (DT);
            //State state = wc->getDefaultState ();
            int cnt     = 0;
            const Image* img;
            while (!simcam->isImageReady ()) {
                //std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
                simcam->update (info, state);
                cnt++;
            }
            img = simcam->getImage ();
            std::string imgname_r = "ImageRight" + std::to_string(i) + ".ppm";
            img->saveAsPPM (imgname_r);
            // simcam->acquire ();
            // while (!simcam->isImageReady ()) {
            //     std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            //     simcam->update (info, state);
            //     cnt++;
            // }
            // std::cout << "Took " << cnt << " steps" << std::endl;
            // img = simcam->getImage ();
            // std::cout << "Image: " << img->getWidth () << "x" << img->getHeight () << " bits "
            //           << img->getBitsPerPixel () << " channels " << img->getNrOfChannels ()
            //           << std::endl;
            // img->saveAsPPM ("Image2.ppm");
            simcam->stop ();
            }

            {
            const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
            const GLFrameGrabber::Ptr framegrabber =
                ownedPtr (new GLFrameGrabber (width_left, height_left, fovy_left));
            framegrabber->init (gldrawer);
            SimulatedCamera::Ptr simcam =
                ownedPtr (new SimulatedCamera ("SimulatedCamera", fovy_left, camera_left, framegrabber));
            simcam->setFrameRate (100);
            simcam->initialize ();
            simcam->start ();
            simcam->acquire ();

            static const double DT = 0.001;
            const Simulator::UpdateInfo info (DT);
            //State state = wc->getDefaultState ();
            int cnt     = 0;
            const Image* img;
            while (!simcam->isImageReady ()) {
                //std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
                simcam->update (info, state);
                cnt++;
            }
            img = simcam->getImage ();
            std::string imgname_l = "ImageLeft" + std::to_string(i) + ".ppm";
            img->saveAsPPM (imgname_l);
            simcam->stop ();
            // REMEMBER TO SAVE THE GROUND TRUTH POSITION AS WELL
            }

        }
        app.close ();
    }
    RWS_END ()
    }
    std::cout << "Images Loaded" << std::endl;
}
