#ifndef RW_IMAGE_HANDLER_H
#define RW_IMAGE_HANDLER_H

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


void getImageFromCameras_test(std::vector<std::vector<double>> &ground_truth)
{
    {
    static const std::string WC_FILE = "../workcell/Scene_ball.wc.xml";
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
            std::string imgname_r = "../experiment_data/combination/images/ImageRight" + std::to_string(i) + ".ppm";
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
            std::string imgname_l = "../experiment_data/combination/images/ImageLeft" + std::to_string(i) + ".ppm";
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


std::vector<std::vector<double>> saveTestImages()
{
    std::vector<std::vector<double>> ground_truth;
    getImageFromCameras_test(ground_truth);
    return ground_truth;
}



#endif