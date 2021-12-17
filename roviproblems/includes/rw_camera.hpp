
// std includes
#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <fstream>

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

class Camera
{
    public:
    Camera(WorkCell::Ptr wc, std::string camera_name, RobWorkStudio* rwstudio)
    {
        Frame* camera_frame = wc->findFrame(camera_name);
        if(camera_frame == nullptr)
            std::cout << "Could not find camera frame" << std::endl;
        const PropertyMap& properties = camera_frame->getPropertyMap();
        const std::string parameters = properties.get< std::string > ("Camera");
        
        std::istringstream iss (parameters, std::istringstream::in);
        double fovy;
        int width;
        int height;
        iss >> fovy >> width >> height;

        const SceneViewer::Ptr gldrawer = rwstudio->getView ()->getSceneViewer ();
        const GLFrameGrabber::Ptr framegrabber = ownedPtr (new GLFrameGrabber (width, height, fovy));
        bool is_framegrabber = framegrabber->init (gldrawer);
        if(!is_framegrabber)
            std::cout << "Framegravver was not initiated" << std::endl;
        
        simcam = ownedPtr (new SimulatedCamera ("SimulatedCamera", fovy, camera_frame, framegrabber));
        simcam->setFrameRate(100);
        bool is_simcam_initiated = simcam->initialize ();
        if(!is_simcam_initiated)
            std::cout << "Simcam was not initiated" << std::endl;
    }

    void get_image(State state, std::string filename)
    {
        simcam->start();
        simcam->acquire ();

        double DT = 0.001;
        const Simulator::UpdateInfo info (DT);
        const Image* img;
        while (!simcam->isImageReady ()) {
            //std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcam->update (info, state);
        }
        img = simcam->getImage ();    
        img->saveAsPPM (filename + ".ppm");
        simcam->stop();
    }

    private:
    SimulatedCamera::Ptr simcam;
};

void ppmToMat(cv::Mat &img_right, cv::Mat &img_left, std::string image_right, std::string image_left)
{
    cv::Mat img_right_file = cv::imread(image_right);
    cv::Mat img_left_file = cv::imread(image_left);
    img_right_file.cv::Mat::copyTo(img_right);
    img_left_file.cv::Mat::copyTo(img_left);
}

void generateImages(WorkCell::Ptr wc, State state)
{
    {
        RobWorkStudioApp app ("");
        RWS_START(app)
        {
            std::cout << "App started" << std::endl;
            RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
            //rwstudio->postOpenWorkCell (WC_FILE);
            rwstudio->postWorkCell(wc);
            //rwstudio->
            rwstudio->setState(state);
            TimerUtil::sleepMs (5000);
            std::cout << "Timer util started" << std::endl;

            Camera camera_right(wc, "Camera_Right", rwstudio);
            Camera camera_left(wc, "Camera_Left", rwstudio);

            camera_right.get_image(state, "../experiment_data/p4_dense/images/Camera_right");
            camera_left.get_image(state, "../experiment_data/p4_dense/images/Camera_left");            
            
            app.close ();
        }
        RWS_END ()
    }   
}

rw::math::Transform3D<> getProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr _wc, State _state) {
    rw::math::Transform3D<> H;
    Frame* cameraFrame = _wc->findFrame(frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            rw::math::Transform3D<> camPosOGL = cameraFrame->wTf(_state);
            rw::math::Transform3D<> openGLToVis = rw::math::Transform3D<>(rw::math::RPY<>(-1 * rw::math::Pi, 0, rw::math::Pi).toRotation3D());
            H = inverse(camPosOGL * openGLToVis);

        }
    }
    return H;
}