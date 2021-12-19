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
#include "roviproblems/includes/p3_2.hpp"
#include "roviproblems/includes/p3_3.hpp"
#include "roviproblems/includes/p4_1_M3.hpp"

std::string project_path = "../";
std::string workcell_path = project_path + "workcell";

int main(int argc, char** argv)
{
    //load workcell
    //std::cout << "Loading workcell..." << std::endl;
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../workcell/Scene_ball.wc.xml");
    if(NULL == wc){
        RW_THROW("COULD NOT LOAD scene... check path!");
    }
    // get the default state
    rw::kinematics::State state = wc->getDefaultState();




    //--------------------- REACHABILITY ANALYSIS ----------------------//
    //simulateCollisionLandscape(20, wc);

    // Function below is for testing a single base position side or top grip, for either pick or place operation, replay is saved if last parameter is true
    //std::cout << reachabilityAnalysis_pick(0.0, 0.0, true, true, wc) << std::endl;
    //std::cout << reachabilityAnalysis_place(0.0, 0.0, true, true, wc) << std::endl;
    //------------------------------------------------------------------//


    //--------------------- P4_1_M3 ----------------------//
    // Generate 20 images that are going to be used for testing
    //std::vector<std::vector<double>> ground_truth = saveTestImages();
    
    // Run tests

    // Blur Tests
    /*
    for (int i = 0; i <= 200; i+=5) // Increasing the amount of times an average kernel of 3x3 is used on the image
    {
        std::cout << i << " Out Of: " << 200 << std::endl;
        triangulationTest(i, 1, 0.0, "BlurTest", ground_truth, wc, state);
    }
    */

    // Salt and Pepper (black and white) Tests
    /*
    for (int i = 0; i <= 100; i++) // Increasing the percentage of Salt and Pepper noise
    {
        std::cout << i << " Out Of: " << 100 << std::endl;
        triangulationTest(0, 2, i, "SPTest", ground_truth, wc, state);
    }
    */

    // Blur and Salt and Pepper (black and white) Tests
    /*
    for (int i = 0; i <= 100; i+=2) // Increasing the percentage of Salt and Pepper noise
    {
        std::cout << i << " Out Of: " << 100 << std::endl;
        triangulationTest(i, 0, i, "BlurSPTest", ground_truth, wc, state);
    }
    */

    //----------------------------------------------------//

    return 0;
}