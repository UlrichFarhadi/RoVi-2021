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
    // Function below is for simulating the colision landscape writing to .csv files that can be read in the folder /experiment_data/p3_2
    // In that file there wil also be a matlab file p3_2_Contourplot.m that plots the heatmaps from the .csv files. It is dynamic so should be able to run as is no matter
    // the parameter beow. The parameter below (20) means you create a 21x21 grid equalling 441 base positions, this is quite a lot to test for so it can be changed lower
    //simulateCollisionLandscape(20, wc);

    // Function below is for testing a single base position side or top grip, for either pick or place operation, replay is saved if last parameter is true
    // The replay can be watched in the file /experiment_data/p3_2
    //std::cout << reachabilityAnalysis_pick(0.0, 0.0, true, true, wc) << std::endl;
    //std::cout << reachabilityAnalysis_place(0.0, 0.0, true, true, wc) << std::endl;
    //------------------------------------------------------------------//


    //--------------------- P4_1_M3 ----------------------//
    // Generate 20 images that are going to be used for testing
    std::vector<std::vector<double>> ground_truth = saveTestImages();

    // Functions below can be used for testing single noise values.
    //triangulationTest(150, 1, 0.0, "ImageTest", ground_truth, wc, state);
    //triangulationTest(0, 2, 25.0, "ImageTest", ground_truth, wc, state);
    //triangulationTest(0, 2, 50.0, "ImageTest", ground_truth, wc, state);

    // Run tests

    // IMPORTANT:
    // Before any of the tests below are run, the .csv files in \experiment_data\P4_1_M3 needs to be deleted since the functions below APPEND data to those files
    // The Matlab file Data_Analysis_P4_1M3.m only works for the parameters given below (ex 150 iterations for gaussian blur), so the plots will not work if tests are
    // made with any less iterations. (can of course be changed manually in the matlab file)

    // Blur Tests --> Remove comments for this for loop below to run the gaussian blur tests described in the report with 150 iterations
    /*
    for (int i = 0; i <= 150; i++) // Increasing the amount of times an average kernel of 3x3 is used on the image
    {
        std::cout << i << " Out Of: " << 150 << std::endl;
        triangulationTest(i, 1, 0.0, "BlurTest", ground_truth, wc, state);
    }
    */

    // Salt and Pepper (black and white) Tests --> Remove comments for this for loop below to run the salt and pepper noise tests described in the report
    /*
    for (int i = 0; i <= 100; i++) // Increasing the percentage of Salt and Pepper noise
    {
        std::cout << i << " Out Of: " << 100 << std::endl;
        triangulationTest(0, 2, i, "SPTest", ground_truth, wc, state);
    }
    */

    // Blur and Salt and Pepper (black and white) Tests --> Remove comments for this for loop below to run the gaussian blur combined with salt and pepper noise test described in the report
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