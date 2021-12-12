#include <iostream>
#include <string>

#include "roviproblems/includes/p3_2.hpp"
#include "roviproblems/includes/p3_3.hpp"
#include "roviproblems/includes/p4_1_M3.hpp"

std::string project_path = "../";
std::string workcell_path = project_path + "workcell";

int main(int argc, char** argv)
{
    //simulateCollisionLandscape(20);
    //std::cout << reachabilityAnalysis_pick(0, -0.5, false, true) << std::endl;

 
    //reachabilityAnalysis();  

    //test(0, 1, 0.0, "Gaussian_NoNoise");
    //test(20, 1, 0.0, "Gaussian_HighNoise");
    test(80, 1, 0.0, "Gaussian_VeryHighNoise");

    //test(0, 2, 0.0, "saltpepper_NoNoise");
    //test(0, 2, 10.0, "saltpepper_HighNoise");
    //test(0, 2, 80.0, "saltpepper_VeryHighNoise");

    return 0;
}