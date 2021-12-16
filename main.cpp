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



    triangulationTest(0, 1, 0.0, "NoNoise");
    //triangulationTest(10, 1, 0.0, "G_10");
    //triangulationTest(20, 1, 0.0, "G_20");
    //triangulationTest(40, 1, 0.0, "G_40");
    //triangulationTest(80, 1, 0.0, "G_80");
    //triangulationTest(160, 1, 0.0, "G_160");
    //triangulationTest(200, 1, 0.0, "G_200");
    //triangulationTest(10, 2, 10.0, "SP_10");
    //triangulationTest(20, 2, 20.0, "SP_20");
    //triangulationTest(40, 2, 40.0, "SP_40");
    //triangulationTest(50, 2, 50.0, "SP_50");
    //triangulationTest(60, 2, 60.0, "SP_60");
    //triangulationTest(80, 2, 80.0, "SP_80");

    return 0;
}