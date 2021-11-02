#include <iostream>
#include <string>

#include "roviproblems/includes/p3_2.hpp"

std::string project_path = "../";
std::string workcell_path = project_path + "workcell";

int main(int argc, char** argv)
{
    std::cout << "Loading workcell..." << std::endl;
    //simulateCollisionLandscape(4);
    std::cout << reachabilityAnalysis_place(0, 0, false, true) << std::endl;

 

    return 0;
}