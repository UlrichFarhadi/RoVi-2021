#include <iostream>
#include <string>

#include "roviproblems/includes/p3_2.hpp"
#include "roviproblems/includes/p3_3.hpp"

std::string project_path = "../";
std::string workcell_path = project_path + "workcell";

int main(int argc, char** argv)
{
    simulateCollisionLandscape(20);
    //std::cout << reachabilityAnalysis_place(0, 0, false, true) << std::endl;

 
    //reachabilityAnalysis();  

    return 0;
