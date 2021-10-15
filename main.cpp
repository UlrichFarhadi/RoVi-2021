#include <iostream>
#include <string>

#include "roviproblems/includes/p3_2.hpp"

std::string project_path = "../";
std::string workcell_path = project_path + "workcell";

int main(int argc, char** argv)
{
    std::cout << "Loading workcell" << std::endl;
    reachabilityAnalysis();  

    return 0;
}