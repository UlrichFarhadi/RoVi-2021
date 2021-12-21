#include <iostream>

#include "roviproblems/includes/combination.hpp"

//#include "roviproblems/includes/rw_image_handler.hpp"

////////TEST

// std includes
#include <cstring>
#include <cassert>
#include <vector>
#include <math.h>
#include <algorithm>    // std::reverse
#include <fstream>

// Robworks includes
#include <rw/rw.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/trajectory/Timed.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/trajectory/Path.hpp>

#include <random>
#include <iostream>

using namespace std;


int main(int argc, char** argv)
{

    std::cout << "main started" << std::endl;
    testCombined();

 


    return 0;
}
