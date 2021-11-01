// std includes
#include <iostream>
#include <cstring>
#include <boost/filesystem.hpp>

// Robworks includes
#include <rw/rw.hpp>

// rovi includes
#include "../includes/p3_3.hpp"


USE_ROBWORK_NAMESPACE
using namespace robwork;

void motion_planning(const std::string &path)
{
    std::cout << "Running motion planning..." << std::endl;
    const std::string project_path = "../";
    const std::string workcell_path = project_path + "workcell/Scene.wc.xml";
    std::cout << "Loading workcell..." << std::endl;
    std::cout << "Current path is : " << path << std::endl;
    
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("../workcell/Scene.wc.xml");
    if(wc == nullptr)
    {
        Log::infoLog() << "Workcell could not be loaded from " << workcell_path << std::endl;
    }


}