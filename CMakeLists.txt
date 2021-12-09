# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(rovi2021)
MESSAGE(${PROJECT_NAME} ":")
include(CTest)
enable_testing()

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Verbose mode prints more info when doing make
# set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
# SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWorks
SET(RobWork_DIR ${RW_ROOT}/cmake) 
#FIND_PACKAGE(RobWork REQUIRED HINTS /home/ulrich/RobWork)
FIND_PACKAGE(RobWork REQUIRED HINTS ~/RobWork)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )


# Add the source files to build'
SET(SrcFiles main.cpp roviproblems/src/p3_2.cpp roviproblems/src/p3_3.cpp)


ADD_EXECUTABLE(${PROJECT_NAME} ${SrcFiles})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ROBWORK_LIBRARIES})

MESSAGE(${PROJECT_NAME} " done!")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)