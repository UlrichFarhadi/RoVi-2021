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
#SET(RW_ROOT $ENV{RW_ROOT})
#SET(RWSTUDIO_ROOT $ENV{RWS_ROOT})
if(DEFINED ENV{RW_ROOT} AND DEFINED ENV{RWS_ROOT})
  file(TO_CMAKE_PATH "$ENV{RW_ROOT}" RW_ROOT)
  file(TO_CMAKE_PATH "$ENV{RWS_ROOT}" RWSTUDIO_ROOT)
else()
  set(RW_ROOT "${ROOT}/../../../RobWork")
  set(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
endif()

#if(DEFINED ENV{RW_ROOT} AND DEFINED ENV{RWS_ROOT})
#    file(TO_CMAKE_PATH "$ENV{RW_ROOT}" RW_ROOT)
#    file(TO_CMAKE_PATH "$ENV{RWS_ROOT}" RW_ROOT)

# Verbose mode prints more info when doing make
# set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
# SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWorks
SET(RobWork_DIR ${RW_ROOT}/cmake)
SET(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)


#FIND_PACKAGE(RobWork REQUIRED HINTS ~/RobWork)
FIND_PACKAGE(RobWork REQUIRED HINTS ~/RobWork)
FIND_PACKAGE(RobWorkStudio REQUIRED HINTS HINTS ~/RobWorkStudio)
#FIND_PACKAGE(RobWork REQUIRED HINTS /home/ulrich/RobWork)
#FIND_PACKAGE(RobWorkStudio REQUIRED HINTS /home/ulrich/RobWork/RobWorkStudio)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

INCLUDE_DIRECTORIES( ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

# Add the source files to build'
SET(SrcFiles main.cpp roviproblems/src/p4_1_M3.cpp roviproblems/src/p3_2.cpp)



# The shared library to build:
ADD_EXECUTABLE(${PROJECT_NAME} ${SrcFiles})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${OpenCV_LIBS} Qt5Core Qt5Widgets)

MESSAGE(${PROJECT_NAME} " done!")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)