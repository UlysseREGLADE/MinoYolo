cmake_minimum_required(VERSION 3.0)

#Configure project
project(mainRobotCode)
set(PROJECT_DESCRIPTION "Code for the main robot for Eurobot")

# Set compiler to arm compiler (cross-compile)
set(CMAKE_CXX_COMPILER "g++")

# Set C++ compiler flags.
set(CMAKE_CXX_FLAGS "-Wall -Wno-psabi -fdiagnostics-color -lpthread -lwiringPi")

# Use pkg-config to find external libraries : Glib (compiled for arm) and BBBEurobot.
# find_package(PkgConfig REQUIRED)
# Look for external libraries, and link them to the project.
#unset(MINOTAURELIB_FOUND CACHE)
#pkg_search_module(MINOTAURELIB REQUIRED MinotaureLib>=2.0)
#message(STATUS "Found MinotaureLib v." ${MINOTAURELIB_VERSION})

#include_directories(${MINOTAURELIB_INCLUDE_DIRS})
#link_directories(${MINOTAURELIB_LIBRARY_DIRS})

# Include source files from src directory.
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.cpp" "src/rplidar/*.cpp")
# Add directory containing the headers.
include_directories(include)
include_directories(include/rplidar)

# Set executable target and link libraries.
add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} pthread)

install(TARGETS ${PROJECT_NAME} DESTINATION ".")
