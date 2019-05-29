cmake_minimum_required(VERSION 3.0)

#Configure project
project(MinotaureLib VERSION 2.2.0)
set(PROJECT_DESCRIPTION "C++ library for Minotaure")

# Option to choose between cross-compiling or compiling for local computer.
# If compiling for the current computer, _intel is appended to the library name.

option(CROSS_COMPILE "True to cross compile to arm, false to compile on current platform" ON)

# Set compiler and library name.

if(CROSS_COMPILE)
    set(CMAKE_CXX_COMPILER "arm-linux-gnueabihf-g++")
    set(LIBRARY_NAME ${PROJECT_NAME})
else()
    set(CMAKE_CXX_COMPILER "g++")
    set(LIBRARY_NAME ${PROJECT_NAME}_intel)
endif()

# Set compiler flags.
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -fdiagnostics-color -lpthread -lm")

# Specify library content :
# Recursively include all .c files
file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES "src/*.c" "src/*.cpp")
set_source_files_properties(${${PROJECT_NAME}_SOURCES} PROPERTIES LANGUAGE CXX )

# Add directory containing the headers, and put all headers in a variable.
include_directories(include)
file(GLOB_RECURSE ${PROJECT_NAME}_HEADERS "include/*.h")

# Setup static library
add_library(${LIBRARY_NAME} STATIC ${${PROJECT_NAME}_SOURCES})

# Create package config file from template.
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/MinotaureLibTemplate.pc" "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc")

# Set install rules: copy library and headers.
install(TARGETS ${LIBRARY_NAME} DESTINATION "lib")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/" DESTINATION "include")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${LIBRARY_NAME}.pc" DESTINATION "lib/pkgconfig/")
