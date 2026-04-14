cmake_minimum_required(VERSION 3.12)
project(TANKSTATUS)

add_subdirectory(lib/)

include(FetchContent)

FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.1 # Replace with the latest stable version
)

FetchContent_MakeAvailable(pybind11)

# Find Python and pybind11
find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)


# Add your C source file as a library
add_library(tankstatus_lib STATIC lib/TankStatus.h lib/TankStatus.c)

# Create the Python module
pybind11_add_module(tankstatus_wrapper tankstatuswrapper.cpp TankStatusClass.h TankStatusClass.cpp)

# Link your C library to the Python module
target_link_libraries(tankstatus_wrapper PRIVATE tankstatus_lib)



