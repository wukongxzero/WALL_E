# 1. Setup and silence the deprecation warning
cmake_minimum_required(VERSION 3.10...3.28)
project(TankStatusProtocol C CXX)

# Set standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PYBIND11_FINDPYTHON ON)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 2. Download pybind11 automatically (No find_package needed!)
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.1
)
FetchContent_MakeAvailable(pybind11)

# 3. Create the Python module
# Note: I am using the "lib/" prefix based on your earlier 'cat lib/TankStatusClass.h' command.
# If your .c and .cpp files are actually in 'src/', change 'lib/' to 'src/'.

# Add tankstatuswrapper.cpp to the list of source files!
pybind11_add_module(tankstatus_wrapper
    tankstatuswrapper.cpp
    lib/TankStatusClass.cpp
    lib/TankStatus.c
)

# 4. Tell the compiler where to find the .h files
target_include_directories(tankstatus_wrapper PRIVATE lib)
