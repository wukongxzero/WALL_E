cmake_minimum_required(VERSION 3.12...3.28)
project(TANKSTATUS C CXX)

add_subdirectory(lib/)

# 1. Find Python FIRST (Crucial for Pybind11 on NixOS)
set(PYBIND11_FINDPYTHON ON)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# 2. Fetch Pybind11 locally
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.13.1
)
FetchContent_MakeAvailable(pybind11)
# (Notice we completely removed find_package(pybind11 REQUIRED)!)

# 3. Build your C/C++ libraries
add_library(tankstatus_lib SHARED lib/TankStatus.h lib/TankStatus.c)
set_target_properties(tankstatus_lib PROPERTIES CXX_VISIBILITY_PRESET hidden)

add_library(tankstatus_class SHARED lib/TankStatusClass.h lib/TankStatusClass.cpp)
# Modern CMake uses target_link_libraries instead of link_libraries
target_link_libraries(tankstatus_class PUBLIC tankstatus_lib)

# 4. Create the Python module
pybind11_add_module(tankstatus_wrapper tankstatuswrapper.cpp lib/TankStatusClass.cpp)

# Link your C library to the Python module
target_link_libraries(tankstatus_wrapper PRIVATE tankstatus_lib)
