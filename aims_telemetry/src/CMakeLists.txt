cmake_minimum_required(VERSION 3.15)
project(aims_telemetry)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python COMPONENTS Interpreter Development REQUIRED)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

# ==========================================
# Automated Dependency Fetching (FetchContent)
# ==========================================
include(FetchContent)

# 1. Fetch Pybind11
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.12.0
)
FetchContent_MakeAvailable(pybind11)

# 2. Fetch Eigen (Header-only)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(Eigen)

# ==========================================
# C++ Target Configuration
# ==========================================
pybind11_add_module(flight_control 
    "${CMAKE_CURRENT_SOURCE_DIR}/telemetry.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/scheduler.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/aero_controller.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/autothrottle.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/flight_executive.cpp"
)

target_include_directories(flight_control PRIVATE 
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
)

target_link_libraries(flight_control PRIVATE Eigen3::Eigen)