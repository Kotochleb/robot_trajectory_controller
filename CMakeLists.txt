cmake_minimum_required(VERSION 3.15)
project(robot_controller)

set(CMAKE_CXX_STANDARD 17)


find_package(PkgConfig REQUIRED)
pkg_check_modules(Ipopt REQUIRED ipopt)

include(FetchContent)
FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master)
FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

FetchContent_Declare(Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE)
set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
FetchContent_MakeAvailable(Eigen)

include_directories(include)

add_executable(robot_controller src/main.cpp src/robot_dynamics.cpp)
# target_link_libraries(robot_controller PRIVATE ${Ipopt_LIBRARIES} Eigen3::Eigen matplot)
target_link_libraries(robot_controller PRIVATE Eigen3::Eigen matplot)
# target_include_directories(robot_controller PUBLIC ${Ipopt_INCLUDE_DIRS})
# target_compile_options(robot_controller PUBLIC ${Ipopt_CFLAGS_OTHER})