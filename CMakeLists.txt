cmake_minimum_required(VERSION 3.15)
project(robot_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Ipopt REQUIRED ipopt)

include(FetchContent)
FetchContent_Declare(sciplot_content
  GIT_REPOSITORY https://github.com/sciplot/sciplot.git
  GIT_TAG master)
FetchContent_GetProperties(sciplot_content)
if(NOT sciplot_content_POPULATED)
  FetchContent_Populate(sciplot_content)
endif()

include_directories(${sciplot_content_SOURCE_DIR})

include_directories(include)

add_executable(robot_controller src/main.cpp src/robot_tnlp.cpp)
target_link_libraries(robot_controller PRIVATE ${Ipopt_LIBRARIES})
target_include_directories(robot_controller PUBLIC ${Ipopt_INCLUDE_DIRS})
target_compile_options(robot_controller PUBLIC ${Ipopt_CFLAGS_OTHER})