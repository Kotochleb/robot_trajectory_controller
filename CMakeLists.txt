cmake_minimum_required(VERSION 3.15)
project(inverted_pendulum)

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

add_executable(inverted_pendulum src/main.cpp src/pendulum.cpp)
target_link_libraries(inverted_pendulum PRIVATE ${Ipopt_LIBRARIES})
target_include_directories(inverted_pendulum PUBLIC ${Ipopt_INCLUDE_DIRS})
target_compile_options(inverted_pendulum PUBLIC ${Ipopt_CFLAGS_OTHER})