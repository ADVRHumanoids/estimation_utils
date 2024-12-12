cmake_minimum_required(VERSION 3.10)
project(estimation_utils VERSION 1.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(BUILD_SHARED_LIBS OFF)

find_package(Eigen3 REQUIRED)

set(ESTIMATION_UTILS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(src/kalman)

add_subdirectory(src/force_estimation)

add_subdirectory(src/payload)

add_subdirectory(src/nodes)

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN *.h)

install(DIRECTORY launch
        DESTINATION launch)
install(DIRECTORY config
        DESTINATION launch)

include(cmake/ExportProject.cmake)
export_project()
