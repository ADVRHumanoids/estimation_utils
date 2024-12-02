find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot2_interface REQUIRED)

# add_executable(payload_estimation_executor
#     payload_estimation_executor.cpp)
# ament_target_dependencies(payload_estimation_executor
#     rclcpp
# )
add_library(force_estimation_node
    force_estimation_node.cpp
)
ament_target_dependencies(force_estimation_node
    rclcpp
    geometry_msgs
    std_srvs
    visualization_msgs
    xbot_msgs
)
target_link_libraries(force_estimation_node
    force_estimation
    xbot2_interface::xbot2_interface
)

target_include_directories(force_estimation_node
    PUBLIC
    $<BUILD_INTERFACE:${ESTIMATION_UTILS_SRC_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

set_target_properties(force_estimation_node PROPERTIES PREFIX "${PROJECT_NAME}_")

add_executable(force_estimation_executor
    force_estimation_executor.cpp)

target_link_libraries(force_estimation_executor
    force_estimation_node
    ${roscpp_LIBRARIES}
)

install(TARGETS 
  force_estimation_executor  
  force_estimation_node
#   payload_estimation_executor
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib
)

ament_package()

