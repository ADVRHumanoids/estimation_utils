find_package(rclcpp REQUIRED)
find_package(eigen_conversions REQUIRED)

include_directories(
    ${eigen_conversions_INCLUDE_DIRECTORIES}
)

# add_executable(payload_estimation_executor
#     payload_estimation_executor.cpp)
# ament_target_dependencies(payload_estimation_executor
#     rclcpp
# )
# target_link_libraries(payload_estimation_executor
#     ${eigen_conversions_LIBRARIES}
# )

add_library(force_estimation_node
    force_estimation_node.cpp
)
ament_target_dependencies(force_estimation_node
    rclcpp
)
target_link_libraries(force_estimation_node
    force_estimation
    ${eigen_conversions_LIBRARIES}
)

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
  DESTINATION lib
)

