find_package(roscpp REQUIRED)
find_package(eigen_conversions REQUIRED)

include_directories(
    ${roscpp_INCLUDE_DIRECTORIES}
    ${eigen_conversions_INCLUDE_DIRECTORIES}
)
add_executable(payload_estimation_node
    payload_estimation_node.cpp)


target_link_libraries(payload_estimation_node
    payload_estimation
    ${roscpp_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    )

add_executable(force_estimation_node
    force_estimation_node.cpp)


target_link_libraries(force_estimation_node
    force_estimation
    ${roscpp_LIBRARIES}
)

install(TARGETS 
  force_estimation_node  
  payload_estimation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

