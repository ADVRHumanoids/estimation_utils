add_library(kalman
    kalman.cpp)

set_target_properties(kalman PROPERTIES PREFIX "${PROJECT_NAME}_")

target_link_libraries(kalman
    PUBLIC Eigen3::Eigen)

target_include_directories(kalman
    PUBLIC
    $<BUILD_INTERFACE:${ESTIMATION_UTILS_SRC_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS kalman
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib)
