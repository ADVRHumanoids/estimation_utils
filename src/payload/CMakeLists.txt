find_package(XBotInterface REQUIRED)
find_package(matlogger2 REQUIRED)

add_library(payload_estimation
    payload_estimation.cpp
    force_estimation.cpp)

set_target_properties(payload_estimation PROPERTIES PREFIX "${PROJECT_NAME}_")

target_link_libraries(payload_estimation
    PUBLIC
    kalman
    Eigen3::Eigen
    XBotInterface::XBotInterface
    matlogger2::matlogger2
)

target_include_directories(payload_estimation
    PUBLIC
    $<BUILD_INTERFACE:${ESTIMATION_UTILS_SRC_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS payload_estimation
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib)
