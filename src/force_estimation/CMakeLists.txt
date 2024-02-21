find_package(XBotInterface REQUIRED)
find_package(matlogger2 REQUIRED)

add_library(force_estimation SHARED
    ForceEstimation.cpp)

set_target_properties(force_estimation PROPERTIES PREFIX "${PROJECT_NAME}_")

target_link_libraries(force_estimation
    PUBLIC
    Eigen3::Eigen
    XBotInterface::XBotInterface
    matlogger2::matlogger2
)

target_include_directories(force_estimation
    PUBLIC
    $<BUILD_INTERFACE:${ESTIMATION_UTILS_SRC_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS force_estimation
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib)
