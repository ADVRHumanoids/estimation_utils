#include <estimation_utils/nodes/force_estimation_node.h>

//https://github.com/ADVRHumanoids/CartesianInterface/blob/2.0-devel/src/ros/ForceEstimationNode.cpp


int main(int argc, char ** argv)
{
    
    rclcpp::init(argc, argv);
    auto force_estimation_node = std::make_shared<estimation_utils::ForceEstimationNode>();

    rclcpp::spin(force_estimation_node);
    rclcpp::shutdown();
    
    return 0;
}
