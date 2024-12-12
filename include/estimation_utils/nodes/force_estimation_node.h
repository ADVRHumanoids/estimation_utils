#ifndef ESTIMATION_UTILS_FORCE_ESTIMATION_NODE_H
#define ESTIMATION_UTILS_FORCE_ESTIMATION_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <xbot2_interface/robotinterface2.h>
#include <xbot2_interface/ros2/config_from_param.hpp>

#include <matlogger2/matlogger2.h>
//for waitForXbotCore method
#include <xbot_msgs/srv/plugin_status.hpp>

#include <estimation_utils/force_estimation/force_estimation.h>

namespace estimation_utils {
    
class ForceEstimationNode : public rclcpp::Node {
    
public:
    ForceEstimationNode();

private:              

    rclcpp::TimerBase::SharedPtr _timer;

    double _rate;

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    XBot::ImuSensor::ConstPtr _imu;

    std::shared_ptr<estimation_utils::ForceEstimation> _f_est_ptr;
    Eigen::VectorXd _tau, _tau_offset;
    geometry_msgs::msg::WrenchStamped _wrench_msg;

    std::map<std::string, XBot::ForceTorqueSensor::ConstPtr> _ft_map;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr> _ft_pub_map;

    void publishArrows();  
    bool run();

    //offset reset
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _request_wrench_zero_offset;
    double _reset_time_sec;
    bool _reset_requested = false;
    bool wrenchZeroOffsetClbk(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                              std::shared_ptr<std_srvs::srv::Empty::Response> res);
   
    //markers pub
    bool _pub_markers;
    double _arrow_scale_factor, _arrow_max_norm;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _arrows_pub;
    visualization_msgs::msg::Marker _marker;

    std::string _ref_frame;

    //log
    XBot::MatLogger2::Ptr _logger;

    bool waitForXbotCore(double timeout_sec = -1);
};
    
} //namespace

#endif // ESTIMATION_UTILS_FORCE_ESTIMATION_NODE_H

