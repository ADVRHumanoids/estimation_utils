#ifndef FORCE_ESTIMATION_NODE_H
#define FORCE_ESTIMATION_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <matlogger2/matlogger2.h>
//for waitForXbotCore method
#include <xbot_msgs/PluginStatus.h>

#include <estimation_utils/force_estimation/ForceEstimation.h>

namespace estimation_utils {
    
class ForceEstimationNode {
    
public:
    ForceEstimationNode(ros::NodeHandle* nh, const double& rate);
    
    bool init();
            
    void publishArrows();  
    
    bool run();
    
private:

    ros::NodeHandle* _nh;

    const double _rate;

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;
    XBot::ImuSensor::ConstPtr _imu;

    std::shared_ptr<estimation_utils::ForceEstimation> _f_est_ptr;
    Eigen::VectorXd _tau, _tau_offset;
    geometry_msgs::WrenchStamped _wrench_msg;

    std::map<std::string, XBot::ForceTorqueSensor::ConstPtr> _ft_map;
    std::map<std::string, ros::Publisher> _ft_pub_map;

    //offset reset
    ros::ServiceServer _request_wrench_zero_offset;
    double _reset_time_sec;
    bool _reset_requested = false;
    bool wrenchZeroOffsetClbk(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
   
    //markers pub
    bool _pub_markers;
    double _arrow_scale_factor, _arrow_max_norm;
    ros::Publisher _arrows_pub;
    visualization_msgs::Marker _marker;

    std::string _ref_frame;

    //log
    XBot::MatLogger2::Ptr _logger;

    bool waitForXbotCore(ros::NodeHandle* nh, ros::Duration timeout = ros::Duration(-1));
};
    
} //namespace

#endif // FORCE_ESTIMATION_NODE_H

