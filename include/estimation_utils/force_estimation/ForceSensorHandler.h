#ifndef FORCE_ESTIMATION_HANDLER_H
#define FORCE_ESTIMATION_HANDLER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <utils/UtilsEigen.h>
#include <utils/SecondOrderFilter.h>

#include <mutex>

namespace tpo {
    
class ForceSensorHandler{
    
public:
    ForceSensorHandler();
    
    bool init(const std::unique_ptr<ros::NodeHandle>& nh, std::string group_name, std::string sensor_link,
                std::string ref_link, bool filter, double force_dead_zone_limit, 
                                   double period_sec, bool pub_arrows = true);
    
    bool reset();
        
    void getSensedWrench(Eigen::Matrix<double,6,1>& w) const;
    void getFilteredSensedWrench(Eigen::Matrix<double, 6, 1> & w) const;

    void publishArrows();  
    
    void update();
    
    std::string getSensorLink() const;

    
private:
    
    std::string group_name;
    std::string sensor_link;
    std::string ref_link;
        
    ros::Subscriber sensed_wrench_sub;
    geometry_msgs::WrenchStamped sensed_wrench_raw_msg;
    bool sensed_wrench_new_data;
    void sensedWrenchSubCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    std::mutex sensed_wrench_mutex;
    
    Eigen::Matrix<double,6,1> sensed_wrench;
    Eigen::Matrix<double,6,1> sensed_wrench_filtered;
    
    bool filter;
    void filterSensedForce();
    double force_dead_zone_limit;
    tpo::utils::SecondOrderFilter<Eigen::Matrix<double,6,1>> filter_sensed_force;
    bool initSensedForceFilter();
    double period_sec;
    
    ros::Publisher sensed_force_filtered_pub;
    
    ros::Publisher sensed_force_marker_pub;
    bool pub_arrows;
 
    tf2_ros::Buffer tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
};
    
} //namespace

#endif // FORCE_ESTIMATION_HANDLER_H

