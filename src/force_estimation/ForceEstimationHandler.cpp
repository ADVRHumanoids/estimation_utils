#include <telephysicaloperation/ForceSensorHandler.h>

using tpo::ForceSensorHandler;

ForceSensorHandler::ForceSensorHandler() {}


bool tpo::ForceSensorHandler::init(const std::unique_ptr<ros::NodeHandle>& nh, std::string group_name, std::string sensor_link, 
                                   std::string ref_link, bool filter, double force_dead_zone_limit, 
                                   double period_sec, bool pub_arrows)
{
    this->group_name = group_name;
    this->sensor_link = sensor_link;
    this->ref_link = ref_link;
        
    this->period_sec = period_sec;
    this->filter = filter;
    this->pub_arrows = pub_arrows;
    
    this-> force_dead_zone_limit = force_dead_zone_limit; //lower than this sensed forces are ignored
     
    std::string topic_name = "/force_estimator/force_estimation/" + sensor_link;
    
    sensed_wrench_sub = nh->subscribe<geometry_msgs::WrenchStamped>(topic_name, 1, &ForceSensorHandler::sensedWrenchSubCallback, this);
    sensed_wrench_raw_msg = geometry_msgs::WrenchStamped();
    sensed_wrench_new_data= false;
    sensed_wrench= Eigen::Matrix<double, 6, 1>::Zero();
    
    tf_listener = std::make_unique<tf2_ros::TransformListener>(tf_buffer);

    if (filter) {
        initSensedForceFilter();
        sensed_force_filtered_pub = nh->advertise<geometry_msgs::WrenchStamped>("/force_estimation/force_filtered/" + sensor_link, 1);
    }
    
    if (pub_arrows) {
        sensed_force_marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/force_estimation/force_marker/" + sensor_link, 1);
    }
    
    return true;
}

bool tpo::ForceSensorHandler::reset()
{
    
    Eigen::VectorXd resetVect = Eigen::Matrix<double, 6,1>::Zero();
    
    // reset filter
    filter_sensed_force.reset(resetVect);
    
    sensed_wrench_raw_msg = geometry_msgs::WrenchStamped();
    sensed_wrench = Eigen::Matrix<double, 6, 1>::Zero();
    sensed_wrench_filtered = Eigen::Matrix<double, 6, 1>::Zero();
    sensed_wrench_new_data = false;
    
    return true;
    
}   

void tpo::ForceSensorHandler::publishArrows() {
    
    visualization_msgs::MarkerArray markers;
    
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tf_buffer.lookupTransform(sensor_link, ref_link,  //first is target, second is source
                                                    ros::Time(0)); //latest available transform
    }
    catch (tf2::TransformException &ex) {

        ROS_WARN("FORCE VISUALIZER: %s",ex.what());
        //ros::Duration(1.0).sleep();
        return;
    }

    Eigen::Quaterniond quat(
        transformStamped.transform.rotation.w,
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = sensor_link;
    marker.id = 0;
    marker.header.stamp = ros::Time::now();
    marker.scale.x = 0.02; //shaft diameter
    marker.scale.y = 0.04; //head diameter
    marker.scale.z = 0.05; //head length auto computed
    marker.pose.orientation.w = 1;
    marker.frame_locked = true;
    marker.points.push_back(geometry_msgs::Point());
    marker.points.push_back(geometry_msgs::Point());
    
    if (!filter) {
         
        const auto & it = sensed_wrench;

        marker.ns = "raw";

        marker.color.a = 0.8;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;

        Eigen::Vector3d vect;
        geometry_msgs::Point point1;

        vect = quat * it.head<3>();
        if (vect.norm() > 0.5) {
            vect *= 0.5 / vect.norm() ;
        }
        point1.x = vect(0);
        point1.y = vect(1);
        point1.z = vect(2);
        marker.points.at(1) = point1;

    } else {

        const auto & it = sensed_wrench_filtered;
        
        marker.ns = "filtered";

        marker.color.a = 0.8;
        marker.color.r = 0.2;
        marker.color.g = 1.0;
        marker.color.b = 0.9;

        Eigen::Vector3d vect;
        geometry_msgs::Point point1;

        vect = quat * it.head<3>();
        if (vect.norm() > 0.5) {
            vect *= 0.5 / vect.norm() ;
        }
        point1.x = vect(0);
        point1.y = vect(1);
        point1.z = vect(2);
        marker.points.at(1) = point1;
    }
    
    markers.markers.push_back(marker);
    
    sensed_force_marker_pub.publish(markers);
}


void tpo::ForceSensorHandler::sensedWrenchSubCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    
    const std::lock_guard<std::mutex> lock(sensed_wrench_mutex);

    sensed_wrench_raw_msg = *msg;
    sensed_wrench_new_data = true;
}

void tpo::ForceSensorHandler::update() {
    
    const auto &it = sensed_wrench_raw_msg;
        
    if (sensed_wrench_new_data) {
        
        const std::lock_guard<std::mutex> lock(sensed_wrench_mutex);
        
        geometry_msgs::WrenchStamped msg_transf;

        try {
            tf_buffer.transform(it, msg_transf, ref_link);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        //DEBUG PRINTS
        //std::cout << sensor_link << " : " << it.wrench.force.x << " " << it.wrench.force.y << " " << it.wrench.force.z << std::endl;
        //std::cout << "after tf trasf: : " << msg_transf.wrench.force.x << " " << msg_transf.wrench.force.y << " " << msg_transf.wrench.force.z << std::endl;
        
        tf::wrenchMsgToEigen(msg_transf.wrench, sensed_wrench);
        //std::cout << "after eigen trasf: : " << sensed_wrench.transpose() << std::endl;

        sensed_wrench_new_data = false;
    }
    
    //even if not data, we must filtered because filter should be run at each loop rate given...?
    if (filter) {
        filterSensedForce();
        geometry_msgs::WrenchStamped msg;
        tf::wrenchEigenToMsg(sensed_wrench_filtered, msg.wrench);
        
        msg.header.frame_id = ref_link;
        msg.header.stamp = it.header.stamp;

        sensed_force_filtered_pub.publish(msg);
    } 
    if (pub_arrows) {
        publishArrows();
    }
}

void tpo::ForceSensorHandler::filterSensedForce() {
    
    sensed_wrench_filtered = sensed_wrench;
    
    tpo::utils::cutOff(sensed_wrench_filtered, force_dead_zone_limit);
    
    sensed_wrench_filtered= filter_sensed_force.process(sensed_wrench_filtered);
    
}

bool tpo::ForceSensorHandler::initSensedForceFilter() {
    
    const double DAMPING_FACT = 0.8;
    const double BW_MEDIUM = 3;
    double omega = 2.0 * M_PI * BW_MEDIUM;
    
    Eigen::VectorXd resetVect = Eigen::Matrix<double, 6,1>::Zero();

    filter_sensed_force = tpo::utils::SecondOrderFilter<Eigen::Matrix<double,6,1>>();
    filter_sensed_force.setDamping ( DAMPING_FACT );
    filter_sensed_force.setTimeStep ( period_sec );
    filter_sensed_force.setOmega ( omega );
    
    // reset filter
    filter_sensed_force.reset(resetVect);
    
    sensed_wrench_filtered = Eigen::Matrix<double, 6, 1>::Zero();
    
    return true;
} 

void tpo::ForceSensorHandler::getSensedWrench(Eigen::Matrix<double, 6, 1> & w) const { w = sensed_wrench; }
void tpo::ForceSensorHandler::getFilteredSensedWrench(Eigen::Matrix<double, 6, 1> & w) const { w = sensed_wrench_filtered; }
std::string tpo::ForceSensorHandler::getSensorLink() const { return sensor_link; }
