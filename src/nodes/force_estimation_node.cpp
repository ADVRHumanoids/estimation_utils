#include <estimation_utils/nodes/force_estimation_node.h>

using estimation_utils::ForceEstimationNode;

ForceEstimationNode::ForceEstimationNode(): Node("force_estimation_node") {

    //lets wait for xbot
    if (!waitForXbotCore()) {
        
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "WaitForXbotCore failed, exiting..");
        throw(std::runtime_error("WaitForXbotCore failed"));
    }
    
    // get robot, model, and one imu
    _robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer(*_nh));
    _model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer(*_nh));
    

    if(_robot->getImu().size() > 0)
    {
        _imu = _robot->getImu().begin()->second;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IMU not found: map is empty\n");
    }
    
    // configure node
    this->declare_parameter("rate", 100);
    _rate = this->get_parameter("rate").as_double();

    this->declare_parameter("links", std::vector<std::string>());
    auto links = this->get_parameter("links").as_string_array();
    
    if(links.size() == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Private parameter ~/links is empty, exiting..");
        throw(std::runtime_error("WaitForXbotCore failed"));

    }
    
    this->declare_parameter("chains", std::vector<std::string>());
    auto chains = this->get_parameter("chains").as_string_array();
    if(chains.size() == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Private parameter ~/chains is empty, will use all torque sensors");
    }
    
    this->declare_parameter("svd_threshold", (double)ForceEstimation::DEFAULT_SVD_THRESHOLD);
    double svd_th = this->get_parameter("svd_threshold").as_double();

    // get torque offset map
    this->declare_parameter("torque_offset", std::map<std::string, double>());
    auto tau_off_map = _nh->param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    _tau_offset.setZero(_model->getJointNum());
    _model->mapToEigen(tau_off_map_xbot, _tau_offset);
    
    // ros service to request for zeroing the force estimation offset
    _request_wrench_zero_offset = this->create_service("wrench_zero_offset", &ForceEstimationNode::wrenchZeroOffsetClbk, this);
    this->declare_parameter("sec_to_reset", 3);
    _reset_time_sec = this->get_parameter("sec_to_reset").as_double();
    
    // construct force estimation class
    this->declare_parameter("use_momentum_based", false);
    bool use_momentum_based = this->get_parameter("use_momentum_based").as_bool();
    if(use_momentum_based)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "using momentum based estimation");
        _f_est_ptr = std::make_shared<estimation_utils::ForceEstimationMomentumBased>(_model, _rate, svd_th);
    }
    else
    {
        _f_est_ptr = std::make_shared<estimation_utils::ForceEstimation>(_model, _rate, svd_th);
    }


    // set ignored joints
    this->declare_parameter("ignored_joints", std::vector<std::string>());
    auto ignored_joints = this->get_parameter("ignored_joints").as_string_array();
    for(auto jname : ignored_joints)
    {
        _f_est_ptr->setIgnoredJoint(jname);
    }
    
    // generate virtual fts
    for(const auto& l : links)
    {
        auto dofs = _nh->param(l + "/dofs", std::vector<int>());
        
        auto pub = _nh->advertise<geometry_msgs::WrenchStamped>("force_estimation/" + l, 1);
        
        _ft_map[l] = _f_est_ptr->add_link(l, dofs, chains);
        _ft_pub_map[l] = pub;
    }

    // log
    this->declare_parameter("enable_log", false);
    bool enable_log = this->get_parameter("enable_log").as_bool();
    if(enable_log)
    {
        _logger = XBot::MatLogger2::MakeLogger("/tmp/force_estimation_log");
    }

    this->declare_parameter("ref_frame", std::string());
    _ref_frame = this->get_parameter("ref_frame").as_string();

    //filter
    this->declare_parameter("enable_filter", false);
    bool enable_filter = this->get_parameter("enable_filter").as_bool();
    if(enable_filter) {
        double filter_damp = _nh->param("filter_damp", 0.8);
        double filter_bw = _nh->param("filter_bw", 3);
        double filter_dead_zone = _nh->param("filter_dead_zone", 0); //lower than this forces are ignored
        _f_est_ptr->initFilter(filter_damp, filter_bw, filter_dead_zone);
    }

    //markers
    this->declare_parameter("pub_arrows", false);
    _pub_markers = this->get_parameter("pub_arrows").as_bool();
    if (_pub_markers) {

        _arrows_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("force_estimation/force_markers/", 1);

        _marker.id = 0;
        _marker.scale.x = 0.02; //shaft diameter
        _marker.scale.y = 0.04; //head diameter
        _marker.scale.z = 0.05; //head length auto computed
        _marker.color.a = 0.8;
        _marker.color.r = 0.5;
        _marker.color.g = 0.5;
        _marker.color.b = 0.5;
        _marker.pose.orientation.w = 1;
        _marker.frame_locked = true;
        _marker.points.resize(2);
        _marker.points.at(0) = geometry_msgs::msg::Point();

        this->declare_parameter("arrow_scale_factor", 1);
        this->declare_parameter("arrow_max_norm", 5);
        _arrow_scale_factor = this->get_parameter("arrow_scale_factor").as_double();
        _arrow_max_norm = this->get_parameter("arrow_max_norm").as_double();
    }

    _timer = this->create_wall_timer(
        1.0/_rate, std::bind(&ForceEstimationNode::run, this));
}


bool estimation_utils::ForceEstimationNode::run() {

    // update model from robot, set imu
    _robot->sense(false);
    _model->syncFrom(*_robot, XBot::Sync::All, XBot::Sync::MotorSide);
    if(_model->isFloatingBase() && _imu)
    {
        _model->setFloatingBaseState(_imu);
        _model->update();
    }
    _model->update();
    _model->getJointEffort(_tau);
    _tau += _tau_offset;
    _model->setJointEffort(_tau);
    
    //reset offset stuff
    if (_reset_requested) {
        
        _f_est_ptr->resetOffset(_reset_time_sec);
        _reset_requested = false;
    }
    
    // update force estimation
    _f_est_ptr->update();

    // publish to topics
    
    for(const auto& ft : _ft_map)
    {
        Eigen::Vector6d wrench;
        ft.second->getWrench(wrench);

        if (_ref_frame.size()>0) {

            Eigen::Matrix3d sensor_R_ref;
            //source target
            if (! _model->getOrientation(ft.second->getSensorName(), _ref_frame, sensor_R_ref)) {
                return false;
            }
            
            wrench.head<3>() = sensor_R_ref * wrench.head<3>();
            wrench.tail<3>() = sensor_R_ref * wrench.tail<3>();

            _wrench_msg.header.frame_id = _ref_frame;

        } else {
            _wrench_msg.header.frame_id = ft.second->getSensorName();

        }

        tf::wrenchEigenToMsg(wrench, _wrench_msg.wrench);
        _wrench_msg.header.stamp = rclcpp::Time::now();
        _ft_pub_map.at(ft.first).publish(_wrench_msg);
    }

    if (_pub_markers) {
        publishArrows();
    }

    // log
    if(_logger)
    {
        _f_est_ptr->log(_logger);
    }

    return true;

}

bool ForceEstimationNode::wrenchZeroOffsetClbk(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    _reset_requested = true;
    return true;
}

bool ForceEstimationNode::waitForXbotCore(double timeout) {
    
    //ros::Time startTime = ros::Time::now();
    rclcpp::Client client = this->create_client<xbot_msgs::srv::PluginStatus>("/xbotcore/ros_io/state");

    auto wait_lambda = [&](){
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for XbotCore...");
        if (!client->wait_for_service(timeout)) {
            return false;
        }
        
        return true;
    };
    
    if (! wait_lambda()) {
        return false;
    }
    
    xbot_msgs::PluginStatus status;
    rclcpp::Rate r(10);
    rclcpp::Duration(1).sleep(); //necessary because idk but service is not really ready and the client.call hands 4ever
    while(status.response.status.compare("Running") != 0) {
        
        if (!client.call(status)) {
//              RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Call for XbotCore fail, I will retry...");
//             if (! wait_lambda()) {
//                 return false;
//             }
        }
        r.sleep();
    }

    return true;
    
}


void ForceEstimationNode::publishArrows() {
    
    visualization_msgs::MarkerArray markers;

    for (const auto& ft : _ft_map) {

    //    geometry_msgs::TransformStamped transformStamped;
        // try{
        //     transformStamped = tf_buffer.lookupTransform(sensor_link, ref_frame,  //first is target, second is source
        //                                                 ros::Time(0)); //latest available transform
        // }
        // catch (tf2::TransformException &ex) {

        //     ROS_WARN("FORCE VISUALIZER: %s",ex.what());
        //     //ros::Duration(1.0).sleep();
        //     return;
        // }

        // Eigen::Quaterniond quat(
        //     transformStamped.transform.rotation.w,
        //     transformStamped.transform.rotation.x,
        //     transformStamped.transform.rotation.y,
        //     transformStamped.transform.rotation.z);
        
        
        _marker.header.frame_id = ft.second->getSensorName();
        _marker.header.stamp = rclcpp::Time::now();
        _marker.ns = _marker.header.frame_id;

        Eigen::Vector6d wrench;
        ft.second->getWrench(wrench);

        //Eigen::Vector3d vect = quat * wrench.head<3>();
        Eigen::Vector3d vect = wrench.head<3>();

        vect /= _arrow_scale_factor;

        //saturate arrow
        if (vect.norm() > _arrow_max_norm) {
            vect *= _arrow_max_norm / vect.norm() ;
        }

        geometry_msgs::Point point1;
        point1.x = vect(0);
        point1.y = vect(1);
        point1.z = vect(2);
        _marker.points.at(1) = point1;
        
        markers.markers.push_back(_marker);
    }

    _arrows_pub.publish(markers);
}