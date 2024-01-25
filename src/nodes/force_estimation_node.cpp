#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <estimation_utils/force_estimation/ForceEstimation.h>
#include <matlogger2/matlogger2.h>

#include <std_srvs/Empty.h>

//for waitForXbotCore method
#include <xbot_msgs/PluginStatus.h>


//https://github.com/ADVRHumanoids/CartesianInterface/blob/2.0-devel/src/ros/ForceEstimationNode.cpp

bool reset_requested = false;

bool wrenchZeroOffsetClbk(std_srvs::Empty::Request &req,
                          std_srvs::Empty::Response &res)
{
    reset_requested = true;
    
    return true;
}

bool waitForXbotCore(ros::NodeHandle* nh, ros::Duration timeout = ros::Duration(-1)) {
    
    //ros::Time startTime = ros::Time::now();
    ros::ServiceClient client = nh->serviceClient<xbot_msgs::PluginStatus>("/xbotcore/ros_io/state");

    auto wait_lambda = [&](){
        
        ROS_INFO_STREAM("Waiting for XbotCore...");
        if (!client.waitForExistence(timeout)) {
            return false;
        }
        
        return true;
    };
    
    if (! wait_lambda()) {
        return false;
    }
    
    xbot_msgs::PluginStatus status;
    ros::Rate r(10);
    ros::Duration(1).sleep(); //necessary because idk but service is not really ready and the client.call hands 4ever
    while(status.response.status.compare("Running") != 0) {
        
        if (!client.call(status)) {
//             ROS_INFO_STREAM("Call for XbotCore fail, I will retry...");
//             if (! wait_lambda()) {
//                 return false;
//             }
        }
        r.sleep();
    }

    return true;
    
}

int main(int argc, char ** argv)
{
    
    // init ros, get handles
    ros::init(argc, argv, "force_estimation_node");
    ros::NodeHandle nh_priv("~");
    
    //lets wait for xbot
    if (!waitForXbotCore(&nh_priv)) {
        
        ROS_ERROR_STREAM("WaitForXbotCore failed, exiting..");
        return -1;
    }
    
    
    // get robot, model, and one imu
    auto robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer(nh_priv));
    auto model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer(nh_priv));
    XBot::ImuSensor::ConstPtr imu;

    if(robot->getImu().size() > 0)
    {
        imu = robot->getImu().begin()->second;
    }
    else
    {
        ROS_INFO("IMU not found: map is empty\n");
    }
    
    // configure node
    double rate = nh_priv.param("rate", 100.0);
    auto links = nh_priv.param("links", std::vector<std::string>());
    
    if(links.size() == 0)
    {
        ROS_INFO("Private parameter ~/links is empty, exiting..");
        return 1;
    }
    
    auto chains = nh_priv.param("chains", std::vector<std::string>());
    if(links.size() == 0)
    {
        ROS_INFO("Private parameter ~/chains is empty, will use all torque sensors");
    }
    
    double svd_th = nh_priv.param("svd_threshold", (double)estimation_utils::ForceEstimation::DEFAULT_SVD_THRESHOLD);

    // get torque offset map
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getJointNum());
    model->mapToEigen(tau_off_map_xbot, tau_offset);
    
    /////  ros service to request for zeroing the force estimation offset
    ros::ServiceServer request_wrench_zero_offset;
    request_wrench_zero_offset = nh_priv.advertiseService("wrench_zero_offset", wrenchZeroOffsetClbk);
    double reset_time_sec = nh_priv.param("sec_to_reset", 3);;
    
    // construct force estimation class
    std::shared_ptr<estimation_utils::ForceEstimation> f_est_ptr;

    if(nh_priv.param("use_momentum_based", false))
    {
        ROS_INFO("using momentum based estimation");
        f_est_ptr = std::make_shared<estimation_utils::ForceEstimationMomentumBased>(model, rate, svd_th);
    }
    else
    {
        f_est_ptr = std::make_shared<estimation_utils::ForceEstimation>(model, rate, svd_th);
    }

    estimation_utils::ForceEstimation& f_est = *f_est_ptr;

    // set ignored joints
    auto ignored_joints = nh_priv.param("ignored_joints", std::vector<std::string>());
    for(auto jname : ignored_joints)
    {
        //f_est.setIgnoredJoint(jname);
    }
    
    // generate virtual fts
    std::map<XBot::ForceTorqueSensor::ConstPtr, ros::Publisher> ft_map;
    
    for(auto l : links)
    {
        auto dofs = nh_priv.param(l + "/dofs", std::vector<int>());
        
        auto pub = nh_priv.advertise<geometry_msgs::WrenchStamped>("force_estimation/" + l, 1);
        
        ft_map[f_est.add_link(l, dofs, chains)] = pub;
    }

    // log
    XBot::MatLogger2::Ptr logger;
    if(nh_priv.param("enable_log", false))
    {
        logger = XBot::MatLogger2::MakeLogger("/tmp/force_estimation_log");
    }

    
    // start looping
    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);
    Eigen::Vector6d wrench;
    
    while(ros::ok())
    {
        // update model from robot, set imu
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        if(model->isFloatingBase() && imu)
        {
            model->setFloatingBaseState(imu);
            model->update();
        }
        model->update();
        model->getJointEffort(tau);
        tau += tau_offset;
        model->setJointEffort(tau);
        
        //reset offset stuff
        if (reset_requested) {
            
            f_est.resetOffset(reset_time_sec);
            reset_requested = false;
        }
        
        // update force estimation
        f_est.update();
        
        // publish to topics
        
        for(const auto& ft_pub : ft_map)
        {
            geometry_msgs::WrenchStamped msg;

            ft_pub.first->getWrench(wrench);
            
            tf::wrenchEigenToMsg(wrench, msg.wrench);
            
            msg.header.frame_id = ft_pub.first->getSensorName();
            msg.header.stamp = ros::Time::now();
            
            ft_pub.second.publish(msg);
        }

        // log
        if(logger)
        {
            f_est.log(logger);
        }
        
        
        ros::spinOnce();
        // sync with desired loop freq
        loop_rate.sleep();
    }
    
    return 0;
}
