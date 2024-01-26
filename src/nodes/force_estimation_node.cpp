#include <estimation_utils/nodes/ForceEstimationNode.h>

//https://github.com/ADVRHumanoids/CartesianInterface/blob/2.0-devel/src/ros/ForceEstimationNode.cpp


int main(int argc, char ** argv)
{
    
    // init ros, get handles
    ros::init(argc, argv, "force_estimation_node");
    ros::NodeHandle nh_priv("~");

    double rate = nh_priv.param("rate", 100.0);
    estimation_utils::ForceEstimationNode force_estimation_node(&nh_priv, rate);
    if (! force_estimation_node.init()) {

        ROS_ERROR_STREAM("Init failed");
        return -1;
    }

    ros::Rate loop_rate(rate);
    
    while(ros::ok())
    {

        force_estimation_node.run();
        
        ros::spinOnce();
        // sync with desired loop freq
        loop_rate.sleep();
    }
    
    return 0;
}
