#include <ros/ros.h>

#include "g_compensation/g_compensator_class.h"

main(int argc, char **argv)
{
    ros::init(argc, argv, "g_compensator");
    ros::NodeHandle nh;

    GCompensator comp(nh);

    ROS_INFO("Running gravity compensator");

    ros::spin();

    ROS_INFO("Shutting down gravity compensator");

    return 0;
}
