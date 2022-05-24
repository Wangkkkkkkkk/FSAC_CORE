#include <ros/ros.h>

#include "loam_handle.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "loam");
    ros::NodeHandle nodeHandle("~");

    loam_localization::LoamHandle myLoamHandle(nodeHandle);

    ros::Rate loop_rate(10);
    while (ros::ok()) {

        myLoamHandle.run();

        ros::spinOnce();                // Keeps node alive basically
        loop_rate.sleep();              // Sleep for loop_rate
    }
    return 0;
}