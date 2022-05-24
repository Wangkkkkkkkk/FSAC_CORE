#include <ros/ros.h>
#include "lidar_filter_handle.hpp"

typedef ns_lidar_filter::LidarFilterHandle LidarFilterHandle;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_cluster");
  ros::NodeHandle nodeHandle("~");

  LidarFilterHandle myLidarFilterHandle(nodeHandle);

  ros::Rate loop_rate(10);
  while (ros::ok()) {

    myLidarFilterHandle.run();

    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}