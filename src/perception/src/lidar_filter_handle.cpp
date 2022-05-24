#include <ros/ros.h>
#include "lidar_filter_handle.hpp"

namespace ns_lidar_filter {

// Constructor
LidarFilterHandle::LidarFilterHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    lidar_filter_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  subscribeToTopics();
  publishToTopics();
}

// 设定订阅节点
void LidarFilterHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  rawLidarSubscriber_ = nodeHandle_.subscribe("/velodyne_points", 1, &LidarFilterHandle::rawLidarCallback, this);
}

// 设定发布节点
void LidarFilterHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  lidarFilterPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("/lidar_filter", 1);
  lidarFilterGroundPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/lidar_filter_ground", 1);
  lidarFilterCornerPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/lidar_filter_corner", 1);
}


// 主函数
void LidarFilterHandle::run() {
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  lidar_filter_.runAlgorithm();
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
  std::cout<<"time:"<<time_round<<std::endl;
  sendMsg();
}

// 订阅回调函数，读取lidar点云信息
void LidarFilterHandle::rawLidarCallback(const sensor_msgs::PointCloud2 &msg) {
  lidar_filter_.setRawLidar(msg);
}

void LidarFilterHandle::sendMsg() {
  lidarFilterPublisher_.publish(lidar_filter_.getLidarFilter());
  lidarFilterGroundPublisher_.publish(lidar_filter_.getLidarGroundFilter());
  lidarFilterCornerPublisher_.publish(lidar_filter_.getLidarCornerFilter());
}

}