#include <ros/ros.h>

#include "loam_handle.hpp"

namespace loam_localization {

// Constructor
LoamHandle::LoamHandle(ros::NodeHandle &nodeHandle) :
        nodeHandle_(nodeHandle),
        loam_localization_(nodeHandle) {
    ROS_INFO("Constructing Handle");
    subscribeToTopics();
    publishToTopics();
}

// 设定订阅节点
void LoamHandle::subscribeToTopics() {
    ROS_INFO("subscribe to topics");
    LidarFilterSubscriber_ = nodeHandle_.subscribe("/lidar_filter", 1, &LoamHandle::LidarFilterCallback, this);
    LidarFilterGroundSubscriber_ = nodeHandle_.subscribe("/lidar_filter_ground", 1, &LoamHandle::LidarFilterGroundCallback, this);
    LidarFilterCornerSubscriber_ = nodeHandle_.subscribe("/lidar_filter_corner", 1, &LoamHandle::LidarFilterCornerCallback, this);
}

// 设定发布节点
void LoamHandle::publishToTopics() {
    ROS_INFO("publish to topics");
//  lidarFilterPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud>("/lidar_filter", 1);
//  lidarFilterGroundPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/lidar_filter_ground", 1);
//  lidarFilterCornerPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("/lidar_filter_corner", 1);
}

// 回调函数
void LoamHandle::LidarFilterCallback(const sensor_msgs::PointCloud &msg) {
    loam_localization_.setLidarFilter(msg);
}

// 回调函数
void LoamHandle::LidarFilterGroundCallback(const sensor_msgs::PointCloud2 &msg) {
    loam_localization_.setLidarGroundFilter(msg);
}

// 回调函数
void LoamHandle::LidarFilterCornerCallback(const sensor_msgs::PointCloud2 &msg) {
    loam_localization_.setLidarCornerFilter(msg);
}

// 主函数
void LoamHandle::run() {
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    loam_localization_.runAlgorithm();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t2).count();
//  std::cout<<"time:"<<time_round<<std::endl;
    sendMsg();
}

void LoamHandle::sendMsg() {
//   lidarFilterPublisher_.publish(lidar_filter_.getLidarFilter());
//   lidarFilterGroundPublisher_.publish(lidar_filter_.getLidarGroundFilter());
//   lidarFilterCornerPublisher_.publish(lidar_filter_.getLidarCornerFilter());
}

}