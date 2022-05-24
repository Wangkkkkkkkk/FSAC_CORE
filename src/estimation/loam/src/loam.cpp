#include <ros/ros.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <opencv2/opencv.hpp>
#include <vector>

#include "loam.hpp"

namespace loam_localization {

// Constructor
loam::loam(ros::NodeHandle &nh) : nh_(nh) {
    isInitial = false;
};

// Setters
void loam::setLidarFilter(const sensor_msgs::PointCloud &msg) {
    lidar_filter_ = msg;
}

void loam::setLidarGroundFilter(const sensor_msgs::PointCloud2 &msg) {
    lidar_filter_ground_ = msg;
}

void loam::setLidarCornerFilter(const sensor_msgs::PointCloud2 &msg) {
    lidar_filter_corner_ = msg;
}

void loam::runAlgorithm() {

    if (lidar_filter_.points.empty()) {
        return;
    }

    pcl::fromROSMsg(lidar_filter_, pcl_lidar_filter_);

    if (!isInitial) {
        pcl::copyPointCloud(pcl_lidar_filter_, last_pcl_lidar_filter_);
        return;
    }

    // 点云匹配
    vector<cv::Point3f> points1;
    vector<cv::Point3f> points2;


    // ICP 求解位姿
    cv::Mat R;
    cv::Mat t;
    pose_estimation_3d3d(points1, points2, R, t);

    cout<< "Get lidar filter success!" <<endl;

}

void loam::pose_estimation_3d3d(const vector<cv::Point3f> &pts1,
                                const vector<cv::Point3f> &pts2,
                                Mat &R, Mat &t) 
{

    cv::Point3f p1, p2;
    int N = pts1



}



}