#ifndef LOAM_HPP
#define LOAM_HPP

#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include <chrono>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

namespace loam_localization {

class loam {

public:
    loam(ros::NodeHandle &nh);

    void setLidarFilter(const sensor_msgs::PointCloud &msg);
    void setLidarGroundFilter(const sensor_msgs::PointCloud2 &msg);
    void setLidarCornerFilter(const sensor_msgs::PointCloud2 &msg);

    void runAlgorithm();

    void pose_estimation_3d3d(const vector<cv::Point3f> &pts1,
                              const vector<cv::Point3f> &pts2,
                              Mat &R, Mat &t);


private:
    ros::NodeHandle &nh_;

    bool isInitial;

    sensor_msgs::PointCloud lidar_filter_;
    sensor_msgs::PointCloud2 lidar_filter_ground_;
    sensor_msgs::PointCloud2 lidar_filter_corner_;

    pcl::PointCloud<pcl::PointXYZI> pcl_lidar_filter_;
    pcl::PointCloud<pcl::PointXYZI> last_pcl_lidar_filter_;

};

}

#endif