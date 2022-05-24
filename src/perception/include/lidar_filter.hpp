#ifndef LIDAR_FILTER_HPP
#define LIDAR_FILTER_HPP

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

namespace ns_lidar_filter {

class LidarFilter {

 public:
  // Constructor
  LidarFilter(ros::NodeHandle &nh);

  sensor_msgs::PointCloud getLidarFilter();
  sensor_msgs::PointCloud2 getLidarGroundFilter();
  sensor_msgs::PointCloud2 getLidarCornerFilter();

  // Setters
  void setRawLidar(const sensor_msgs::PointCloud2 &msg);

  void runAlgorithm();

  void preprocessing(
    pcl::PointCloud<pcl::PointXYZI> &raw,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones);
  void ClusterProcessing(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold);

 private:
  ros::NodeHandle &nh_;

  sensor_msgs::PointCloud filter_;

  sensor_msgs::PointCloud2 raw_pc2_;

  pcl::PointCloud<pcl::PointXYZI> raw_pc_;

  sensor_msgs::PointCloud2 filter_ground_, filter_cones_;
};
} // namespace ns_lidar_filter

#endif // LIDAR_CLUSTER_HPP
