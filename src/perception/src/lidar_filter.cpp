#include "lidar_filter.hpp"
#include <ros/ros.h>
#include <sstream>
#include <utility>

#include <pcl/search/impl/search.hpp>
 
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

namespace ns_lidar_filter {
// Constructor
LidarFilter::LidarFilter(ros::NodeHandle &nh) : nh_(nh) {};

// Getters
sensor_msgs::PointCloud LidarFilter::getLidarFilter() { return filter_; }
sensor_msgs::PointCloud2 LidarFilter::getLidarGroundFilter() { return filter_ground_; }
sensor_msgs::PointCloud2 LidarFilter::getLidarCornerFilter() { return filter_cones_; }

// Setters
void LidarFilter::setRawLidar(const sensor_msgs::PointCloud2 &msg) {
    raw_pc2_ = msg;
}

void LidarFilter::runAlgorithm() {
    if (raw_pc2_.fields.empty()) {
        return;
    }

    pcl::fromROSMsg(raw_pc2_, raw_pc_);  // ros 的点云信息转到 pcl

//   std::cout<< "Get lidar points size: " << raw_pc_.points.size() <<endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground(
        new pcl::PointCloud<pcl::PointXYZI>);  // 初始化 地面点云集
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cones(
        new pcl::PointCloud<pcl::PointXYZI>);  // 初始化 锥桶点云集

    // 分割地面点云并得到原始的扁平化
    preprocessing(raw_pc_, cloud_ground, cloud_cones);

    // 使用集群和多过滤器来获取锥体位置
    ClusterProcessing(cloud_cones, 0.5);

    filter_.header.frame_id = "/base_link";
    filter_.header.stamp = raw_pc2_.header.stamp;
}

void LidarFilter::preprocessing(
        pcl::PointCloud<pcl::PointXYZI> &raw,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones) {

    pcl::PointCloud<pcl::PointXYZI> filtered;

    // 剔除偏离点，标准：
    // 1.点云距离 < sqrt(2)
    // 2.点云高度 > 0.7 （考虑最大锥桶高度不超过 0.7）
    // 3.点云坐标 x < 0 （考虑点云 x 小于 0 是车后方点云，忽略）
    // 4.点云距离 > 7 && 点云高度 < 0.03 （考虑距离过远，地面点云高度不准确）
    for (auto &iter : raw.points) {
        //   cout<< "points x: " << iter.x << " y: " << iter.y << " z: " << iter.z <<endl;
        // if (iter.z > 1.0 || iter.x < 0 || (std::hypot(iter.x, iter.y) > 7 && iter.z < 0.03))
        //   continue;
        filtered.points.push_back(iter);
    }

    // PCL 点云分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // set Distance Threshold
    seg.setDistanceThreshold(0.07);
    seg.setInputCloud(filtered.makeShared());
    seg.segment(*inliers, *coefficients);

    /* Debug
        for (auto iter : coefficients->values) {
        std::cout << iter << " ";
        }
        std::cout << "\n-------------\n";
    */

    // extract ground
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(raw.makeShared());
    extract.setIndices(inliers);
    extract.filter(*cloud_ground);

    // extract cone
    extract.setNegative(true);
    extract.filter(*cloud_cones);

    pcl::toROSMsg(*cloud_ground, filter_ground_);
    pcl::toROSMsg(*cloud_cones, filter_cones_);
}

void LidarFilter::ClusterProcessing(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold) {

    filter_.points.clear();

    // Creating the KdTree object for the search method of the extraction 为提取的搜索方法创建 KdTree 对象
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    // PCL 欧式聚类分割
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(threshold);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(200);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &iter : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cone(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto it : iter.indices) {
        cone->points.push_back(cloud->points[it]);
        }
        cone->width = cone->points.size();
        cone->height = 1;
        cone->is_dense = true;

        Eigen::Vector4f centroid;
        Eigen::Vector4f min;
        Eigen::Vector4f max;
        pcl::compute3DCentroid(*cone, centroid);  // 计算 3D 质心
        pcl::getMinMax3D(*cone, min, max);  // 得到点云的最小 x, y, z 和最大 x, y, z

        float bound_x = std::fabs(max[0] - min[0]);
        float bound_y = std::fabs(max[1] - min[1]);
        float bound_z = std::fabs(max[2] - min[2]);

        // filter based on the shape of cones 基于圆锥形状的过滤器
        // 1.点云集的宽度 < 0.5 && 长度 < 0.5 && 高度 < 0.4
        // 2.质心高度 < 0.4 （考虑排除掉符合尺寸但是高度不对的点云集）
        if (bound_x < 0.8 && bound_y < 0.8 && bound_z < 1.0 && centroid[2] < 0.5) {

            for (auto it : iter.indices) {
                geometry_msgs::Point32 tmp;
                tmp.x = cloud->points[it].x;
                tmp.y = cloud->points[it].y;
                tmp.z = cloud->points[it].z;
                filter_.points.push_back(tmp);
            }

            //  geometry_msgs::Point32 tmp;
            //  tmp.x = centroid[0];
            //  tmp.y = centroid[1];
            //  tmp.z = centroid[2];
            //  filter_.points.push_back(tmp);
        }
    }
}

}