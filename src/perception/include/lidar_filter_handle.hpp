#ifndef LIDAR_FILTER_HANDLE_HPP
#define LIDAR_FILTER_HANDLE_HPP

#include "lidar_filter.hpp"

namespace ns_lidar_filter {

class LidarFilterHandle {

 public:
  // Constructor
  LidarFilterHandle(ros::NodeHandle &nodeHandle);

  void subscribeToTopics();
  void publishToTopics();
  void run();
  void rawLidarCallback(const sensor_msgs::PointCloud2 &msg);
  void sendMsg();


 private:
  ros::NodeHandle nodeHandle_;

  ros::Subscriber rawLidarSubscriber_;

  ros::Publisher lidarFilterPublisher_;
  ros::Publisher lidarFilterGroundPublisher_;
  ros::Publisher lidarFilterCornerPublisher_;

  LidarFilter lidar_filter_;

};
}

#endif //LIDAR_CLUSTER_HANDLE_HPP