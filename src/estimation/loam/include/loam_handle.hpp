#ifndef LOAM_HANDLE_HPP
#define LOAM_HANDLE_HPP

#include "loam.hpp"

namespace loam_localization {

class LoamHandle {

 public:
  // Constructor
  LoamHandle(ros::NodeHandle &nodeHandle);

  void subscribeToTopics();
  void publishToTopics();
  void run();
  void LidarFilterCallback(const sensor_msgs::PointCloud &msg);
  void LidarFilterGroundCallback(const sensor_msgs::PointCloud2 &msg);
  void LidarFilterCornerCallback(const sensor_msgs::PointCloud2 &msg);
  void sendMsg();


 private:
  ros::NodeHandle nodeHandle_;

  ros::Subscriber LidarFilterSubscriber_;
  ros::Subscriber LidarFilterGroundSubscriber_;
  ros::Subscriber LidarFilterCornerSubscriber_;

  ros::Publisher lidarFilterPublisher_;
  ros::Publisher lidarFilterGroundPublisher_;
  ros::Publisher lidarFilterCornerPublisher_;

  loam loam_localization_;

};

}

#endif // LOAM_HANDLE_HPP