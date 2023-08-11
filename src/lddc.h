//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_ROS_DRIVER2_LDDC_H_
#define LIVOX_ROS_DRIVER2_LDDC_H_

#include "include/livox_ros_driver2.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "driver_node.h"
#include "lds.h"
#include <deque>

namespace livox_ros {

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum {
  kOutputToRos = 0,
  kOutputToRosBagFile = 1,
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
  kLivoxQuaternionMsg = 4,
} TransferType;

/** The service type of transfer */
typedef enum {
  kCalibrationSrv = 0,
} TransferSrvType;

/** Type-Definitions based on ROS versions */
#ifdef BUILDING_ROS1
using Publisher = ros::Publisher;
using PublisherPtr = ros::Publisher*;
using PointCloud2 = sensor_msgs::PointCloud2;
using PointField = sensor_msgs::PointField;
using CustomMsg = livox_ros_driver2::CustomMsg;
using CustomPoint = livox_ros_driver2::CustomPoint;
using QuaternionMsg = geometry_msgs::Quaternion
using ImuMsg = sensor_msgs::Imu;
#elif defined BUILDING_ROS2
template <typename MessageT> using Publisher = rclcpp::Publisher<MessageT>;
template <typename MessageT> using Service = rclcpp::Service<MessageT>;
using PublisherPtr = std::shared_ptr<rclcpp::PublisherBase>;
using ServicePtr = std::shared_ptr<rclcpp::ServiceBase>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;
using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;
using QuaternionMsg = geometry_msgs::msg::Quaternion;
using ImuMsg = sensor_msgs::msg::Imu;
using TriggerSrv = std_srvs::srv::Trigger;
#endif

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class DriverNode;

class Lddc final {
 public:
#ifdef BUILDING_ROS1
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id, bool lidar_bag, bool imu_bag);
#elif defined BUILDING_ROS2
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
      std::string &frame_id);
#endif
  ~Lddc();

  int RegisterLds(Lds *lds);
  void DistributePointCloudData(void);
  void DistributeImuData(void);
  void CreateBagFile(const std::string &file_name);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }
  void SetRosNode(livox_ros::DriverNode *node) { cur_node_ = node; }

  // void SetRosPub(ros::Publisher *pub) { global_pub_ = pub; };  // NOT USED
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

 public:
  Lds *lds_;

 private:
  void PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t index, LidarDevice *lidar);

  void PublishPointcloud2(LidarDataQueue *queue, uint8_t index);
  void PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index);
  void PublishPclMsg(LidarDataQueue *queue, uint8_t index);

  void PublishImuData(LidarImuDataQueue& imu_data_queue, const uint8_t index);

  void InitPointcloud2MsgHeader(PointCloud2& cloud);
  void InitPointcloud2Msg(const StoragePacket& pkg, PointCloud2& cloud, uint64_t& timestamp);
  void PublishPointcloud2Data(const uint8_t index, uint64_t timestamp, const PointCloud2& cloud);

  void InitCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg, uint8_t index);
  void FillPointsToCustomMsg(CustomMsg& livox_msg, const StoragePacket& pkg);
  void PublishCustomPointData(const CustomMsg& livox_msg, const uint8_t index);

  void InitPclMsg(const StoragePacket& pkg, PointCloud& cloud, uint64_t& timestamp);
  void FillPointsToPclMsg(const StoragePacket& pkg, PointCloud& pcl_msg);
  void PublishPclData(const uint8_t index, const uint64_t timestamp, const PointCloud& cloud);

  void InitImuMsg(const ImuData& imu_data, ImuMsg& imu_msg, uint64_t& timestamp);

  void StoreAccelerationValues(ImuMsg& imu_msg);

  void FillPointsToPclMsg(PointCloud& pcl_msg, LivoxPointXyzrtlt* src_point, uint32_t num);
  void FillPointsToCustomMsg(CustomMsg& livox_msg, LivoxPointXyzrtlt* src_point, uint32_t num,
      uint32_t offset_time, uint32_t point_interval, uint32_t echo_num);

  bool CalibrateLivoxCb(std_srvs::srv::Trigger::Request::SharedPtr req,
                      std_srvs::srv::Trigger::Response::SharedPtr res);

#ifdef BUILDING_ROS2
  PublisherPtr CreatePublisher(uint8_t msg_type, std::string &topic_name, uint32_t queue_size);
  ServicePtr CreateService(uint8_t service_type, std::string &service_name);
#endif

  PublisherPtr GetCurrentPublisher(uint8_t index);
  PublisherPtr GetCurrentImuPublisher(uint8_t index);
  PublisherPtr GetCurrentQuaternionPublisher();
  ServicePtr GetCurrentCalibService();

 private:
  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;
  std::deque<double> _imu_accel_x_vector;
  std::deque<double> _imu_accel_y_vector;
  std::deque<double> _imu_accel_z_vector;
  bool quaternion_vector_filled = false;

#ifdef BUILDING_ROS1
  bool enable_lidar_bag_;
  bool enable_imu_bag_;
  PublisherPtr private_pub_[kMaxSourceLidar];
  PublisherPtr global_pub_;
  PublisherPtr private_imu_pub_[kMaxSourceLidar];
  PublisherPtr global_imu_pub_;
  rosbag::Bag *bag_;
#elif defined BUILDING_ROS2
  PublisherPtr private_pub_[kMaxSourceLidar];
  PublisherPtr global_pub_;
  PublisherPtr private_imu_pub_[kMaxSourceLidar];
  PublisherPtr global_imu_pub_;
  PublisherPtr private_quaternion_pub_[kMaxSourceLidar];
  PublisherPtr quaternion_imu_pub_;
  ServicePtr calib_service_;
#endif

  livox_ros::DriverNode *cur_node_;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER2_LDDC_H_
