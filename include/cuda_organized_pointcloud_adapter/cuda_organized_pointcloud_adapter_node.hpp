#ifndef cuda_organized_pointcloud_adapter__cuda_organized_pointcloud_adapter_node_HPP_
#define cuda_organized_pointcloud_adapter__cuda_organized_pointcloud_adapter_node_HPP_

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_point_types/types.hpp>
#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <deque>
#include <memory>
#include <vector>

namespace cuda_organized_pointcloud_adapter
{

class CudaOrganizedPointcloudAdapterNode : public rclcpp::Node
{
public:
  explicit CudaOrganizedPointcloudAdapterNode(const rclcpp::NodeOptions & node_options);
  ~CudaOrganizedPointcloudAdapterNode() = default;

private:
  // Callback
  void pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr);

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_{};

  // CUDA pub
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>> pub_;

  static constexpr std::size_t MAX_RINGS{128};
  static constexpr std::size_t MAX_POINTS_PER_RING{2048};

  std::array<std::size_t, MAX_RINGS> next_ring_index_;
  std::vector<autoware_point_types::PointXYZIRCAEDT> buffer_;
  autoware_point_types::PointXYZIRCAEDT * device_buffer_;

  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;
};

}  // namespace cuda_organized_pointcloud_adapter

#endif  // CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_NODE_HPP_
