
#include "cuda_organized_pointcloud_adapter/cuda_organized_pointcloud_adapter_node.hpp"

#include <cuda_runtime.h>

#include <vector>

namespace cuda_organized_pointcloud_adapter
{
using sensor_msgs::msg::PointCloud2;

CudaOrganizedPointcloudAdapterNode::CudaOrganizedPointcloudAdapterNode(
  const rclcpp::NodeOptions & node_options)
: Node("cuda_organized_pointcloud_adapter", node_options)
{

  pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud");
  
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&CudaOrganizedPointcloudAdapterNode::pointcloudCallback, this, std::placeholders::_1));

  next_ring_index_.fill(0);
  buffer_.resize(MAX_RINGS * MAX_POINTS_PER_RING);

  cudaMalloc(&device_buffer_, MAX_RINGS * MAX_POINTS_PER_RING * sizeof(autoware_point_types::PointXYZIRCAEDT));
}


void CudaOrganizedPointcloudAdapterNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr)
{
  assert(input_pointcloud_msg_ptr->point_step == sizeof(autoware_point_types::PointXYZIRCAEDT));
  const autoware_point_types::PointXYZIRCAEDT * input_buffer = reinterpret_cast<const autoware_point_types::PointXYZIRCAEDT*>(input_pointcloud_msg_ptr->data.data());

  for (std::size_t i = 0; i < input_pointcloud_msg_ptr->width * input_pointcloud_msg_ptr->height; i++)
  {
    const autoware_point_types::PointXYZIRCAEDT & point = input_buffer[i];
    const std::size_t ring = point.channel;
    const std::size_t index = next_ring_index_[ring];
    buffer_[ring * MAX_POINTS_PER_RING + index] = point;
    next_ring_index_[ring] = (index + 1) % MAX_POINTS_PER_RING;
  }

  // Copy to cuda memory
  cudaMemcpy(device_buffer_, buffer_.data(), MAX_RINGS * MAX_POINTS_PER_RING * sizeof(autoware_point_types::PointXYZIRCAEDT), cudaMemcpyHostToDevice);

  auto cuda_pointcloud_msg_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  cuda_pointcloud_msg_ptr->width = MAX_POINTS_PER_RING;
  cuda_pointcloud_msg_ptr->height = MAX_RINGS;
  cuda_pointcloud_msg_ptr->point_step = sizeof(autoware_point_types::PointXYZIRCAEDT);
  cuda_pointcloud_msg_ptr->row_step = MAX_POINTS_PER_RING * sizeof(autoware_point_types::PointXYZIRCAEDT);
  cuda_pointcloud_msg_ptr->data = reinterpret_cast<uint8_t*>(device_buffer_);
  cuda_pointcloud_msg_ptr->is_dense = input_pointcloud_msg_ptr->is_dense;
  cuda_pointcloud_msg_ptr->header = input_pointcloud_msg_ptr->header;

  pub_->publish(std::move(cuda_pointcloud_msg_ptr));

  // Allocate cuda memory
  cudaMalloc(&device_buffer_, MAX_RINGS * MAX_POINTS_PER_RING * sizeof(autoware_point_types::PointXYZIRCAEDT));
  // Clear indexes
  next_ring_index_.fill(0);

  // Clear pointcloud buffer
  std::fill(buffer_.begin(), buffer_.end(), autoware_point_types::PointXYZIRCAEDT{});
}

}  // namespace cuda_organized_pointcloud_adapter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cuda_organized_pointcloud_adapter::CudaOrganizedPointcloudAdapterNode)
