// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl_conversions/pcl_conversions.h"

#include "encoder_module.h"
#include "decoder_module.h"
#include "struct.h"

#include "rcpcc/msg/compressed_point_cloud.hpp"

using namespace std::chrono_literals;


class RCPCC : public rclcpp::Node
{
public:
  RCPCC()
  : Node("minimal_publisher")
  {
    this->declare_parameter<std::string>("csv_folder_path", "/tmp");
    this->declare_parameter<std::string>("pointcloud_topic", "/velodyne/velodyne_points");
    this->declare_parameter<int>("q_level", 2);


    q_level_ = this->get_parameter("q_level").as_int();

    std::string folder_path;
    this->get_parameter("csv_folder_path", folder_path);

    csv_file_path_ = folder_path + "/compress_rcpcc.csv" +"_"+ std::to_string(q_level_);

    publisher_point_cloud_ = this->create_publisher<rcpcc::msg::CompressedPointCloud>("compressed_pointcloud_rcpcc", 10);
    

    std::string pointcloud_topic;
    this->get_parameter("pointcloud_topic", pointcloud_topic);
    RCLCPP_INFO(this->get_logger(), "Subscribing to point cloud topic: %s", pointcloud_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Compression level: %i", q_level_);


    subscriber_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10, std::bind(&RCPCC::point_cloud_callback, this, std::placeholders::_1));

    if (!std::filesystem::exists(csv_file_path_)) {
      std::filesystem::create_directories(std::filesystem::path(csv_file_path_).parent_path()); 
      std::ofstream file(csv_file_path_);
      if (file.is_open()) {
        file << "time_stamp, points_number, compresion_time, size_after_compresion[B]\n";  // Header row
        file.close();
      }
    }
  }

private:

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud with width: %u", msg->width);

    auto start_compresion = std::chrono::steady_clock::now();
    std::vector<point_cloud> pcloud_data;

    

    // convert pointCloud2 to pointCloud from pcl
    ros_pcl_2_rcppc_pcl(msg, pcloud_data);


    EncoderModule encoder(4, this->q_level_);
    std::vector<char> encoded_data = encoder.encodeToData(pcloud_data, true);

    auto size = encoded_data.size();
    
    // RCLCPP_INFO(this->get_logger(), "size: %i", size);

    auto end_compresion = std::chrono::steady_clock::now();
    auto elapsed_compresion = std::chrono::duration_cast<std::chrono::microseconds>(end_compresion - start_compresion).count();

    auto msg_pub = rcpcc::msg::CompressedPointCloud();
    msg_pub.header = msg->header;
    msg_pub.data = std::vector<uint8_t>(encoded_data.begin(), encoded_data.end());
    msg_pub.qlevel = q_level_;



    publisher_point_cloud_->publish(msg_pub);

    std::ofstream ofs(csv_file_path_, std::ios_base::app);
    if (!ofs) {
      std::filesystem::create_directories(std::filesystem::path(csv_file_path_).parent_path()); 
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", csv_file_path_.c_str());
      return;
    }
    double stamp = rclcpp::Time(msg->header.stamp).seconds();
    ofs<< std::fixed << std::setprecision(6) <<stamp << ", " << msg->width * msg->height << ", " << elapsed_compresion << ", " << size << "\n";
    ofs.close();

  }

  void ros_pcl_2_rcppc_pcl(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    std::vector<point_cloud> &pcloud_data)
    {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msg, point_cloud);

    pcloud_data.resize(point_cloud.size());
    for (size_t i = 0; i < point_cloud.size(); ++i)
    {
      pcloud_data[i].x = point_cloud.points[i].x;
      pcloud_data[i].y = point_cloud.points[i].y;
      pcloud_data[i].z = point_cloud.points[i].z;
    }
   }
  int q_level_;
  std::string csv_file_path_;
  rclcpp::Publisher<rcpcc::msg::CompressedPointCloud>::SharedPtr publisher_point_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_point_cloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RCPCC>());
  rclcpp::shutdown();
  return 0;
}
