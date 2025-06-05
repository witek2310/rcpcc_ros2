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

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RCPCC : public rclcpp::Node
{
public:
  RCPCC()
  : Node("RCPCC")
  {
    this->declare_parameter<std::string>("csv_folder_path", "/tmp");
    this->declare_parameter<std::string>("output_topic", "/rcpcc_decompressed");
    
    std::string folder_path;
    this->get_parameter("csv_folder_path", folder_path);

    std::string output_topic;
    this->get_parameter("output_topic", output_topic);

    csv_file_path_ = folder_path + "/decompress.csv";

    publisher_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    
    subscriber_point_cloud_ = this->create_subscription<rcpcc::msg::CompressedPointCloud>(
      "compressed_pointcloud_rcpcc", 10, std::bind(&RCPCC::point_cloud_callback, this, std::placeholders::_1));

    if (!std::filesystem::exists(csv_file_path_)) {
      std::filesystem::create_directories(std::filesystem::path(csv_file_path_).parent_path()); 
      std::ofstream file(csv_file_path_);
      if (file.is_open()) {
        file << "cloud_size,number_of_poionts,time_decompresion,compressed_size\n";  // Header row
        file.close();
      }
    }
  }

private:

  void point_cloud_callback(const rcpcc::msg::CompressedPointCloud::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Received point cloud with width: %u", msg->width);
      RCLCPP_INFO(this->get_logger(), "recived compressed point cloud");

    auto start_decompresion = std::chrono::steady_clock::now();
    const std::vector<uint8_t>& compressed_data_u8 = msg->data;

    std::vector<char> encoded_data(compressed_data_u8.begin(), compressed_data_u8.end());

    int size = encoded_data.size(); 

    DecoderModule decoder(encoded_data, 4, true, msg->qlevel);
    auto restored_pcloud = decoder.restored_pcloud;



    // Convert restored point cloud back to PointCloud2
    pcl::PointCloud<pcl::PointXYZ> restored_pcl;
    restored_pcl.width = restored_pcloud.size();
    restored_pcl.height = 1;
    restored_pcl.is_dense = true;
    restored_pcl.points.resize(restored_pcloud.size());


    for (size_t i = 0; i < restored_pcloud.size(); ++i)
    {
      restored_pcl.points[i].x = restored_pcloud[i].x;
      restored_pcl.points[i].y = restored_pcloud[i].y;
      restored_pcl.points[i].z = restored_pcloud[i].z;
    }
    sensor_msgs::msg::PointCloud2 ros_pointcloud2_msg;
    pcl::toROSMsg(restored_pcl, ros_pointcloud2_msg);
    
    auto end_decompresion = std::chrono::steady_clock::now();
    auto elapsed_decompresion = std::chrono::duration_cast<std::chrono::microseconds>(end_decompresion - start_decompresion).count();
    
    
    
    ros_pointcloud2_msg.header= msg->header;
    publisher_point_cloud_->publish(ros_pointcloud2_msg);


    std::ofstream ofs(csv_file_path_, std::ios_base::app);
    if (!ofs) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", csv_file_path_.c_str());
      return;
    }
  ofs << ros_pointcloud2_msg.row_step * ros_pointcloud2_msg.height << "," <<ros_pointcloud2_msg.width * ros_pointcloud2_msg.height<<","<< elapsed_decompresion << ","<< size << "\n";
    ofs.close();
  }


  std::string csv_file_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_;
  rclcpp::Subscription<rcpcc::msg::CompressedPointCloud>::SharedPtr subscriber_point_cloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RCPCC>());
  rclcpp::shutdown();
  return 0;
}
