/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/serialization.hpp>
#include <fstream>
#include "common_lib.h"

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // Solid-state (e.g. Livox)
    Mech    = 2    // Mechanical multi-line
};

class DataPreprocess
{
public:
    // Point cloud with ring field for mechanical LiDAR support
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        string bag_path    = params.bag_path;
        string image_path  = params.image_path;
        string lidar_topic = params.lidar_topic;

        // Load image
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::string msg = "Loading the image " + image_path + " failed";
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "%s", msg.c_str());
            return;
        }

        // Check if bag file exists
        std::fstream file_;
        file_.open(bag_path, ios::in);
        if (!file_)
        {
            std::string msg = "Loading the rosbag " + bag_path + " failed";
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "%s", msg.c_str());
            return;
        }
        file_.close();
        
        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), "Loading the rosbag %s", bag_path.c_str());
        
        // ROS 2 rosbag reading
        rosbag2_cpp::readers::SequentialReader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        try {
            reader.open(storage_options, converter_options);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), "LOADING BAG FAILED: %s", e.what());
            return;
        }

        // Discover available topics and their types
        auto topics = reader.get_all_topics_and_types();
        bool topic_found = false;
        std::string actual_topic_type;
        
        for (const auto& topic_info : topics) {
            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                       "Available topic: %s, type: %s", 
                       topic_info.name.c_str(), topic_info.type.c_str());
            
            if (topic_info.name == lidar_topic) {
                actual_topic_type = topic_info.type;
                topic_found = true;
                break;
            }
        }
        
        if (!topic_found) {
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
                        "Topic %s not found in rosbag", lidar_topic.c_str());
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                   "Found topic %s with type: %s", 
                   lidar_topic.c_str(), actual_topic_type.c_str());

        // Set topic filter
        rosbag2_storage::StorageFilter filter;
        filter.topics.push_back(lidar_topic);
        reader.set_filter(filter);

        int message_count = 0;
        
        if (actual_topic_type == "sensor_msgs/msg/PointCloud2") {
            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                       "Processing standard PointCloud2 messages...");

            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pcl_serialization;
            
            while (reader.has_next()) {
                auto bag_message = reader.read_next();
                
                if (bag_message->topic_name == lidar_topic) {
                    try {
                        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                        sensor_msgs::msg::PointCloud2 pcl_msg;
                        pcl_serialization.deserialize_message(&serialized_msg, &pcl_msg);

                        // Check for ring field to determine LiDAR type
                        bool has_ring = false;
                        for (const auto& f : pcl_msg.fields) {
                            if (f.name == "ring") { has_ring = true; break; }
                        }

                        if (message_count == 0) {
                            lidar_type_ = has_ring ? LiDARType::Mech : LiDARType::Solid;
                        }

                        // Convert to Common::Point preserving ring if available
                        for (size_t i = 0; i < pcl_msg.width * pcl_msg.height; ++i) {
                            Common::Point p;
                            // Use pcl_conversions for field extraction
                            sensor_msgs::PointCloud2ConstIterator<float> it_x(pcl_msg, "x");
                            sensor_msgs::PointCloud2ConstIterator<float> it_y(pcl_msg, "y");
                            sensor_msgs::PointCloud2ConstIterator<float> it_z(pcl_msg, "z");
                            it_x += i; it_y += i; it_z += i;
                            p.x = *it_x;
                            p.y = *it_y;
                            p.z = *it_z;
                            if (has_ring) {
                                sensor_msgs::PointCloud2ConstIterator<uint16_t> it_ring(pcl_msg, "ring");
                                it_ring += i;
                                p.ring = *it_ring;
                            } else {
                                p.ring = 0xFFFF;
                            }
                            cloud_input_->push_back(p);
                        }
                        message_count++;
                        
                        if (message_count % 10 == 0) {
                            RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                                       "Processed %d messages, total points: %ld", 
                                       message_count, cloud_input_->size());
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
                                    "Error deserializing message %d: %s", message_count, e.what());
                        continue;
                    }
                }
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("data_preprocess"), 
                        "Unsupported topic type: %s. Only sensor_msgs/msg/PointCloud2 is supported.",
                        actual_topic_type.c_str());
            return;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("data_preprocess"), 
                   "Loaded %ld points from %d messages in the rosbag.", 
                   cloud_input_->size(), message_count);
                   
        if (cloud_input_->size() == 0) {
            RCLCPP_WARN(rclcpp::get_logger("data_preprocess"), 
                       "No points loaded! Check your rosbag and topic configuration.");
        }
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP
