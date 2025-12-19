// Copyright 2012, 2013, 2019 Open Source Robotics Foundation, Joshua Whitley
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

#ifndef IMAGE_VIEW__VIDEO_RECORDER_NODE_HPP_
#define IMAGE_VIEW__VIDEO_RECORDER_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace image_view
{

class VideoRecorderNode
  : public rclcpp::Node
{
public:
  explicit VideoRecorderNode(const rclcpp::NodeOptions & options);
  explicit VideoRecorderNode(const VideoRecorderNode &) = default;
  explicit VideoRecorderNode(VideoRecorderNode &&) = default;
  VideoRecorderNode & operator=(const VideoRecorderNode &) = default;
  VideoRecorderNode & operator=(VideoRecorderNode &&) = default;
  ~VideoRecorderNode();

private:
  cv::VideoWriter outputVideo;

  int g_count;
  rclcpp::Time g_last_wrote_time;
  std::string encoding;
  std::string codec;
  double fps;
  double min_depth_range;
  double max_depth_range;
  bool use_dynamic_range;
  int colormap;
  image_transport::Subscriber sub_image;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trigger_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr nav_status_sub;
  bool recording_started;
  bool recording_enabled;
  std::string filename;
  std::string trigger_topic;
  std::string robot_poses_file;
  int total_waypoints;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void trigger_callback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void task_status_callback(const std_msgs::msg::Int32::ConstSharedPtr & msg);
  int load_waypoint_count_from_yaml(const std::string & file_path);
};

}  // namespace image_view

#endif  // IMAGE_VIEW__VIDEO_RECORDER_NODE_HPP_