// Copyright 2021, 2022 Eric Slaghuis
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

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("odom_tf2_frame_publisher")
  {
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
   
    // Call on_timer function four times a second 
    timer_ = this->create_wall_timer(
      35ms, std::bind(&FramePublisher::on_timer, this));   // PX4 publishes odometry at 28 hz.
      
    RCLCPP_INFO(this->get_logger(), "Publishing transforms from odom -> base_link");  
  }

private:
  void on_timer() //handle_imu_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
    // Received odometry from the IMU.
    // Lookup the transform from base_link_ned to odom
    // Publish a odom->base_link transform
    std::string fromFrameRel = "base_link_ned";
    std::string toFrameRel = "odom";
    rclcpp::Time now = this->get_clock()->now();
    
    geometry_msgs::msg::TransformStamped transformStamped;
    
    try {
      transformStamped = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Transformed fromFrame %s toFrame %s %0.1f, %0.1f, %0.1f", 
        fromFrameRel.c_str(),
        toFrameRel.c_str(),
        transformStamped.transform.translation.x, 
        transformStamped.transform.translation.y, 
        transformStamped.transform.translation.z);
    
    transformStamped.header.frame_id = "odom";
    transformStamped.header.stamp = now;
    transformStamped.child_frame_id = "base_link";
    
    // Send the transformation
    tf_broadcaster_->sendTransform(transformStamped);
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}

