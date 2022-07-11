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

/* **********************************************************************
 * Listens to odometry messages (that typically are published in the
 * odom_ned frame by the drone node. Looks up a transform of the pose 
 * to the "odom" frame and publishes an odom -> base_link tranform.
 * Typically the odom_ned->map frame and the
 *                    map->odom transform are handled by static transform
 * broadcasters. 
 * Together with this node the transform tree is completed.
 * Sensor transforms from base_link is handled elsewhere.
 * **********************************************************************/
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/exceptions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>

#include "message_filters/subscriber.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class FrameBroadcaster : public rclcpp::Node
{
public:
  FrameBroadcaster()
  : Node("frame_broadcaster")
  {
    // Declare and acquire parameters
    double transform_tolerance = 0.1;
    this->declare_parameter<double>("transform_tolerance", 0.1);
    this->get_parameter("transform_tolerance", transform_tolerance);

    this->declare_parameter<std::string>("odometry_frame", "odom");
    this->get_parameter("odometry_frame", odom_frame_);

    this->declare_parameter<std::string>("base_frame", "base_link");
    this->get_parameter("base_frame", target_frame_);
      
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);  
      
    one_off_timer_ = this->create_wall_timer(500ms, std::bind(&FrameBroadcaster::init, this)); 
      
  }

private:

  void init()
  {
    this->one_off_timer_->cancel();
     
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); 
    
    // Initialize the transform filter   
    message_filters::Subscriber<nav_msgs::msg::Odometry> point_sub(this, "/drone/odom");
    tf2_filter_ = new tf2_ros::MessageFilter<nav_msgs::msg::Odometry>(point_sub, *tf_buffer_, odom_frame_, 10, shared_from_this(), 1ms);  
    tf2_filter_->registerCallback(&FrameBroadcaster::msg_callback, this);
    
    // Initialize the transform broadcaster    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);        
    
  }
  
  void msg_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
  {      
    // Copy the odomery message to a suitible structure
    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = msg->header;                     // odometry is broadcast in a odom_ned frame;
    odom_pose.pose.position = msg->pose.pose.position;
    odom_pose.pose.orientation=msg->pose.pose.orientation; 
      
    // Tranform the data to a farmiliar frame
    geometry_msgs::msg::PoseStamped transformed_pose;
    try
    {
      tf_buffer_->transform(odom_pose, transformed_pose, odom_frame_, transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Exception in transform_pose: %s", ex.what());
      return;
    }
    
    // Populate a transform message
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odom_frame_.c_str(); 
    t.child_frame_id = target_frame_.c_str();

    t.transform.translation.x = transformed_pose.pose.position.x;
    t.transform.translation.y = transformed_pose.pose.position.y;
    t.transform.translation.z = transformed_pose.pose.position.z;

    t.transform.rotation.x = transformed_pose.pose.orientation.x;
    t.transform.rotation.y = transformed_pose.pose.orientation.y;
    t.transform.rotation.z = transformed_pose.pose.orientation.z;
    t.transform.rotation.w = transformed_pose.pose.orientation.w;

    // Send the transformation
    RCLCPP_INFO(this->get_logger(), "sending transform from: %s to: %s", t.header.frame_id, t.child_frame_id );
    tf_broadcaster_->sendTransform(t);   
  }  
    
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometrySubscription_;  
  /// 
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::Duration transform_tolerance_;
  std::string target_frame_;
  std::string odom_frame_;
  
  tf2_ros::MessageFilter<nav_msgs::msg::Odometry> * tf2_filter_;
  
  // General Timers
  rclcpp::TimerBase::SharedPtr one_off_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
