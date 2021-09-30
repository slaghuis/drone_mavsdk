#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// MAVSDK Sepecific
#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

// From drone_interfaces
#include "drone_interfaces/action/takeoff.hpp"
#include "drone_interfaces/action/land.hpp"
#include "drone_interfaces/srv/arm.hpp"
#include "drone_interfaces/srv/offboard.hpp"

// ROS2 Message Types
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drone_mavsdk/visibility_control.h"

using namespace mavsdk;

struct ConnectionException : public std::exception {
  const char * what () const throw () {
    return "Connection Failed. Check Connection String Parameter.";
  }
};

namespace drone_node
{
class DroneNode : public rclcpp::Node
{
public:
  using Takeoff = drone_interfaces::action::Takeoff;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using Land = drone_interfaces::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

  DRONE_NODE_PUBLIC
  explicit DroneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("drone_commander", options)
  {
    
    using namespace std::placeholders;
    
    _have_global_origin = false;
          
    // Services
    arm_service_ = this->create_service<drone_interfaces::srv::Arm>("drone/arm", 
      std::bind(&DroneNode::arm, this, _1, _2));

    offboard_service_ = this->create_service<drone_interfaces::srv::Offboard>("drone/offboard", 
      std::bind(&DroneNode::offboard, this, _1, _2));
    
    // Action servers
    this->takeoff_action_server_ = rclcpp_action::create_server<Takeoff>(
      this,
      "drone/takeoff",
      std::bind(&DroneNode::takeoff_handle_goal, this, _1, _2),
      std::bind(&DroneNode::takeoff_handle_cancel, this, _1),
      std::bind(&DroneNode::takeoff_handle_accepted, this, _1));

    this->land_action_server_ = rclcpp_action::create_server<Land>(
      this,
      "drone/land",
      std::bind(&DroneNode::land_handle_goal, this, _1, _2),
      std::bind(&DroneNode::land_handle_cancel, this, _1),
      std::bind(&DroneNode::land_handle_accepted, this, _1));


    // Subscribers
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "drone/cmd_vel", 10, std::bind(&DroneNode::cmd_vel_topic_callback, this, _1));
    
    height_subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
      "vl53l1x/range", 5, std::bind(&DroneNode::height_callback, this, _1));

    // Publishers
    using namespace std::chrono_literals;
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("drone/odom", 10);
    battery_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("drone/battery", 5);    
    nav_sat_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("drone/gps", 10);
    
    // TF2 Broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Give the node a second to start up before initiating
    one_off_timer_ = this->create_wall_timer(1000ms, std::bind(&DroneNode::init, this)); 

  }

  private:

  // MAVSDK specific parameters
  std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
  std::shared_ptr<mavsdk::Telemetry> _telemetry;
  std::shared_ptr<mavsdk::Action> _action;
  std::shared_ptr<mavsdk::Offboard> _offboard;
  std::shared_ptr<mavsdk::MavlinkPassthrough> _passthrough;
  std::shared_ptr<mavsdk::Info> _info; 
  
  std::shared_ptr<mavsdk::geometry::CoordinateTransformation> _coordinate_transformation;
  
  // Global Variables
  bool _have_global_origin;
  ConnectionResult connection_result = ConnectionResult::SystemNotConnected;
  float _last_x, _last_y, _last_z;
  
  // ROS2 Parameters
  std::string connection_url;
  
  // General Timers
  rclcpp::TimerBase::SharedPtr one_off_timer_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_publisher_;
  
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  void cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;
  
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr height_subscription_;
  void height_callback(const sensor_msgs::msg::Range::SharedPtr msg) const;
  
  // TF2 Boradcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Action Servers
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_action_server_; 
  rclcpp_action::GoalResponse takeoff_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal);
  rclcpp_action::CancelResponse takeoff_handle_cancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void takeoff_handle_accepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle);
  void takeoff_execute(const std::shared_ptr<GoalHandleTakeoff> goal_handle);

  rclcpp_action::Server<Land>::SharedPtr land_action_server_; 
  rclcpp_action::GoalResponse land_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal);
  rclcpp_action::CancelResponse land_handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle);
  void land_handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle);
  void land_execute(const std::shared_ptr<GoalHandleLand> goal_handle);


  // Services
  rclcpp::Service<drone_interfaces::srv::Arm>::SharedPtr arm_service_;
  void arm(const std::shared_ptr<drone_interfaces::srv::Arm::Request> request,
               std::shared_ptr<drone_interfaces::srv::Arm::Response> response);

  rclcpp::Service<drone_interfaces::srv::Offboard>::SharedPtr offboard_service_;
  void offboard(const std::shared_ptr<drone_interfaces::srv::Offboard::Request> request,
               std::shared_ptr<drone_interfaces::srv::Offboard::Response> response);
               
  // Utiltiy Procedures
  void init();            
  void wait_until_discover();
 
};  // class DroneNode

// Subscriptions //////////////////////////////////////////////////////////////////
void DroneNode::cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{  
  if (!_offboard->is_active()) {
    // Go into offboard mode
      
    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityBodyYawspeed stay{};
    _offboard->set_velocity_body(stay);

    Offboard::Result offboard_result = _offboard->start();
    if(offboard_result != Offboard::Result::Success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode");
    }
  
  }
      
  // Receive messages in ROS base_link FLU ; X->Foreward, Y->Left Z->Up
  // Send to PX4 in FRD : X->Foreward, Y->Rightl Z->Down.
      
  RCLCPP_DEBUG(this->get_logger(), "I heard: foreward = '%f' right = '%f'", msg->linear.x, msg->linear.y);
     
  mavsdk::Offboard::VelocityBodyYawspeed px4_vel_frd{};
  px4_vel_frd.forward_m_s = msg->linear.x;
  px4_vel_frd.right_m_s = -(msg->linear.y);
  px4_vel_frd.down_m_s = -(msg->linear.z);
      
  px4_vel_frd.yawspeed_deg_s = -(msg->angular.z);  // CCW Yaw in ros is positive; CCW Yaw in PX4 is negative.
      
  _offboard->set_velocity_body(px4_vel_frd);

}
  
void DroneNode::height_callback(const sensor_msgs::msg::Range::SharedPtr msg) const
{
  
  // Read the system boot time.  (Is this a hack?)
  std::pair< mavsdk::Info::Result, mavsdk::Info::FlightInfo > information = _info->get_flight_information();
  
  // Pass the height to the flight controller to ease landing  
  mavlink_message_t message;
  float quaternion = 0.0;
  mavlink_msg_distance_sensor_pack(
    _passthrough->get_our_sysid(),                         // ID of this system
    _passthrough->get_our_compid(),                        // ID of this component (e.g. 200 for IMU)
    &message,                                              // The MAVLINK message to compress the data into   
    information.second.time_boot_ms,                       // [ms] Time since system boot
    msg->min_range * 100,                                  // [cm] Minimum distance sensor can measure.  ROS message is in Meters!      
    msg->max_range * 100,                                  // [cm] Maximum distance sensor can measure.  ROS message is in Meters!
    msg->range * 100,                                      // [cm] Current distance reading.  ROS message is in Meters!
    MAV_DISTANCE_SENSOR::MAV_DISTANCE_SENSOR_INFRARED,     // Type of distance sensor. This is not aligned with msg->radiation_type,
    0,                                                     // Onboard ID of sensor     
    MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270, // Direction the sensor faces
    255,                                                   // [cm^2] Measurement Variance.  Mx Standard Deviation is 6cm, UINT8_MAX is unknown 
    msg->field_of_view,                                    // [rad] Horizontal field of view
    msg->field_of_view,                                    // [rad] Vertical field of view
    &quaternion,                                           // This field is required if orientation is set to MAV_SENSOR_ROTATION_CUSTOM.  Set to 0 if invalid)
    0                                                      // Signal quality.  0 = unknown. 1 = invalid signal, 100 = perfect signal
  ); 
  
  _passthrough->send_message(message);
  
  
}

// Action Servers /////////////////////////////////////////////////////////////////
// Takeoff Action Server - Start
// The general strategy here is to use the PX4 Action Plugin to Takeoff. The drone will be armed if it is not already so.

  rclcpp_action::GoalResponse DroneNode::takeoff_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Takeoff::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with target altitude %f meters", goal->target_altitude);
    (void)uuid;
    
    if ( goal->target_altitude < 1.0) {
      RCLCPP_ERROR(this->get_logger(), "Target altitude of %f meters is below the minimum of 1 meter.", goal->target_altitude);
      return rclcpp_action::GoalResponse::REJECT;
    } else if ( goal->target_altitude > 50.0) {
      RCLCPP_ERROR(this->get_logger(), "Target altitude of %f meters is above the maximum floght celing of 50 meters.", goal->target_altitude);
      return rclcpp_action::GoalResponse::REJECT;
    }  
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

  }

  rclcpp_action::CancelResponse DroneNode::takeoff_handle_cancel(
    const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    // Takeoff has been cancelled.  Land the vehicle.

    (void)goal_handle;
    
    const Action::Result land_result = _action->land();
    if (land_result != Action::Result::Success) {
      RCLCPP_ERROR(this->get_logger(), "Landing failed!  Put your helmet on!");
    }

    return rclcpp_action::CancelResponse::ACCEPT;    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DroneNode::takeoff_handle_accepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DroneNode::takeoff_execute, this, _1), goal_handle}.detach();
  }

  void DroneNode::takeoff_execute(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing takeoff");
    rclcpp::Rate loop_rate(5);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Takeoff::Feedback>();
    auto &current_altitude = feedback->current_altitude;

    auto result = std::make_shared<Takeoff::Result>();
    
    _action->set_takeoff_altitude(goal->target_altitude);
    
    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = _telemetry->set_rate_position(5);
    if (set_rate_result != Telemetry::Result::Success) {
        result->result = false;
        RCLCPP_ERROR(this->get_logger(), "Setting rate failed.");
        goal_handle->abort(result);
        return;  
    }
      
    // Set up callback to monitor altitude while the vehicle is in flight
    _telemetry->subscribe_position([&](Telemetry::Position position) {    
      current_altitude = (float)position.relative_altitude_m;      
      // Publish feedback      
      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(this->get_logger(), "Current Altitude: %f", current_altitude);
    });
       
    // Check if armed, else arm
    if (!_telemetry->armed()) {
      Action::Result arm_result = _action->arm();
      if (arm_result != Action::Result::Success) {
        result->result = false;
        RCLCPP_ERROR(this->get_logger(), "Arm failed.");
        goal_handle->succeed(result);
        _telemetry->subscribe_position(nullptr);
        return; 
      }
    }

    const Action::Result takeoff_result = _action->takeoff();
    if (takeoff_result != Action::Result::Success) {
        result->result = false;
        RCLCPP_ERROR(this->get_logger(), "Take off failed.");
        goal_handle->succeed(result);
        _telemetry->subscribe_position(nullptr);
        return;  
    }
    
    while ( (goal->target_altitude- feedback->current_altitude) > 0.1 ) {
      if (goal_handle->is_canceling()) {   // What if the mode has changed??
        result->result = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Takeoff goal cancelled");
        _telemetry->subscribe_position(nullptr);
        return;
      }

      // Publish feedback      
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    _telemetry->subscribe_position(nullptr);

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }  else {
      RCLCPP_WARN(this->get_logger(), "RCLCPP is NOT OK");
      result->result = false;
      goal_handle->succeed(result);
    }
  }

// Takeoff Action Server End
// Land Action Server Start
// The general strategy here is to use the PX4 Action Plugin to Land at the current location. The flight controller 
// will disarm the drone after landing.

  rclcpp_action::GoalResponse DroneNode::land_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal)
  {    
    if (goal->gear_down) {
      // Set the gear down first
      // Not implimented yet
    };

    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse DroneNode::land_handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel land.  What shall I do?");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void DroneNode::land_handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DroneNode::land_execute, this, _1), goal_handle}.detach();
  }

  void DroneNode::land_execute(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    rclcpp::Rate loop_rate(5);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Land::Feedback>();
    auto &current_altitude = feedback->current_altitude;

    auto result = std::make_shared<Land::Result>();

    Offboard::Result offboard_result = _offboard->stop();
    if (offboard_result != Offboard::Result::Success) {
      RCLCPP_INFO(this->get_logger(), "Failed to stop offboard mode.");
    }
    
    RCLCPP_INFO(this->get_logger(), "Landing...");

    // We want to listen to the altitude of the drone at 20 Hz.
    const Telemetry::Result set_rate_result = _telemetry->set_rate_position(5.0);
    if (set_rate_result != Telemetry::Result::Success) {
        result->result = false;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Setting rate failed.");
        return;  
    }
      
    // Set up callback to monitor altitude while the vehicle is in flight
    _telemetry->subscribe_position([&](Telemetry::Position position) {    
      current_altitude = (float)position.relative_altitude_m;      
      // Publish feedback      
      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(this->get_logger(), "Current Altitude: %f", position.relative_altitude_m);
    });

    const Action::Result land_result = _action->land();
    if (land_result != Action::Result::Success) {
      RCLCPP_ERROR(this->get_logger(), "Landing failed!  Put your helmet on!");
      result->result = false;
      goal_handle->canceled(result);
      _telemetry->subscribe_position(nullptr);

      return;

    }

    // Check if vehicle is still in air
    while (_telemetry->in_air()) {
    
      if (goal_handle->is_canceling()) {
        result->result = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Landing goal cancelled");
        _telemetry->subscribe_position(nullptr);

        return;
      }

      // Publish feedback      
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }
    
    _telemetry->subscribe_position(nullptr);
    RCLCPP_INFO(this->get_logger(), "Landed!");

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }  
  }

// Land Action Server End

// Simple Services /////////////////////////////////////////////////////////////////////

void DroneNode::arm(const std::shared_ptr<drone_interfaces::srv::Arm::Request> request,
               std::shared_ptr<drone_interfaces::srv::Arm::Response> response) {
               
  // Check if vehicle is ready to arm
  while (_telemetry->health_all_ok() != true) {
    rclcpp::sleep_for(std::chrono::seconds(1));
  } 
    
  Action::Result arm_result;
  if (request->arm == 0) {
    // Arm vehicle
    RCLCPP_INFO(this->get_logger(), "Arming...");
    arm_result = _action->arm();
  } else {
    // Disarm vehicle
    RCLCPP_INFO(this->get_logger(), "Disarming...");
    arm_result = _action->disarm();
  }  

  response->result = (arm_result == Action::Result::Success);
}

void DroneNode::offboard(const std::shared_ptr<drone_interfaces::srv::Offboard::Request> request,
               std::shared_ptr<drone_interfaces::srv::Offboard::Response> response) {

  Offboard::Result offboard_result;
  if (request->enable == 1) {
    // Switch to offboard
    // Send it once before starting offboard, otherwise it will be rejected.
    Offboard::VelocityBodyYawspeed stay{};
    _offboard->set_velocity_body(stay);

    offboard_result = _offboard->start();
  } else {
    // Disable offboard
    offboard_result = _offboard->stop();
  }  
    
  response->result = (offboard_result == Offboard::Result::Success);    
  
}

// Utilities ////////////////////////////////////////////////////////////////////////////
void DroneNode::init()
{
  // Only run this once.  Stop the timer that triggered this.
  // Should one not run this every second to check if we are still connected to the FC?  Just a though
  this->one_off_timer_->cancel();
  
  // Connect to the flight controller and set up the interfaces for the rest to work.
  _mavsdk = std::make_shared<mavsdk::Mavsdk>();

  Mavsdk::Configuration config( Mavsdk::Configuration::UsageType::CompanionComputer);
  _mavsdk->set_configuration(config);

  this->declare_parameter<std::string>("connection_url", "udp://:14540");
  this->get_parameter("connection_url", connection_url);
  
  RCLCPP_INFO(this->get_logger(), "Connecting with string: %s", connection_url.c_str());
      
  connection_result = _mavsdk->add_any_connection(connection_url.c_str());
      
  if (connection_result != ConnectionResult::Success) {
        throw ConnectionException();
  }
      
  wait_until_discover();     

  auto target_system = _mavsdk->systems().at(0);    
  
  _action = std::make_shared<mavsdk::Action>(target_system);
  _offboard = std::make_shared<mavsdk::Offboard>(target_system);
  _telemetry = std::make_shared<mavsdk::Telemetry>(target_system);
  _passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(target_system);
  _info = std::make_shared<mavsdk::Info>(target_system);

  while (!_telemetry->health_all_ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for system to be ready");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  
  // Subscribe and publish odometry messages
    _telemetry->subscribe_odometry(
        [this](mavsdk::Telemetry::Odometry odometry) { 
      
      // We read maxsdk::Telemetry::Odometry in he FRD frame.  This is called "odom"
      // ROS works in the FLU orientation.  Implimented as "base_link"
      // Publish a transform broadcast to roll the odom PI radians (180 degrees) and update the position from odom to base_link      
      
      _last_x = odometry.position_body.x_m;
      _last_y = -odometry.position_body.y_m;  // Turn Left into right
      _last_z = -odometry.position_body.z_m;  // Turn down into up
          
      rclcpp::Time now = this->get_clock()->now();
          
      // Publish transform odom->base_link    
      geometry_msgs::msg::TransformStamped t;
      
      t.header.stamp = now;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";
          
      t.transform.translation.x = odometry.position_body.x_m;
      t.transform.translation.y = -odometry.position_body.y_m;  // Turn Left into right
      t.transform.translation.z = -odometry.position_body.z_m;  // Turn down into up
         
      //tf2::Quaternion q;
      //q.setRPY(3.1415, 0, 0);
      // Adopt the roll pitch and yaw from the drone.
      t.transform.rotation.x = odometry.q.x;
      t.transform.rotation.y = odometry.q.y;
      t.transform.rotation.z = odometry.q.z;
      t.transform.rotation.w = odometry.q.w;
      tf_broadcaster_->sendTransform(t);
          
      // Publish odometry    
      auto message = nav_msgs::msg::Odometry();

      message.header.stamp = now;
      message.header.frame_id ="odom";

      message.pose.pose.position.x = odometry.position_body.x_m;
      message.pose.pose.position.y = odometry.position_body.y_m;
      message.pose.pose.position.z = odometry.position_body.z_m;
      
      message.pose.pose.orientation.x = odometry.q.x;
      message.pose.pose.orientation.y = odometry.q.y;
      message.pose.pose.orientation.z = odometry.q.z;
      message.pose.pose.orientation.w = odometry.q.w; 
        
      message.pose.covariance[0] = odometry.pose_covariance.covariance_matrix[0];
      message.pose.covariance[7] = odometry.pose_covariance.covariance_matrix[6];
      message.pose.covariance[14] = odometry.pose_covariance.covariance_matrix[11];      
      message.pose.covariance[21] = odometry.pose_covariance.covariance_matrix[15];
      message.pose.covariance[28] = odometry.pose_covariance.covariance_matrix[18];
      message.pose.covariance[35] = odometry.pose_covariance.covariance_matrix[20];
        
      message.twist.twist.linear.x = odometry.velocity_body.x_m_s;
      message.twist.twist.linear.y = odometry.velocity_body.y_m_s;
      message.twist.twist.linear.z = odometry.velocity_body.z_m_s;
            
      message.twist.twist.angular.x = odometry.angular_velocity_body.pitch_rad_s;
      message.twist.twist.angular.y = odometry.angular_velocity_body.roll_rad_s;
      message.twist.twist.angular.z = odometry.angular_velocity_body.yaw_rad_s;

      message.twist.covariance[0] = odometry.velocity_covariance.covariance_matrix[0];
      message.twist.covariance[7] = odometry.velocity_covariance.covariance_matrix[6];
      message.twist.covariance[14] = odometry.velocity_covariance.covariance_matrix[11];      
      message.twist.covariance[21] = odometry.velocity_covariance.covariance_matrix[15];
      message.twist.covariance[28] = odometry.velocity_covariance.covariance_matrix[18];
      message.twist.covariance[35] = odometry.velocity_covariance.covariance_matrix[20];
      
      odom_publisher_->publish(message); 
        
   });  
  
  _telemetry->subscribe_battery(
        [this](mavsdk::Telemetry::Battery battery) {
          
    rclcpp::Time now = this->get_clock()->now();
    
    auto message = sensor_msgs::msg::BatteryState();
       
    message.header.stamp = now;
    message.header.frame_id ="battery";
          
    message.voltage = battery.voltage_v;
    message.temperature = std::numeric_limits<float>::quiet_NaN(); // declared in  <limits>  
    message.current = std::numeric_limits<float>::quiet_NaN();
    message.charge = std::numeric_limits<float>::quiet_NaN();
    message.capacity = std::numeric_limits<float>::quiet_NaN(); 
    message.design_capacity = std::numeric_limits<float>::quiet_NaN();
    message.percentage = battery.remaining_percent;          
    message.present = true;
    message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
          
    battery_publisher_->publish(message);
          
  });     
  
  // Obtain the GPS cooridnates where the estimator has been initialised (map frame [0;0;0])
  // What if I never obtain a fix?  Then I will keep in publishing a zero transform?
  _telemetry->get_gps_global_origin_async(
        [this](mavsdk::Telemetry::Result res, 
               mavsdk::Telemetry::GpsGlobalOrigin origin) {
          
    if(res == mavsdk::Telemetry::Result::Success) {      
      // Setup a projection reference      
      mavsdk::geometry::CoordinateTransformation::GlobalCoordinate global_coordinate;
      global_coordinate.latitude_deg = origin.latitude_deg;
      global_coordinate.longitude_deg = origin.longitude_deg;
      _coordinate_transformation = std::make_shared<mavsdk::geometry::CoordinateTransformation>(global_coordinate); 
      
      _have_global_origin = true;          
    }  
          
  });

  // Use the _global_origin as reference and calculate where we are in the map frame. This shouls give 
  // a good map->odom frame transformation
  _telemetry->subscribe_position([this](Telemetry::Position position) {
      // std::cout << "Vehicle is at: " << position.latitude_deg << ", " << position.longitude_deg
      //            << " degrees\n";
    
    // Publish the gps position as drone/gps_position of type sensor_msgs/mag/NavSatFix
    rclcpp::Time now = this->get_clock()->now();
    
    auto message = sensor_msgs::msg::NavSatFix();       
    message.header.stamp = now;
    message.header.frame_id ="world";
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS; 
    message.latitude = position.latitude_deg;
    message.longitude = position.longitude_deg;
    message.altitude = position.absolute_altitude_m;   // Above MSL    
    nav_sat_publisher_->publish(message);        
    // message.position_covariance[0] = 0.0;  // Leave blank
    message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;          

    if (_have_global_origin) {
        
      // Transform current position
      mavsdk::geometry::CoordinateTransformation::GlobalCoordinate global_coordinate;
      global_coordinate.latitude_deg = position.latitude_deg;
      global_coordinate.longitude_deg = position.longitude_deg;
      auto local = _coordinate_transformation->local_from_global(global_coordinate);
        
      // Publish a map-> odom transform based on the calculated position relative to odometry
      // SHOULD this be done by listening to a transform?  Is there a better way.  What if this is old?
      
      float err_x = _last_x - local.north_m;
      float err_y = _last_y - local.east_m;
      float err_z = _last_z - position.relative_altitude_m;
      RCLCPP_DEBUG(this->get_logger(), "Publishing transform from map->odom [%.2f;%.2f;%.2f]", err_x, err_y, err_z);
              
      //Publish a transform.   
      geometry_msgs::msg::TransformStamped t;
      
      t.header.stamp = now;
      t.header.frame_id = "map";
      t.child_frame_id = "odom";
          
      t.transform.translation.x = err_x;  
      t.transform.translation.y = err_y;  // TEST well, is NED map frames handled correctly???
      t.transform.translation.z = err_z;  
         
      t.transform.rotation.x = 0;
      t.transform.rotation.y = 0;
      t.transform.rotation.z = 0;
      t.transform.rotation.w = 1;
      tf_broadcaster_->sendTransform(t);
    }
    
  });
  
  RCLCPP_INFO(this->get_logger(), "Node is ready");
}
  
void DroneNode::wait_until_discover()
{
    RCLCPP_INFO(this->get_logger(), "Waiting to discover system...");
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    _mavsdk->subscribe_on_new_system([&]() {
        const auto system = _mavsdk->systems().at(0);

        if (system->is_connected()) {
            RCLCPP_INFO(this->get_logger(), "Discovered system");

            discover_promise.set_value();
        }
    });

    discover_future.wait();
}


}  // namespace drone_node

RCLCPP_COMPONENTS_REGISTER_NODE(drone_node::DroneNode)


