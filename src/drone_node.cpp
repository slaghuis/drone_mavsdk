#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// MAVSDK Sepecific
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// From drone_interfaces
#include "drone_interfaces/action/takeoff.hpp"
#include "drone_interfaces/action/land.hpp"
#include "drone_interfaces/srv/arm.hpp"
#include "drone_interfaces/srv/offboard.hpp"

// ROS2 Message Types
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

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

    // Publishers
    using namespace std::chrono_literals;
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("drone/odom", 10);
    
    // Give the node a second to start up before initiating
    one_off_timer_ = this->create_wall_timer(1000ms, std::bind(&DroneNode::init, this)); 

  }

  private:

  // MAVSDK specific parameters
  std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
  std::shared_ptr<mavsdk::Telemetry> _telemetry;
  std::shared_ptr<mavsdk::Action> _action;
  std::shared_ptr<mavsdk::Offboard> _offboard;
    
  ConnectionResult connection_result = ConnectionResult::SystemNotConnected;
  
  // ROS2 Parameters
  std::string connection_url;
  
  // General Timers
  rclcpp::TimerBase::SharedPtr one_off_timer_;
  
  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  void cmd_vel_topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;

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
      
  // Receive meaages in ROS base_link FLU ; X->Foreward, Y->Left Z->Up
  // Send to PX4 in FRD : X->Foreward, Y->Rightl Z->Down.
      
  RCLCPP_INFO(this->get_logger(), "I heard: foreward = '%f' right = '%f'", msg->linear.x, msg->linear.y);
     
  mavsdk::Offboard::VelocityBodyYawspeed px4_vel_frd{};
  px4_vel_frd.forward_m_s = msg->linear.x;
  px4_vel_frd.right_m_s = -(msg->linear.y);
  px4_vel_frd.down_m_s = -(msg->linear.z);
      
  px4_vel_frd.yawspeed_deg_s = -(msg->angular.z);  // CCW Yaw in ros is positive; CCW Yaw in PX4 is negative.
      
  _offboard->set_velocity_body(px4_vel_frd);

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
    RCLCPP_INFO(this->get_logger(), "Received request to cancel takeoff");
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
      RCLCPP_INFO(this->get_logger(), "Current Altitude: %f", current_altitude);
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
      RCLCPP_INFO(this->get_logger(), "Current Altitude: %f", position.relative_altitude_m);
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

// Services /////////////////////////////////////////////////////////////////////

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

  while (!_telemetry->health_all_ok()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for system to be ready");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  
  // Subscribe and publish odometry messages
    _telemetry->subscribe_odometry(
        [this](mavsdk::Telemetry::Odometry odometry) { 
      
      // We read maxsdk::Telemetry::Odometry in he FRD frame.  This is called "odom"
      // ROS works in the FLU orientation.  Implimented as "base_link"
      // Publish a static transform broadcaset in your launch file to roll the odom PI radians (180 degrees) from odom to base_link
      
      auto message = nav_msgs::msg::Odometry();

      message.header.stamp = rclcpp::Node::now();//timestamp_.load();
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

