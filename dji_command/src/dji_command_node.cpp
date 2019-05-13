#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <control_toolbox/pid.h>

#include <iostream>

#include <dji_sdk/dji_drone.h>
#include <dji_sdk_lib/DJI_Flight.h>
#include <cstdlib>

using namespace std;
using namespace dji_sdk;
using namespace DJI::onboardSDK;

#include <mav_msgs/default_topics.h>

#include <dji_command_node.h>


PIDAttitudeControllerNode::PIDAttitudeControllerNode(const ros::NodeHandle& nh,
                                                     const ros::NodeHandle private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      got_first_attitude_command_(false)
  {

  drone = new DJIDrone(nh_);

  command_roll_pitch_yawrate_thrust_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
      &PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback, this);

//   odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
//                                 &PIDAttitudeControllerNode::OdometryCallback, this,
//                                 ros::TransportHints().tcpNoDelay());

//   dynamic_reconfigure::Server<mav_linear_mpc::PIDAttitudeConfig>::CallbackType f;
//   f = boost::bind(&PIDAttitudeControllerNode::DynConfigCallback, this, _1, _2);
//   dyn_config_server_.setCallback(f);
}

PIDAttitudeControllerNode::~PIDAttitudeControllerNode()
{
}

void PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference)
{
//   roll_pitch_yawrate_thrust_reference->roll
//   roll_pitch_yawrate_thrust_reference->pitch
//   roll_pitch_yawrate_thrust_reference->yaw_rate
//   roll_pitch_yawrate_thrust_reference->thrust.z
//   got_first_attitude_command_ = true;

if ((drone->flight_control_info.cur_ctrl_dev_in_navi_mode != 2 /*onboard*/) ||
    (drone->flight_control_info.serial_req_status != 1 /*enabled*/))
{
    drone->request_sdk_permission_control();

    // tmp car odom emulation
    //            car_odom.pose = drone->odometry.pose;
    ROS_INFO("Requesting control permission");
    ros::Duration(0.5).sleep();
    return;
}
else
{
    ROS_INFO_ONCE("Got control permission");
}

drone->attitude_control(Flight::HorizontalLogic::HORIZONTAL_ANGLE |
                            Flight::VerticalLogic::VERTICAL_THRUST/*VELOCITY*/ |
                            Flight::YawLogic::YAW_RATE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                        roll_pitch_yawrate_thrust_reference->roll * 180.0/M_PI,
                        roll_pitch_yawrate_thrust_reference->pitch * 180.0/M_PI,
                        roll_pitch_yawrate_thrust_reference->thrust.z,
                        roll_pitch_yawrate_thrust_reference->yaw_rate * 180.0/M_PI);

}

void PIDAttitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("PIDAttitudeController got first odometry message.");

//   if (!got_first_attitude_command_)
//     return;

//   mav_msgs::EigenOdometry odometry;
//   eigenOdometryFromMsg(*odometry_msg, &odometry);

//   PID_attitude_controller_.SetOdometry(odometry);

//   Eigen::VectorXd ref_rotor_velocities;
//   PID_attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

//   mav_msgs::Actuators turning_velocities_msg;

//   turning_velocities_msg.angular_velocities.clear();
//   for (int i = 0; i < ref_rotor_velocities.size(); i++)
//     turning_velocities_msg.angular_velocities.push_back(ref_rotor_velocities[i]);
//   turning_velocities_msg.header.stamp = odometry_msg->header.stamp;

//   motor_velocity_reference_pub_.publish(turning_velocities_msg);
}

// void PIDAttitudeControllerNode::DynConfigCallback(mav_linear_mpc::PIDAttitudeConfig &config,
//                                                   uint32_t level)
// {



int main(int argc, char** argv)
{
  ros::init(argc, argv, "DJIAttitudeControllerNode");
  ros::NodeHandle nh, private_nh("~");
  PIDAttitudeControllerNode PID_attitude_controller(nh, private_nh);
  ros::spin();
  return 0;
}
