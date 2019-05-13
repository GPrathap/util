/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <stdio.h>
#include <memory.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <dji_sdk/RCChannels.h>

#include <dynamic_reconfigure/server.h>

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

class DJIIdentificationNode
{
 public:
  DJIIdentificationNode(const ros::NodeHandle& nh, const ros::NodeHandle private_nh);
  ~DJIIdentificationNode();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  DJIDrone* drone;

  // subscribers
  ros::Subscriber rc_sub_;
  ros::Subscriber odometry_sub_;
  bool got_first_attitude_command_;

  ros::Publisher rpy_ref_pub_;


  void RCCallback(
      const dji_sdk::RCChannelsPtr& rc_cmd);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

//   void DynConfigCallback(mav_linear_mpc::PIDAttitudeConfig &config, uint32_t level);

//   dynamic_reconfigure::Server<mav_linear_mpc::PIDAttitudeConfig> dyn_config_server_;

};

