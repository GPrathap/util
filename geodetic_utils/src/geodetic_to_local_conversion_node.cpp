/*
  Use altitude from 'external_altitude' topic if messages are received
  (To enable the messages arriving, publish to the topic by remapping in the launch file
  Otherwise, altitude from GPS is taken
*/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geodetic_utils/geodetic_conv.hpp>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

bool g_is_sim;
bool g_publish_pose;

geodetic_converter::GeodeticConverter g_geodetic_converter;
sensor_msgs::Imu g_latest_imu_msg;
std_msgs::Float32 g_latest_altitude_msg;
ros::Time g_latest_altitude_msg_receive_time;
geometry_msgs::Vector3Stamped g_latest_velocity_msg;
bool g_got_imu;
bool g_got_altitude;
bool g_got_velocity;

ros::Publisher g_gps_pose_pub;
ros::Publisher g_gps_transform_pub;
ros::Publisher g_gps_position_pub;
ros::Publisher g_odometry_pub;

bool g_trust_gps;
double g_covariance_position_x;
double g_covariance_position_y;
double g_covariance_position_z;
double g_covariance_orientation_x;
double g_covariance_orientation_y;
double g_covariance_orientation_z;
std::string g_frame_id;
std::string g_tf_child_frame_id;

// origin init params
double g_lat_ref;
double g_lon_ref;
double g_alt_ref;
int g_count = 1;
bool gps_ref_is_init = false;
int g_its = 5;

enum EMode
{
  MODE_AVERAGE = 0,
  MODE_WAIT
};
// average over, or wait for, n GPS fixes
EMode g_mode = MODE_AVERAGE;


std::shared_ptr<tf::TransformBroadcaster> p_tf_broadcaster;

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  g_latest_imu_msg = *msg;
  g_got_imu = true;
}

void altitude_callback(const std_msgs::Float32ConstPtr& msg)
{
  // Only the z value in the PointStamped message is used
  g_latest_altitude_msg = *msg;
  g_latest_altitude_msg_receive_time = ros::Time::now();
  g_got_altitude = true;
}

void vel_callback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  g_latest_velocity_msg = *msg;
  g_got_velocity = true;
}

void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  if (!g_geodetic_converter.isInitialised()){
    // Origin init part
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
      return;
    }

    g_lat_ref += msg->latitude;
    g_lon_ref += msg->longitude;
    g_alt_ref += msg->altitude;

    ROS_INFO("Current measurement: %3.8f, %3.8f, %4.2f", msg->latitude, msg->longitude, msg->altitude);

    if (g_count == g_its) {
      if (g_mode == MODE_AVERAGE) {
        g_lat_ref /= g_its;
        g_lon_ref /= g_its;
//        g_alt_ref /= g_its;
        g_alt_ref = 0;
      } else {
        g_lat_ref = msg->latitude;
        g_lon_ref = msg->longitude;
//        g_alt_ref = msg->altitude;
        g_alt_ref = 0;
      }

      ros::NodeHandle nh;
      nh.setParam("/gps_ref_latitude", g_lat_ref);
      nh.setParam("/gps_ref_longitude", g_lon_ref);
      nh.setParam("/gps_ref_altitude", /*g_alt_ref*/0);
      nh.setParam("/gps_ref_is_init", true);

      ROS_INFO("Final reference position: %3.8f, %3.8f, %4.2f", g_lat_ref, g_lon_ref, /*g_alt_ref*/0);
      g_geodetic_converter.initialiseReference(g_lat_ref, g_lon_ref, /*g_alt_ref*/0);

      return;
    } else {
      ROS_INFO("    Still waiting for %d measurements", g_its - g_count);
    }

    g_count++;
  }

  else
  {
    // Conversion part

    if( (ros::Time::now() - g_latest_imu_msg.header.stamp).toSec() > 1.5 )
      g_got_imu = false;
    if( (ros::Time::now() - g_latest_altitude_msg_receive_time).toSec() > 1.5 ){
      //ROS_WARN("No altitude msgs");
      g_got_altitude = false;
    }
    if( (ros::Time::now() - g_latest_velocity_msg.header.stamp).toSec() > 1.5 ){
      //ROS_WARN("No velocity msgs");
      g_got_velocity = false;
    }

    if (!g_got_imu) {
      ROS_WARN_STREAM_THROTTLE(1, "No IMU data yet");
      //return;
    }

    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      ROS_WARN_STREAM_THROTTLE(1, "No GPS fix");
      //return;
    }

    if (!g_geodetic_converter.isInitialised()) {
      ROS_WARN_STREAM_THROTTLE(1, "No GPS reference point set, not publishing");
      //return;
    }

    double x, y, z;
    g_geodetic_converter.geodetic2Enu(msg->latitude, msg->longitude, msg->altitude, &x, &y, &z);

    // Fill up pose message
    geometry_msgs::PoseWithCovarianceStampedPtr pose_msg(
          new geometry_msgs::PoseWithCovarianceStamped);
    pose_msg->header = msg->header;
    pose_msg->header.frame_id = g_frame_id;
    pose_msg->pose.pose.position.x = x;
    pose_msg->pose.pose.position.y = y;
    pose_msg->pose.pose.position.z = z;
    pose_msg->pose.pose.orientation = g_latest_imu_msg.orientation;

    // Fill up position message
    geometry_msgs::PointStampedPtr position_msg(
          new geometry_msgs::PointStamped);
    position_msg->header = pose_msg->header;
    position_msg->header.frame_id = g_frame_id;
    position_msg->point = pose_msg->pose.pose.position;

    // If external altitude messages received, include in pose and position messages
    if (g_got_altitude) {
      pose_msg->pose.pose.position.z = g_latest_altitude_msg.data;
      position_msg->point.z = g_latest_altitude_msg.data;
    }

    pose_msg->pose.covariance.assign(0);

    // Set default covariances
    pose_msg->pose.covariance[6 * 0 + 0] = g_covariance_position_x;
    pose_msg->pose.covariance[6 * 1 + 1] = g_covariance_position_y;
    pose_msg->pose.covariance[6 * 2 + 2] = g_covariance_position_z;
    pose_msg->pose.covariance[6 * 3 + 3] = g_covariance_orientation_x;
    pose_msg->pose.covariance[6 * 4 + 4] = g_covariance_orientation_y;
    pose_msg->pose.covariance[6 * 5 + 5] = g_covariance_orientation_z;

    // Take covariances from GPS
    if (g_trust_gps) {
      if (msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN
          || msg->position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED) {
        // Fill in completely
        for (int i = 0; i <= 2; i++) {
          for (int j = 0; j <= 2; j++) {
            pose_msg->pose.covariance[6 * i + j] = msg->position_covariance[3 * i + j];
          }
        }
      } else if (msg->position_covariance_type
                 == sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN) {
        // Only fill in diagonal
        for (int i = 0; i <= 2; i++) {
          pose_msg->pose.covariance[6 * i + i] = msg->position_covariance[3 * i + i];
        }
      }
    }

    if (g_publish_pose) {
      g_gps_pose_pub.publish(pose_msg);
    }
    g_gps_position_pub.publish(position_msg);

    // Fill up transform message
    geometry_msgs::TransformStampedPtr transform_msg(
          new geometry_msgs::TransformStamped);
    transform_msg->header = msg->header;
    transform_msg->header.frame_id = g_frame_id;
    transform_msg->transform.translation.x = pose_msg->pose.pose.position.x;
    transform_msg->transform.translation.y = pose_msg->pose.pose.position.y;
    transform_msg->transform.translation.z = pose_msg->pose.pose.position.z;
    transform_msg->transform.rotation = g_latest_imu_msg.orientation;

    g_gps_transform_pub.publish(transform_msg);

    // Fill up TF broadcaster
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z));
    transform.setRotation(tf::Quaternion(g_latest_imu_msg.orientation.x,
                                         g_latest_imu_msg.orientation.y,
                                         g_latest_imu_msg.orientation.z,
                                         g_latest_imu_msg.orientation.w));
    p_tf_broadcaster->sendTransform(tf::StampedTransform(transform,
                                                         msg->header.stamp,
                                                         g_frame_id,
                                                         g_tf_child_frame_id));


    // Fill up odometry message
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.frame_id = g_frame_id;
    odometry_msg.header.stamp = msg->header.stamp;
    odometry_msg.header.seq = msg->header.seq;
    odometry_msg.pose = pose_msg->pose;

    // velocity_B = orientation_W_B.inverse() * velocity_world;
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(odometry_msg.pose.pose.orientation, tf_quat);
    tf::Vector3 velocity_world(g_latest_velocity_msg.vector.x, g_latest_velocity_msg.vector.y, g_latest_velocity_msg.vector.z);
    tf::Vector3 velocity_B = tf::quatRotate(tf_quat.inverse(), velocity_world);

    odometry_msg.twist.twist.angular = g_latest_imu_msg.angular_velocity;
    odometry_msg.twist.twist.linear.x = velocity_B.x();
    odometry_msg.twist.twist.linear.y = velocity_B.y();
    odometry_msg.twist.twist.linear.z = velocity_B.z();
    g_odometry_pub.publish(odometry_msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "geodetic_to_local_conversion_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  g_got_imu = false;
  g_got_altitude = false;
  p_tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();

  // Specify whether covariances should be set manually or from GPS
  ros::param::param("~trust_gps", g_trust_gps, false);

  // Get manual parameters
  ros::param::param("~fixed_covariance/position/x", g_covariance_position_x,
                    4.0);
  ros::param::param("~fixed_covariance/position/y", g_covariance_position_y,
                    4.0);
  ros::param::param("~fixed_covariance/position/z", g_covariance_position_z,
                    100.0);
  ros::param::param("~fixed_covariance/orientation/x",
                    g_covariance_orientation_x, 0.02);
  ros::param::param("~fixed_covariance/orientation/y",
                    g_covariance_orientation_y, 0.02);
  ros::param::param("~fixed_covariance/orientation/z",
                    g_covariance_orientation_z, 0.11);
  ros::param::param<std::string>("~frame_id",
                                 g_frame_id, "world");
  ros::param::param<std::string>("~tf_child_frame_id",
                                 g_tf_child_frame_id, "gps_receiver");

  // Specify whether to publish pose or not
  ros::param::param("~publish_pose", g_publish_pose, false);

  // Initialize publishers
  g_gps_pose_pub =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps_pose", 1);
  g_gps_transform_pub =
      nh.advertise<geometry_msgs::TransformStamped>("gps_transform", 1);
  g_gps_position_pub =
      nh.advertise<geometry_msgs::PointStamped>("gps_position", 1);
  g_odometry_pub =
      nh.advertise<nav_msgs::Odometry>("odometry", 1);

  // Subscribe to IMU and GPS fixes, and convert in GPS callback
  ros::Subscriber imu_sub = nh.subscribe("imu", 1, &imu_callback);
  ros::Subscriber gps_sub = nh.subscribe("gps", 1, &gps_callback);
  ros::Subscriber vel_sub = nh.subscribe("vel", 1, &vel_callback);
  ros::Subscriber altitude_sub =
     nh.subscribe("external_altitude", 1, &altitude_callback);

  ros::spin();
}
