/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/MagneticField.h>

#include <iostream>
#include <gnss/geo_pos_conv.hpp>

#include "message_filters/subscriber.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;
// true if position history is long enough to compute orientation
static bool _orientation_ready = false;

static int _plane;

static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
  geo_pos_conv geo;

  geo.set_plane(_plane);
  geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  // pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = geo.x(); //note-tianyu exchange x and y 
  pose.pose.position.y = geo.y();
  pose.pose.position.z = geo.z();

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  std::cout << "distance : " << distance << std::endl;

  if (distance > 0.1)
  {
    yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _prev_pose = pose;
    _orientation_ready = true;
  }

  if (_orientation_ready)
  {
    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
    stat_publisher.publish(gnss_stat_msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "gps"));
  }
}

static void GnssMagCallback(const sensor_msgs::NavSatFixConstPtr &gnss_msg, const sensor_msgs::MagneticFieldConstPtr &mag_msg)
{
  geo_pos_conv geo;

  geo.set_plane(_plane);
  geo.llh_to_xyz(gnss_msg->latitude, gnss_msg->longitude, gnss_msg->altitude);

  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseStamped pose;
  pose.header = gnss_msg->header;
  // pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  pose.pose.position.x = geo.x(); //note-tianyu exchange x and y 
  pose.pose.position.y = geo.y();
  pose.pose.position.z = geo.z();

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  yaw=atan2(mag_msg->magnetic_field.y,mag_msg->magnetic_field.x);
	std::cout<<"magnetic yaw: "<<yaw/M_PI*180<<std::endl;

  _quat = tf::createQuaternionMsgFromYaw(-yaw);
  _prev_pose = pose;
  _orientation_ready = true;

  if (_orientation_ready)
  {
    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
    stat_publisher.publish(gnss_stat_msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, gnss_msg->header.stamp, "map", "gps"));
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fix2tfpose");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("plane", _plane);
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
  stat_publisher = nh.advertise<std_msgs::Bool>("/gnss_stat", 1000);
  //ros::Subscriber gnss_pose_subscriber = nh.subscribe("gps", 100, GNSSCallback);

  //note-tianyu
  message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "/gps", 100);
  message_filters::Subscriber<sensor_msgs::MagneticField> magnetic_sub(nh, "/magnetic", 1000);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::MagneticField> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gps_sub, magnetic_sub);
	sync.registerCallback(boost::bind(&GnssMagCallback, _1, _2));
  

  ros::spin();
  return 0;
}
