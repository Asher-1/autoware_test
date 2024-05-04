#include <ros/ros.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <eigen_conversions/eigen_msg.h>

std::vector<pcl::PointCloud<pcl::PointXYZRGB> > color_clouds;
std::vector<geometry_msgs::PoseStamped> poses;
std::vector<Eigen::Matrix4d> tf_base_link;

Eigen::Matrix4d getTransform(std::string frame_id, std::string child_frame_id)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tfBuffer.lookupTransform(frame_id, child_frame_id, ros::Time::now(), ros::Duration(5.0));
  return tf2::transformToEigen(transformStamped).matrix();
}

static void color_cloud_callback(const sensor_msgs::PointCloud2& input)
{
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    pcl::fromROSMsg(input, color_cloud);
    color_clouds.push_back(color_cloud);
    ROS_INFO("Get color_clouds (lidar): %lu", color_clouds.size());

}

static void current_pose_callback(const geometry_msgs::PoseStamped& input)
{
  Eigen::Affine3d affine;
  tf::poseMsgToEigen (input.pose, affine);
  tf_base_link.push_back(affine.matrix());
    poses.push_back(input);
    ROS_INFO("Get poses (base_link): %lu", tf_base_link.size());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "build_rgb_map");
  ros::NodeHandle n;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB> rgb_map;
  // Eigen::Matrix4d lidar2vehicle_transform = getTransform("vehicle", "lidar");

  ros::Subscriber color_cloud_sub = n.subscribe("/color_cloud", 100, color_cloud_callback);
  ros::Subscriber current_pose_sub = n.subscribe("/current_pose", 100, current_pose_callback);
  
  static Eigen::Matrix4d tf_btol = getTransform("vehicle", "lidar");
  static int i=0;
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);


  while(ros::ok())
  {
    if(i<color_clouds.size() && i<tf_base_link.size())
    {
      pcl::PointCloud<pcl::PointXYZRGB> scan = color_clouds[i];
      Eigen::Matrix4d transform = tf_base_link[i]*tf_btol;
      pcl::transformPointCloud(scan, *transformed_scan_ptr, transform);
      voxel_grid_filter.setInputCloud(transformed_scan_ptr);
      voxel_grid_filter.filter(*transformed_scan_ptr);
      rgb_map += *transformed_scan_ptr;
      ROS_INFO("concat %d scans to rgb_map(%lu points)", ++i, rgb_map.points.size());
    }
    if(i==99)
    {
      pcl::io::savePCDFileASCII("rgb_map.pcd",rgb_map);
      ROS_INFO("save pointcloud to .ros/rgb_map.pcd");
      return 0;
    }
      ros::spinOnce();
  }


  return 0;
}
