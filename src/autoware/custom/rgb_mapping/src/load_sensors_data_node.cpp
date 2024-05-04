#include <ros/ros.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include "rgb_mapping/PointCloudShader.h"
// 查找文件夹下的制定文件, 如png, pcd文件, 同时输出的按照文件名排名
// https://www.cnblogs.com/chrislzy/p/14942305.html
bool getFileNames(const std::string &path, const std::string &sub_name, std::vector<std::string> &file_name_v)
{
  file_name_v.clear();
  // find
  DIR *p_dir;
  struct dirent *ptr;
  if (!(p_dir = opendir(path.c_str())))
  {
    ROS_WARN("file path don't exist!");
    return false;
  }
  std::string file_name;
  while ((ptr = readdir(p_dir)) != 0)
  {
    file_name = ptr->d_name;
    if (file_name.find(sub_name) != -1) //没有找到返回-1
    {
      file_name_v.emplace_back(file_name);
    }
  }
  closedir(p_dir);

  if (file_name_v.empty())
  {
    ROS_WARN("no file in file path!");
    return false;
  }

  // TODO: 按照string方式排序, 如何按照int方式排序
  std::sort(file_name_v.begin(), file_name_v.end());
  return true;
}


// sensor_msgs::PointCloud2 transformPointCloudMsg(sensor_msgs::PointCloud2 cloud_ptr, const Eigen::Matrix4d& transform)
// {
//     // sensor_msgs to pointxyz
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::fromROSMsg(cloud_ptr, *cloud);
//   pcl::transformPointCloud(*cloud, *cloud, transform);
//   sensor_msgs::PointCloud2 cloud_out;
//   pcl::toROSMsg(*cloud, cloud_out);
//   return cloud_out;
// }


Eigen::Matrix4d getTransform(std::string frame_id, std::string child_frame_id)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tfBuffer.lookupTransform(frame_id, child_frame_id, ros::Time::now(), ros::Duration(3.0));
  return tf2::transformToEigen(transformStamped).matrix();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_sensors_data_node");
  ros::NodeHandle n;

  // Eigen::Matrix4d lidar2vehicle_transform = getTransform("vehicle", "lidar");

  std::string image_data_front_path, image_data_left_back_path, image_data_right_back_path, pcd_data_path;
  if (!n.getParam("/image_data_front_path", image_data_front_path))
  {
    ROS_ERROR("Failed to get param: %s", image_data_front_path.c_str());
    return -1;
  }
  if (!n.getParam("/image_data_left_back_path", image_data_left_back_path))
  {
    ROS_ERROR("Failed to get param: %s", image_data_left_back_path.c_str());
    return -1;
  }
  if (!n.getParam("/image_data_right_back_path", image_data_right_back_path))
  {
    ROS_ERROR("Failed to get param: %s", image_data_right_back_path.c_str());
    return -1;
  }
  if (!n.getParam("/pcd_data_path", pcd_data_path))
  {
    ROS_ERROR("Failed to get param: %s", pcd_data_path.c_str());
    return -1;
  }

  std::vector<std::string> v_image_front_filename, v_image_left_back_filename, v_image_right_back_filename, v_pcd_filename;
  if (!getFileNames(image_data_front_path, ".jpg", v_image_front_filename))
  {
    return -1;
  }
  else
  {
    ROS_INFO("Number of Files in image_data_front_path: %lu", v_image_front_filename.size());
  }
  if (!getFileNames(image_data_left_back_path, ".jpg", v_image_left_back_filename))
  {
    return -1;
  }
  else
  {
    ROS_INFO("Number of Files in image_data_left_back_path: %lu", v_image_left_back_filename.size());
  }
  if (!getFileNames(image_data_right_back_path, ".jpg", v_image_right_back_filename))
  {
    return -1;
  }
  else
  {
    ROS_INFO("Number of Files in image_data_right_back_path: %lu", v_image_right_back_filename.size());
  }
  if (!getFileNames(pcd_data_path, ".pcd", v_pcd_filename))
  {
    return -1;
  }
  else
  {
    ROS_INFO("Number of Files in pcd_data_path: %lu", v_pcd_filename.size());
  }

  ros::Publisher image_front_pub = n.advertise<sensor_msgs::Image>("/camera/front", 1000);
  ros::Publisher image_left_back_pub = n.advertise<sensor_msgs::Image>("/camera/left_back", 1000);
  ros::Publisher image_right_back_pub = n.advertise<sensor_msgs::Image>("/camera/right_back", 1000);
  ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2>("/lidar/points_raw", 20);
  ros::Publisher color_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/color_cloud", 20);
  // process image and pcd one by one
  cv::Mat image_front, image_left_back, image_right_back;
  //定义发布的消息
  pcl::PointCloud<pcl::PointXYZ> pcd_cloud;//cloud_in_vehicle;
  pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
  sensor_msgs::PointCloud2 cloud_msg;
  // Eigen::Matrix4d lidar2vehicle_transform = load_lidar2vehicle();
  // 创建客户端,调用PointCloudShader服务,传入图像和点云,经过服务器计算后,返回彩色
  ros::ServiceClient client = n.serviceClient<rgb_mapping::PointCloudShader>("PointCloudShader");
  rgb_mapping::PointCloudShader srv;
  bool cloud_shader_server_is_on = false;
  if (!n.getParam("cloud_shader_server_is_on", cloud_shader_server_is_on))
  {
    ROS_ERROR("Failed to get param: cloud_shader_server_is_on");
  }

  bool repeat_load_sensors_data = false;
  if (!n.getParam("repeat_load_sensors_data", repeat_load_sensors_data))
  {
    ROS_ERROR("Failed to get param: repeat_load_sensors_data");
  }

  ros::Rate rate(1); //更新频率
  ros::Duration(1).sleep();//稍微延时一点，等待着色服务器初始化
  while (ros::ok())
  {
    for (size_t i = 0; i < v_image_front_filename.size(); ++i)
    {
      // 读取图像和点云
      image_front = cv::imread(image_data_front_path + "/" + v_image_front_filename[i],cv::IMREAD_COLOR);
      image_left_back = cv::imread(image_data_left_back_path + "/" + v_image_left_back_filename[i]),cv::IMREAD_COLOR;
      image_right_back = cv::imread(image_data_right_back_path + "/" + v_image_right_back_filename[i],cv::IMREAD_COLOR);
      pcl::io::loadPCDFile(pcd_data_path + "/" + v_pcd_filename[i], pcd_cloud);
      // pcl::transformPointCloud(pcd_cloud, cloud_in_vehicle, lidar2vehicle_transform);
      // cv::imshow("/image_front", image_front);
      // cv::imshow("/image_left_back", image_left_back);
      // cv::imshow("/image_right_back", image_right_back);
      // cv::waitKey(50);
      // 将图像和点云转换为sensor_msgs消息格式
      sensor_msgs::ImagePtr image_front_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_front).toImageMsg();
      sensor_msgs::ImagePtr image_left_back_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left_back).toImageMsg();
      sensor_msgs::ImagePtr image_right_back_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_right_back).toImageMsg();
      pcl::toROSMsg(pcd_cloud, cloud_msg);
      cloud_msg.header.frame_id = "lidar";
      // 发布图像和点云
      image_front_pub.publish(image_front_msg);
      image_left_back_pub.publish(image_left_back_msg);
      image_right_back_pub.publish(image_right_back_msg);
      pcl_pub.publish(cloud_msg);

      // 如果着色服务器启动了，那么将数据整理好发送给服务器，获得着色后的RGB点云，发布到"/color_cloud"话题上
      if (cloud_shader_server_is_on)
      {
        srv.request.image_front = *image_front_msg;
        srv.request.image_left_back = *image_left_back_msg;
        srv.request.image_right_back = *image_right_back_msg;
        srv.request.cloud = cloud_msg;
        if (client.call(srv))
        {
          color_cloud_pub.publish(srv.response.color_cloud);
          ROS_INFO("Publish color_cloud[frame_id: %s]",srv.response.color_cloud.header.frame_id.c_str());
          // pcl::fromROSMsg(srv.response.color_cloud, color_cloud);
          // std::string filename = pcd_data_path + "/color_cloud/" + std::to_string(i) + ".pcd";
          // pcl::io::savePCDFileASCII(filename, color_cloud);
        }
        else
        {
          ROS_WARN("Failed to call service PointCloudShader");
          // return 1;
        }
      }
      rate.sleep();
    }
    if(repeat_load_sensors_data==false) break;
  }
  ros::spin();
  return 0;
}
