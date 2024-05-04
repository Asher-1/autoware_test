#include "ros/ros.h"
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "rgb_mapping/PointCloudShader.h" //这是从*.srv文件生成头文件
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include "rgb_mapping/getCameraInfo.h"
#include <tf2_eigen/tf2_eigen.h> //http://wiki.ros.org/tf2_eigen
//#include "colored_pointcloud.h"
#include <cv_bridge/cv_bridge.h>

//相机内参、雷达到相机的坐标变换
//在当前节点启动时，查询tf树、调用相机内参服务，获得相关信息，缓存在节点内，然后不再更新
cv::Mat front_camera_intrinsic;
cv::Mat front_camera_distcoeff;
cv::Size front_camera_imageSize;
Eigen::Matrix4d front_camera_transform;
Eigen::Matrix4d front_camera_transform_backto_vehicle;
cv::Mat left_back_camera_intrinsic;
cv::Mat left_back_camera_distcoeff;
cv::Size left_back_camera_imageSize;
Eigen::Matrix4d left_back_camera_transform;
Eigen::Matrix4d left_back_camera_transform_backto_vehicle;
cv::Mat right_back_camera_intrinsic;
cv::Mat right_back_camera_distcoeff;
cv::Size right_back_camera_imageSize;
Eigen::Matrix4d right_back_camera_transform;
Eigen::Matrix4d right_back_camera_transform_backto_vehicle;

//几个常数矩阵
cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation vector
cv::Mat rMat = cv::Mat::eye(3, 3, CV_64FC1);
cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // Translation vector


// 给定一张图片、一个点云、相机内参(包括畸变系数)、雷达到相机的坐标变化，将像素的RGB赋给点云，得到RGB点云返回.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr shader_pcl(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
    cv::Mat input_image,
    cv::Mat cam_intrinsic,
    cv::Mat cam_distcoeff,
    Eigen::Matrix4d lidar2cam_transform,
    Eigen::Matrix4d cam2vehicle_transform,
    cv::Size img_size)
{
  Eigen::Matrix4d cam2lidar_transform = lidar2cam_transform.inverse();
  if (input_cloud_ptr->size() == 0)
  {

    ROS_WARN("input cloud is empty, please check it out!");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // transform lidar points from lidar coordinate to camera coordiante
  pcl::transformPointCloud(*input_cloud_ptr, *transformed_cloud, lidar2cam_transform); // lidar coordinate(forward x+, left y+, up z+)
                                                                                       // front camera coordiante(right x+, down y+, forward z+) (3D-3D)
                                                                                       // using the extrinsic matrix between this two coordinate system
  std::vector<cv::Point3d> lidar_points;
  std::vector<float> intensity;
  std::vector<cv::Point2d> imagePoints;

  // reserve the points in front of the camera(z>0)
  for (int i = 0; i <= transformed_cloud->points.size(); i++)
  {
    if (transformed_cloud->points[i].z > 0)
    {
      lidar_points.push_back(cv::Point3d(transformed_cloud->points[i].x, transformed_cloud->points[i].y, transformed_cloud->points[i].z));
      // intensity.push_back(transformed_cloud->points[i].intensity);
    }
  }

  // project lidar points from the camera coordinate to the image coordinate(right x+, down y+)
  cv::projectPoints(lidar_points, rMat, tVec, cam_intrinsic, cam_distcoeff, imagePoints);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_transback(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < imagePoints.size(); i++)
  {
    if (imagePoints[i].x >= 0 && imagePoints[i].x < img_size.width && imagePoints[i].y >= 0 && imagePoints[i].y < img_size.height)
    {
      pcl::PointXYZRGB point;          // reserve the lidar points in the range of image
      point.x = lidar_points[i].x; // use 3D lidar points and RGB value of the corresponding pixels
      point.y = lidar_points[i].y; // to create colored point clouds
      point.z = lidar_points[i].z;
      point.r = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[2];
      point.g = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[1];
      point.b = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[0];
      // point.i = intensity[i];
      colored_cloud->points.push_back(point);
    }
  }
  // transform colored points from camera coordinate to lidar coordinate
  // pcl::transformPointCloud(*colored_cloud, *colored_cloud_transback, cam2vehicle_transform);
  pcl::transformPointCloud(*colored_cloud, *colored_cloud_transback, cam2lidar_transform);
  return colored_cloud_transback;
}
// 服务回调函数
// 请求: 3张图片+原始点云
// 响应: RGB点云
bool shader(rgb_mapping::PointCloudShader::Request &req,
            rgb_mapping::PointCloudShader::Response &res)
{

  cv::Mat front_image,left_back_image,right_back_image;
  cv_bridge::CvImagePtr front_cv_ptr,left_back_cv_ptr,right_back_cv_ptr;
  // sensor_msgs to cv image
  try
  {
    front_cv_ptr = cv_bridge::toCvCopy(req.image_front, sensor_msgs::image_encodings::BGR8);
    left_back_cv_ptr = cv_bridge::toCvCopy(req.image_left_back, sensor_msgs::image_encodings::BGR8);
    right_back_cv_ptr = cv_bridge::toCvCopy(req.image_right_back, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception e)
  {
    ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
    return false;
  }
  front_image = front_cv_ptr->image;
  left_back_image = left_back_cv_ptr->image;
  right_back_image = right_back_cv_ptr->image;

  // sensor_msgs to pointxyz
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(req.cloud, *input_cloud_ptr);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_transback(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_transback_front = shader_pcl(input_cloud_ptr, front_image,front_camera_intrinsic, front_camera_distcoeff,front_camera_transform,front_camera_transform_backto_vehicle,front_camera_imageSize);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_transback_left_back = shader_pcl(input_cloud_ptr, left_back_image,left_back_camera_intrinsic, left_back_camera_distcoeff,left_back_camera_transform,left_back_camera_transform_backto_vehicle,left_back_camera_imageSize);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_transback_right_back = shader_pcl(input_cloud_ptr, right_back_image,right_back_camera_intrinsic, right_back_camera_distcoeff,right_back_camera_transform,right_back_camera_transform_backto_vehicle,right_back_camera_imageSize);
  // 三个相机视角下的点云合并
  *colored_cloud_transback += (*colored_cloud_transback_front);
  *colored_cloud_transback += (*colored_cloud_transback_left_back);
  *colored_cloud_transback += (*colored_cloud_transback_right_back);
  pcl::toROSMsg(*colored_cloud_transback, res.color_cloud);
  res.color_cloud.header = req.cloud.header;
  res.color_cloud.header.frame_id = "lidar";
  return true;
}

// 查询tf树和相机内参服务器，将结果缓存在本地
void lookupParams(ros::NodeHandle &n,
  Eigen::Matrix4d& lidar2cam_transform,
  Eigen::Matrix4d& cam2vehicle_transform,
  std::string cam_frame_id, 
  cv::Mat& cam_intrinsic,
  cv::Mat& cam_distcoeff,
  cv::Size& cam_size)
{
  // 查询相机和雷达之间的坐标变换
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped, transformStamped_cam2vehicle;
  transformStamped = tfBuffer.lookupTransform(cam_frame_id, "lidar", ros::Time::now(), ros::Duration(3.0));
  ROS_INFO("translation: [%f %f %f], rotation: [%f %f %f %f]",
           transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z,
           transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  lidar2cam_transform = tf2::transformToEigen(transformStamped).matrix();

  transformStamped_cam2vehicle = tfBuffer.lookupTransform("lidar", cam_frame_id, ros::Time::now(), ros::Duration(3.0));

  cam2vehicle_transform = tf2::transformToEigen(transformStamped_cam2vehicle).matrix();
  // 查询相机内参
  ros::ServiceClient client = n.serviceClient<rgb_mapping::getCameraInfo>("/rgb_mapping/load_params_node/getCameraInfo");
  rgb_mapping::getCameraInfo srv;
  srv.request.frame_id =  cam_frame_id + "_camera_intrinsic";
  if (client.call(srv))
  {
    sensor_msgs::CameraInfo cam = srv.response.camera_info;
    ROS_INFO("get CameraInfo: %s", cam.header.frame_id.c_str());
    cam_size = cv::Size(cam.width, cam.height);
    cam_intrinsic = (cv::Mat_<double>(3, 3) << cam.K[0], cam.K[1], cam.K[2], cam.K[3], cam.K[4], cam.K[5], cam.K[6], cam.K[7], cam.K[8]);
    cam_distcoeff = (cv::Mat_<double>(5, 1) << cam.D[0], cam.D[1], cam.D[2], cam.D[3], cam.D[4]);
  }
  else
  {
    ROS_WARN("Failed to call service getCameraInfo");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_shader_server");
  ros::NodeHandle n;
  //查询三个相机的相关参数，缓存在全局变量中
  lookupParams(n, front_camera_transform, front_camera_transform_backto_vehicle, "front", front_camera_intrinsic,front_camera_distcoeff,front_camera_imageSize);
  lookupParams(n, left_back_camera_transform, left_back_camera_transform_backto_vehicle, "left_back", left_back_camera_intrinsic,left_back_camera_distcoeff,left_back_camera_imageSize);
  lookupParams(n, right_back_camera_transform, right_back_camera_transform_backto_vehicle, "right_back", right_back_camera_intrinsic,right_back_camera_distcoeff,right_back_camera_imageSize);
  //启动着色服务器，等待图像和点云数据，返回RGB点云
  ros::ServiceServer service = n.advertiseService("PointCloudShader", shader);
  ROS_INFO("Ready to shader the cloud.");
  ros::spin();
  return 0;
}
