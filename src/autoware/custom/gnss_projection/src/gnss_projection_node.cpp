// C++ includes
#include <string>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/approximate_voxel_grid.h>// 滤波文件头



// Default values
static int max_iter = 100;        // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 1;   // Step size
static double trans_eps = 0.001;  // Transformation epsilon


pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>());
bool map_loaded = false;
Eigen::Matrix4f map2world_transform = Eigen::Matrix4f::Identity();
bool if_get_gnss_pose = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr dowmsample(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, double leaf_sz = 2)
{
// Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(leaf_sz, leaf_sz, leaf_sz);
  approximate_voxel_filter.setInputCloud(in_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
  std::cout << "Filtered cloud from "<< in_cloud->size()<< " to " << filtered_cloud->size() << "points."<< std::endl;
  return filtered_cloud;
}



static void gnss_pose_callback(const geometry_msgs::PoseStamped& input)
{
  if(if_get_gnss_pose) return;
  Eigen::Affine3d affine;
  tf::poseMsgToEigen (input.pose, affine);
  Eigen::Matrix4d matrix4d = affine.matrix();
  map2world_transform = matrix4d.cast<float>();
  if_get_gnss_pose = true;
  ROS_INFO("Get gnss_pose:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", 
    map2world_transform(0,0), map2world_transform(0,1), map2world_transform(0,2), map2world_transform(0,3), 
    map2world_transform(1,0), map2world_transform(1,1), map2world_transform(1,2), map2world_transform(1,3), 
    map2world_transform(2,0), map2world_transform(2,1), map2world_transform(2,2), map2world_transform(2,3), 
    map2world_transform(3,0), map2world_transform(3,1), map2world_transform(3,2), map2world_transform(3,3)
    );
}

static void map_callback(const sensor_msgs::PointCloud2& input)
{
  pcl::fromROSMsg(input, *global_map);
  map_loaded = true;
  ROS_INFO("Get global map (%lu points)", global_map->points.size());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gnss_projection");
  ros::NodeHandle n;
  ros::Subscriber gnss_pose_sub = n.subscribe("/gnss_pose", 10, gnss_pose_callback);
  ros::Subscriber map_sub = n.subscribe("/points_map", 1, map_callback);
  ros::Publisher local_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/local_map_transformed", 20);
  while(ros::ok())
  {
    if(if_get_gnss_pose && map_loaded)
    {
      break;
    }
    ros::spinOnce();
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_filtered = dowmsample(global_map);


  std::string map_data_path;
  n.getParam("map_data_path", map_data_path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_transformed(new pcl::PointCloud<pcl::PointXYZ>());
  sensor_msgs::PointCloud2 cloud_msg;


  pcl::io::loadPCDFile(map_data_path + "local_map.pcd", *local_map);
  ROS_INFO("Get local_map (%lu points)", local_map->points.size());
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_filtered =  dowmsample(local_map);
  
  ROS_INFO("Registrate local_map to global_map start ...");
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
  // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
  new_ndt.setResolution(ndt_res);
  new_ndt.setInputTarget(global_map_filtered);
  new_ndt.setMaximumIterations(max_iter);
  new_ndt.setStepSize(step_size);
  new_ndt.setTransformationEpsilon(trans_eps);
  new_ndt.setInputSource(local_map_filtered);
  new_ndt.align(*local_map_transformed, map2world_transform);


  ROS_INFO("Registrate local_map to global_map done! has_converged: %d", new_ndt.hasConverged());

  map2world_transform = new_ndt.getFinalTransformation();
  ROS_INFO("Get registration results:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", 
    map2world_transform(0,0), map2world_transform(0,1), map2world_transform(0,2), map2world_transform(0,3), 
    map2world_transform(1,0), map2world_transform(1,1), map2world_transform(1,2), map2world_transform(1,3), 
    map2world_transform(2,0), map2world_transform(2,1), map2world_transform(2,2), map2world_transform(2,3), 
    map2world_transform(3,0), map2world_transform(3,1), map2world_transform(3,2), map2world_transform(3,3)
    );
  //把原始的未经降采样的local_map转换到global_map坐标系
  pcl::transformPointCloud(*local_map, *local_map_transformed, map2world_transform);
  ROS_INFO("Transformed local_map to global_map axis");

  pcl::io::savePCDFileASCII(map_data_path+"local_map_transformed.pcd", *local_map_transformed);
  ROS_INFO("PointCloud saved in to file");

  pcl::toROSMsg(*local_map_transformed, cloud_msg);
  cloud_msg.header.frame_id = "map";
  local_cloud_pub.publish(cloud_msg);
  ROS_INFO("published local map to /local_map_transformed");
  ros::spin();
  return 0;
}
