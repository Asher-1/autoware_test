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

#ifndef GNSSPROJECTION_CORE_H
#define GNSSPROJECTION_CORE_H

// C++ includes
#include <string>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>

#include <gnss/geo_pos_conv.hpp>
//Third lib
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <ndt_cpu/NormalDistributionsTransform.h>

class GnssProjectionNode
{
public:
  GnssProjectionNode();
  ~GnssProjectionNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_;

  // subscriber
  ros::Subscriber sub1_;

  // constants
  const std::string MAP_FRAME_;
  const std::string GPS_FRAME_;

  // variables
  std::string yaml_path_;
  std::string pcd_path_;
  std::string save_path_;
  int32_t plane_number_;

  geo_pos_conv geo_;
  geo_pos_conv last_geo_;
  double roll_, pitch_, yaw_;
  double orientation_time_, position_time_;
  ros::Time current_time_, orientation_stamp_;
  tf::TransformBroadcaster br_;
  bool orientation_ready_;  // true if position history is long enough to compute orientation

  cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> cpu_ndt_;

  //ndt config
  double voxel_leaf_size_ = 2.0;

  int max_iter_ = 30;        // Maximum iterations
  float ndt_res_ = 1.0;      // Resolution
  double step_size_ = 0.1;   // Step size
  double trans_eps_ = 0.01;  // Transformation epsilon

  // callbacks
  void callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg);

  // initializer
  void initForROS();

  // functions
  void TransformToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                            Eigen::Matrix4f f_mat);

  Eigen::Matrix4f NdtMatching(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
                              pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
                              Eigen::Matrix4f init_mat);

  void publishPoseStamped();
  void createOrientation();
  void convert(std::vector<std::string> nmea, ros::Time current_stamp);
};

std::vector<std::string> split(const std::string &string);
void ReadExtrinsicsParam(std::string yaml_path);
bool ReadPointsCloud(std::string pcd_file, pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr);

#endif  
