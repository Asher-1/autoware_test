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

#include "gnss_projection_core.h"


Eigen::Matrix4d T_w2m;
cv::Mat T_w2m_cv;

// Constructor
GnssProjectionNode::GnssProjectionNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(-std::numeric_limits<double>::infinity())
  , position_time_(-std::numeric_limits<double>::infinity())
  , current_time_(0)
  , orientation_stamp_(0)
  , orientation_ready_(false)
{
  initForROS();
  geo_.set_plane(plane_number_);
}

// Destructor
GnssProjectionNode::~GnssProjectionNode()
{
}

void GnssProjectionNode::initForROS()
{
  // ros parameter settings
  private_nh_.getParam("plane", plane_number_);
  private_nh_.getParam("yaml_path", yaml_path_);
  private_nh_.getParam("pcd_path", pcd_path_);
  private_nh_.getParam("save_path", save_path_);

  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 100, &GnssProjectionNode::callbackFromNmeaSentence, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
}

void GnssProjectionNode::run()
{
  ros::spin();
}

void GnssProjectionNode::publishPoseStamped()
{
  static bool l_first_pose_flag = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr_b(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr_a(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr_b(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if(l_first_pose_flag && ReadPointsCloud(pcd_path_ + "originB_csf.pcd", in_cloud_ptr_a) 
                       && ReadPointsCloud(pcd_path_ + "originA_csf.pcd", in_cloud_ptr_b) 
                       && ReadPointsCloud(pcd_path_ + "bin_Laser-00147_-00849.pcd", target_cloud_ptr))
  {
    l_first_pose_flag = false;

    //只考虑yaw，假设处于理想平面
    Eigen::Quaternionf q(tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_).w,
                         tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_).x,
                         tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_).y,
                         tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_).z);

    Eigen::Matrix<float, 3, 3> R = q.toRotationMatrix();
    Eigen::Matrix<float, 3, 1> t;
    t<<geo_.y(), geo_.x(), geo_.z();

    Eigen::Matrix4f T_nm2m; //这里计算的是新建地图的原点所在坐标系到原地图坐标系下的坐标
    T_nm2m.setIdentity();
    T_nm2m.topLeftCorner<3,3>()=R;
    T_nm2m.topRightCorner<3,1>()=t;

    ReadExtrinsicsParam(yaml_path_);

    std::cout << T_nm2m << std::endl;
    //基于gnss换算后的坐标位置，将原始点云投影，可以理解为粗匹配
    TransformToMap(in_cloud_ptr_a, out_cloud_ptr_a, T_nm2m);
    TransformToMap(in_cloud_ptr_b, out_cloud_ptr_b, T_nm2m);

    pcl::io::savePCDFileASCII(save_path_ + "temp_result_a.pcd", *out_cloud_ptr_a);
    pcl::io::savePCDFileASCII(save_path_ + "temp_result_b.pcd", *out_cloud_ptr_b);

    //设定ndt初始值，凭感觉和经验，初始值很重要
    Eigen::Matrix<float, 3, 1> t1;
    t1<<5, 5, -2.5;

    Eigen::Matrix4f T_init;
    T_init.setIdentity();
    T_init.topRightCorner<3,1>()=t1;
    
    Eigen::Matrix4f T_final = NdtMatching(out_cloud_ptr_a, target_cloud_ptr, T_init);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_a;
    cloud_transformed_a.reset(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_b;
    cloud_transformed_b.reset(new pcl::PointCloud<pcl::PointXYZ>());

    //just set lidar and base coordinate coincide
    //基于ndt结果进行点云投影，可以理解为精匹配
    pcl::transformPointCloud(*out_cloud_ptr_a, *cloud_transformed_a, T_final);
    pcl::transformPointCloud(*out_cloud_ptr_b, *cloud_transformed_b, T_final);
    //最终的结果保存 可以直接用在官方demo中的地图加载
    pcl::io::savePCDFileASCII(save_path_ + "final_result_a.pcd", *cloud_transformed_a);
    pcl::io::savePCDFileASCII(save_path_ + "final_result_b.pcd", *cloud_transformed_b);
  }
}

void GnssProjectionNode::TransformToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                                           Eigen::Matrix4f f_mat)
{
    out_cloud_ptr->points.clear();
    int j = 0;
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        pcl::PointXYZ p = in_cloud_ptr->points[i];
        cv::Mat p_in = (cv::Mat_<double>(4,1) << p.x, p.y, p.z, 1);
        
        double p_out_x =  f_mat(0,0)*p_in.at<double>(0,0)+f_mat(0,1)*p_in.at<double>(1,0)+f_mat(0,2)*p_in.at<double>(2,0)+f_mat(0,3);
        double p_out_y =  f_mat(1,0)*p_in.at<double>(0,0)+f_mat(1,1)*p_in.at<double>(1,0)+f_mat(1,2)*p_in.at<double>(2,0)+f_mat(1,3);
        double p_out_z =  f_mat(2,0)*p_in.at<double>(0,0)+f_mat(2,1)*p_in.at<double>(1,0)+f_mat(2,2)*p_in.at<double>(2,0)+f_mat(2,3);

        p.x = p_out_x;
        p.y = p_out_y;
        p.z = p_out_z;

        if(!isnan(p.x) && !isnan(p.y) && !isnan(p.z))
        {
          j++;
          out_cloud_ptr->points.push_back(p);
        }
    }
    out_cloud_ptr->height = 1;
	  out_cloud_ptr->width = j;
}

Eigen::Matrix4f GnssProjectionNode::NdtMatching(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, 
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, 
                                                Eigen::Matrix4f init_mat)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_filtered;
    source_cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
    
    cpu_ndt_.setInputTarget(target_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid_filter.setInputCloud(source_cloud);
    voxel_grid_filter.filter(*source_cloud_filtered);
    
    // set ndt source
    cpu_ndt_.setTransformationEpsilon(trans_eps_);
    cpu_ndt_.setStepSize(step_size_);
    cpu_ndt_.setResolution(ndt_res_);
    cpu_ndt_.setMaximumIterations(max_iter_);
    cpu_ndt_.setInputSource(source_cloud_filtered);  

    cpu_ndt_.align(init_mat);  

    std::cout<<"run here!"<<std::endl;
    double fitness_score = cpu_ndt_.getFitnessScore();
    Eigen::Matrix4f t_localizer = cpu_ndt_.getFinalTransformation();
    bool has_converged = cpu_ndt_.hasConverged();
    int final_num_iteration = cpu_ndt_.getFinalNumIteration();
    std::cout<<t_localizer<<std::endl;

    return t_localizer;
}

//copy autoware
void GnssProjectionNode::createOrientation()
{
  yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  roll_ = 0;
  pitch_ = 0;
}
//copy autoware
void GnssProjectionNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  try
  {
    if (nmea.at(0).compare(0, 2, "QQ") == 0)
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
      orientation_stamp_ = current_stamp;
      orientation_ready_ = true;
      ROS_INFO("QQ is subscribed.");
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      orientation_ready_ = true;
      ROS_INFO("PASHR is subscribed.");
    }
    else if (nmea.at(0).compare(3, 3, "GGA") == 0)
    {
      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      double lon = stod(nmea.at(4));
      double h = stod(nmea.at(9));

      if (nmea.at(3) == "S")
        lat = -lat;

      if (nmea.at(5) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GGA is subscribed.");
    }
    else if (nmea.at(0) == "$GPRMC")
    {
      position_time_ = stoi(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;

      if (nmea.at(4) == "S")
        lat = -lat;

      if (nmea.at(6) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GPRMC is subscribed.");
    }
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}
//copy autoware
void GnssProjectionNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  current_time_ = msg->header.stamp;
  convert(split(msg->sentence), msg->header.stamp);

  double timeout = 10.0;

  if (orientation_stamp_.isZero()
      || fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
    double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
    double threshold = 0.2;
    if (dt > threshold)
    {
      if (orientation_ready_)
      {
        ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
        createOrientation();
        publishPoseStamped();
      }
      else
      {
        orientation_ready_ = true;
      }
      last_geo_ = geo_;
    }
    return;
  }

  double e = 1e-2;
  if ((fabs(orientation_time_ - position_time_) < e) && orientation_ready_)
  {
    publishPoseStamped();
    return;
  }
}
//copy autoware
std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);
  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

void ReadExtrinsicsParam(std::string yaml_path)
{
  cv::FileStorage fsSettings(yaml_path, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cout << "File path is" <<yaml_path<< std::endl;
    std::cerr << "ERROR:Wrong path to settings" << std::endl;
    return;
  }

  fsSettings["world2map_ext"] >> T_w2m_cv;

  cv::cv2eigen(T_w2m_cv, T_w2m);
}

bool ReadPointsCloud(std::string pcd_file, pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *in_cloud_ptr )== -1)
    {
        PCL_ERROR("Couldn't read file pcd file\n");
        return false;
    }
    
    return true;
}



