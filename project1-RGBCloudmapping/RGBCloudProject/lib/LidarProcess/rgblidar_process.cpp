#include "rgblidar_process.h"

template<typename PointT>
RgbLidarProcess<PointT>::RgbLidarProcess(std::string config_file, 
                                                                                            std::vector<std::string> sensor_names):
config_file_(config_file),
sensor_names_(sensor_names)
{
  ReadParameters(config_file, sensor_names_);
}

template<typename PointT>
RgbLidarProcess<PointT>::RgbLidarProcess(std::string config_file):
config_file_(config_file)
{
  ReadParameters(config_file_);
}

template<typename PointT>
RgbLidarProcess<PointT>::~RgbLidarProcess() 
{

}

template<typename PointT>
bool RgbLidarProcess<PointT>::ReadParameters(std::string config_file)
{
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cout << "File path is" <<config_file<< std::endl;
    std::cerr << "ERROR:Wrong path to settings" << std::endl;
    return false;
  }
  
  fsSettings["T_lidar_world"] >> T_lidar_world;
  std::cout <<" T_lidar_world \n" << T_lidar_world << std::endl; 
  fsSettings["T_cam_world"] >> T_cam_world;
  std::cout <<" T_cam_world \n" << T_cam_world << std::endl; 
  fsSettings["T_lidar_cam_offset"] >> T_lidar_cam_offset;
  std::cout <<" T_lidar_cam_offset \n" << T_lidar_cam_offset << std::endl; 
  fsSettings["T_lidar_cam"] >> T_lidar_cam;
  CheckTransform(T_lidar_cam);
  std::cout <<" T_lidar_cam \n" << T_lidar_cam << std::endl; 
  fsSettings["K_cam"] >> K_cam;
  std::cout <<" K_cam \n" << K_cam << std::endl; 
  fsSettings["D_cam"] >> D_cam;
  std::cout <<" D_cam \n" << D_cam << std::endl; 
  return true;
}

template<typename PointT>
bool RgbLidarProcess<PointT>::ReadParameters(std::string config_file, std::vector<std::string> sensor_names)
{
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cout << "File path is" <<config_file<< std::endl;
    std::cerr << "ERROR:Wrong path to settings" << std::endl;
    return false;
  }

  for(int i = 0; i < sensor_names.size(); i++)
  {
    if(sensor_names[i] == "lidar" && i==0)
    {
      std::string lidar_ext = "T_vechicle_" + sensor_names[i];
      cv::Mat T_vechicle_lidar_temp;
      fsSettings[lidar_ext] >> T_vechicle_lidar_temp;
      std::cout <<" T_vechicle_lidar_temp \n" << T_vechicle_lidar_temp << std::endl; 
      T_vechicle_lidar_= TransformCvmat(T_vechicle_lidar_temp);
    }
    else if(sensor_names[i] == "front_camera")
    {
      std::string camera_ext = "T_vechicle_" + sensor_names[i];
      std::string camera_int_k = "K_" + sensor_names[i];
      std::string camera_int_d = "D_" + sensor_names[i];
      cv::Mat T_vechicle_front_camera_temp;
      fsSettings[camera_ext] >> T_vechicle_front_camera_temp;
      std::cout<<camera_ext<<std::endl;
       std::cout <<" T_vechicle_front_camera_temp \n" << T_vechicle_front_camera_temp << std::endl; 
      T_vechicle_front_camera_= TransformCvmat(T_vechicle_front_camera_temp);
      fsSettings[camera_int_k] >> K_cam_front_camera_;
       std::cout <<" K_cam_front_camera_ \n" << K_cam_front_camera_ << std::endl; 
      fsSettings[camera_int_d] >> D_cam_front_camera_;
       std::cout <<" D_cam_front_camera_ \n" << D_cam_front_camera_ << std::endl; 

      cv::Mat T_front_camera_vechicle;
      cv::invert(T_vechicle_front_camera_, T_front_camera_vechicle);
      T_front_camera_lidar_ =  T_front_camera_vechicle*T_vechicle_lidar_;
      std::cout <<" T_front_camera_lidar_ \n" << T_front_camera_lidar_ << std::endl; 
    }
    else if(sensor_names[i] == "left_back_camera")
    {
      std::string camera_ext = "T_vechicle_" + sensor_names[i];
      std::string camera_int_k = "K_" + sensor_names[i];
      std::string camera_int_d = "D_" + sensor_names[i];
      cv::Mat T_vechicle_left_back_camera_temp;
      fsSettings[camera_ext] >> T_vechicle_left_back_camera_temp;
       std::cout <<" T_vechicle_left_back_camera_temp \n" << T_vechicle_left_back_camera_temp << std::endl; 
      T_vechicle_left_back_camera_= TransformCvmat(T_vechicle_left_back_camera_temp);
      fsSettings[camera_int_k] >> K_cam_left_back_camera_;
       std::cout <<" K_cam_left_back_camera_ \n" << K_cam_left_back_camera_ << std::endl; 
      fsSettings[camera_int_d] >> D_cam_left_back_camera_;
       std::cout <<" D_cam_left_back_camera_ \n" << D_cam_left_back_camera_ << std::endl; 

      cv::Mat T_left_back_camera_vechicle;
      cv::invert(T_vechicle_left_back_camera_, T_left_back_camera_vechicle);
      T_left_back_camera_lidar_ = T_left_back_camera_vechicle * T_vechicle_lidar_;
    }
    else if(sensor_names[i] == "right_back_camera")
    {
      std::string camera_ext = "T_vechicle_" + sensor_names[i];
      std::string camera_int_k = "K_" + sensor_names[i];
      std::string camera_int_d = "D_" + sensor_names[i];
      cv::Mat T_vechicle_right_back_camera_temp;
      fsSettings[camera_ext] >> T_vechicle_right_back_camera_temp;
       std::cout <<" T_vechicle_right_back_camera_temp \n" << T_vechicle_right_back_camera_temp << std::endl; 
      T_vechicle_right_back_camera_= TransformCvmat(T_vechicle_right_back_camera_temp);
      fsSettings[camera_int_k] >> K_cam_right_back_camera_;
       std::cout <<" K_cam_right_back_camera_ \n" << K_cam_right_back_camera_ << std::endl; 
      fsSettings[camera_int_d] >> D_cam_right_back_camera_;
       std::cout <<" D_cam_right_back_camera_ \n" << D_cam_right_back_camera_ << std::endl; 

      cv::Mat T_right_back_camera_vechicle;
      cv::invert(T_vechicle_right_back_camera_, T_right_back_camera_vechicle);
      T_right_back_camera_lidar_ =  T_right_back_camera_vechicle*T_vechicle_lidar_ ;
    }
    else
    {
      std::cout<<"sensor name is not exist! or lidar should be first!"<<std::endl;
      return false;
    }
  }
  return true;
}

template<typename PointT>
bool RgbLidarProcess<PointT>::ReadPointsCloud(std::string file_path, 
                                           PtCdtr<pcl::PointXYZ> raw_cloud_ptr, 
                                           char separator)
{
  std::ifstream ifile(file_path);
  if (!ifile.is_open())
  {
    std::cerr << "Wrong path of lidar datas file! " << std::endl;
    return false;
  }

  int j = 0;
  //PtCdtr<PointT> raw_cloud_ptr(new pcl::PointCloud<PointT>());

  std::string lineStr;
  while (std::getline(ifile, lineStr))
  {
    std::stringstream ss(lineStr);
    std::string str;
   
    pcl::PointXYZ p;

    if (separator == ' ')
    {
      if(typeid(PointT).name() == typeid(p_xyz).name())
      {
        ss >> str; p.x = atof(str.c_str());
        ss >> str; p.y = atof(str.c_str());
        ss >> str; p.z = atof(str.c_str());
        std::cout<<p.x<<","<<p.y<<","<<p.z<<std::endl;
      }
      else if(typeid(PointT).name() == typeid(p_xyzi).name()) //注释位置—code—判断typename的类型 ？？？
      {
        ss >> str; p.x = atof(str.c_str());
        ss >> str; p.y = atof(str.c_str());
        ss >> str; p.z = atof(str.c_str());
        //ss >> str; p.intensity = atof(str.c_str());
        //std::cout<<p.x<<","<<p.y<<","<<p.z<<std::endl;
      }
      else if(typeid(PointT).name() == typeid(p_xyzrgb).name())
      {
        ss >> str; p.x = atof(str.c_str());
        ss >> str; p.y = atof(str.c_str());
        ss >> str; p.z = atof(str.c_str());
        //ss >> str; p.intensity = atof(str.c_str());
        //std::cout<<p.x<<","<<p.y<<","<<p.z<<std::endl;
      }
    }
    else if (separator == ',')
    {
      std::getline(ss, str, separator); p.x = atof(str.c_str());
      std::getline(ss, str, separator); p.y = atof(str.c_str());
      std::getline(ss, str, separator); p.z = atof(str.c_str());
    }
    else
    {
      std::cerr << "Wrong type of separator " << std::endl;
      return false;
    }

    j++;
    raw_cloud_ptr->points.push_back(p);
  }

	raw_cloud_ptr->height = 1;
	raw_cloud_ptr->width = j;

  //TransformToWorld(raw_cloud_ptr, out_cloud_ptr);

  return true;
}

template<typename PointT>
bool RgbLidarProcess<PointT>::ReadPointsCloud(std::string file_path, 
                                                                                                        PtCdtr<pcl::PointXYZ> raw_cloud_ptr)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *raw_cloud_ptr )== -1)
    {
        PCL_ERROR("Couldn't read file pcd fiile\n");
        return false;
    }
    
    return true;
}

template<typename PointT>
void RgbLidarProcess<PointT>::CloudFusionMultiplyRGB(PtCdtr<pcl::PointXYZ> in_cloud_ptr, 
                                                                                                                       std::unordered_map<std::string, std::string> &image_hash,
                                                                                                                       PtCdtr<pcl::PointXYZRGB> out_cloud_ptr )
{
    PtCdtr<pcl::PointXYZ> front_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    PtCdtr<pcl::PointXYZ> left_back_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    PtCdtr<pcl::PointXYZ> right_back_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    PtCdtr<pcl::PointXYZRGB> front_rgb_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    PtCdtr<pcl::PointXYZRGB> left_back_rgb_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    PtCdtr<pcl::PointXYZRGB> right_back_rgb_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(size_t i = 0; i< in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ p = in_cloud_ptr->points[i];
       //front_cloud_ptr->points.push_back(p); //beixing

      if(p.x >= 0) 
      {
        front_cloud_ptr->points.push_back(p);
      }
      else if(p.y > 0)
      {
        left_back_cloud_ptr->points.push_back(p);
      }
      else 
      {
        right_back_cloud_ptr->points.push_back(p);
      }

    }
    
    for(size_t i = 1; i<sensor_names_.size(); i++)
    {
      if(sensor_names_[i] == "front_camera")
      {
        std::unordered_map<std::string, std::string>::const_iterator iterator_image_hash;
        std::string image_path;
        iterator_image_hash = image_hash.find(sensor_names_[i]);
        if(iterator_image_hash == image_hash.end())
        {
          continue;
        }
        image_path = iterator_image_hash->second;
        cv::Mat image = cv::imread(image_path);
        CloudFusionRGB(front_cloud_ptr, image, T_front_camera_lidar_, K_cam_front_camera_, D_cam_front_camera_, front_rgb_cloud_ptr);
        std::cout<<front_cloud_ptr->points.size()<<std::endl;
      }
      else if(sensor_names_[i] == "left_back_camera")
      {
        std::unordered_map<std::string, std::string>::const_iterator iterator_image_hash;
        std::string image_path;
        iterator_image_hash = image_hash.find(sensor_names_[i]);
        if(iterator_image_hash == image_hash.end())
        {
          continue;
        }
        image_path = iterator_image_hash->second;
        cv::Mat image = cv::imread(image_path);
        CloudFusionRGB(left_back_cloud_ptr, image, T_left_back_camera_lidar_, K_cam_left_back_camera_, D_cam_left_back_camera_, left_back_rgb_cloud_ptr);
      }
      else if(sensor_names_[i] == "right_back_camera")
      {
        std::unordered_map<std::string, std::string>::const_iterator iterator_image_hash;
        std::string image_path;
        iterator_image_hash = image_hash.find(sensor_names_[i]);
        if(iterator_image_hash == image_hash.end())
        {
          continue;
        }
        image_path = iterator_image_hash->second;
        cv::Mat image = cv::imread(image_path);
        CloudFusionRGB(right_back_cloud_ptr, image, T_right_back_camera_lidar_, K_cam_right_back_camera_, D_cam_right_back_camera_, right_back_rgb_cloud_ptr);
      }
    }

    std::vector<PtCdtr<pcl::PointXYZRGB>> rgb_clouds;
    rgb_clouds.push_back(front_rgb_cloud_ptr);
    rgb_clouds.push_back(left_back_rgb_cloud_ptr);
    rgb_clouds.push_back(right_back_rgb_cloud_ptr);
    RgbCloudMerge(rgb_clouds, out_cloud_ptr);

}

template<typename PointT>
void RgbLidarProcess<PointT>::RgbCloudMerge(std::vector<PtCdtr<pcl::PointXYZRGB>> &rgb_clouds, PtCdtr<pcl::PointXYZRGB> out_cloud_ptr)
{
  int m = 0;

  for(int i = 0; i<rgb_clouds.size(); i++)
  {
    if(rgb_clouds[i]->points.size() == 0)
      continue;
    
    for(size_t j = 0; j < rgb_clouds[i]->points.size(); j++)
    {
      pcl::PointXYZRGB p = rgb_clouds[i]->points[j];
      out_cloud_ptr->points.push_back(p);
      m++;
    }
  }

  out_cloud_ptr->height = 1;
	out_cloud_ptr->width = m;
}

template<typename PointT>
void RgbLidarProcess<PointT>::CloudFusionRGB(PtCdtr<pcl::PointXYZ> in_cloud_ptr, 
                                                                                                      cv::Mat image,
                                                                                                      cv::Mat T_cam_lidar, //lidar to cam
                                                                                                      cv::Mat K_cam,
                                                                                                      cv::Mat D_cam,
                                                                                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr)
{
    cv::Mat undistort_img = image.clone();
    //beixing 注释掉
    cv::undistort(image, undistort_img, K_cam, D_cam);
    //IplImage image_saved;
    //image_saved = IplImage(undistort_img);
    //cvSaveImage("../data/xiasha/slave/01.jpg", &image_saved);

    std::cout<<"T_cam_lidar: "<<T_cam_lidar<<std::endl;
    std::cout<<"K_cam: "<<K_cam<<std::endl;
    std::cout<<"D_cam: "<<D_cam<<std::endl;

    std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;
    
    int j = 0;

    for(size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ p_lidar(in_cloud_ptr->points[i].x, in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].z);
      pcl::PointXYZ p_cam;
      cv::Mat p_in = (cv::Mat_<double>(4,1) << p_lidar.x, p_lidar.y, p_lidar.z, 1);
      p_cam.x = T_cam_lidar.at<double>(0,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(0,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(0,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(0,3);
      p_cam.y = T_cam_lidar.at<double>(1,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(1,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(1,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(1,3);
      p_cam.z = T_cam_lidar.at<double>(2,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(2,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(2,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(2,3);

      int u = int(p_cam.x*K_cam.at<double>(0,0)/p_cam.z + K_cam.at<double>(0,2));
      int v = int(p_cam.y*K_cam.at<double>(1,1)/p_cam.z + K_cam.at<double>(1,2));
      if((u<=0) || (v<=0) || (u>=undistort_img.cols) || (v>=undistort_img.rows))
        continue;
      projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(u,v), p_lidar));
    }
    
    for (int row = 0; row < undistort_img.rows; row++)
    {
      for (int col = 0; col < undistort_img.cols; col++)
      {
        std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator iterator_3d_2d;
        pcl::PointXYZ corresponding_3d_point;
        pcl::PointXYZRGB colored_3d_point;
        iterator_3d_2d = projection_map.find(cv::Point(col, row));
        if (iterator_3d_2d != projection_map.end())
        {
          corresponding_3d_point = iterator_3d_2d->second;
          cv::Vec3b rgb_pixel = undistort_img.at<cv::Vec3b>(row, col);
          colored_3d_point.x = corresponding_3d_point.x;
          colored_3d_point.y = corresponding_3d_point.y;
          colored_3d_point.z = corresponding_3d_point.z;
          colored_3d_point.r = rgb_pixel[2];
          colored_3d_point.g = rgb_pixel[1];
          colored_3d_point.b = rgb_pixel[0];
          j++;
          out_cloud_ptr->points.push_back(colored_3d_point);
          //std::cout<< colored_3d_point.r <<","<< colored_3d_point.g<<","<< colored_3d_point.b<<std::endl;
        }
      }
    }

    out_cloud_ptr->height = 1;
	  out_cloud_ptr->width = j;
}

template<typename PointT>
void RgbLidarProcess<PointT>::CloudProjectImg(PtCdtr<pcl::PointXYZI> in_cloud_ptr,
                                                                                                      cv::Mat T_cam_lidar, 
                                                                                                      cv::Mat image)
{
    std::unordered_map<cv::Point, pcl::PointXYZI> projection_map;
    cv::Mat project_img = image.clone();
    cv::undistort(image, project_img, K_cam, D_cam);
    
    int j = 0;

    for(size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZI p_lidar = in_cloud_ptr->points[i];
      pcl::PointXYZI p_cam;
      cv::Mat p_in = (cv::Mat_<double>(4,1) << p_lidar.x, p_lidar.y, p_lidar.z, 1);
      p_cam.x = T_cam_lidar.at<double>(0,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(0,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(0,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(0,3);
      p_cam.y = T_cam_lidar.at<double>(1,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(1,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(1,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(1,3);
      p_cam.z = T_cam_lidar.at<double>(2,0)*p_in.at<double>(0,0)+T_cam_lidar.at<double>(2,1)*p_in.at<double>(1,0)+T_cam_lidar.at<double>(2,2)*p_in.at<double>(2,0)+T_cam_lidar.at<double>(2,3);

      int u = int(p_cam.x*K_cam.at<double>(0,0)/p_cam.z + K_cam.at<double>(0,2));
      int v = int(p_cam.y*K_cam.at<double>(1,1)/p_cam.z + K_cam.at<double>(1,2));
      if((u<=0) || (v<=0) || (u>=project_img.cols) || (v>=project_img.rows))
        continue;
      projection_map.insert(std::pair<cv::Point, pcl::PointXYZI>(cv::Point(u,v), p_lidar));
    }
    
    for (int row = 0; row < project_img.rows; row++)
    {
      for (int col = 0; col < project_img.cols; col++)
      {
        std::unordered_map<cv::Point, pcl::PointXYZI>::const_iterator iterator_3d_2d;
        pcl::PointXYZI corresponding_3d_point;
        pcl::PointXYZRGB colored_3d_point;
        iterator_3d_2d = projection_map.find(cv::Point(col, row));
        if (iterator_3d_2d != projection_map.end())
        {
          corresponding_3d_point = iterator_3d_2d->second;
          int pixel_value = 0;
          if(corresponding_3d_point.intensity>=35)
            pixel_value = 255;
          project_img.at<cv::Vec3b>(row, col)[0] = pixel_value;
          project_img.at<cv::Vec3b>(row, col)[1] = pixel_value;
          project_img.at<cv::Vec3b>(row, col)[2] = pixel_value;
        }
      }
    }
    IplImage image_saved;
    image_saved = IplImage(project_img);
    cvSaveImage("reproject_img.jpg", &image_saved);
}

template<typename PointT>
void RgbLidarProcess<PointT>::CheckTransform(cv::Mat &T)
{
  if(T.at<double>(0,3) == 0 && T.at<double>(1,3) == 0 && T.at<double>(2,3) == 0)
  {
    std::cout<<"Update the transform!"<<std::endl;
    cv::Mat T_world_cam;
    cv::Mat T_world_cam_R;
    cv::invert(T_cam_world, T_world_cam);

    T = T_lidar_world*T_world_cam;
    cv::Mat T_t;
    cv::invert(T, T_t);

    cv::Mat T_offset = CalOffsetTransform(T_lidar_cam_offset);
    T = T_t*T_offset;
  }
}

template<typename PointT>
cv::Mat RgbLidarProcess<PointT>::CalOffsetTransform(cv::Mat euler_mat)
{
    Eigen::Vector3d Euler(euler_mat.at<double>(0,0)/180*M_PI, euler_mat.at<double>(1,0)/180*M_PI, euler_mat.at<double>(2,0)/180*M_PI);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(Euler(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(Euler(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(Euler(0),Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d R;
    R=yawAngle*pitchAngle*rollAngle;

    cv::Mat T_ = (cv::Mat_<double>(4, 4) <<   R(0,0), R(0,1), R(0,2), 0,
                                              R(1,0), R(1,1), R(1,2), 0,
                                              R(2,0), R(2,1), R(2,2), 0,
                                              0, 0, 0, 1);
    return T_;
}

template<typename PointT>
cv::Mat RgbLidarProcess<PointT>::TransformCvmat(cv::Mat &T)
{
    if(T.cols == 4 && T.rows == 4)
    {
        return T;
    }
    Eigen::Quaterniond q(T.at<double>(3,0), T.at<double>(0,0), T.at<double>(1,0), T.at<double>(2,0));
    Eigen::Matrix<double, 3, 3> R = q.toRotationMatrix();
    Eigen::Matrix<double, 3, 1> t;
    t<<T.at<double>(4,0), T.at<double>(5,0), T.at<double>(6,0);

    Eigen::Matrix4d T_temp;
    T_temp.setIdentity();
    T_temp.topLeftCorner<3,3>()=R;
    T_temp.topRightCorner<3,1>()=t;
    cv::Mat T_final;
    cv::eigen2cv(T_temp, T_final);
    return T_final;
}

template<typename PointT>
void RgbLidarProcess<PointT>::TransformToWorld(const PtCdtr<PointT> in_cloud_ptr,
                                            PtCdtr<PointT> out_cloud_ptr)
{
    out_cloud_ptr->points.clear();
    int j = 0;
    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        PointT p = in_cloud_ptr->points[i];
        cv::Mat p_in = (cv::Mat_<double>(4,1) << p.x, p.y, p.z, 1);
        
        double p_out_x =  T_lidar_world.at<double>(0,0)*p_in.at<double>(0,0)+T_lidar_world.at<double>(0,1)*p_in.at<double>(1,0)+T_lidar_world.at<double>(0,2)*p_in.at<double>(2,0)+T_lidar_world.at<double>(0,3);
        double p_out_y =  T_lidar_world.at<double>(1,0)*p_in.at<double>(0,0)+T_lidar_world.at<double>(1,1)*p_in.at<double>(1,0)+T_lidar_world.at<double>(1,2)*p_in.at<double>(2,0)+T_lidar_world.at<double>(1,3);
        double p_out_z =  T_lidar_world.at<double>(2,0)*p_in.at<double>(0,0)+T_lidar_world.at<double>(2,1)*p_in.at<double>(1,0)+T_lidar_world.at<double>(2,2)*p_in.at<double>(2,0)+T_lidar_world.at<double>(2,3);

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

template<typename PointT> 
void RgbLidarProcess<PointT>::SaveRgbCloudFiles(std::vector<std::vector<cv::String>> &sensors_data_pathes, int saved_frame_num)
{
  if(sensor_names_.size()==0 || sensor_names_[0] != "lidar")
  {
    std::cout<<"[SaveRgbCloudFiles]Sensor_names is not correct!"<<std::endl;
    return;
  }

  if(sensors_data_pathes.size() != sensor_names_.size())
  {
    std::cout<<"[SaveRgbCloudFiles]Sensor_pathes is not correct!"<<std::endl;
    return;
  }

  if(saved_frame_num >= sensors_data_pathes[0].size())
  {
    saved_frame_num = sensors_data_pathes[0].size();
  }

  std::cout<<"saved_frame_num: "<<saved_frame_num<<std::endl;

  for (size_t i = 0; i < saved_frame_num; i++)
  {
     PtCdtr<pcl::PointXYZ> raw_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
     PtCdtr<pcl::PointXYZRGB> out_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // ReadPointsCloud(sensors_data_pathes[0][i], raw_cloud_ptr, ' '); //beixing
     ReadPointsCloud(sensors_data_pathes[0][i], raw_cloud_ptr);

    std::unordered_map<std::string, std::string> image_hash;
    for(size_t j= 1; j<sensor_names_.size(); j++)
    {
      if(sensor_names_[j] == "front_camera")
      {
        image_hash.insert(std::pair<std::string, std::string>(sensor_names_[j], sensors_data_pathes[1][i]));
      }
      else if(sensor_names_[j] == "left_back_camera")
      {
        image_hash.insert(std::pair<std::string, std::string>(sensor_names_[j], sensors_data_pathes[2][i]));
      }
      else if(sensor_names_[j] == "right_back_camera")
      {
        image_hash.insert(std::pair<std::string, std::string>(sensor_names_[j], sensors_data_pathes[3][i]));
      }
    }

    CloudFusionMultiplyRGB(raw_cloud_ptr, image_hash, out_cloud_ptr);

    std::string saved_path = "../data/saved_rgb_clouds/rgb_pcd_" + std::to_string(i) + ".pcd";

    pcl::io::savePCDFileASCII(saved_path, *out_cloud_ptr);

    std::cout<<"saved "<<i+1<<" th rgb cloud success !"<<std::endl;
  }
}

