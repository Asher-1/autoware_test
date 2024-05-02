#ifndef RGBLIDAR_PROCESS_H
#define RGBLIDAR_PROCESS_H

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

#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <typeinfo>
 
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace std {
	template <>
	class hash< cv::Point >{
	public :
		size_t operator()(const cv::Point &pixel_cloud ) const
		{
			return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
		}
	};
};

template<typename PointT>
using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

template<typename PointT>
class RgbLidarProcess
{
public:
    RgbLidarProcess(std::string config_file, std::vector<std::string> sensor_names);
    RgbLidarProcess(std::string config_file);
    RgbLidarProcess();
    ~RgbLidarProcess();

    bool ReadParameters(std::string config_file);
    bool ReadParameters(std::string config_file, std::vector<std::string> sensor_names);
    bool ReadPointsCloud(std::string file_path,
                         PtCdtr<pcl::PointXYZ> raw_cloud_ptr,
                         char separator ); // char separator = ' '
    bool ReadPointsCloud(std::string file_path,
                         PtCdtr<pcl::PointXYZ> raw_cloud_ptr);

    void CloudFusionMultiplyRGB(PtCdtr<pcl::PointXYZ> in_cloud_ptr, 
                                                                    std::unordered_map<std::string, std::string> &image_hash,
                                                                    PtCdtr<pcl::PointXYZRGB> out_cloud_ptr);

    void CloudFusionRGB(PtCdtr<pcl::PointXYZ> in_cloud_ptr, 
                        cv::Mat image,
                        cv::Mat T_cam_lidar,
                        cv::Mat K_cam,
                        cv::Mat D_cam,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr);

    void RgbCloudMerge(std::vector<PtCdtr<pcl::PointXYZRGB>> &rgb_clouds, PtCdtr<pcl::PointXYZRGB> out_cloud_ptr);

    void CloudProjectImg(PtCdtr<pcl::PointXYZI> in_cloud_ptr, 
                        cv::Mat T_cam_lidar,
                        cv::Mat image);

    void SaveRgbCloudFiles(std::vector<std::vector<cv::String>> &sensors_data_pathes, int saved_frame_num);

    cv::Mat T_left_back_camera_lidar_;
    cv::Mat T_right_back_camera_lidar_;
    cv::Mat T_front_camera_lidar_;
    cv::Mat K_cam_front_camera_;
    cv::Mat D_cam_front_camera_;
    cv::Mat K_cam_left_back_camera_;
    cv::Mat D_cam_left_back_camera_;
    cv::Mat K_cam_right_back_camera_;
    cv::Mat D_cam_right_back_camera_;

private:
    //lidar sensor config 
    std::string config_file_;
    std::vector<std::string> sensor_names_; 
    std::string lidar_folder_path_;

    cv::Mat Twl_;
    cv::Mat T_offset;
    cv::Mat T_lidar_world;
    cv::Mat T_cam_world;
    cv::Mat T_lidar_cam;
    cv::Mat T_lidar_cam_offset;
    cv::Mat K_cam;
    cv::Mat D_cam;

    //apollo sensor config
    cv::Mat T_vechicle_lidar_; //lidar to vechicle
    cv::Mat T_vechicle_front_camera_; 
    cv::Mat T_vechicle_left_back_camera_;
    cv::Mat T_vechicle_right_back_camera_;

    //judge template class
    pcl::PointXYZ p_xyz;
    pcl::PointXYZI p_xyzi;
    pcl::PointXYZRGB p_xyzrgb;


    //ndt config
     int max_iter = 30;        // Maximum iterations
    float ndt_res = 1.0;      // Resolution
    double step_size = 0.1;   // Step size
    double trans_eps = 0.01;  // Transformation epsilon
                         
    void TransformToWorld(const PtCdtr<PointT> in_cloud_ptr,
                                                                   PtCdtr<PointT> out_cloud_ptr);

    void CheckTransform(cv::Mat &T);
    cv::Mat CalOffsetTransform(cv::Mat euler_mat);
    cv::Mat TransformCvmat(cv::Mat &T);

};

#endif
