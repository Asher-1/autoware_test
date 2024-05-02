#ifndef NDT_MAPPING_H
#define NDT_MAPPING_H

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

#include <NormalDistributionsTransform.h>

class SubMap
{
public:
    SubMap();
    ~SubMap();

};

struct LidarFrame
{
    std::string time_stamp;
    double time_s;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    Eigen::Matrix4f T_map_lidar; //lidar to map
};

struct Pose
{
    double x;
    double y;
    double z;
    //radian
    double roll;
    double pitch;
    double yaw;

    void initPose(double x_, double y_, double z_, double roll_, double pitch_, double yaw_)
    {
        x = x_;
        y = y_;
        z = z_;
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
    }
};

class NdtMapping
{
public:
    NdtMapping();
    ~NdtMapping();
    void CreateNdtRgbMap(std::vector<cv::String> lidar_pathes);


private:
    void ReadLidarFrames(std::vector<cv::String> lidar_pathes);
    void NdtFrameMatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_frame_rgb, double time_s);
    double calcDiffForRadian(const double lhs_rad, const double rhs_rad);
    void CreateMapBasedLidarFrames(std::vector<LidarFrame> &lidar_frames);

    std::vector<LidarFrame> mapping_frames_; //lidar frames for mapping

    //initial
    int initial_frame_loaded_ = 0;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rgb_ptr_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rgb_filtered_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_filtered_ptr_;
    cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> cpu_ndt_;

    double pre_time_stamp_;
    
    //global variable pose
    Pose previous_pose_;
    Pose diff_pose_;
    Pose guess_pose_;
    Pose add_pose_; //the pose of frame will insert a map

    //config
    double min_scan_range_ = 5.0;
    double max_scan_range_ = 200.0;

    double voxel_leaf_size_ = 2.0;

    int max_iter_ = 30;        // Maximum iterations
    float ndt_res_ = 1.0;      // Resolution
    double step_size_ = 0.1;   // Step size
    double trans_eps_ = 0.01;  // Transformation epsilon

    double min_add_scan_shift_ = 1.0;
};

#endif NDT_MAPPING_H
