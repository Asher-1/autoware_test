//
// Created by Rsx on 2020/11/09.
//

// for visualizer for tracking target in 3d pointcloud map

#ifndef POINTSCLOUDPROCESS_H_
#define POINTSCLOUDPROCESS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

template<typename PointT>
using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

template<typename PointT>
class PointsCloudProcess
{
public:
    PointsCloudProcess();
    ~PointsCloudProcess();
    
    PtCdtr<PointT> loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath); 
    PtCdtr<PointT> FilterCloud(PtCdtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
    void TransformCloud(PtCdtr<PointT> &incloud);
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> RansacSegmentPlane(PtCdtr<PointT> cloud);
    std::vector<bool> GetPlaneInliners(PtCdtr<PointT> cloud);
    std::vector<PtCdtr<PointT>> EuclidCluster(PtCdtr<PointT> cloud);
    void GenerateCluster(int ind, PtCdtr<PointT> cloud, std::vector<int> &cluster, pcl::KdTreeFLANN<PointT> *kdtree);

    std::vector<bool> processed_flag;//for EuclidCluster
    float clusterTolerance;
};


#endif /* PROCESSPOINTCLOUDS_H_ */
