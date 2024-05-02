//
// Created by Rsx on 2020/11/09.
//

// for visualizer for tracking target in 3d pointcloud map

#ifndef VISUALIZERSET_H_
#define VISUALIZERSET_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_set>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>

struct Color
{
	float r, g, b;
    Color(float setR, float setG, float setB)
	    : r(setR), g(setG), b(setB)
	{}
};

enum CameraAngle
{
	XY, TopDown, Side, FPS, SKEW
};

struct BoundingBox
{
    float x; // x-component of top left coordinate
    float y; // y-component of top left coordinate
    float w; // width of the box
    float h; // height of the box
    //float score; // score of the box;
/*
    cv::RotatedRect rect;

    cv::RotatedRect rect_world;
*/
    unsigned int track_id;
    unsigned int obj_id;
    int untracked_num;

    float utm_x;
    float utm_y;

    bool cerred = false;
    unsigned long TimeS;    //时间，单位：秒
    unsigned long TimeNS;   //时间，单位：纳秒
}; 

class VisualizerSet
{
public:  
    VisualizerSet(pcl::visualization::PCLVisualizer::Ptr& viewer_, CameraAngle setAngle_, Color color_); 
    VisualizerSet();   
    ~VisualizerSet();

    void InitialVisual();
    void PointsCloudVisual(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name);
    void CloudClustersVisual(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CloudClusters);
    void BoxVisual(BoundingBox box, int id);
    void ResetVisual();
    void CloudClusters2Boxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CloudClusters);
    void getMinMax3D(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointXYZI &minPoint_ ,pcl::PointXYZI &maxPoint_);

    pcl::visualization::PCLVisualizer::Ptr& viewer;
    CameraAngle setAngle;
    Color color;
    std::vector<BoundingBox> boxes;
    
};
#endif /* VISUALIZERSET_H_ */