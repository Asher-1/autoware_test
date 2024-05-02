//
// Created by Rsx on 2020/11/09.
//

// for visualizer for tracking target in 3d pointcloud map

#include "VisualizerSet.h"

//class VisualizerSet
//add—sixu——code？
VisualizerSet::VisualizerSet(pcl::visualization::PCLVisualizer::Ptr& viewer_, CameraAngle setAngle_, Color color_):
viewer(viewer_),
setAngle(setAngle_),
color(color_)
{
    InitialVisual();  
}

VisualizerSet::~VisualizerSet()
{
    
}

void VisualizerSet::InitialVisual()
{
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    int distance = 50;
    switch (setAngle) 
    {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);

        case SKEW :
            viewer->setCameraPosition(0, 0, distance, 0, 0.8, 1);
    }
    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

void VisualizerSet::PointsCloudVisual(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name)
{
    viewer->addPointCloud<pcl::PointXYZI> (cloud, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void VisualizerSet::CloudClustersVisual(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CloudClusters)
{
    int id = 0;
    for(auto CloudCluster:CloudClusters)
    {
        std::string name = "Cluster"+std::to_string(id);
        PointsCloudVisual(CloudCluster, name); 
        id++;      
    }
}

void VisualizerSet::ResetVisual()
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
}

void VisualizerSet::getMinMax3D(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointXYZI &minPoint_, pcl::PointXYZI &maxPoint_)
{
    double xmax = -1000;
    double xmin = 1000;
    double ymax = -1000;
    double ymin = 1000;
    double zmax = -1000;
    double zmin = 1000;

    for(int i = 0; i<cloud->points.size(); i++)
    {
        if(cloud->points[i].x>xmax)
            xmax = cloud->points[i].x;
        if(cloud->points[i].x<xmin)
            xmin = cloud->points[i].x;
        if(cloud->points[i].y>ymax)
            ymax = cloud->points[i].y;
        if(cloud->points[i].y<ymin)
            ymin = cloud->points[i].y;
        if(cloud->points[i].z>zmax)
            zmax = cloud->points[i].z;
        if(cloud->points[i].y<zmin)
            zmin = cloud->points[i].z;
    }

    minPoint_.x = xmin;
    minPoint_.y = ymin;
    minPoint_.z = zmin;
    maxPoint_.x = xmax;
    maxPoint_.y = ymax;
    maxPoint_.z = zmax;

    //std::cout<<"x = "<<maxPoint_.x<<" y = "<<maxPoint_.y<<" z = "<<maxPoint_.z<<std::endl;
}

void VisualizerSet::CloudClusters2Boxes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> CloudClusters)
{
    pcl::PointXYZI minPoint_;
    pcl::PointXYZI maxPoint_;
    int id = 0;
    for(auto CloudCluster:CloudClusters)
    {      
        getMinMax3D(CloudCluster, minPoint_, maxPoint_);
        BoundingBox box;
        box.x = minPoint_.x;
        box.y = minPoint_.y;
        box.w = maxPoint_.x-minPoint_.x;
        box.h = maxPoint_.y-minPoint_.y;
        BoxVisual(box, id);
        boxes.push_back(box);
        id++;
        std::cout<<"A cluster convert a box success!"<<std::endl;
        std::cout<<"id = "<<id<<std::endl;
        std::cout<<"x = "<<boxes[id-1].x<<" y = "<<boxes[id-1].y<<" w = "<<boxes[id-1].w<<" h = "<<boxes[id-1].h<<std::endl;
    }  
          
}

void VisualizerSet::BoxVisual(BoundingBox box, int id)
{	
       
    std::string cube = "box"+std::to_string(id);
    viewer->addCube(box.x, box.x+box.w, box.y, box.y+box.h, float(-1.2), float(0), color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);
}
