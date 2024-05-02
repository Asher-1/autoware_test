//
// Created by Rsx on 2020/11/09.
//

// for visualizer for tracking target in 3d pointcloud map

#include "PointsCloudProcess.h"
#include "PointsCloudProcess.cpp"
#include "VisualizerSet.h"


int main(int argc, char **argv) 
{
      
    std::cout<<"test begin."<<std::endl;
    PointsCloudProcess<pcl::PointXYZI> *pointProcessorI = new PointsCloudProcess<pcl::PointXYZI>();
    PtCdtr<pcl::PointXYZI> mapCloud;
    PtCdtr<pcl::PointXYZI> FiltermapCloud;
    Eigen::Vector4f minPoint_map(-1000,-1000,-1000, 1);
    Eigen::Vector4f maxPoint_map( 1000, 1000, 1000, 1);
    mapCloud=pointProcessorI->loadPcd("../data/pcd/demo_map_around.pcd");
    FiltermapCloud=pointProcessorI->FilterCloud(mapCloud, 0.3, minPoint_map, maxPoint_map);
    pointProcessorI->TransformCloud(FiltermapCloud);//地图位姿调整
    
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../data/pcd/data_1");
    auto streamIterator = stream.begin();

    pcl::visualization::PCLVisualizer::Ptr visual(new pcl::visualization::PCLVisualizer("3D Viewer"));
    Color color(0,0,1);//调节颜色
    CameraAngle setAngle=TopDown;//调节视角
    VisualizerSet *visualset = new VisualizerSet(visual, setAngle, color); 

    while (1)
    { 
        visualset->ResetVisual();
        PtCdtr<pcl::PointXYZI> Cloud_(new pcl::PointCloud<pcl::PointXYZI>());
        PtCdtr<pcl::PointXYZI> FilterCloud_(new pcl::PointCloud<pcl::PointXYZI>());

        Eigen::Vector4f minPoint_cloud(-10, -6, -2, 1);
        Eigen::Vector4f maxPoint_cloud(30, 6, 1, 1);
        Cloud_=pointProcessorI->loadPcd((*streamIterator).string());
        FilterCloud_=pointProcessorI->FilterCloud(Cloud_, 0.3, minPoint_cloud, maxPoint_cloud);//设置region内降采样

        std::pair<PtCdtr<pcl::PointXYZI>, PtCdtr<pcl::PointXYZI>> SegmentCloud_=pointProcessorI->RansacSegmentPlane(FilterCloud_);//分割平面
        std::vector<PtCdtr<pcl::PointXYZI>> clusters_ = pointProcessorI->EuclidCluster(SegmentCloud_.first);//欧式聚类
        
        visualset->CloudClusters2Boxes(clusters_); //将聚类转成box并显示
        //visualset->CloudClustersVisual(clusters_); //显示聚类点云

        visualset->PointsCloudVisual(Cloud_, "lidar");//显示原始输入点云
        //visualset->PointsCloudVisual(FiltermapCloud, "map");//显示地图

        streamIterator++;
        if (streamIterator == stream.end()) 
        {
            streamIterator = stream.begin();
        }
        visual->spinOnce();
    }
 
}
