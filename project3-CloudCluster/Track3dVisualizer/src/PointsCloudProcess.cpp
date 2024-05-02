//
// Created by Rsx on 2020/11/09.
//

// for visualizer for tracking target in 3d pointcloud map

#include "PointsCloudProcess.h"

//class PointsCloudProcess
template<typename PointT>
PointsCloudProcess<PointT>::PointsCloudProcess()
{ 
   clusterTolerance=0.8;
}

template<typename PointT>
PointsCloudProcess<PointT>::~PointsCloudProcess()
{ 
    
}

template<typename PointT>
PtCdtr<PointT> PointsCloudProcess<PointT>::loadPcd(std::string file)
{
    PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) 
    { 
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> PointsCloudProcess<PointT>::streamPcd(std::string dataPath) 
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    sort(paths.begin(), paths.end());

    return paths;
}

//点云过滤，这里参考autoware里面代码
template<typename PointT>
PtCdtr<PointT> PointsCloudProcess<PointT>::FilterCloud(PtCdtr<PointT> cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> vg;
    PtCdtr<PointT> cloudFiltered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    PtCdtr<PointT> cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices) {
        inliers->indices.push_back(point);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    return cloudRegion;
}

template<typename PointT>
void PointsCloudProcess<PointT>::TransformCloud(PtCdtr<PointT> &incloud)
{
    Eigen::Matrix<double, 3, 3> R_;
    R_<<1, 0, 0, 
        0, 0, -1, 
        0, 1, 0;

    Eigen::Vector3d t_(0,0,0);

    for(size_t i=0; i<incloud->points.size(); i++)
    {
        Eigen::Vector3d point(incloud->points[i].x,incloud->points[i].y,incloud->points[i].z);
        point = R_*point+t_; 
        incloud->points[i].x=point(0);
        incloud->points[i].y=point(1);
        incloud->points[i].z=point(2); 
    }   
}

template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>> PointsCloudProcess<PointT>::RansacSegmentPlane(PtCdtr<PointT> cloud)
{
    std::pair<PtCdtr<PointT>, PtCdtr<PointT>> result_cloud;

    PtCdtr<PointT> obstacle_cloud(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> plane_cloud(new pcl::PointCloud<PointT>());
    std::vector<bool> inliners_flag = GetPlaneInliners(cloud);
    
    for(size_t i = 0; i<cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        //根据flag将其分为plane_cloud和obstacle_cloud
        if(inliners_flag[i])
            plane_cloud->points.push_back(point);        
        else
            obstacle_cloud->points.push_back(point);
    }
    
    result_cloud.first=obstacle_cloud;
    result_cloud.second=plane_cloud;
    return result_cloud;   
}

template<typename PointT>
std::vector<bool> PointsCloudProcess<PointT>::GetPlaneInliners(PtCdtr<PointT> cloud)
{
    std::vector<bool> inliners_flag;
    inliners_flag.assign(cloud->points.size(), false);
    size_t num_points = cloud->points.size();
    int maxIterations=20;
    double distanceTol=0.1;
    while (maxIterations--) 
    {
        std::unordered_set<int> inliners;
        while (inliners.size() < 3) 
        {
            //这里是如果inlines数量很少，那么相当于随机选取另外三个连续点构成拟合平面（重复几次总会使随机点确实来自于地面上的点）
            inliners.insert(rand()%num_points);
        }
        //基于前面设定的随机数，选取连续三个point
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliners.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        // ax+by+cz+d = 0 高中学习的平面数学表达公式
        float a, b, c, d, sqrt_abc;
        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -(a * x1 + b * y1 + c * z1);
        sqrt_abc = sqrt(a * a + b * b + c * c);
        
        for (int ind = 0; ind < num_points; ind++) 
        {
            if (inliners_flag[ind]) 
            { 
                continue;
            }
            PointT point = cloud->points[ind];
            float x = point.x;
            float y = point.y;
            float z = point.z;
            //计算每一个point到平面的距离，小于一定阈值就说明他是平面点，并打上flag
            float dist = fabs(a * x + b * y + c * z + d) / sqrt_abc; 

            if (dist < distanceTol) 
            {
                inliners.insert(ind);
                inliners_flag[ind]=true;
            }
        }
    }
    return inliners_flag;
}
//递归实现聚类，基本流程和autoware里面是一样的，如果当时感知视频看懂了，这里应该问题不大
template<typename PointT>
std::vector<PtCdtr<PointT>> PointsCloudProcess<PointT>::EuclidCluster(PtCdtr<PointT> cloud)
{
    int points_num = cloud->points.size();
    processed_flag.assign(points_num, false);
    int minClusterSize = 10;
    int maxClusterSize = 140;
       
    std::vector<PtCdtr<PointT>> clusters;
    pcl::KdTreeFLANN<PointT> *kdtree = new pcl::KdTreeFLANN<PointT>();
    kdtree->setInputCloud(cloud);

    for(size_t id = 0; id<points_num; id++)
    {
        if(processed_flag[id])
        {
            continue;
        }
        std::vector<int> cluster;
        PtCdtr<PointT> cloudCluster(new pcl::PointCloud<PointT>);
        GenerateCluster(id, cloud, cluster, kdtree);
        int cluster_size = cluster.size();
        if(cluster_size>minClusterSize && cluster_size<maxClusterSize)
        {
            for(int i = 0; i<cluster_size; i++)
            {
                cloudCluster->points.push_back(cloud->points[cluster[i]]);       
            }

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            clusters.push_back(cloudCluster);
        }

    }
    std::cout<<"Get "<<clusters.size()<<"numbers of clusters from "<<points_num<<"numbers of points!"<<std::endl;
    return clusters;
}

template<typename PointT>
void PointsCloudProcess<PointT>::GenerateCluster(int id, PtCdtr<PointT> cloud, std::vector<int> &cluster, pcl::KdTreeFLANN<PointT> *kdtree)
{
    processed_flag[id] = true;
    cluster.push_back(id);
    std::vector<int> neighbor_points;
    std::vector<float> neighbor_points_dis;
    //基于kd树进行最邻近点的搜索
    if(kdtree->radiusSearch(cloud->points[id], clusterTolerance, neighbor_points, neighbor_points_dis, 0)>0)
    {
        for(int neighbor_id:neighbor_points)
        {
            if(!processed_flag[neighbor_id])
            {
                GenerateCluster(neighbor_id, cloud, cluster, kdtree);
            }
        }
    }
}