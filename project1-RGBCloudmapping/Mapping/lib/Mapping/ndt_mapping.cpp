#include "ndt_mapping.h"

NdtMapping::NdtMapping()
{
    map_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_filtered_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_rgb_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    map_rgb_filtered_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    previous_pose_.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    diff_pose_.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    guess_pose_.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    add_pose_.initPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

NdtMapping::~NdtMapping()
{
    
}

double NdtMapping::calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}

void NdtMapping::NdtFrameMatch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_frame_rgb, double cur_time_stamp)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_frame;
    cur_frame.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_frame_filtered;
    cur_frame_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());

    for(size_t i = 0; i<cur_frame_rgb->points.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = cur_frame_rgb->points[i].x;
        p.y = cur_frame_rgb->points[i].y;
        p.z = cur_frame_rgb->points[i].z;

        //if(std::hypot(p.x, p.y) > min_scan_range_ && std::hypot(p.x, p.y) < max_scan_range_)
        cur_frame->points.push_back(p);
    }

    if (initial_frame_loaded_ == 0)
    {
        //just set lidar and base coordinate coincide
        *map_ptr_ += *cur_frame;
        
        cpu_ndt_.setInputTarget(map_ptr_); // initial set ndt target
        initial_frame_loaded_ = 1;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid_filter.setInputCloud(cur_frame);
    voxel_grid_filter.filter(*cur_frame_filtered);

    // set ndt source
    cpu_ndt_.setTransformationEpsilon(trans_eps_);
    cpu_ndt_.setStepSize(step_size_);
    cpu_ndt_.setResolution(ndt_res_);
    cpu_ndt_.setMaximumIterations(max_iter_);
    cpu_ndt_.setInputSource(cur_frame_filtered);    

    guess_pose_.x = previous_pose_.x + diff_pose_.x;
    guess_pose_.y = previous_pose_.y + diff_pose_.y;
    guess_pose_.z = previous_pose_.z + diff_pose_.z;
    guess_pose_.roll = previous_pose_.roll;
    guess_pose_.pitch = previous_pose_.pitch;
    guess_pose_.yaw = previous_pose_.yaw + diff_pose_.yaw;

    Pose guess_pose_for_ndt;
    //TODO  for imu odom sensor data to guess

    guess_pose_for_ndt = guess_pose_;

    Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

    Eigen::Matrix4f init_guess =
          (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

    std::cout<<"init_guess: "<<init_guess<<std::endl;
    std::cout<<"map_ptr_: "<<map_ptr_->points.size()<<std::endl;
    std::cout<<"cur_frame_filtered: "<<cur_frame_filtered->points.size()<<std::endl;
    cpu_ndt_.align(init_guess);
    std::cout<<"run here!"<<std::endl;
    double fitness_score = cpu_ndt_.getFitnessScore();
    Eigen::Matrix4f t_localizer = cpu_ndt_.getFinalTransformation();
    bool has_converged = cpu_ndt_.hasConverged();
    int final_num_iteration = cpu_ndt_.getFinalNumIteration();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cur_frame_transformed;
    cur_frame_transformed.reset(new pcl::PointCloud<pcl::PointXYZ>());
    //just set lidar and base coordinate coincide
    pcl::transformPointCloud(*cur_frame, *cur_frame_transformed, t_localizer);

    Pose current_pose;
    current_pose.x = static_cast<double>(t_localizer(0, 3));
    current_pose.y = static_cast<double>(t_localizer(1, 3));
    current_pose.z = static_cast<double>(t_localizer(2, 3));

    Eigen::Matrix3d R;
    R << static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)), static_cast<double>(t_localizer(0, 2)),
              static_cast<double>(t_localizer(1, 0)), static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
              static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)), static_cast<double>(t_localizer(2, 2));   

    Eigen::Vector3d Euler=R.eulerAngles(2,1,0);          

    current_pose.roll = Euler(2, 0);
    current_pose.pitch = Euler(1, 0);
    current_pose.yaw = Euler(0, 0);

    std::cout<<"fitness_score: "<<fitness_score<<std::endl;
    std::cout<<"has_converged: "<<has_converged<<std::endl;
    std::cout<<"final_num_iteration: "<<final_num_iteration<<std::endl;
    std::cout<<"x: "<<current_pose.x<<","<<"y: "<<current_pose.y<<","<<"z: "<<current_pose.z<<std::endl;
    std::cout<<"current_pose.roll: "<<current_pose.roll<<","<<"current_pose.pitch: "<<current_pose.pitch<<","<<"current_pose.yaw: "<<current_pose.yaw<<std::endl;

    diff_pose_.x = current_pose.x - previous_pose_.x;
    diff_pose_.y = current_pose.y - previous_pose_.y;
    diff_pose_.z = current_pose.z - previous_pose_.z;
    diff_pose_.yaw = calcDiffForRadian(current_pose.yaw, previous_pose_.yaw);

    if (std::hypot(std::abs(add_pose_.x - current_pose.x), std::abs(add_pose_.y - current_pose.y)) >= min_add_scan_shift_)
    {
        add_pose_ = current_pose;
        *map_ptr_ += *cur_frame_transformed;
        cpu_ndt_.setInputTarget(map_ptr_);

        LidarFrame mapping_frame;
        mapping_frame.time_s = cur_time_stamp;
        mapping_frame.T_map_lidar = t_localizer;
        mapping_frame.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
                
        for(size_t j = 0; j < cur_frame_rgb->points.size(); j++)
        {
            pcl::PointXYZRGB p_rgb;
            p_rgb = cur_frame_rgb->points[j];
            if(std::hypot(p_rgb.x, p_rgb.y) > min_scan_range_ && std::hypot(p_rgb.x, p_rgb.y) < max_scan_range_)
                mapping_frame.cloud->points.push_back(p_rgb);
        }

        mapping_frames_.push_back(mapping_frame);
    }

    previous_pose_ = current_pose;
    pre_time_stamp_ = cur_time_stamp;
}

void NdtMapping::CreateMapBasedLidarFrames(std::vector<LidarFrame> &lidar_frames)
{
    if(lidar_frames.empty() || lidar_frames[0].T_map_lidar.rows() != 4)
    {
        std::cout<<"lidar_frames has no data!"<<std::endl;
        return;
    }

    int map_dispart_count = int(lidar_frames.size()/10 + 1);
    std::cout<<"map_dispart_count is: "<<map_dispart_count<<std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_map;
    sub_map.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    int sub_map_width = 0;
    int submap_id = 0;

    for(size_t i = 0; i < lidar_frames.size(); i++)
    {
        int num = i+1;

        if((num%int(10)) == 0)
        {
            sub_map->width = sub_map_width;
            sub_map->height = 1;
            std::string saved_path = "submap_" + std::to_string(submap_id) + ".pcd";
            std::cout<<submap_id<<"th submap saved success!"<<std::endl;

            pcl::io::savePCDFileASCII(saved_path, *sub_map);

            sub_map ->points.clear();
            sub_map_width = 0;

            submap_id++;
        }

        //std::cout<<lidar_frames[i].cloud->points.size()<<std::endl;

        for(size_t j = 0; j<lidar_frames[i].cloud->points.size(); j++)
        {
            pcl::PointXYZRGB lidar_p = lidar_frames[i].cloud->points[j];
            Eigen::Matrix4f T =  lidar_frames[i].T_map_lidar;
            // std::cout<<"T_read: "<<T<<std::endl;

            double map_p_x =  lidar_frames[i].T_map_lidar(0,0)*lidar_p.x+lidar_frames[i].T_map_lidar(0,1)*lidar_p.y+lidar_frames[i].T_map_lidar(0,2)*lidar_p.z+lidar_frames[i].T_map_lidar(0,3);
            double map_p_y =  lidar_frames[i].T_map_lidar(1,0)*lidar_p.x+lidar_frames[i].T_map_lidar(1,1)*lidar_p.y+lidar_frames[i].T_map_lidar(1,2)*lidar_p.z+lidar_frames[i].T_map_lidar(1,3);
            double map_p_z =  lidar_frames[i].T_map_lidar(2,0)*lidar_p.x+lidar_frames[i].T_map_lidar(2,1)*lidar_p.y+lidar_frames[i].T_map_lidar(2,2)*lidar_p.z+lidar_frames[i].T_map_lidar(2,3);

            //std::cout<<"T: "<<lidar_frames[i].T_map_lidar<<std::endl;
            //std::cout<<"lidar: "<<lidar_p.x<<","<<lidar_p.y<<","<<lidar_p.z<<std::endl;
            //std::cout<<"map: "<<map_p_x<<","<<map_p_y<<","<<map_p_z<<std::endl;

            lidar_p.x = map_p_x;
            lidar_p.y = map_p_y;
            lidar_p.z = map_p_z;

            sub_map->points.push_back(lidar_p);
            sub_map_width++;
        }
    }
    return;
}

void NdtMapping::ReadLidarFrames(std::vector<cv::String> lidar_pathes)
{
    std::vector<LidarFrame> lidar_frames;

    for(size_t i = 0; i < lidar_pathes.size(); i++)
    {
        std::ifstream lidarfile(lidar_pathes[i]);
        std::string lineStr;
        if (!lidarfile.is_open())
        {
            std::cerr << "lidar file cannot openned !" << std::endl;
            return;
        }

        LidarFrame lidar_data;
        lidar_data.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        std::string lidar_path = lidar_pathes[i];
        lidar_data.time_stamp = lidar_path.substr(lidar_path.length()-23,lidar_path.length());//TODO config
        double t_ms = atof(lidar_data.time_stamp.substr(10, 4).c_str())/10000;
        double t_s = atof(lidar_data.time_stamp.substr(6, 4).c_str());
        lidar_data.time_s =  t_s+t_ms;

        std::cout<<"lidar_data.time_stamp: "<<lidar_data.time_stamp<<std::endl;
        std::cout<<"lidar_data.time_s: "<< lidar_data.time_s<<std::endl;
        std::cout<<"t_ms: "<<t_ms<<std::endl;
        std::cout<<"t_s: "<<t_s<<std::endl;

        std::string lidarline;
        while (std::getline(lidarfile, lidarline))
        {
            std::stringstream ss(lidarline);
            std::string str;
            pcl::PointXYZRGB p;

            ss >> str; p.x = atof(str.c_str());
            ss >> str; p.y = atof(str.c_str());
            ss >> str; p.z = atof(str.c_str());
            ss >> str; p.r = uint8_t(atoi(str.c_str()));
            ss >> str; p.g = uint8_t(atoi(str.c_str()));
            ss >> str; p.b = uint8_t(atoi(str.c_str()));
            //std::cout<<p.x<<","<<p.y<<","<<p.z<<","<<p.r<<","<<p.g<<","<<p.b<<std::endl;
            lidar_data.cloud->points.push_back(p);
        }
        std::cout<<"lidar size: "<<lidar_data.cloud->points.size()<<std::endl;
         //lidar_frames.push_back(lidar_data);
         NdtFrameMatch(lidar_data.cloud, lidar_data.time_s);
    }
}

void NdtMapping::CreateNdtRgbMap(std::vector<cv::String> lidar_pathes)
{
    ReadLidarFrames(lidar_pathes);
    CreateMapBasedLidarFrames(mapping_frames_);
}
