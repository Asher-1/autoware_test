/*
 * @Description: 
 * @Version: 2.0
 * @Autor: tianyu
 * @Date: 2021-06-21 14:45:26
 * @LastEditors: lhl
 * @LastEditTime: 2021-06-25 13:48:41
 */

#include "rgblidar_process.h"
#include "rgblidar_process.cpp"

int main(int argc, char **argv)
{   
    std::vector<cv::String> lidar_pathes;
    std::vector<cv::String> front_camera_img_pathes;
    std::vector<cv::String> left_back_camera_img_pathes;
    std::vector<cv::String> right_back_camera_img_pathes;

    cv::glob("../data/apollo/lidar/*.pcd", lidar_pathes);
    cv::glob("../data/apollo/front_camera/*.jpg", front_camera_img_pathes);
    cv::glob("../data/apollo/left_back_camera/*.jpg", left_back_camera_img_pathes);
    cv::glob("../data/apollo/right_back_camera/*.jpg", right_back_camera_img_pathes);

    std::vector<std::vector<cv::String>> sensors_data_pathes;
    sensors_data_pathes.push_back(lidar_pathes);
    sensors_data_pathes.push_back(front_camera_img_pathes);
    sensors_data_pathes.push_back(left_back_camera_img_pathes);
    sensors_data_pathes.push_back(right_back_camera_img_pathes);

    std::vector<std::string> sensor_names;
    sensor_names.push_back("lidar");
    sensor_names.push_back("front_camera");
    sensor_names.push_back("left_back_camera");
    sensor_names.push_back("right_back_camera");
    std::string config_path = "../config/sensors_calib_apollo.yml";

    std::shared_ptr<RgbLidarProcess<pcl::PointXYZRGB>> lidar_processer = std::make_shared<RgbLidarProcess<pcl::PointXYZRGB>>(config_path, sensor_names);

    lidar_processer->SaveRgbCloudFiles(sensors_data_pathes, 1);//后面的数字代表需要存多少帧，如果为1那么就只输出一帧rgb点云，根据输入的原始数据设置

    return 0;
}
