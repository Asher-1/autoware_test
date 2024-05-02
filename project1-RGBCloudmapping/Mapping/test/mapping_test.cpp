/*
 * @Description: 
 * @Version: 2.0
 * @Autor: tianyu
 * @Date: 2021-06-21 14:45:26
 * @LastEditors: lhl
 * @LastEditTime: 2021-06-25 13:48:41
 */

#include "ndt_mapping.h"

int main(int argc, char **argv)
{ 
   std::shared_ptr<NdtMapping> mapping_processer = std::make_shared<NdtMapping>();
   std::vector<cv::String> lidar_pathes;
   cv::glob("../data/rgbcloud/*.txt", lidar_pathes);
   mapping_processer->CreateNdtRgbMap(lidar_pathes);
 
   return 0;
}
