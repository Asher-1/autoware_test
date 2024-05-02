# tools 工具分支

# PointCloud_convert：
简介：用于将ndt_mapping建立好的pcd点云文件数据存储类型进行转换;

指令：build文件夹下， "./TypeConvert <file_in.pcd> <file_out.pcd> 0/1/2 ";

说明：0/1/2 分别对应ascii/binary/binary_compressed，有时候需要将pcd转成ascii，否则map_file读不上来；

