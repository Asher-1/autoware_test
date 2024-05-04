#include <ros/ros.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include "rgb_mapping/getCameraInfo.h"
using namespace std;

//读取参数模板
template <typename T>
T getParam(const std::string &name, const T &defaultValue) // This name must be namespace+parameter_name
{
    T v;
    if (ros::param::get(name, v)) // get parameter by name depend on ROS.
    {
        ROS_INFO_STREAM("Found parameter: " << name << ",\tvalue: " << v);
        return v;
    }
    else
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ",\tassigning default: " << defaultValue);
    return defaultValue; // if the parameter haven't been set,it's value will return defaultValue.
}

//根据指定namespace内的参数，构造tf变换，并以静态tf的形式发布
void publish_by_ns(std::string ns)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = getParam<string>(ns + "/header/frame_id", "");
    transformStamped.child_frame_id = getParam<string>(ns + "/child_frame_id", "");
    transformStamped.transform.translation.x = getParam<float>(ns + "/transform/translation/x", (float).0);
    transformStamped.transform.translation.y = getParam<float>(ns + "/transform/translation/y", (float).0);
    transformStamped.transform.translation.z = getParam<float>(ns + "/transform/translation/z", (float).0);
    transformStamped.transform.rotation.x = getParam<float>(ns + "/transform/rotation/x", (float).0);
    transformStamped.transform.rotation.y = getParam<float>(ns + "/transform/rotation/y", (float).0);
    transformStamped.transform.rotation.z = getParam<float>(ns + "/transform/rotation/z", (float).0);
    transformStamped.transform.rotation.w = getParam<float>(ns + "/transform/rotation/w", (float).0);

    static_broadcaster.sendTransform(transformStamped);
}

//从指定namespace的参数服务器获取相机内参，转换为sensor_msgs::CameraInfo的形式返回
sensor_msgs::CameraInfo construct_CameraInfo_by_ns(std::string ns)
{
    sensor_msgs::CameraInfo ci;
    ci.header.stamp = ros::Time::now();
    ci.header.frame_id = ns;
    ci.distortion_model = getParam<string>(ns + "/distortion_model", "");
    ci.height = (u_int32_t)getParam<int>(ns + "/height", (int)0);
    ci.width = (u_int32_t)getParam<int>(ns + "/width", (int)0);
    XmlRpc::XmlRpcValue vec; // https://cloud.tencent.com/developer/ask/sof/1137443
    vec = getParam<XmlRpc::XmlRpcValue>(ns + "/D", vec);
    if (vec.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // ci.D.resize(vec.size());
        for (int i = 0; i < vec.size(); i++)
        {
            // ci.D[i] = (double)vec[i];
            ci.D.push_back((double)vec[i]);
        }
    }
    ROS_INFO("%s sensor_msgs/CameraInfo/D: [%f %f %f %f %f]", ci.header.frame_id.c_str(), ci.D[0], ci.D[1], ci.D[2], ci.D[3], ci.D[4]);

    vec = getParam<XmlRpc::XmlRpcValue>(ns + "/K", vec);
    if (vec.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // ci.K.resize(vec.size());
        for (int i = 0; i < vec.size(); i++)
        {
            ci.K[i] = (double)vec[i];
            //  ci.K.push_back((double)vec[i]);
        }
    }
    ROS_INFO("%s sensor_msgs/CameraInfo/K: [%f %f %f %f %f %f %f %f %f]", ci.header.frame_id.c_str(),
             ci.K[0], ci.K[1], ci.K[2], ci.K[3], ci.K[4], ci.K[5], ci.K[6], ci.K[7], ci.K[8]);

// https://blog.csdn.net/tfb760/article/details/115935289
    cv::Mat dst;
    cv::Mat cameraMatrix = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
    cv::Mat distCoeffs(cv::Size(1, 5), CV_64FC1);
    cameraMatrix.at<double>(0, 0) = ci.K[0];
    cameraMatrix.at<double>(0, 2) = ci.K[2];
    cameraMatrix.at<double>(1, 1) = ci.K[4];
    cameraMatrix.at<double>(1, 2) = ci.K[5];
    cameraMatrix.at<double>(2, 2) = ci.K[8];

    distCoeffs.at<double>(0) = ci.D[0];
    distCoeffs.at<double>(1) = ci.D[1];
    distCoeffs.at<double>(2) = ci.D[2];
    distCoeffs.at<double>(3) = ci.D[3];
    distCoeffs.at<double>(4) = ci.D[4];
    dst = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(ci.width, ci.height), 0);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            ci.P[i * 4 + j] = dst.at<double>(i, j);
        }
    }
    ci.R[0] = 1;
    ci.R[4] = 1;
    ci.R[8] = 1;
    ci.binning_x = 0;
    ci.binning_y = 0;
    return ci;
}

//服务回调函数: 
//请求: 相机内参的命名空间，例如 front_camera_intrinsic/left_back_camera_intrinsic/right_back_camera_intrinsic
//响应: sensor_msgs::CameraInfo形式的相机内参
bool getCameraInfo_callback(rgb_mapping::getCameraInfo::Request &req,
                            rgb_mapping::getCameraInfo::Response &res)
{
    ROS_INFO("request camera_info of [frame_id: %s]", req.frame_id.c_str());
    res.camera_info = construct_CameraInfo_by_ns(req.frame_id);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "load_params_node");
    ros::NodeHandle n;
    //发布外参tf
    publish_by_ns("front_camera_extrinsic");
    publish_by_ns("left_back_camera_extrinsic");
    publish_by_ns("right_back_camera_extrinsic");
    publish_by_ns("lidar_extrinsic");
    //注册服务，等待客户端调用，向客户端发送相机内参
    ros::ServiceServer service = n.advertiseService("/rgb_mapping/load_params_node/getCameraInfo", getCameraInfo_callback);
    ROS_INFO("Ready to getCameraInfo by frame_id.");
    // sensor_msgs::CameraInfo front_camera_info = construct_CameraInfo_by_ns("front_camera_intrinsic");
    // ros::Publisher pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1000);
    // ros::Rate rate(1); //点云更新频率0.5Hz
    // while (ros::ok())
    // {

    //     pub.publish(front_camera_info); //发布出去
    //     rate.sleep();
    // }
    ros::spin();
    return 0;
}
