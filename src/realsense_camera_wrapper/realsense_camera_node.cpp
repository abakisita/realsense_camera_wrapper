#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <iostream>
#include <realsense_camera_ros.h>

RealsenseCameraRos::RealsenseCameraRos()
{
    device = std::make_shared<RealsenseCamera>(ImageResolution::SD, ImageResolution::SD);
}

int main(int argc, char *argv[]) 
{

    RealsenseCamera device(ImageResolution::SD, ImageResolution::SD);
    ros::init(argc, argv, "example_node_1");
    ros::NodeHandle nh_;
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    ptr_cloud cloud;
    ptr_cloud cloud_new;
    rs2::frame color;
    rs2::frame depth; 
    double t_start;
    double t_end;

    while (nh_.ok())
    {
        ptr_cloud cloud_new(new point_cloud);
        // device->get_data(cloud, color, depth);
        t_start = ros::Time::now().toSec();   
        std::chrono::milliseconds::rep time_stamp;
        device.get_pointcloud(cloud_new, time_stamp);
        t_end = ros::Time::now().toSec();   
        ROS_WARN_STREAM("time required for calculating point cloud " << (t_end - t_start));
        ros::Rate r(30);
        r.sleep();
        viewer.showCloud(cloud_new);
    }
}