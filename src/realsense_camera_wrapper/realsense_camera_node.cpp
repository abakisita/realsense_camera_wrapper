#include "realsense_camera_device_wrapper.h"
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char *argv[]) 
{

    RealsenseCamera device(480, 640);
    ros::init(argc, argv, "example_node_1");
    ros::NodeHandle nh_;
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZRGB>);
    rs2::frame color;
    rs2::frame depth; 
    double t_start;
    double t_end;

    while (nh_.ok())
    {
        //ptr_cloud cloud_new(new point_cloud);
        // device->get_data(cloud, color, depth);
        t_start = ros::Time::now().toSec();   
        device.get_pointcloud(cloud_new);
        t_end = ros::Time::now().toSec();   
        ROS_WARN_STREAM("time required for calculating point cloud " << (t_end - t_start));
        ros::Rate r(30);
        r.sleep();
        viewer.showCloud(cloud_new);
    }
}