#include "realsense_camera_device_wrapper.h"
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <iostream>

int main(int argc, char *argv[]) 
{
    std::shared_ptr<RealsenseCamera> device = std::make_shared<RealsenseCamera>();
    ros::init(argc, argv, "example_node_1");
    ros::NodeHandle nh_;
    device->initialize();
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    ptr_cloud cloud;
    ptr_cloud cloud_new;
    rs2::frame color;
    rs2::frame depth;
    // zmq::context_t context (1);
    // zmq::socket_t publisher (context, ZMQ_PUB);
    // publisher.bind("ipc:///tmp/cloud.ipc");  
    double t_start;
    double t_end;
    // boost::interprocess::managed_shared_memory shared_point_cloud_memory(boost::interprocess::open_or_create, "shared_point_cloud_memory", 100240);
    // point_cloud *shared_point_cloud = shared_point_cloud_memory.find_or_construct<point_cloud>("point_cloud")();
    while (nh_.ok())
    {
        ptr_cloud cloud_new(new point_cloud);
        // device->get_data(cloud, color, depth);
        t_start = ros::Time::now().toSec();   
        device->get_pointcloud(cloud_new);
        t_end = ros::Time::now().toSec();   
        ROS_WARN_STREAM("time required for calculating point cloud " << (t_end - t_start));
        ros::Rate r(30);
        r.sleep();
        viewer.showCloud(cloud_new);
        // shared_point_cloud_memory.find_or_construct<point_cloud>("point_cloud")(*cloud);
        // point_cloud *pt_cloud;
        // pt_cloud = cloud.get();
        // try
        // {
        //     zmq::message_t point_cloud_msg(5 * sizeof(*pt_cloud));
        //     memcpy(point_cloud_msg.data(), pt_cloud, 5 * sizeof (*pt_cloud));
        //     publisher.send(point_cloud_msg);
        // }
        // catch (std::exception e1)
        // {
        // try 
        // {
        //     zmq::message_t point_cloud_msg(sizeof("hi"));
        //     memcpy(point_cloud_msg.data(), "hi", sizeof ("hi"));
        //     publisher.send(point_cloud_msg);
        //     // std::cout << "msg sent" << std::endl;

        // }
        // catch (std::exception e1)
        // {
        //     // publisher.bind("ipc://point_cloud.ipc");      
        //     std::cout << "failed" << std::endl;
        // }
    }
}