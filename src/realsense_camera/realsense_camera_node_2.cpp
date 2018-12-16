#include "realsense_camera_device_wrapper.h"
#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <zmq.hpp>

// #include <boost/shared_pointer.hpp>
#include <iostream>
int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "example_node_2");
    std::cout << "connected";
    ros::NodeHandle nh_;
    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer 2");
    ptr_cloud cloud;
    rs2::frame color;
    rs2::frame depth;
    zmq::context_t context (1);
    zmq::socket_t subscriber (context, ZMQ_SUB);
    std::cout << "connected";
    subscriber.connect("ipc:///tmp/cloud.ipc");
    zmq_connect(subscriber, "ipc:///tmp/cloud.ipc");
    std::cout << "connected";
    ROS_WARN ("connected");

    // boost::interprocess::managed_shared_memory shared_point_cloud_memory(boost::interprocess::open_or_create, "shared_point_cloud_memory", 10240);
    // point_cloud *shared_point_cloud = shared_point_cloud_memory.find_or_construct<point_cloud>("point_cloud")();
    while (nh_.ok())
    {   
        ROS_WARN ("connected");

        // point_cloud *shared_point_cloud = shared_point_cloud_memory.find<point_cloud>("point_cloud");
        // const boost::shared_ptr<point_cloud> pc = boost::make_shared<point_cloud>(*shared_point_cloud);
        // point_cloud *pc_ = (point_cloud *)request.data();
        // const boost::shared_ptr<point_cloud> pc = boost::make_shared<point_cloud>(*pc_);        
        // viewer.showCloud(pc);
        zmq::message_t message;
        subscriber.recv(&message);

        std::string pc = std::string(static_cast<char*>(message.data()), message.size());
        ROS_WARN_STREAM("msg " << pc);
    }
}