#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <algorithm>            // std::min, std::max
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB P_pcl;
typedef pcl::PointCloud<P_pcl> point_cloud;
typedef point_cloud::Ptr ptr_cloud;
ptr_cloud point_cloud_;
class RealsenseCamera 
{
public:
    RealsenseCamera();
    void initialize();
    float get_depth_scale(rs2::device dev);
    void get_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud);
    rs2::pipeline pipe_;
    rs2::pipeline_profile p_profile_;
    float m_invalid_depth_value_ = 0.0;
    float m_max_z_ = 8.0;
    float m_depth_scale_;
    rs2::stream_profile stream_c_;
    rs2::video_stream_profile* vsc_;
    rs2_intrinsics m_color_intrinsics_;      
    rs2::stream_profile stream_d_;
    rs2::video_stream_profile* vsd_;
    rs2_intrinsics m_depth_intrinsics_;
    rs2_extrinsics m_depth_2_color_extrinsics_;
    int color_width_;
    int color_height_;
    bool is_initialized_ = false;
    rs2_format color_format_;
    bool swap_rgb_;
    uint16_t nb_color_pixel_;
};