#include "realsense_camera.h"

RealsenseCamera::RealsenseCamera(uint16_t width, uint16_t height)
{
    rs2::config cfg;
    // set configuration 
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, 30);

    p_profile_ = pipe_.start(cfg);
    is_initialized_ = true;

    pointcloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    initialize();
}

void RealsenseCamera::initialize()
{
    m_depth_scale_ = get_depth_scale(p_profile_.get_device());
    stream_c_ = p_profile_.get_stream(RS2_STREAM_COLOR);
    vsc_ = new rs2::video_stream_profile(stream_c_.as<rs2::video_stream_profile>());
    m_color_intrinsics_ = vsc_->get_intrinsics();      
    stream_d_ = p_profile_.get_stream(RS2_STREAM_DEPTH);
    vsd_ = new rs2::video_stream_profile(stream_d_.as<rs2::video_stream_profile>());
    m_depth_intrinsics_ = vsd_->get_intrinsics();
    m_depth_2_color_extrinsics_ = vsd_->get_extrinsics_to(*vsc_);
    color_width_ = m_color_intrinsics_.width;
    color_height_ = m_color_intrinsics_.height;
    rs2::frameset frames = pipe_.wait_for_frames();
    const rs2::frame &color_frame = frames.get_color_frame();
    const rs2::frame &depth_frame = frames.get_depth_frame();
    color_format_ = color_frame.as<rs2::video_frame>().get_profile().format();
    swap_rgb_ = color_format_ == RS2_FORMAT_BGR8 || color_format_ == RS2_FORMAT_BGRA8;
    nb_color_pixel_ = (color_format_ == RS2_FORMAT_RGB8 || color_format_ == RS2_FORMAT_BGR8) ? 3 : 4;
}

float RealsenseCamera::get_depth_scale(rs2::device dev)
{
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RealsenseCamera::get_pointcloud(std::chrono::milliseconds::rep &time_stamp)
{
    rs2::frameset frames = pipe_.wait_for_frames();
    const rs2::frame &color_frame = frames.get_color_frame();
    const rs2::frame &depth_frame = frames.get_depth_frame();
    time_stamp = 
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    rs2::video_frame vf = depth_frame.as<rs2::video_frame>();
    const int width = vf.get_width();
    const int height = vf.get_height();
    pointcloud_->width = (uint32_t)width;
    pointcloud_->height = (uint32_t)height;
    pointcloud_->resize((size_t)(width * height));
    const uint16_t *p_depth_frame = reinterpret_cast<const uint16_t *>(depth_frame.get_data());
    const unsigned char *p_color_frame = reinterpret_cast<const unsigned char *>(color_frame.get_data());
    for (int i = 0; i < height; i++)
    {
        auto depth_pixel_index = i * width;
        for (int j = 0; j < width; j++, depth_pixel_index++)
        {
            if (p_depth_frame[depth_pixel_index] == 0)
            {
                pointcloud_->points[(size_t)depth_pixel_index].x = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].y = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].z = m_invalid_depth_value_;
            }

            // Get the depth value of the current pixel
            auto pixels_distance = m_depth_scale_ * p_depth_frame[depth_pixel_index];
            float depth_point[3];
            const float pixel[] = {(float)j, (float)i};
            rs2_deproject_pixel_to_point(depth_point, &m_depth_intrinsics_, pixel, pixels_distance);
            if (pixels_distance > m_max_z_)
                depth_point[0] = depth_point[1] = depth_point[2] = m_invalid_depth_value_;

            pointcloud_->points[(size_t)depth_pixel_index].x = depth_point[0];
            pointcloud_->points[(size_t)depth_pixel_index].y = depth_point[1];
            pointcloud_->points[(size_t)depth_pixel_index].z = depth_point[2];

            float color_point[3];
            rs2_transform_point_to_point(color_point, &m_depth_2_color_extrinsics_, depth_point);
            float color_pixel[2];
            rs2_project_point_to_pixel(color_pixel, &m_color_intrinsics_, color_point);

            if (color_pixel[1] < 0 || color_pixel[1] >= color_height_ || color_pixel[0] < 0 || color_pixel[0] >= color_width_)
            {
                pointcloud_->points[(size_t)depth_pixel_index].x = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].y = m_invalid_depth_value_;
                pointcloud_->points[(size_t)depth_pixel_index].z = m_invalid_depth_value_;
            }
            else
            {
                unsigned int i_ = (unsigned int)color_pixel[1];
                unsigned int j_ = (unsigned int)color_pixel[0];
                if (swap_rgb_)
                {
                    pointcloud_->points[(size_t)depth_pixel_index].b =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_];
                    pointcloud_->points[(size_t)depth_pixel_index].g =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 1];
                    pointcloud_->points[(size_t)depth_pixel_index].r =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 2];
                }
                else
                {
                    pointcloud_->points[(size_t)depth_pixel_index].r =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_];
                    pointcloud_->points[(size_t)depth_pixel_index].g =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 1];
                    pointcloud_->points[(size_t)depth_pixel_index].b =
                        (uint32_t)p_color_frame[(i_ * (unsigned int)color_width_ + j_) * nb_color_pixel_ + 2];
                }

            }
        }
    }
    return pointcloud_;
}