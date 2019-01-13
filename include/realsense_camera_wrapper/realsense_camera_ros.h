#include<realsense_camera.h>

class RealsenseCameraRos
{

public:
    
    RealsenseCameraRos();

    std::shared_ptr<RealsenseCamera> device;
    
};