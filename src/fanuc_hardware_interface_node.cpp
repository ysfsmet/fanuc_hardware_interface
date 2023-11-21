#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <fanuc_hardware_interface/fanuc_hardware_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fanuc_hardware_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    fanuc_hardware_interface::FanucHardwareInterface rhi(nh);
    
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}