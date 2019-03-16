#include "ros/ros.h"
#include "tag_detector/kernel_size.h"

bool get_kernel_size(tag_detector::kernel_size::Request &req, tag_detector::kernel_size::Response &res)
{
  res.blur_window_size = req.size;
  return true;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_kernel_size");
	ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("set_blur_window_size",get_kernel_size);
    std::cout << "Response ready" << std::endl;
    ros::spin();
    return 0;
};
