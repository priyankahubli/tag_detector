#include "../include/tag_detector/process_vid.hpp"


ProcessVideo::ProcessVideo()
{
	img_sub = nh.subscribe("image_raw", 1000, &ProcessVideo::imageCallBack,this);
};

void ProcessVideo::imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
	// Check if there is a message published at the topic
	boost::shared_ptr<sensor_msgs::Image const> is_frame;
	is_frame = ros::topic::waitForMessage<sensor_msgs::Image>("image_raw",ros::Duration(5));
	if(is_frame == NULL)
	{
		std::cout<<("No messages received")<<std::endl;
		destroyWindow("view");
		ros::shutdown();
	}
	// If not published, close the window displaying the video and shutdown the node. 
	else 
	{
		imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
		waitKey(30);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"video_processor");
	namedWindow("view");
	ProcessVideo pv;
	ros::spin();
	return 0;
}