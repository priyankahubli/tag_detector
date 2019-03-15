#include "ros/ros.h"
#include "opencv2/core/version.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"

using namespace cv;

class ProcessVideo
{
private:
	ros::NodeHandle nh;
	ros::Subscriber img_sub;

public:
	ProcessVideo();
	/*brief constructor where we create a subscriber to /image_raw*/
	
	void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
	/* Args: The message published by the topic this is subscribing to. 
	This function is called whenever a new message arrives and it displays the msgs in a window. */

};


