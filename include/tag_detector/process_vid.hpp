#include "ros/ros.h"
#include "opencv2/core/version.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>


using namespace cv;
using namespace std;

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
		This function is called whenever a new message arrives, it calls a function to detect the squares of the april tags, and
		draw around those contours. */

		void DetectTags(const cv::Mat& img);
		/* Args: img of type cv::Mat. 
		It detects the squares of the april tags and draws around these contours */

};


