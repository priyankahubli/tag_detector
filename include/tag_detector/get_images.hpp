#include "ros/ros.h"
#include "opencv2/core/version.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"

using namespace cv;
using namespace std;

class ImagePublisher
{
private:
	ros::NodeHandle nh;
	ros::Publisher image_pub; 
public:
	
	ImagePublisher(); 
	/*brief constructor where we create the image_raw topic for publishing */
	
	void cvtImageAndPublish(VideoCapture& vid);  
	/* Args: VideoCapture object with its argument specified by the file name or the device index.
	This function, reads the video in the form of cv::Mat, converts it to sensor_msgs/Image, 
	and then publishes the msgs to /image_raw topic. */

};