#include "ros/ros.h"
#include "opencv2/core/version.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include "tag_detector/kernel_size.h"


using namespace cv;
using namespace std;

class ImagePublisher
{
	private:
		ros::NodeHandle nh;
		ros::Publisher image_pub; 
		ros::ServiceClient kernel_size_client;

	public:
		
		ImagePublisher(); 
		/*brief constructor where we create the image_raw topic for publishing */
		
		void cvtImageAndPublish(VideoCapture& vid, int& blur_size);  
		/* Args: VideoCapture object with its argument specified by the file name or the device index.
		This function, reads the video in the form of cv::Mat, converts it to sensor_msgs/Image, 
		and then publishes the msgs to /image_raw topic. */

		int setBlurWindowSize(int& arg2);
		/* Args: The second argument, when the node is called is passed to this function. 
		In this function, we wait for the service response and set the window size accordingly. */ 


};