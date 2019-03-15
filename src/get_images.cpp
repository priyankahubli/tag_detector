#include "../include/tag_detector/get_images.hpp"

ImagePublisher::ImagePublisher()
{
	image_pub = nh.advertise<sensor_msgs::Image>("image_raw",1);
};
	

void ImagePublisher::cvtImageAndPublish(VideoCapture& vid)
{
	Mat frame;
	sensor_msgs::ImagePtr msg;
	ros::Rate loop_rate(5);

	while (nh.ok()) 
    {
    	vid >> frame;

    	// Check if grabbed frame is actually full with some content
    	if(!frame.empty()) 
    	{
    		// Convert frame to sesor_msgs/Image type
    		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    		image_pub.publish(msg);
    		waitKey(1);
    	}
    	else
    		ros::shutdown();

		loop_rate.sleep();
	}
};
	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_images");

	if (argv[1] == NULL)
	{
		cerr << "No video path given" << endl;
		return -1;		
	}

	VideoCapture vid(argv[1]); 

	// Throw error if video cannot be opened
	if (!vid.isOpened())
       throw invalid_argument("Video not found / Wrong file path");

   	ImagePublisher img_pub;
   	
   	// Load the video and publish it to /image_raw topic. 
   	img_pub.cvtImageAndPublish(vid);
   	
   	return 0;
}
