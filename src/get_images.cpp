#include "../include/tag_detector/get_images.hpp"

ImagePublisher::ImagePublisher()
{
	image_pub = nh.advertise<sensor_msgs::Image>("image_raw",1);
  kernel_size_client = nh.serviceClient<tag_detector::kernel_size>("set_blur_window_size");
};
	
void ImagePublisher::cvtImageAndPublish(VideoCapture& vid, int& blur_size)
{
	Mat frame, blurred_frame;
	sensor_msgs::ImagePtr msg;
	ros::Rate loop_rate(5);

	while (nh.ok()) 
    {
    	vid >> frame;

    	// Check if grabbed frame is actually full with some content
    	if(!frame.empty()) 
    	{
        // Smooth the image
        GaussianBlur(frame, blurred_frame, Size( blur_size, blur_size ), 0, 0 );
        
        // Convert frame to sesor_msgs/Image type
    		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", blurred_frame).toImageMsg();
    		image_pub.publish(msg);
    		waitKey(1);
    	}
    	else
    		ros::shutdown();

		loop_rate.sleep();
	}
};

int ImagePublisher::setBlurWindowSize(int& arg2)
{
  tag_detector::kernel_size srv;
  srv.request.size = arg2;
  int blur_size = 1;   //default

  if (kernel_size_client.call(srv))
  {
    blur_size = srv.response.blur_window_size;
    return blur_size;
  }
  else
  {
    cerr << "Failed to call service set_blur_window_size, using default value 1" << endl;
    return blur_size;
  }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_images");

	if (argv[1] == NULL && argv[2] == NULL)
	{
		cerr << "No video path or blurring window size given" << endl;
		return -1;		
	}

	VideoCapture vid(argv[1]); 

	// Throw error if video cannot be opened
	if (!vid.isOpened())
       throw invalid_argument("Video not found / Wrong file path");

  ImagePublisher img_pub;
  
  int arg_2 = atoi(argv[2]);
  
  //Get kernel_size
  int blur_size = img_pub.setBlurWindowSize(arg_2);

  // Load the video and publish it to /image_raw topic. 
 	img_pub.cvtImageAndPublish(vid, blur_size);
   	
 	return 0;
}
