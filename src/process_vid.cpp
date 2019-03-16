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
		cout << ("No messages received") << endl;
		destroyWindow("view");
		ros::shutdown();
	}
	// If not published, close the window displaying the video and shutdown the node. 
	else 
	{
		Mat img_sharp;
		ros::Time start_, end_;
		// Start timer to calculate computation time for detection and visualization
		start_ = ros::Time::now();

		// Convert sensor_msgs/Image to cv::Mat format 
		Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

		// Evaluate sharpness 
		bool is_blur = ProcessVideo::EvaluateSharpness(img);

		if (is_blur)
		{
			img_sharp = ProcessVideo::DeblurImage(img);
		}
		else
		{
			img_sharp = img.clone();
		}

		// Detect and visualize rectangles around april tags. 
		ProcessVideo::DetectTags(img_sharp);		

		// Stop the timer
		end_ = ros::Time::now();

		// Calculate computation time and display
		double computation_time = (end_ - start_).toNSec() * 1e-6;
		cout << "Computation time (ms): " << computation_time << endl;

	}
};

void ProcessVideo::DetectTags(const cv::Mat& img)
{
	Mat img_gray, img_bw;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Point> approx;

	// Convert color image to gray 
	cvtColor(img, img_gray, CV_BGR2GRAY);

	// Threshold gray image to black and white
	threshold(img_gray, img_bw,127,255,CV_THRESH_BINARY);
	// Apply Canny edge detector
	Canny(img_bw, img_bw, 50, 100, 3);

	// Find Contours 
	findContours(img_bw, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Visualize the detections using drawContours
	for (int i = 0; i< contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
        // Since we want to detect the squares/rectangle of the april tags, we consider only contours with 4 vertices
		if( approx.size() == 4)
		{
			Scalar color = Scalar(rand()%255,rand()%255,rand()%255);
			drawContours(img, contours, i, color, 2, 8, hierarchy, 0, Point());
		}
	}  
	
	imshow("view", img);
	waitKey(1);
	return;
};

bool ProcessVideo::EvaluateSharpness(const cv::Mat& img)
{
	double min, max;
	Mat img_gray, dst;

	// Convert color image to gray 	
	cvtColor(img, img_gray, CV_BGR2GRAY);

	Mat planes[] = {Mat_<float>(img_gray), Mat::zeros(img_gray.size(), CV_32F)};

    //Complex plane to contain the DFT coefficients {[0]-Real,[1]-Img}
    Mat complexI;    
    
    merge(planes, 2, complexI);
    dft(complexI, complexI);  

    // compute magnitude
    split(complexI, planes);
    magnitude(planes[0], planes[1], planes[0]);
    Mat magI = planes[0];
    
    // Switch to logarithmic scale
    magI += Scalar::all(1);                    
	log(magI, magI);

	minMaxLoc(magI, &min, &max);

	// Count the number of pixels displaying high frequencies 
	int threshold_value = (max - min) / 2;
	threshold(magI, dst, threshold_value, 255, THRESH_BINARY);
	int num_high_freqs = countNonZero(dst);
	
	// If more than 5% of the pixels have high frequencies then we consider the image to be blurry. 
	if (num_high_freqs > (0.05*magI.rows*magI.cols))
			return false;
	
	return true;

};

cv::Mat ProcessVideo::DeblurImage(const cv::Mat& img)
{
	// Ideally would like to apply the weiner filter to deblur the image. 
	
	Mat img_sharp;
	float kernel_values[10] = { 0, -1/4, 0, -1/4,
                               8/4, -1/4, 0, -1/4, 0};
	cv::Mat kernel = cv::Mat(3, 3, CV_8U, kernel_values);
    filter2D(img, img_sharp, -1, kernel);

	return img+img_sharp;

};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"video_processor");
	namedWindow("view");
	ProcessVideo pv;
	ros::spin();
	return 0;
}