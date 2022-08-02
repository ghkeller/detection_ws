#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <geometry_msgs/Vector3.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher det_rvec_pub_ = nh_.advertise<geometry_msgs::Vector3>("/marker_detection/pose/rvec",1);
  ros::Publisher det_tvec_pub_ = nh_.advertise<geometry_msgs::Vector3>("/marker_detection/pose/tvec",1);

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iris/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }
	geometry_msgs::Vector3 rvec_msg_;
	geometry_msgs::Vector3 tvec_msg_;
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_bridge::CvImagePtr cv_ptr_out;
    try
    {
      cv_ptr_out = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // run detection on the image
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    double f_x = 277.191356;
    double f_y = 277.191356;
    double c_x = 320.5;
    double c_y = 240.5;

    double cMat[3][3] = {{f_x, 0, c_x},
	    	   {0, f_y, c_y},
		   {0, 0, 1}};
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, cMat);

    double k_1 = 0.0;
    double k_2 = 0.0;
    double p_1 = 0.0;
    double p_2 = 0.0;

    double dVec[4] = {k_1, k_2, p_1, p_2};
    cv::Mat distCoeffs = cv::Mat(1, 4, CV_64F, dVec);

    std::vector<cv::Vec3d> rvecs, tvecs;
    
    // estimate pose from marker
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 1.0, cameraMatrix, distCoeffs, rvecs, tvecs); 


    // Update GUI Window
    cv::Mat outputImage = cv_ptr->image.clone();
    for (int i = 0; i < rvecs.size(); ++i) {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
        cv::aruco::drawAxis(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.5);
	
	// Output the relative pose of the aruco marker 
	rvec_msg_.x = rvecs[0][0];
	rvec_msg_.y = rvecs[0][1];
	rvec_msg_.z = rvecs[0][2];

	tvec_msg_.x = tvecs[0][0];
	tvec_msg_.y = tvecs[0][1];
	tvec_msg_.z = tvecs[0][2];

    	det_rvec_pub_.publish( rvec_msg_ );
    	det_tvec_pub_.publish( tvec_msg_ );
    }
    
    //cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    cv::imshow(OPENCV_WINDOW, outputImage);
    cv::waitKey(3);

    // Output modified video stream
    cv_ptr_out->image = outputImage;
    image_pub_.publish(cv_ptr_out->toImageMsg());

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ROS_INFO("-- ArUco Detector initialized.");
  ImageConverter ic;
  ros::spin();
  return 0;
}
