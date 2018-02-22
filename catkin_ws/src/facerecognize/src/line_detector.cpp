#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Twist.h"
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

cv::Mat dst, cdst, src;
cv::Point center_of_mass;
cv::CascadeClassifier face_cascade, eyes_cascade;
image_transport::Publisher user_image_pub;
ros::Publisher line_error_pub;

std_msgs::Float32 error_msg;
image_transport::Subscriber raw_image_sub;

ros::Publisher vel_pub;

geometry_msgs::Twist vel_msg;

double line_center = 0;
int intersections = 0;

int hue_lower, hue_upper, sat_lower, sat_upper, value_lower, value_upper;
bool first_reconfig = true;
bool go = false;
ros::Subscriber go_sub;

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
	ROS_INFO("Callback!");
	try {
		src = cv_bridge::toCvShare(msg, "bgr8")->image;
ROS_INFO("38"); 
		std::vector<cv::Rect> faces;
		cv::Mat frame_gray;
		cv::cvtColor(src, frame_gray, cv::COLOR_BGR2GRAY);  // Convert to gray scale
		equalizeHist(frame_gray, frame_gray);    // Equalize histogram
 ROS_INFO("43");
		// Detect faces
		face_cascade.detectMultiScale(frame_gray, faces, 1.05, 3,
									  0 | cv::CASCADE_SCALE_IMAGE, cv::Size(15, 15));
		ROS_INFO("47");
		// Iterate over all of the faces
		for (size_t i = 0; i < faces.size(); i++) {

			// Find center of faces
			cv::Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);

			cv::Mat face = frame_gray(faces[i]);
			std::vector<cv::Rect> eyes;

			// Try to detect eyes, inside each face
			eyes_cascade.detectMultiScale(face, eyes, 1.1, 2,
										  0, cv::Size(15, 15));
 ROS_INFO("60");
			// Check to see if eyes inside of face, if so, draw ellipse around face
			if (eyes.size() > 0) {
				ellipse(src, center, cv::Size(faces[i].width / 2, faces[i].height / 2),
						0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);
				error_msg.data = faces[i].x - src.cols / 2;
				line_error_pub.publish(error_msg);
				break;
			}
		} 
		if (faces.size() == 0) {
			error_msg.data = 12345;
			line_error_pub.publish(error_msg);
		}
		sensor_msgs::ImagePtr msg;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
ROS_INFO("76");
		user_image_pub.publish(msg);
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}
void goHandler(const std_msgs::Bool &msg){
	go = msg.data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "line_detector");
	ros::NodeHandle nh;

	face_cascade.load("/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml");
	eyes_cascade.load("/usr/local/share/OpenCV/haarcascades/haarcascade_eye.xml");
	image_transport::ImageTransport it(nh);
	line_error_pub = nh.advertise<std_msgs::Float32>("/line_error", 10);
	//advertise the topic with our processed image
	user_image_pub = it.advertise("/user/image1", 1);
	go_sub = nh.subscribe("/doFollow", 10, goHandler);
	//subscribe to the raw usb camera image
	raw_image_sub = it.subscribe("/raspicam_node/image", 1, imageCallback);
	ROS_INFO("ready!");
	ros::spin();
}
