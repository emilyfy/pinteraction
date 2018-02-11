#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <pinteraction/cblobtracking.h>
using namespace cv;

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
using namespace ros;

const string windowNameOriginal = "Original Image";
//const string windowNameHSV = "HSV";
//const string windowNameThreshold = "Threshold";
const string windowNameMorph = "Morphological Operations";
const string windowNameTrackbars = "Trackbars";

cv_bridge::CvImagePtr cv_ptr;
cv::Mat src;
vector<std_msgs::UInt16MultiArray> output;
int img_width, img_height;
float height;
int row, col;
float clearance = 11.0;                                      //distance btw hand and pins, in cm

void distCb(const std_msgs::UInt16 dst)
{
	//float distance = dst.data/1023.0*10.0;
	float distance = 15.0;
	float curr = output[row].data[col]/1023.0*10.0;
	height = curr + distance - clearance;
	if (height<0) height = 0.0;
	else if (height>10) height = 10.0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //Get the image in OpenCV format
  src = cv_ptr->image;
  if(src.empty())
  {
    ROS_INFO("Empty input. Looping...");
    return;
  }
//   cv::blur(src,src,Size(1,1));
//   cv::imshow("src", src);
}

int main(int argc, char* argv[])
{
  	int h[2] = { 0,8 };//{0,8};
	int s[2] = { 191,256 };//{98,256};
	int v[2] = { 67,256 };//{54,209};
	CBlobTracking tracker = CBlobTracking(h, s, v, 640, 480, 10, 400, (int)((640 * 480) / 1.5));
	tracker.SetIsTrackObjects(true);
	tracker.SetUseDilate(true);
	tracker.SetUseErode(true);

	std_msgs::UInt16MultiArray msg;
	std_msgs::MultiArrayDimension temp;
	msg.layout.dim.push_back(temp);
	msg.layout.dim[0].size = 10;
	msg.layout.dim[0].stride = 1;
	msg.layout.data_offset = 0;
	for (int i=0;i<10;i++) msg.data.push_back(0);

	ros::init(argc, argv, "vision");
	ros::NodeHandle n;
	n.setParam("/mode", 3);
	vector<ros::Publisher> pub;
	for (int i=0;i<10;i++) {
		ostringstream convert;
		convert << "/height/" << i+1;
		string str = convert.str();
		pub.push_back(n.advertise<std_msgs::UInt16MultiArray>(str, 20));
		msg.layout.dim[0].label = str;
		output.push_back(msg);
	}
	ros::Subscriber subd = n.subscribe("distance", 10, distCb);
	image_transport::ImageTransport it(n);
  	image_transport::Subscriber subimg = it.subscribe("/usb_cam/image_raw", 1, imageCb);
  
	ros::Rate loop_rate(10);

	if (n.getParam("/usb_cam/image_width", img_width)) {
		tracker.SetFrameWidth(img_width);
		img_width = img_width/10;
	}
	else img_width = tracker.GetFrameWidth() /10;
	if (n.getParam("/usb_cam/image_height", img_height)) {
		tracker.SetFrameHeight(img_height);
		img_height = img_height/10;
	}
	else img_height = tracker.GetFrameHeight() / 10;

	/*
	Uncomment this two lines of code in case you need to setup the binary image (AKA threshold) that way all you will see is what you want to track (As white)
	Everything else should be black
	*/
	//namedWindow("trackbars",0);
	//tracker.InitTrackbars("trackbars");

	/************************************************************/
		// Mat frame;
	//createTrackbars();
	// VideoCapture capture;
	// capture.open(0);                                                          //errors here
	// capture.set(CV_CAP_PROP_FRAME_WIDTH, tracker.GetFrameWidth());
	// capture.set(CV_CAP_PROP_FRAME_HEIGHT, tracker.GetFrameHeight());          //errors here

	// cout << "frame = " << frame << endl;

	while (ros::ok()) {
		// capture >> frame;
		try
		{
			tracker.Process(src);
			imshow(windowNameOriginal, tracker.GetOriginalImage());
		}
		catch (const std::exception& e)
		{
			// ROS_ERROR("exception: %s", e.what());
			ROS_INFO("waiting for video device");
		}
		// tracker.Process(src);

		//imshow(windowNameThreshold, tracker.GetBinaryThreshold());
		//imshow(windowNameHSV, tracker.GetHSVImage());
		
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command

		vector<TrackingObjects> objects = tracker.GetTrackingCoordinates();
		for (int i=0;i<objects.size();i++) {
			row = floor(objects[i].point.y / img_height);
			col = floor(objects[i].point.x / img_width);
			
			if (row>0) {
				if (col>0) output[row-1].data[col-1] = int(height/10.0*1023);
				output[row-1].data[col] = int(height/10.0*1023);
				if (col<9) output[row-1].data[col+1] = int(height/10.0*1023);
				pub[row-1].publish(output[row-1]);
			}

			if (row<9) {
				if (col>0) output[row+1].data[col-1] = int(height/10.0*1023);
				output[row+1].data[col] = int(height/10.0*1023);
				if (col<9) output[row+1].data[col+1] = int(height/10.0*1023);
				pub[row+1].publish(output[row+1]);
			}
			
			output[row].data[col] = int(height/10.0*1023);
			if (col>0) output[row].data[col-1] = int(height/10.0*1023);
			if (col<9) output[row].data[col+1] = int(height/10.0*1023);
            pub[row].publish(output[row]);
		}
		
		for (int i=0;i<10;i++) {
			for (int j=0;j<10;j++) output[i].data[j] = 0;
			if (objects.size()==0) pub[i].publish(output[i]);
		}
		
		ros::spinOnce();
		loop_rate.sleep();

		if (waitKey(30) == 13)
			break;
	}
	return 0;
}



/*
int main(int argc, char ** argv)
{
VideoCapture cap(0);
if(!cap.isOpened())
return -1;
Mat edges;
Mat grey;
namedWindow("edges",1);
namedWindow("original",2);
bool changeToGray = true;
bool useGaussian = false;
bool useCanny = false;
int screenX = GetSystemMetrics(SM_CXSCREEN);
bool stopLoop = false;
while(true)
{
Mat frame;
cap>>frame;
cvSetImageROI(&frame.operator IplImage(),cvRect(100,100,800,500));
cvtColor(frame,grey,CV_BGR2GRAY);
cvThreshold(&grey,&grey,100,255,CV_THRESH_BINARY);
IplImage* labelImg = cvCreateImage(cvGetSize(&grey),IPL_DEPTH_LABEL,1);
CvBlobs blobs;
unsigned int result = cvLabel(&grey.operator IplImage(), labelImg, blobs);
IplImage *imgOut = cvCreateImage(cvGetSize(&frame), IPL_DEPTH_8U, 3);
cvZero(imgOut);
cvRenderBlobs(labelImg, blobs, &frame.operator IplImage(), imgOut);

if(changeToGray)
cvtColor(frame, edges, CV_BGR2GRAY);
if(useGaussian)
GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
if(useCanny)
Canny(edges, edges, 0, 30, 3);
imshow("edges", edges);
int keyCode = waitKey(30);


//imshow("original",imgOut);
cvShowImage("original",imgOut);
switch(keyCode)
{
case 13:
stopLoop = true;;
break;
case 49:
changeToGray = !changeToGray;
break;
case 50:
useGaussian = !useGaussian;
break;
case 51:
useCanny = !useCanny;
}
if(stopLoop)
break;
}
return 0;
}

*/