#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <iostream>
#include <opencv/cv.h>
#include "Header.h"
#include <math.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

const string windowNameOriginal = "Original Image";
//const string windowNameHSV = "HSV";
//const string windowNameThreshold = "Threshold";
const string windowNameMorph = "Morphological Operations";
const string windowNameTrackbars = "Trackbars";

int main(int argc, char* argv[])
{
	int h[2] = { 0,8 };//{0,8};
	int s[2] = { 191,256 };//{98,256};
	int v[2] = { 67,256 };//{54,209};
	CBlobTracking tracker = CBlobTracking(h, s, v, 640, 480, 10, 400, (int)((640 * 480) / 1.5));
	tracker.SetIsTrackObjects(true);
	tracker.SetUseDilate(true);
	tracker.SetUseErode(true);

	/*
	Uncomment this two lines of code in case you need to setup the binary image (AKA threshold) that way all you will see is what you want to track (As white)
	Everything else should be black
	*/
	//namedWindow("trackbars",0);
	//tracker.InitTrackbars("trackbars");

	/************************************************************/
		Mat frame;
	//createTrackbars();
	VideoCapture capture;
	capture.open(0);
	capture.set(CV_CAP_PROP_FRAME_WIDTH, tracker.GetFrameWidth());
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, tracker.GetFrameHeight());
	while (1) {
		capture >> frame;
		tracker.Process(frame);

		//imshow(windowNameThreshold, tracker.GetBinaryThreshold());
		imshow(windowNameOriginal, tracker.GetOriginalImage());
		//imshow(windowNameHSV, tracker.GetHSVImage());
		
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command

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