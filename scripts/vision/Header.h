#pragma once
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

using namespace std;
using namespace cv;
struct TrackingObjects
{
	CvPoint point;
	double area;
};
class CBlobTracking
{
public:
	CBlobTracking(void);
	CBlobTracking(int hMinMax[2], int sMinMax[2], int vMinMax[2], int frameWidth, int frameHeight, int maxObjects, int minObjectArea, int maxObjectArea);
	~CBlobTracking(void);

	bool Process(Mat &originalImage);
	void InitTrackbars(string windowName);

	/**************************************/
	/******* Setters And Getters **********/
	/**************************************/
	/**************************************/
	/*************** Setters **************/
	/**************************************/

	void SetFrameWidth(int frameWidth) { m_frameWidth = frameWidth; }
	void SetFrameHeight(int frameHeight) { m_frameHeight = frameHeight; }
	void SetMaxObjects(int maxObjects) { m_maxObjects = maxObjects; }
	void SetMinObjectArea(int minObjectArea) { m_minObjectArea = minObjectArea; }
	void SetMaxObjectArea(int maxObjectArea) { m_maxObjectArea = maxObjectArea; }
	//void SetOriginalWindowName(string name){m_originalWindowName.assign(name);}
	//void SetTrackingWindowName(string name){m_trackingWindowName.assign(name);}
	//void SetTrackbarWindowName(string name){m_trackbarWindowName.assign(name);}
	//void SetHSVWindowName(string name){m_hsvWindowName.assign(name);}
	//void SetThresholdWindowName(string name){m_thresholdWindowName.assign(name);}
	void SetShowTrackbar(bool isShow) { m_showTrackbar = isShow; }
	void SetUseErode(bool useErode) { m_useErode = useErode; }
	void SetUseDilate(bool useDilate) { m_useDilate = useDilate; }
	void SetIsTrackObjects(bool isTrackable) { m_isTrackObjects = isTrackable; }

	/**************************************/
	/*************** Getters **************/
	/**************************************/
	int GetFrameWidth() { return m_frameWidth; }
	int GetFrameHeight() { return m_frameHeight; }
	int GetMaxObjects() { return m_maxObjects; }
	int GetMinObjectArea() { return m_minObjectArea; }
	int GetMaxObjectArea() { return m_maxObjectArea; }
	//string GetOriginalWindowName(){return m_originalWindowName;}
	//string GetTrackingWindowName(){return m_trackingWindowName;}
	//string GetTrackbarWindowName(){return m_trackbarWindowName;}
	//string GetHSVWindowName(){return m_hsvWindowName;}
	//string GetThresholdWindowName(){return m_thresholdWindowName;}
	bool GetShowTrackbar() { return m_showTrackbar; }
	bool GetUseErode() { return m_useErode; }
	bool GetUseDilate() { return m_useDilate; }
	bool GetIsTrackObjects() { return m_isTrackObjects; }
	vector<TrackingObjects> GetTrackingCoordinates() {
		if (m_isTrackObjects)
			return m_coordinates;

		return vector<TrackingObjects>();
	}
	int GetTotalObjectTracked() {
		if (m_isTrackObjects)
			return m_totalObjectTracked;
		return 0;
	}

	Mat GetOriginalImage() { return m_originalImage; }
	Mat GetHSVImage() { return m_hsvImage; }
	Mat GetBinaryThreshold() { return m_binaryThreshold; }
	int* GetHueValues() { return m_hValues; }
	int* GetSaturatedValues() { return m_sValues; }
	int* GetValueValues() { return m_vValues; }


	/**************************************/



private:
	/**************************************/
	/********* Class Variables ************/
	/**************************************/
	//string m_originalWindowName;
	//string m_trackingWindowName;
	//string m_trackbarWindowName;
	//string m_hsvWindowName;
	//string m_thresholdWindowName;

	bool m_showTrackbar;
	bool m_useErode;
	bool m_useDilate;
	bool m_isTrackObjects;

	int m_maxObjects;
	int m_frameWidth;
	int m_frameHeight;
	int m_minObjectArea;
	int m_maxObjectArea;
	int m_totalObjectTracked;

	int m_hValues[2];
	int m_sValues[2];
	int m_vValues[2];

	Mat m_originalImage;
	Mat m_hsvImage;
	Mat m_binaryThreshold;

	vector<TrackingObjects> m_coordinates;

	/**************************************/
	/********** Private Functions *********/
	/**************************************/

	void MorphImage(Mat&);
	int TrackObjects(Mat, Mat&);

};


