#include <pinteraction/cblobtracking.h>

CBlobTracking::CBlobTracking(void)
{
	m_hValues[0] = m_sValues[0] = m_vValues[0] = 0;
	m_hValues[1] = m_sValues[1] = m_vValues[1] = 256;
	m_frameWidth = 640;
	m_frameHeight = 480;
	m_maxObjects = 10;
	m_minObjectArea = 360;
	m_maxObjectArea = (int)((m_frameWidth * m_frameHeight) / 1.5);
	//m_originalWindowName = "Original Image";
	//m_trackbarWindowName = "HSV Min/Max Values";
	//m_hsvWindowName		 = "HSV";
	//m_thresholdWindowName = "Threshold Image";


}

CBlobTracking::CBlobTracking(int hMinMax[2], int sMinMax[2], int vMinMax[2], int frameWidth, int frameHeight, int maxObjects, int minObjectArea, int maxObjectArea)
{
	m_hValues[0] = hMinMax[0];
	m_sValues[0] = sMinMax[0];
	m_vValues[0] = vMinMax[0];
	m_hValues[1] = hMinMax[1];
	m_sValues[1] = sMinMax[1];
	m_vValues[1] = vMinMax[1];
	m_frameHeight = frameHeight;
	m_frameWidth = frameWidth;
	m_maxObjects = maxObjects;
	m_minObjectArea = minObjectArea;
	m_maxObjectArea = maxObjectArea;
	//m_originalWindowName = originalWindowName;

}


CBlobTracking::~CBlobTracking(void)
{
}

bool CBlobTracking::Process(Mat& originalImage)
{
	m_originalImage = originalImage;
	// convert original image to HSV
	cvtColor(m_originalImage, m_hsvImage, COLOR_BGR2HSV);
	inRange(m_hsvImage, Scalar(m_hValues[0], m_sValues[0], m_vValues[0]), Scalar(m_hValues[1], m_sValues[1], m_vValues[1]), m_binaryThreshold);
	if (m_useDilate || m_useErode)
		MorphImage(m_binaryThreshold);

	if (m_isTrackObjects)
		m_totalObjectTracked = TrackObjects(m_binaryThreshold, m_originalImage);

	return true;
}

int CBlobTracking::TrackObjects(Mat threshold, Mat& original)
{
	Mat thresholdCopy;
	threshold.copyTo(thresholdCopy);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	m_coordinates.clear();
	findContours(thresholdCopy, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	//drawContours(original,contours,3,Scalar(0,0,255),3,8,hierarchy,0);
	bool isObjectFound = false;
	int totalObjects = 0;
	double holdArea = 0.0;
	if ((int)hierarchy.size()>0 && (int)hierarchy.size()<m_maxObjects)
	{
		int numObjects = hierarchy.size();
		int index = 0;
		double area;
		while (index >= 0)
		{
			Moments moment = moments((Mat)contours[index]);
			area = moment.m00;
			if (area >m_minObjectArea && area<m_maxObjectArea && area > holdArea)
			{
				TrackingObjects c;
				c.point.x = (int)(moment.m10 / area);
				c.point.y = (int)(moment.m01 / area);
				c.area = area;
				m_coordinates.push_back(c);
				holdArea = area;
				circle(original, Point(c.point), 50, Scalar(0, 0, 255), 1);
				ostringstream ss;
				ss << c.point.x << "," << c.point.y;
				string str(ss.str());
				putText(original, str, Point(c.point.x, c.point.y + 30), 1, 1, Scalar(0, 0, 255), 2);

				//cout << c.point.x << ", " << c.point.y << endl;
				//cout << xco << ", " << yco << endl;
				//x-coordinate (column number) of the pin
				//y-coordinate (row number) of the pin

				//xco = floor(c.point.x / 64);
				//yco = floor(c.point.y / 48);

			}
			index = hierarchy[index][0];
		}

	}
	
	/*
	else
	{
		cout << "too many objects. Might be noise!" << endl;
	}
	*/

	return totalObjects;

}

void CBlobTracking::MorphImage(Mat &threshold)
{

	if (m_useErode)
	{
		Mat e = getStructuringElement(MORPH_RECT, Size(2, 2));
		erode(threshold, threshold, e);
		erode(threshold, threshold, e);
	}

	if (m_useDilate)
	{
		Mat d = getStructuringElement(MORPH_RECT, Size(6, 6));
		dilate(threshold, threshold, d);
		dilate(threshold, threshold, d);
	}


}
void on_trackbar(int, void*)
{
}
void CBlobTracking::InitTrackbars(string windowName)
{
	createTrackbar("H_MIN", windowName, &m_hValues[0], m_hValues[1], on_trackbar);
	createTrackbar("H_MAX", windowName, &m_hValues[1], m_hValues[1], on_trackbar);
	createTrackbar("S_MIN", windowName, &m_sValues[0], m_sValues[1], on_trackbar);
	createTrackbar("S_MAX", windowName, &m_sValues[1], m_sValues[1], on_trackbar);
	createTrackbar("V_MIN", windowName, &m_vValues[0], m_vValues[1], on_trackbar);
	createTrackbar("V_MAX", windowName, &m_vValues[1], m_vValues[1], on_trackbar);
}

