#ifndef FIND_MARKER_H
	#define FIND_MARKER_H
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include "config.h"
	
		
	using namespace cv;
	using namespace std;
	typedef struct marker{	Point m_corners[4];}Marker;
	IplImage* iFindMarkerImage(IplImage* src,CvPoint2D32f* rec,int flag);
	IplImage* iFindMarkerImage3Ch(IplImage* src,CvPoint2D32f* rec,int flag);
#endif
