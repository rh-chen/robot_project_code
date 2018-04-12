#ifndef CONTOUR_H
	#define CONTOUR_H
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include "findmarker.h"	
	
	using namespace cv;
	using namespace std;
	
	
	void iCvFindContour(IplImage* src,CvPoint2D32f* rec,int nAreaThresholdMin,int nAreaThresholdMax);
	void iCvFindContourS(IplImage* src,CvPoint2D32f* rec,int nAreaThresholdMin,int nAreaThresholdMax);
#endif
