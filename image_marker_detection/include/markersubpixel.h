#ifndef MARKER_REFINE
	#define MARKER_REFINE
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include "vector"
	using namespace cv;
	using namespace std;
	
	void iMarkerRefine(IplImage* srcGray,CvPoint2D32f* rec);
#endif
