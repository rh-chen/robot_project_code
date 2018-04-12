#ifndef DECODE_MARKER_H
	#define DECODE_MARKER_H	
	#include "config.h"
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include "vector"
	#include "findmarker.h"	
	#include "hammingdistance.h"
	#include "bitmatrixrotate.h"
	#include "bitmatrixtoid.h"	
	#include "contour.h"
	#include "string.h"	
	#include "processunitmarker.h"
					
	using namespace cv;
	using namespace std;
	
	
	
	int iDecodeMarker(IplImage* src,IplImage* mask);
    int iDecodeMarkerAnother(IplImage* src,IplImage* mask,int arrRightDirection[5][5]);
#endif
