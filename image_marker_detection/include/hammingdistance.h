#ifndef HAMMING_DISTANCE_H
	#define HAMMING_DISTANCE_H
	#include <opencv2/highgui/highgui.hpp>
	#include <opencv2/imgproc/imgproc.hpp>
	#include "stdlib.h"
	#include "vector"

	using namespace cv;
	using namespace std;
	int iHammingDistance(Mat& bitMatrix);
    int iHammingDistanceAnother(int arrRightDirection[5][5],Mat& bitMatrix);
#endif
