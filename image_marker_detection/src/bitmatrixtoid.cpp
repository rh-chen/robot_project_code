#include "bitmatrixtoid.h"

int iBitMatrixToId(Mat& bitMatrix)
{
	int nId = 0;
	
	for(int y = 0;y < 5;y++)
		{
			nId <<= 1;
			nId |= bitMatrix.at<uchar>(y,1);
			nId <<= 1;
			nId |= bitMatrix.at<uchar>(y,3);
		}
	return nId;

}
