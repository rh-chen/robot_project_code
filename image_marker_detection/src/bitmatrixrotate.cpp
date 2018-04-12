#include "bitmatrixrotate.h"

Mat iBitMatrixRotate(Mat& bitMatrix)
{
	Mat outMat = bitMatrix.clone();
	
	int nRows = bitMatrix.rows;
	int nCols = bitMatrix.cols;

	int i,j;
		
	for(i = 0;i < nRows;i++)
		{
			for(j = 0;j < nCols;j++)
			{
				outMat.at<uchar>(i,j) = bitMatrix.at<uchar>(nCols-j-1,i);

			}
		}
	return outMat;

}
