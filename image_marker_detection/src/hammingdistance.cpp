#include "hammingdistance.h"

int iHammingDistance(Mat& bitMatrix)
{
	int y,x;
	//int j;

	/*model*/	
	const int ids[5][5] = {
				{1,0,0,0,0},{1,0,0,0,0},{1,1,1,1,1},{1,1,1,0,0},{1,0,0,1,1}//
				//{0,0,1,0,0},{1,1,0,0,1},{1,0,0,1,0},{0,0,0,1,1},{1,0,1,0,1}//
				//{1,0,1,1,1},{0,0,1,1,1},{1,1,1,1,0},{1,0,0,0,0},{0,1,0,1,1}//
				
				
			};
	int nDist = 0;
	
		
	for(y = 0;y < 5;y++)
		{
			int nMin = INT_MAX;
			for(int p = 0;p < 5;p++)
				{
					int nSum = 0;
					for(x = 0;x < 5;x++)
						{
							nSum += !(bitMatrix.at<uchar>(y,x) == ids[p][x]);
						}
					nMin = nMin < nSum?nMin:nSum;
				}
		
			nDist += nMin;
		}
	
	return nDist;		
}
int iHammingDistanceAnother(int arrRightDirection[5][5],Mat& bitMatrix)
{
    int y,x;
    //int j;


    int nDist = 0;

    for(y = 0;y < 5;y++)
        {
            int nMin = INT_MAX;
            for(int p = 0;p < 5;p++)
                {
                    int nSum = 0;
                    for(x = 0;x < 5;x++)
                        {
                            nSum += !(bitMatrix.at<uchar>(y,x) == arrRightDirection[p][x]);
                        }
                    nMin = nMin < nSum?nMin:nSum;
                }

            nDist += nMin;
        }

    return nDist;
}
