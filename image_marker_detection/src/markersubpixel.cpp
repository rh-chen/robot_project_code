#include "markersubpixel.h"

void iMarkerRefine(IplImage* srcGray,CvPoint2D32f* rec)
{
	vector<Point2f>::iterator i;
	int j;
	
	//Mat iMat(srcGray,0);
	Mat iMat = cvarrToMat(srcGray,true);
	
	vector<Point2f> res;
	for(j = 0;j <4;j++)
		{
			res.push_back(Point2f(rec[j].x,rec[j].y));
		}
	
	cornerSubPix(iMat,res,Size(3,3),Size(-1,-1),TermCriteria(CV_TERMCRIT_ITER,30,0.1));
	
	j = 0;
	for(i = res.begin();i != res.end();i++)
		{	
			rec[j].x = (*i).x;
			rec[j].y = (*i).y;
			j++;
		}
		
	
	vector<Point2f>().swap(res);
	
}
