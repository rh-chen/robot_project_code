#include "findmarker.h"


IplImage* iFindMarkerImage(IplImage* src,CvPoint2D32f* rec,int flag)
{
	int nScale = -3;
	CvPoint2D32f arrPoint[4];
	
	arrPoint[2].x = rec[2].x > nScale?rec[2].x-nScale:0;
	arrPoint[2].y = (rec[2].y + nScale) > src->height-1?src->height-1:rec[2].y+nScale;
	
	arrPoint[0].x = rec[1].x > nScale?rec[1].x-nScale:0;
	arrPoint[0].y = rec[1].y > nScale?rec[1].y-nScale:0;

	arrPoint[1].x = (rec[0].x + nScale) > src->width-1?src->width-1:rec[0].x+nScale;
	arrPoint[1].y = rec[0].y > nScale?rec[0].y-nScale:0;

	arrPoint[3].x = (rec[3].x + nScale) > src->width-1?src->width-1:rec[3].x+nScale;
	arrPoint[3].y = (rec[3].y + nScale) > src->height-1?src->height-1:rec[3].y+nScale;

	CvPoint2D32f dstArrPoint[4];
	
	int nScaleImage = 0;
	if(flag == MARKER_SIZE)
		nScaleImage = MARKER_SIZE;
	else if(flag == MARKER_SIZE_SMALL)
		nScaleImage = MARKER_SIZE_SMALL;
			
	dstArrPoint[0].x = 0;
	dstArrPoint[0].y = 0;
		
	dstArrPoint[1].x = nScaleImage-1;
	dstArrPoint[1].y = 0;
	
	dstArrPoint[2].x = 0;
	dstArrPoint[2].y = nScaleImage-1;

	dstArrPoint[3].x = nScaleImage-1;
	dstArrPoint[3].y = nScaleImage-1;
	
	CvMat* warpMat = cvCreateMat(3,3,CV_32FC1);
	
	cvGetPerspectiveTransform(arrPoint,dstArrPoint,warpMat);
	IplImage* pRectImage = cvCreateImage(cvSize(nScaleImage,nScaleImage),src->depth,1);
	
	cvWarpPerspective(src,pRectImage,warpMat);
		
	cvReleaseMat(&warpMat);
	
	
	return pRectImage;	


}
IplImage* iFindMarkerImage3Ch(IplImage* src,CvPoint2D32f* rec,int flag)
{
    int nScale = -3;
    CvPoint2D32f arrPoint[4];

    arrPoint[2].x = rec[2].x > nScale?rec[2].x-nScale:0;
    arrPoint[2].y = (rec[2].y + nScale) > src->height-1?src->height-1:rec[2].y+nScale;

    arrPoint[0].x = rec[1].x > nScale?rec[1].x-nScale:0;
    arrPoint[0].y = rec[1].y > nScale?rec[1].y-nScale:0;

    arrPoint[1].x = (rec[0].x + nScale) > src->width-1?src->width-1:rec[0].x+nScale;
    arrPoint[1].y = rec[0].y > nScale?rec[0].y-nScale:0;

    arrPoint[3].x = (rec[3].x + nScale) > src->width-1?src->width-1:rec[3].x+nScale;
    arrPoint[3].y = (rec[3].y + nScale) > src->height-1?src->height-1:rec[3].y+nScale;

    CvPoint2D32f dstArrPoint[4];

    int nScaleImage = 0;
    if(flag == MARKER_SIZE)
        nScaleImage = MARKER_SIZE;
    else if(flag == MARKER_SIZE_SMALL)
        nScaleImage = MARKER_SIZE_SMALL;

    dstArrPoint[0].x = 0;
    dstArrPoint[0].y = 0;

    dstArrPoint[1].x = nScaleImage-1;
    dstArrPoint[1].y = 0;

    dstArrPoint[2].x = 0;
    dstArrPoint[2].y = nScaleImage-1;

    dstArrPoint[3].x = nScaleImage-1;
    dstArrPoint[3].y = nScaleImage-1;

    CvMat* warpMat = cvCreateMat(3,3,CV_32FC1);

    cvGetPerspectiveTransform(arrPoint,dstArrPoint,warpMat);
    IplImage* pRectImage = cvCreateImage(cvSize(nScaleImage,nScaleImage),src->depth,3);

    cvWarpPerspective(src,pRectImage,warpMat);

    cvReleaseMat(&warpMat);


    return pRectImage;


}

