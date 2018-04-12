#include "processunitmarker.h"
#include "otsu.h"

IplImage* iProcessUnitMarker(IplImage* src)
{
    
    IplImage* pGray = cvCreateImage(cvGetSize(src),8,1);
    
    cvCvtColor(src,pGray,CV_RGB2GRAY);

	int nGray2Value = otsu(pGray);
	IplConvKernel* pKernel = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);

	//IplImage* pUnitMarkerImage = cvCloneImage(pGray);
	//cvZero(pUnitMarkerImage);
	
	//cvThreshold(src,pUnitMarkerImage,nGray2Value,255,CV_THRESH_BINARY);

	//cvMorphologyEx(pUnitMarkerImage,pUnitMarkerImage,pUnitMarkerImage,pKernel,CV_MOP_CLOSE,3);

	//cvFloodFill(pUnitMarkerImage,cvPoint(0,0),cvScalarAll(0),cvScalarAll(0),cvScalarAll(0),NULL,8,NULL);

	cvThreshold(pGray,pGray,nGray2Value,255,CV_THRESH_BINARY);

	cvReleaseStructuringElement(&pKernel);
	return pGray;
}
IplImage* iProcessUnitMarkerS(IplImage* src)
{
        int nGray2Value = otsu(src);
        IplConvKernel* pKernel = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);

        IplImage* pUnitMarkerImage = cvCloneImage(src);
        cvZero(pUnitMarkerImage);

        cvThreshold(src,pUnitMarkerImage,nGray2Value,255,CV_THRESH_BINARY);

        //cvMorphologyEx(pUnitMarkerImage,pUnitMarkerImage,pUnitMarkerImage,pKernel,CV_MOP_CLOSE,3);


        cvReleaseStructuringElement(&pKernel);
        return pUnitMarkerImage;
}

