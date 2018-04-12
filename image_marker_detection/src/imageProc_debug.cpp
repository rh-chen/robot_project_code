#include "imageProc.h"
#include "math.h"
#include "malloc.h"
#include "time.h"

#define SORT
//#define OTSU
#define ADAPTIVE_THRESHOLD
#define SCALE_IMAGE
#define SHOW_IMAGE
//#define SAVE_IMAGE
#define VIDEO
#define COLOR

int main(int argc,char** argv)
{

	IplImage* pSrcImage;
#ifndef VIDEO
	pSrcImage = cvLoadImage(argv[1],1);
#endif
#ifdef VIDEO
    	CvCapture* pCapture = cvCaptureFromCAM(atoi(argv[1]));
	
	if(pCapture == NULL)
		return 0;
	
	cvSetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_WIDTH,1280);
	cvSetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_HEIGHT,720);
	
	
    	cvNamedWindow("camera",0);
    	cvMoveWindow("camera",0,0);
    	cvNamedWindow("imageProc",0);
    	cvMoveWindow("imageProc",512,1024);
    	cvNamedWindow("imageMarker",0);
    	cvMoveWindow("imageMaker",1024,0);
	cvNamedWindow("image2Value",0);
	cvMoveWindow("image2Value",512,512);
#endif

	CvFont nFont;
	cvInitFont(&nFont,CV_FONT_HERSHEY_COMPLEX,0.5,0.5,0,1,8);
	
	char arrIntToData[255] = {"\0"};
	char arrXCorToData[255] = {"\0"};
	char arrYCorToData[255] = {"\0"};

#ifdef VIDEO	
for(;;)
{	
#endif
#ifdef VIDEO
    	pSrcImage = cvQueryFrame(pCapture);
#endif
	clock_t nStartTime = clock();
#ifdef SCALE_IMAGE
	IplImage* pImage = cvCreateImage(cvSize(pSrcImage->width*FSCALE,pSrcImage->height*FSCALE),pSrcImage->depth,3); 
	cvResize(pSrcImage,pImage,CV_INTER_LINEAR);
#endif
	IplImage* pImageRect = cvCloneImage(pImage);


	IplConvKernel* pKernel = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);
	
	
	IplImage* pImageGrayR = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	IplImage* pImageGrayG = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	IplImage* pImageGrayB = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	
	
	cvSplit(pImage,pImageGrayB,pImageGrayG,pImageGrayR,NULL);
	
	IplImage* pImageGray = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	
	cvCopy(pImageGrayG,pImageGray);
	
	
	cvSmooth(pImageGray,pImageGray,CV_GAUSSIAN);
		
#ifdef SHOW_IMAGE
	//cvShowImage("pImageGray",pImageGray);
#endif
	
	
	IplImage* pImageGray2Value = cvCloneImage(pImageGray);
#ifdef OTSU
	int nGrayValue = otsu(pImageGray);
	cvThreshold(pImageGray,pImageGray2Value,nGrayValue,255,CV_THRESH_BINARY_INV);	

#endif
#ifdef ADAPTIVE_THRESHOLD
	cvAdaptiveThreshold(pImageGray,pImageGray2Value,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY,11,9);
#endif
	
	
#ifdef SHOW_IMAGE
	//ShowImage("pImageGray2Value",pImageGray2Value);
#endif
   	int i,j;
#ifdef COLOR
    /*颜色特征排除噪声点*/
    	for(i = 0;i < pImageGray->width;i++)
        	for(j = 0;j < pImageGray->height;j++)
            		{
                		if(CV_IMAGE_ELEM(pImageGray2Value,unsigned char,j,i) == 255)
					{
						
						unsigned char bGray = cvGet2D(pImage,j,i).val[0];
						unsigned char gGray = cvGet2D(pImage,j,i).val[1];
						unsigned char rGray = cvGet2D(pImage,j,i).val[2];
                    				if(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i) > 75)
                    					CV_IMAGE_ELEM(pImageGray2Value,unsigned char,j,i) = 0;
					}
					
            }
	
	cvMorphologyEx(pImageGray2Value,pImageGray2Value,pImageGray2Value,pKernel,CV_MOP_CLOSE,3);
#endif
#ifdef SHOW_IMAGE
	cvShowImage("image2Value",pImageGray2Value);
#endif
	
	CvPoint2D32f* pRect = new CvPoint2D32f[4];
	CvPoint2D32f* pRectPoint = new CvPoint2D32f[4];
	memset(pRect,0,4*sizeof(CvPoint2D32f));
	memset(pRectPoint,0,4*sizeof(CvPoint2D32f));
	
	/*分辨率*/
	double dRatio = RATIO;//mm/pixel:1680/1280
	double dMarkerWidth = MARKERWIDTH*FSCALE;//mm
	double dMarkerHeight = MARKERHEIGHT*FSCALE;//mm
	double dMaxDist = sqrt(dMarkerWidth*dMarkerWidth+dMarkerHeight*dMarkerHeight);

	int dAreaThresholdMin = cvRound((dMarkerWidth/dRatio)*(dMarkerHeight/dRatio)*TAN30);//tan(30)
	int dAreaThresholdMax = cvRound((dMaxDist/dRatio)*(dMaxDist/dRatio)*TAN30);//tan(30)
	
	
		
	printf("dAreaThresholdMin:%d\n",dAreaThresholdMin);
	printf("dAreaThresholdMax:%d\n",dAreaThresholdMax);
		
	
	iCvFindContour(pImageGray2Value,pRect,dAreaThresholdMin,dAreaThresholdMax);
	
	/*排序*/
#ifdef SORT
	CvPoint2D32f nCenter = cvPoint2D32f(0,0);//初始化，must
	
	for(i = 0;i < 4;i++)
		{
			nCenter.x = nCenter.x+pRect[i].x;
			nCenter.y = nCenter.y+pRect[i].y;
		}
	
	
	nCenter.x /=4;
	nCenter.y /=4;
	
	printf("nCenter:[%f,%f]\n",nCenter.x,nCenter.y);
	
	for(j = 0;j < 4;j++)
		{
			if(pRect[j].x > nCenter.x && pRect[j].y < nCenter.y)
				{
					pRectPoint[0].x = pRect[j].x;
					pRectPoint[0].y = pRect[j].y;
				}
			else if(pRect[j].x > nCenter.x && pRect[j].y > nCenter.y)
				{
					pRectPoint[3].x = pRect[j].x;
					pRectPoint[3].y = pRect[j].y;
				}
			else if(pRect[j].x < nCenter.x && pRect[j].y > nCenter.y )
				{
					pRectPoint[2].x = pRect[j].x;
					pRectPoint[2].y = pRect[j].y;
				}
			else if(pRect[j].x < nCenter.x && pRect[j].y < nCenter.y)
				{
					pRectPoint[1].x = pRect[j].x;
					pRectPoint[1].y = pRect[j].y;
				}
			


		}
	
#endif	
		
	
	
		
	printf("%s\n","/***************************/");
	printf("%f,%f\n",pRectPoint[0].x,pRectPoint[0].y);
	printf("%f,%f\n",pRectPoint[1].x,pRectPoint[1].y);
	printf("%f,%f\n",pRectPoint[2].x,pRectPoint[2].y);
	printf("%f,%f\n",pRectPoint[3].x,pRectPoint[3].y);
	printf("%s\n","/***************************/");
	
	iMarkerRefine(pImageGray,pRectPoint);

	printf("%s\n","/***************************/");
	printf("%f,%f\n",pRectPoint[0].x,pRectPoint[0].y);
	printf("%f,%f\n",pRectPoint[1].x,pRectPoint[1].y);
	printf("%f,%f\n",pRectPoint[2].x,pRectPoint[2].y);
	printf("%f,%f\n",pRectPoint[3].x,pRectPoint[3].y);
	printf("%s\n","/***************************/");
			
	IplImage* pImageUnitMarker = iFindMarkerImage(pImageGray,pRectPoint,128);//64
#ifdef SHOW_IMAGE
	//cvShowImage("imageProc",pImageUnitMarker);
#endif

	
	IplImage* pUnitMarker = iProcessUnitMarker(pImageUnitMarker);
#ifdef SHOW_IMAGE
	cvShowImage("imageProc",pUnitMarker);
#endif
		
	
	int nResultId = iDecodeMarker(pUnitMarker,pImageUnitMarker);
	
	if(nResultId > 0 && nResultId < 1024 ){	
		/*画线*/
        	cvLine(pImageRect,cvPointFrom32f(pRect[0]),cvPointFrom32f(pRect[1]),CV_RGB(0,255,0),2,CV_AA,0);
        	cvLine(pImageRect,cvPointFrom32f(pRect[1]),cvPointFrom32f(pRect[2]),CV_RGB(0,255,0),2,CV_AA,0);
        	cvLine(pImageRect,cvPointFrom32f(pRect[2]),cvPointFrom32f(pRect[3]),CV_RGB(0,255,0),2,CV_AA,0);
       	 	cvLine(pImageRect,cvPointFrom32f(pRect[0]),cvPointFrom32f(pRect[3]),CV_RGB(0,255,0),2,CV_AA,0);
        	cvCircle(pImageRect,cvPointFrom32f(pRect[0]),3,CV_RGB(255,0,0),2,8,0);
        	cvCircle(pImageRect,cvPointFrom32f(pRect[1]),3,CV_RGB(0,255,255),2,8,0);
        	cvCircle(pImageRect,cvPointFrom32f(pRect[2]),3,CV_RGB(0,0,255),2,8,0);
        	cvCircle(pImageRect,cvPointFrom32f(pRect[3]),3,CV_RGB(255,255,0),2,8,0);
	}

		
	printf("nResultId:%d\n",nResultId);
	
	clock_t nEndTime = clock();
	
	printf("Time:%f\n",(double)(nEndTime-nStartTime)/CLOCKS_PER_SEC);

	float x0 = pRectPoint[1].x;
	float y0 = pRectPoint[1].y;

	float x1 = pRectPoint[3].x;
	float y1 = pRectPoint[3].y;
	
	float x2 = pRectPoint[0].x;
	float y2 = pRectPoint[0].y;

	float x3 = pRectPoint[2].x;
	float y3 = pRectPoint[2].y;

	float y = ((y0-y1)*(y3-y2)*x0+(y3-y2)*(x1-x0)*y0+(y1-y0)*(y3-y2)*x2+(x2-x3)*(y1-y0)*y2)/((x1-x0)*(y3-y2)+(y0-y1)*(x3-x2));
	float x = x2+(x3-x2)*(y-y2)/(y3-y2);
	
	printf("X:%6.2f\n",x);
	printf("Y:%6.2f\n",y);
	
	
	if(nResultId > 0 && nResultId < 1024)
	{
		/*画坐标系*/	
      		cvLine(pImageRect,cvPoint(x,0),cvPoint(x,pImageRect->height),CV_RGB(255,0,255),2,CV_AA,0);
        	cvLine(pImageRect,cvPoint(0,y),cvPoint(pImage->width,y),CV_RGB(255,0,255),2,CV_AA,0);
        	cvLine(pImageRect,cvPoint(pImageRect->width/2,0),cvPoint(x,y),CV_RGB(255,255,0),2,CV_AA,0);
		
	
		sprintf(arrXCorToData,"%s:%6.2f","X",x);
		sprintf(arrYCorToData,"%s:%6.2f","Y",y);
		sprintf(arrIntToData,"%s:%d","DecodeData",nResultId);
		cvPutText(pImageRect,arrIntToData,cvPoint(0,30),&nFont,CV_RGB(0,0,0));
		cvPutText(pImageRect,arrXCorToData,cvPoint(0,60),&nFont,CV_RGB(0,0,0));
		cvPutText(pImageRect,arrYCorToData,cvPoint(0,90),&nFont,CV_RGB(0,0,0));
	}

	

#ifdef SHOW_IMAGE
	cvShowImage("camera",pImageRect);
#endif
	
	delete[] pRect;
	delete[] pRectPoint;
	cvReleaseImage(&pImage);
	cvReleaseImage(&pImageGrayR);
	cvReleaseImage(&pImageGrayG);
	cvReleaseImage(&pImageGrayB);
	cvReleaseImage(&pImageGray);
	cvReleaseImage(&pImageGray2Value);
	cvReleaseStructuringElement(&pKernel);
	cvReleaseImage(&pImageUnitMarker);
	cvReleaseImage(&pUnitMarker);
	cvReleaseImage(&pImageRect);
#ifdef VIDEO
	char c = cvWaitKey(30);
        if(c == 27)
                break;
}//end of while
#else
	cvWaitKey(0);
#endif
#ifdef VIDEO
	cvReleaseCapture(&pCapture);
#endif
	return 1;

}


