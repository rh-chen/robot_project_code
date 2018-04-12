#include "imageProc.h"
#include "math.h"
#include "malloc.h"
#include "time.h"

#define SORT
#define OTSU
//#define ADAPTIVE_THRESHOLD
#define SCALE_IMAGE
//#define SHOW_IMAGE
//#define SAVE_IMAGE
//#define DEBUG
#define RELEASE
#define DELETE_EDGE_FALSE
#define HALF_EDGE

int main(int argc,char** argv)
{
	CvFont nFont;
	cvInitFont(&nFont,CV_FONT_HERSHEY_COMPLEX,0.5,0.5,0,1,8);
	
	char arrIntToData[255] = {"\0"};
	char arrXCorToData[255] = {"\0"};
	char arrYCorToData[255] = {"\0"};
    char arrFlagToData[255] = {"\0"};

for(;;)
{	
	clock_t nStartTime = clock();
    IplImage* pSrcImage = NULL;

	
#ifdef SCALE_IMAGE
	//printf("width=%d,height=%d,depth=%d",pSrcImage->width*FSCALE,pSrcImage->height*FSCALE,pSrcImage->depth);
	IplImage* pImage = cvCreateImage(cvSize(pSrcImage->width*FSCALE,pSrcImage->height*FSCALE),pSrcImage->depth,3); 
	//printf("resize\n");
	cvResize(pSrcImage,pImage,CV_INTER_LINEAR);
#endif
	//printf("cloneImage\n");
	IplImage* pImageRect = cvCloneImage(pImage);

	//printf("create structur element.\n");
	IplConvKernel* pKernel = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);
	
	//printf("depth=%d\n",pImage->depth);	
	IplImage* pImageGrayR = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	IplImage* pImageGrayG = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	IplImage* pImageGrayB = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	
//	printf("start to split image.\n");	
	cvSplit(pImage,pImageGrayB,pImageGrayG,pImageGrayR,NULL);
	
	IplImage* pImageGray = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
	
	cvCopy(pImageGrayG,pImageGray);

    //cvCvtColor(pImage,pImageGray,CV_RGB2GRAY);
	
//	printf("smooth the image,will cause crash.\n");
	//TODO: below line will cause crash.
	cvSmooth(pImageGray,pImageGray,CV_GAUSSIAN);
	
//	printf("clone imge\n");
	IplImage* pImageGray2Value = cvCloneImage(pImageGray);
#ifdef OTSU
	int nGrayValue = otsu(pImageGray);
    int nGrayValueScale = -15;
	cvThreshold(pImageGray,pImageGray2Value,nGrayValue-nGrayValueScale,255,CV_THRESH_BINARY_INV);	

#endif
    //cvCanny(pImageGray,pImageGray2Value,75,150,3);
#ifdef ADAPTIVE_THRESHOLD
	cvAdaptiveThreshold(pImageGray,pImageGray2Value,255,CV_ADAPTIVE_THRESH_GAUSSIAN_C,CV_THRESH_BINARY_INV,11,9);
#endif

   	int i,j;
//	printf("filter for color, crash point!\n");
#ifdef DELETE_EDGE_FALSE
    /*颜色特征排除噪声点*/
    	for(i = 1;i < pImageGray->width-1;i++)
        	for(j = 1;j < pImageGray->height-1;j++)
            		{
                		if(CV_IMAGE_ELEM(pImageGray2Value,unsigned char,j,i) == 255)
    					{
						
	    					//unsigned char bGray = cvGet2D(pImage,j,i).val[0];
		    				//unsigned char gGray = cvGet2D(pImage,j,i).val[1];
			    			//unsigned char rGray = cvGet2D(pImage,j,i).val[2];
                    		if(
                              abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j,i-1)) > 8
                            ||abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j,i+1)) > 8
                            ||abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j+1,i)) > 8
                            ||abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j-1,i)) > 8
                            ||abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j-1,i-1)) > 8
                            ||abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j-1,i+1)) > 8
                            ||abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j+1,i-1)) > 8
                            ||abs(CV_IMAGE_ELEM(pImageGray,unsigned char,j,i)-CV_IMAGE_ELEM(pImageGray,unsigned char,j+1,i+1)) > 8
                            )
                    		    CV_IMAGE_ELEM(pImageGray2Value,unsigned char,j,i) = 0;
					    }
					
            }
//	printf("cvMorphologyEx, crash point08251115.\n");	
	cvMorphologyEx(pImageGray2Value,pImageGray2Value,pImageGray2Value,pKernel,CV_MOP_CLOSE,3);
#endif

#ifdef SHOW_IMAGE
	cvShowImage("image2Value",pImageGray2Value);
#endif

//	printf("new cv point.\n");
	CvPoint2D32f* pRect = new CvPoint2D32f[4];
	CvPoint2D32f* pRectPoint = new CvPoint2D32f[4];
	memset(pRect,0,4*sizeof(CvPoint2D32f));
	memset(pRectPoint,0,4*sizeof(CvPoint2D32f));
	
	iCvFindContour(pImageGray2Value,pRect,-1,-1);
	//printf("before SORT\n");	

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
	
	//printf("nCenter:[%f,%f]\n",nCenter.x,nCenter.y);
	
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
		
//	printf("%s\n","/***************************/");
//	printf("%f,%f\n",pRectPoint[0].x,pRectPoint[0].y);
//	printf("%f,%f\n",pRectPoint[1].x,pRectPoint[1].y);
//	printf("%f,%f\n",pRectPoint[2].x,pRectPoint[2].y);
//	printf("%f,%f\n",pRectPoint[3].x,pRectPoint[3].y);
//	printf("%s\n","/***************************/");
	
	iMarkerRefine(pImageGray,pRectPoint);

//	printf("%s\n","/***************************/");
//	printf("%f,%f\n",pRectPoint[0].x,pRectPoint[0].y);
//	printf("%f,%f\n",pRectPoint[1].x,pRectPoint[1].y);
//	printf("%f,%f\n",pRectPoint[2].x,pRectPoint[2].y);
//	printf("%f,%f\n",pRectPoint[3].x,pRectPoint[3].y);
//	printf("%s\n","/***************************/");
	

    pRectPoint[0].x = pRectPoint[0].x/FSCALE;
    pRectPoint[0].y = pRectPoint[0].y/FSCALE;
    pRectPoint[1].x = pRectPoint[1].x/FSCALE;
    pRectPoint[1].y = pRectPoint[1].y/FSCALE;
    pRectPoint[2].x = pRectPoint[2].x/FSCALE;
    pRectPoint[2].y = pRectPoint[2].y/FSCALE;
    pRectPoint[3].x = pRectPoint[3].x/FSCALE;
    pRectPoint[3].y = pRectPoint[3].y/FSCALE;

    
    //printf("aaaaaaaaaaaaaaaaaaaaaaaaa\n");
    IplImage* pImageUnitMarker = iFindMarkerImage3Ch(pSrcImage,pRectPoint,128);
	//IplImage* pImageUnitMarker = iFindMarkerImage(pImageGray,pRectPoint,256);//64
//	printf("find marker image\n");
#ifdef SHOW_IMAGE
	cvShowImage("imageProc",pImageUnitMarker);
#endif

	
	IplImage* pUnitMarker = iProcessUnitMarker(pImageUnitMarker);
#ifdef SHOW_IMAGE
	cvShowImage("imageMarker",pUnitMarker);
#endif
		
	
	//int nResultId = iDecodeMarker(pUnitMarker,pImageUnitMarker);//2015.8.13
    
    int nResultId = 0;
    int nRightMarker = 0;
    
  //  printf("NMarker:%d\n",NMarker);

    for(i = 0;i < NMarker;i++)
    {
       nResultId = iDecodeMarkerAnother(pUnitMarker,pImageUnitMarker,arrMarker[i].arrDirection);
       if(nResultId > 0 && nResultId < 1023)
       {
      //      printf("nResultId=%d\n", nResultId);
            nRightMarker = i;
            break;
       }
       //else
            //printf("nResultId=%d\n", -1);
    }
    
    //printf("nRightMarker:%d\n",nRightMarker);
#ifdef DEBUG
	if(nResultId > 0 && nResultId < 1023 ){	
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
#endif
		
	clock_t nEndTime = clock();
	//printf("Time:%f\n",(double)(nEndTime-nStartTime)/CLOCKS_PER_SEC);

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
	
	//printf("X:%6.2f\n",x);
	//printf("Y:%6.2f\n",y);
	
#ifdef DEBUG	
	if(nResultId > 0 && nResultId < 1023)
	{
		/*画坐标系*/	
        cvLine(pImageRect,cvPoint(x,0),cvPoint(x,pImageRect->height),CV_RGB(255,0,255),2,CV_AA,0);
        cvLine(pImageRect,cvPoint(0,y),cvPoint(pImage->width,y),CV_RGB(255,0,255),2,CV_AA,0);
        cvLine(pImageRect,cvPoint(pImageRect->width/2,0),cvPoint(x,y),CV_RGB(255,255,0),2,CV_AA,0);
		
	
		sprintf(arrXCorToData,"%s:%6.2f","X",x);
		sprintf(arrYCorToData,"%s:%6.2f","Y",y);
		sprintf(arrIntToData,"%s:%d","DecodeData",nResultId);
        sprintf(arrFlagToData,"%s","Find Marker Success!");
		cvPutText(pImageRect,arrIntToData,cvPoint(0,30),&nFont,CV_RGB(0,0,0));
		cvPutText(pImageRect,arrXCorToData,cvPoint(0,60),&nFont,CV_RGB(0,0,0));
		cvPutText(pImageRect,arrYCorToData,cvPoint(0,90),&nFont,CV_RGB(0,0,0));
	}
#endif

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
#ifdef RELEASE
    cvReleaseImage(&pSrcImage);
#endif

#ifdef DEBUG
	char c = cvWaitKey(10);
    if(c == 27)
        break;
#endif
}//end of while

	return 1;

}


