#include "decodemarker.h"
#define SORT
#define SHOW_IMAGE
int iDecodeMarker(IplImage* src,IplImage* mask)
{	
	
	int x,y;
	int i,j;
		
	CvPoint2D32f* pRect = new CvPoint2D32f[4];
    memset(pRect,0,4*sizeof(CvPoint2D32f));
    CvPoint2D32f* pRectPoint = new CvPoint2D32f[4];
    memset(pRectPoint,0,4*sizeof(CvPoint2D32f));
	
	int nAreaThresholdMin = MARKER_SIZE*MARKER_SIZE/16;
    int nAreaThresholdMax = MARKER_SIZE*MARKER_SIZE;
	
    iCvFindContourS(src,pRect,nAreaThresholdMin,nAreaThresholdMax);

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
	
    	IplImage* pMarker = iFindMarkerImage(mask,pRectPoint,64);
	
	IplImage* pResult = iProcessUnitMarkerS(pMarker);
#ifdef SHOW_IMAGE	
	//cvShowImage("imageMarker",pResult);
#endif
    //Mat markerImage(src,0);
	//Mat markerImageSmall(pResult,0);

	Mat markerImage = cvarrToMat(src,true);
	Mat markerImageSmall = cvarrToMat(pResult,true);

	Mat bitMatrix(5,5,CV_8UC1);
		
	/*black border*/
	/*for(y = 0;y < 7;y++)
		{
			int inc = (y == 0 || y == 6)?1:6;
			int cell_y = y*MARKER_CELL_SIZE;
			
			for(x = 0;x < 7;x += inc)
				{
					int cell_x = x*MARKER_CELL_SIZE;
					int none_zero_count = countNonZero(markerImage(Rect(cell_x,cell_y,MARKER_CELL_SIZE,MARKER_CELL_SIZE)));
					if(none_zero_count > MARKER_CELL_SIZE*MARKER_CELL_SIZE/4)
						{
							delete[] pRect;
							delete[] pRectPoint;
							cvReleaseImage(&pMarker);
							cvReleaseImage(&pResult);
							
							printf("%s\n","black border fail!");
							return -1;
						}
				}

		}*/
	/*decode marker*/
	for(y = 0;y < 5;y++)
		{
			int cell_y = (y+1)*MARKER_CELL_SIZE_SMALL;
			for(x = 0;x < 5;x++)
				{
					int cell_x = (x+1)*MARKER_CELL_SIZE_SMALL;
					int none_zero_count = countNonZero(markerImageSmall(Rect(cell_x,cell_y,MARKER_CELL_SIZE_SMALL,MARKER_CELL_SIZE_SMALL)));
					//printf("none_zero_count:%d\n",none_zero_count);

					if(none_zero_count > MARKER_CELL_SIZE_SMALL*MARKER_CELL_SIZE_SMALL/3)
						bitMatrix.at<uchar>(y,x) = 1;
					else
						bitMatrix.at<uchar>(y,x) = 0;

				}
		}
	
	/*find marker orientation*/
	bool rightMarker = false;
	int rotationIdx;
	
    //printf("%s\n","/********************************marker bit*******************/");
	for(y = 0;y < 5;y++)
		for(x = 0;x < 5;x++)
		{	
			//printf("%d",bitMatrix.at<uchar>(y,x));
			//printf(" ");
			//if(x == 4)
				//printf("\n");
		}
	//printf("%s\n","/***************************************************/");

    //printf("%s\n","/***************************right bit************************/");
    for(y = 0;y < 5;y++)
        for(x = 0;x < 5;x++)
        {
            //printf("%d",bitMatrix.at<uchar>(y,x));
            //printf(" ");
      //      if(x == 4)
                //printf("\n");
        }
    //printf("%s\n","/***************************************************/");

    int arrRightDirection[5][5] = {{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0},{0,0,0,0,0}};
    
	
	for(rotationIdx = 0;rotationIdx < 4;rotationIdx++)
		{
			
			if(iHammingDistanceAnother(arrRightDirection,bitMatrix) == 0)
				{
					rightMarker = true;
					break;
				}
			bitMatrix = iBitMatrixRotate(bitMatrix);
		
		}
	if(!rightMarker)
		{
			delete[] pRect;
                        delete[] pRectPoint;
                        cvReleaseImage(&pMarker);
                        cvReleaseImage(&pResult);
			//printf("%s\n","hamming distance fail!");
			return -1;
		}
			
	//printf("%s\n","/***************************************************/");
       /* for(y = 0;y < 5;y++)
                for(x = 0;x < 5;x++)
                {
                        printf("%d",bitMatrix.at<uchar>(y,x));
                        printf(" ");
                        if(x == 4)
                                printf("\n");
                }*/
//        printf("%s\n","/***************************************************/");
	
	/*marker id*/
	int nId = iBitMatrixToId(bitMatrix);
	
	cvReleaseImage(&pMarker);
	cvReleaseImage(&pResult);
	delete[] pRect;
	delete[] pRectPoint;
	
	if(iHammingDistance(bitMatrix) == 0)
		return nId;
	else 
		return -1;
}
int iDecodeMarkerAnother(IplImage* src,IplImage* mask,int arrRightDirection[5][5])
{

    int x,y;
    int i,j;

    CvPoint2D32f* pRect = new CvPoint2D32f[4];
    memset(pRect,0,4*sizeof(CvPoint2D32f));
    CvPoint2D32f* pRectPoint = new CvPoint2D32f[4];
    memset(pRectPoint,0,4*sizeof(CvPoint2D32f));

    int nAreaThresholdMin = MARKER_SIZE*MARKER_SIZE/16;
    int nAreaThresholdMax = MARKER_SIZE*MARKER_SIZE;

    iCvFindContourS(src,pRect,nAreaThresholdMin,nAreaThresholdMax);

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

    IplImage* pMarker = iFindMarkerImage(src,pRectPoint,64);

    //IplImage* pResult = iProcessUnitMarkerS(pMarker);
    //Mat markerImage(src,0);
    //Mat markerImageSmall(pMarker,0);
	Mat markerImageSmall = cvarrToMat(pMarker,true);

    Mat bitMatrix(5,5,CV_8UC1);

    /*black border*/
    /*for(y = 0;y < 7;y++)
        {
            int inc = (y == 0 || y == 6)?1:6;
            int cell_y = y*MARKER_CELL_SIZE;

            for(x = 0;x < 7;x += inc)
                {
                    int cell_x = x*MARKER_CELL_SIZE;
                    int none_zero_count = countNonZero(markerImage(Rect(cell_x,cell_y,MARKER_CELL_SIZE,MARKER_CELL_SIZE)));
                    if(none_zero_count > MARKER_CELL_SIZE*MARKER_CELL_SIZE/4)
                        {
                            delete[] pRect;
                            delete[] pRectPoint;
                            cvReleaseImage(&pMarker);
                            cvReleaseImage(&pResult);

                            printf("%s\n","black border fail!");
                            return -1;
                        }
                }

        }*/
    /*decode marker*/
    for(y = 0;y < 5;y++)
        {
            int cell_y = (y+1)*MARKER_CELL_SIZE_SMALL;
            for(x = 0;x < 5;x++)
                {
                    int cell_x = (x+1)*MARKER_CELL_SIZE_SMALL;
                    int none_zero_count = countNonZero(markerImageSmall(Rect(cell_x,cell_y,MARKER_CELL_SIZE_SMALL,MARKER_CELL_SIZE_SMALL)));
                    //printf("none_zero_count:%d\n",none_zero_count);

                    if(none_zero_count > MARKER_CELL_SIZE_SMALL*MARKER_CELL_SIZE_SMALL/3)
                        bitMatrix.at<uchar>(y,x) = 1;
                    else
                        bitMatrix.at<uchar>(y,x) = 0;

                }
        }

    /*find marker orientation*/
    bool rightMarker = false;
    int rotationIdx;

    //printf("%s\n","/************************marker bit***************************/");
   /* for(y = 0;y < 5;y++)
        for(x = 0;x < 5;x++)
        {
            //printf("%d",bitMatrix.at<uchar>(y,x));
            printf(" ");
            if(x == 4)
                printf("\n");
        }
*/
    /*for(y = 0;y < 5;y++)
        for(x = 0;x < 5;x++)
        {
            printf("%d",arrRightDirection[y][x]);
            printf(" ");
            if(x == 4)
                printf("\n");
        }*/
//    printf("%s\n","/***************************************************/");


    for(rotationIdx = 0;rotationIdx < 4;rotationIdx++)
        {

            if(iHammingDistanceAnother(arrRightDirection,bitMatrix) == 0)
                {
                    rightMarker = true;
                    break;
                }
            bitMatrix = iBitMatrixRotate(bitMatrix);

            //printf("rotaionIdx:%d\n",rotationIdx);
/*            for(y = 0;y < 5;y++)
                for(x = 0;x < 5;x++)
                {
                    printf("%d",bitMatrix.at<uchar>(y,x));
                    printf(" ");
                    if(x == 4)
                        printf("\n");
                }
(*/            
        }
  //  printf("rightMarker:%d\n",rightMarker);

    if(!rightMarker)
        {
            delete[] pRect;
            delete[] pRectPoint;
            cvReleaseImage(&pMarker);
            //cvReleaseImage(&pResult);
            //printf("%s\n","hamming distance fail!");
            return -1;
        }
/*
        for(y = 0;y < 5;y++)
                for(x = 0;x < 5;x++)
                {
                        printf("%d",bitMatrix.at<uchar>(y,x));
                        printf(" ");
                        if(x == 4)
                                printf("\n");
                }
*/
    /*marker id*/
    int nId = iBitMatrixToId(bitMatrix);

    cvReleaseImage(&pMarker);
    //cvReleaseImage(&pResult);
    delete[] pRect;
    delete[] pRectPoint;
    if(rightMarker == true)
      return nId;
    else
      return -1;
}

