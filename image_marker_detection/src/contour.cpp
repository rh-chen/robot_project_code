#include "contour.h"
#include "string"
#include "vector"
	
//#define TRA

void iCvFindContour(IplImage* src,CvPoint2D32f* rec,int nAreaThresholdMin,int nAreaThresholdMax)
{
#ifdef TRA
	CvMemStorage* pStorage = cvCreateMemStorage(0);
	CvSeq* pContour = NULL;
	CvMemStorage* pStorageTemp = cvCreateMemStorage(0);
#endif
		
#ifdef TRA	
	int num = cvFindContours(src,pStorage,&pContour,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
#endif

#ifndef TRA
	
	//Mat iMat(src,0);
	Mat iMat = cvarrToMat(src,true);
	
	vector< vector<Point> > all_contours;
	vector< vector<Point> > contours;
	
	findContours(iMat,all_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//CV_RETR_TREE
	
	unsigned int i,j;
	unsigned int nPosContour = 0;
	unsigned int nRightContour = 0;

	//printf("总轮廓的数目:%d\n",(int)all_contours.size());

	for(i = 0;i < all_contours.size();i++)
		{
			/*nAreaThresholdMin,nAreaThresholdMax*/
			if(all_contours[i].size() >256  && all_contours[i].size() < 512)
				{
					nPosContour++;
					contours.push_back(all_contours[i]);
					//printf("轮廓点的数目:%d\n",(int)all_contours[i].size());
				}

		}
	
	
	//printf("可能轮廓的数目:%d\n",(int)contours.size());
	
	vector<Point> approx_poly;
	for(i = 0;i < contours.size();i++)
		{
			double eps = contours[i].size()*0.08;//3
            //double eps = 3;
			approxPolyDP(contours[i],approx_poly,eps,true);
			if(approx_poly.size() != 4)
				continue;
			if(!isContourConvex(approx_poly))
				continue;
			float minSide = 65536;
			for(j = 0;j < 4;j++)
				{
					Point sidePoint = approx_poly[j]-approx_poly[(j+1)%4];
					minSide = minSide < sidePoint.dot(sidePoint)?minSide:sidePoint.dot(sidePoint);
				}
			if(cvRound(minSide) < 64)
				continue;
			Marker marker;
			marker.m_corners[0] = approx_poly[0];
			marker.m_corners[1] = approx_poly[1];
			marker.m_corners[2] = approx_poly[2];
			marker.m_corners[3] = approx_poly[3];
			
			Point2f vl = marker.m_corners[1]-marker.m_corners[0];
			Point2f vh = marker.m_corners[2]-marker.m_corners[0];
			
			if(vl.cross(vh) > 0)//图像y轴向下，所以大于零表示逆时针
				swap(marker.m_corners[1],marker.m_corners[3]);
				
			rec[0].x = (float)(marker.m_corners[0].x);
			rec[0].y = (float)(marker.m_corners[0].y);
			rec[1].x = (float)(marker.m_corners[1].x);
			rec[1].y = (float)(marker.m_corners[1].y);
			rec[2].x = (float)(marker.m_corners[2].x);
			rec[2].y = (float)(marker.m_corners[2].y);
			rec[3].x = (float)(marker.m_corners[3].x);
			rec[3].y = (float)(marker.m_corners[3].y);
			
			nRightContour++;
						
			
		}
	
#endif
	
	
	//printf("Marker轮廓的数目:%d\n",nRightContour);
#ifdef TRA	
	for(;pContour != NULL;pContour = pContour->h_next)
		{	
			CvSeq* pContourTemp = cvApproxPoly(pContour,sizeof(CvContour),pStorageTemp,CV_POLY_APPROX_DP,cvContourPerimeter(pContour)*0.02,0);
			CvBox2D rectTemp = cvMinAreaRect2(pContourTemp);
			if(rectTemp.size.width*rectTemp.size.height > nAreaThresholdMin && rectTemp.size.width*rectTemp.size.height < nAreaThresholdMax)
				{
					cvBoxPoints(rectTemp,rec);
				}

		}

	cvReleaseMemStorage(&pStorage);
	cvReleaseMemStorage(&pStorageTemp);
#endif
#ifndef TRA
    for(i = 0;i < nPosContour;i++)
        vector<Point>().swap(contours[i]);
    for(i = 0;i < all_contours.size();i++)
        vector<Point>().swap(all_contours[i]);
    vector<Point>().swap(approx_poly);
    vector< vector<Point> >().swap(all_contours);
    vector< vector<Point> >().swap(contours);
#endif


}
void iCvFindContourS(IplImage* src,CvPoint2D32f* rec,int nAreaThresholdMin,int nAreaThresholdMax)
{
#ifdef TRA
    CvMemStorage* pStorage = cvCreateMemStorage(0);
    CvSeq* pContour = NULL;
    CvMemStorage* pStorageTemp = cvCreateMemStorage(0);
#endif

#ifdef TRA
    int num = cvFindContours(src,pStorage,&pContour,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
#endif

#ifndef TRA

    //Mat iMat(src,0);
	Mat iMat = cvarrToMat(src,true);

    vector< vector<Point> > all_contours;
    vector< vector<Point> > contours;

    findContours(iMat,all_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    unsigned int i,j;
    unsigned int nPosContour = 0;

    for(i = 0;i < all_contours.size();i++)
        {
            /*nAreaThresholdMin,nAreaThresholdMax*/
            if((all_contours[i].size()/4)*(all_contours[i].size()/4) > 128 && (all_contours[i].size()/4)*(all_contours[i].size()/4) < 65536)
                {
		    nPosContour++;
                    contours.push_back(all_contours[i]);
                    //printf("轮廓点的数目:%d\n",(int)all_contours[i].size());
                }

        }


    //printf("轮廓的数目:%d\n",(int)contours.size());

    vector<Point> approx_poly;
    for(i = 0;i < contours.size();i++)
        {
            double eps = contours[i].size()*0.08;
	    //printf("eps=%f\n",eps);
            approxPolyDP(contours[i],approx_poly,eps,true);
            if(approx_poly.size() != 4)
                continue;
            if(!isContourConvex(approx_poly))
                continue;
	
	    float minSide = 65536;
            for(j = 0;j < 4;j++)
                {
                  	Point sidePoint = approx_poly[j]-approx_poly[(j+1)%4];
                        minSide = minSide < sidePoint.dot(sidePoint)?minSide:sidePoint.dot(sidePoint);
                }
            if(cvRound(minSide) < 64)
		continue;

            Marker marker;
            marker.m_corners[0] = approx_poly[0];
            marker.m_corners[1] = approx_poly[1];
            marker.m_corners[2] = approx_poly[2];
            marker.m_corners[3] = approx_poly[3];

            Point2f vl = marker.m_corners[1]-marker.m_corners[0];
            Point2f vh = marker.m_corners[2]-marker.m_corners[0];

            if(vl.cross(vh) > 0)//图像y轴向下，所以大于零表示逆时针
                swap(marker.m_corners[1],marker.m_corners[3]);

            rec[0].x = (float)(marker.m_corners[0].x);
            rec[0].y = (float)(marker.m_corners[0].y);
            rec[1].x = (float)(marker.m_corners[1].x);
            rec[1].y = (float)(marker.m_corners[1].y);
            rec[2].x = (float)(marker.m_corners[2].x);
            rec[2].y = (float)(marker.m_corners[2].y);
            rec[3].x = (float)(marker.m_corners[3].x);
            rec[3].y = (float)(marker.m_corners[3].y);


        }

#endif
#ifdef TRA
    for(;pContour != NULL;pContour = pContour->h_next)
        {
            CvSeq* pContourTemp = cvApproxPoly(pContour,sizeof(CvContour),pStorageTemp,CV_POLY_APPROX_DP,cvContourPerimeter(pContour)*0.02,0);
            CvBox2D rectTemp = cvMinAreaRect2(pContourTemp);
            if(rectTemp.size.width*rectTemp.size.height > nAreaThresholdMin && rectTemp.size.width*rectTemp.size.height < nAreaThresholdMax)
                {
                    cvBoxPoints(rectTemp,rec);
                }

        }

    cvReleaseMemStorage(&pStorage);
    cvReleaseMemStorage(&pStorageTemp);
#endif
#ifndef TRA
    //printf("nPosCount=%d, \n",nPosContour);
    for(i = 0;i < nPosContour;i++)
        vector<Point>().swap(contours[i]);
    for(i = 0;i < all_contours.size();i++)
        vector<Point>().swap(all_contours[i]);
    vector<Point>().swap(approx_poly);
    vector< vector<Point> >().swap(all_contours);
    vector< vector<Point> >().swap(contours);
#endif

}

