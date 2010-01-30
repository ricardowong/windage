/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek
 *   Woontack Woo
 *   U-VR Lab, GIST of Gwangju in Korea.
 *   http://windage.googlecode.com/
 *   http://uvr.gist.ac.kr/
 *
 * Copyright of the derived and new portions of this work
 *     (C) 2009 GIST U-VR Lab.
 *
 * This framework is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This framework is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this framework; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * For further information please contact 
 *   Woonhyuk Baek
 *   <windage@live.com>
 *   GIST U-VR Lab.
 *   Department of Information and Communication
 *   Gwangju Institute of Science and Technology
 *   1, Oryong-dong, Buk-gu, Gwangju
 *   South Korea
 * ========================================================================
 ** @author   Woonhyuk Baek
 * ======================================================================== */

#include <iostream>

#include <highgui.h>
#include <windage.h>

#include "PoseEstimation.h"

#define FLIP
#define RECTIFICATION
#define ADAPTIVE_THRESHOLD

//#define USE_IMAGE_SEQUENCE

const int FIND_FEATURE_COUNT = 10;

const int MAX_FAST_THRESHOLD = 80;
const int MIN_FAST_THRESHOLD = 20;
const int ADAPTIVE_THRESHOLD_VALUE = 200;
const int THRESHOLD_STEP = 1;

const int WIDTH = 640;
const int HEIGHT = 480;
const double intrinsicValues[8] = {1029.400, 1028.675, 316.524, 211.395, -0.206360, 0.238378, 0.001089, -0.000769};

windage::Logger* trace;

windage::ModifiedSURFTracker* CreateTracker(IplImage* refImage, int index)
{
	windage::ModifiedSURFTracker* tracker = new windage::ModifiedSURFTracker();
	tracker->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7], 30);
	tracker->RegistReferenceImage(refImage, WIDTH, HEIGHT, 4.0, 8);
	tracker->SetPoseEstimationMethod(windage::PROSAC);
	tracker->SetOutlinerRemove(false);
	tracker->InitializeOpticalFlow(WIDTH, HEIGHT, 10, cvSize(8, 8), 3);
	tracker->SetOpticalFlowRunning(true);
	tracker->SetRefinement(false);
//	tracker->GetCameraParameter()->InitUndistortionMap(WIDTH, HEIGHT);
	tracker->SetFeatureExtractThreshold(30);

	tracker->SetSetpIndex(index);
	
	return tracker;
}

void CalcHomography(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* homography)
{
	int count = refPoints->size();

	CvPoint2D64f cM={0,0}, cm={0,0}, sM={0,0}, sm={0,0};
    for(int i=0; i<count; i++)
    {
		double x = (*scePoints)[i].x;
		double y = (*scePoints)[i].y;
        double X = (*refPoints)[i].x;
		double Y = (*refPoints)[i].y;

        cm.x += x;
		cm.y += y;
        cM.x += X;
		cM.y += Y;
    }
    cm.x /= count;
	cm.y /= count;
    cM.x /= count;
	cM.y /= count;

    for(int i=0; i<count; i++)
    {
		double x = (*scePoints)[i].x;
		double y = (*scePoints)[i].y;
        double X = (*refPoints)[i].x;
		double Y = (*refPoints)[i].y;

        sm.x += fabs(x - cm.x);
        sm.y += fabs(y - cm.y);
        sM.x += fabs(X - cM.x);
        sM.y += fabs(Y - cM.y);
    }
    sm.x = count/sm.x;
	sm.y = count/sm.y;
    sM.x = count/sM.x;
	sM.y = count/sM.y;

	double LtL[9][9];
	CvMat _LtL = cvMat( 9, 9, CV_64F, LtL );
	cvZero(&_LtL);
	for(int i=0; i<count; i++)
	{
		double x = ((*scePoints)[i].x - cm.x)*sm.x;
		double y = ((*scePoints)[i].y - cm.y)*sm.y;
        double X = ((*refPoints)[i].x - cM.x)*sM.x;
		double Y = ((*refPoints)[i].y - cM.y)*sM.y;

		double Lx[] = { X, Y, 1, 0, 0, 0, -x*X, -x*Y, -x };
        double Ly[] = { 0, 0, 0, X, Y, 1, -y*X, -y*Y, -y };

		for(int j = 0; j < 9; j++ )
            for(int k = j; k < 9; k++ )
                LtL[j][k] += Lx[j]*Lx[k] + Ly[j]*Ly[k];
	}
	cvCompleteSymm( &_LtL );

	double W[9][9], V[9][9];
    CvMat _W = cvMat( 9, 9, CV_64F, W );
    CvMat _V = cvMat( 9, 9, CV_64F, V );

	// borrowed algorithm from opencv
	cvEigenVV( &_LtL, &_V, &_W );

	double h[9];
	CvMat _H = cvMat(3, 3, CV_64FC1, h);
    CvMat _H0 = cvMat( 3, 3, CV_64F, V[8] );
    CvMat _Htemp = cvMat( 3, 3, CV_64F, V[7] );

	double invHnorm[9] = { 1./sm.x, 0, cm.x, 0, 1./sm.y, cm.y, 0, 0, 1 };
    double Hnorm2[9] = { sM.x, 0, -cM.x*sM.x, 0, sM.y, -cM.y*sM.y, 0, 0, 1 };
    CvMat _invHnorm = cvMat( 3, 3, CV_64FC1, invHnorm );
    CvMat _Hnorm2 = cvMat( 3, 3, CV_64FC1, Hnorm2 );
	
    cvMatMul( &_invHnorm, &_H0, &_Htemp );
    cvMatMul( &_Htemp, &_Hnorm2, &_H0 );
    cvConvertScale( &_H0, &_H, 1./_H0.data.db[8] );

	// __H is final result
	CvMat __H = cvMat(3, 3, CV_64FC1, homography);
	cvConvert(&_H, &__H);
}

double CalcHomographyError(CvPoint2D32f refPoints, CvPoint2D32f scePoints, double* h)
{
	double _errNorm = 0;

	double Mx = refPoints.x;
	double My = refPoints.y;
	double mx = scePoints.x;
	double my = scePoints.y;

	double ww = 1./(h[6]*Mx + h[7]*My + 1.);
	double _xi = (h[0]*Mx + h[1]*My + h[2])*ww;
	double _yi = (h[3]*Mx + h[4]*My + h[5])*ww;
	double err[] = { _xi - mx, _yi - my };

	_errNorm += err[0]*err[0] + err[1]*err[1];
	return _errNorm;
}

double CalcHomographyError(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* h)
{
	const int count = refPoints->size();
	const CvPoint2D32f* M = (const CvPoint2D32f*)&(*refPoints)[0];
    const CvPoint2D32f* m = (const CvPoint2D32f*)&(*scePoints)[0];

	double _errNorm = 0;
	for(int i=0; i<count; i++)
	{
		double Mx = M[i].x;
		double My = M[i].y;

		double ww = 1./(h[6]*Mx + h[7]*My + 1.);
		double _xi = (h[0]*Mx + h[1]*My + h[2])*ww;
		double _yi = (h[3]*Mx + h[4]*My + h[5])*ww;
		double err[] = { _xi - m[i].x, _yi - m[i].y };

		_errNorm += err[0]*err[0] + err[1]*err[1];
	}
	return _errNorm;
}

#if 1
bool RefineHomography(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* homography, int maxIters )
{
    CvLevMarq solver(8, 0, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, maxIters, DBL_EPSILON));

    int i, j, k, count = refPoints->size();
    const CvPoint2D32f* M = (const CvPoint2D32f*)&(*refPoints)[0];
    const CvPoint2D32f* m = (const CvPoint2D32f*)&(*scePoints)[0];

    CvMat modelPart = cvMat( solver.param->rows, solver.param->cols, CV_64FC1, homography );
    cvCopy( &modelPart, solver.param );

    for(;;)
    {
        const CvMat* _param = 0;
        CvMat *_JtJ = 0, *_JtErr = 0;
        double* _errNorm = 0;

        if( !solver.updateAlt( _param, _JtJ, _JtErr, _errNorm ))
            break;

        for( i = 0; i < count; i++ )
        {
            const double* h = _param->data.db;
            double Mx = M[i].x, My = M[i].y;
            double ww = 1./(h[6]*Mx + h[7]*My + 1.);
            double _xi = (h[0]*Mx + h[1]*My + h[2])*ww;
            double _yi = (h[3]*Mx + h[4]*My + h[5])*ww;
            double err[] = { _xi - m[i].x, _yi - m[i].y };
            if( _JtJ || _JtErr )
            {
                double J[][8] =
                {
                    { Mx*ww, My*ww, ww, 0, 0, 0, -Mx*ww*_xi, -My*ww*_xi },
                    { 0, 0, 0, Mx*ww, My*ww, ww, -Mx*ww*_yi, -My*ww*_yi }
                };

                for( j = 0; j < 8; j++ )
                {
                    for( k = j; k < 8; k++ )
                        _JtJ->data.db[j*8+k] += J[0][j]*J[0][k] + J[1][j]*J[1][k];
                    _JtErr->data.db[j] += J[0][j]*err[0] + J[1][j]*err[1];
                }
            }
            if( _errNorm )
                *_errNorm += err[0]*err[0] + err[1]*err[1];
        }
    }

    cvCopy( solver.param, &modelPart );
    return true;
}
#else
bool RefineHomography(std::vector<CvPoint2D32f>* refPoints, std::vector<CvPoint2D32f>* scePoints, double* homography, int maxIters )
{
    HomographyLMmethod lmMethod;
	lmMethod.SetMaxIteration(maxIters);
	lmMethod.CalcHomography(refPoints, scePoints, homography);

    return true;
}
#endif

void CalcHomography(std::vector<SURFDesciription>* refPoints, std::vector<SURFDesciription>* scePoints, double* homography)
{
	int count = refPoints->size();
	std::vector<bool> inlier;
	std::vector<bool> maxInlier;
	inlier.resize(count);
	maxInlier.resize(count);
	int maxInlierCount = 0;

	if(count > 4)
	{
		std::vector<CvPoint2D32f> refSamplePoints;
		std::vector<CvPoint2D32f> sceSamplePoints;

		// local optimization
		refSamplePoints.clear();
		sceSamplePoints.clear();
		for(int i=0; i<count; i++)
		{
			refSamplePoints.push_back((*refPoints)[i].point);
			sceSamplePoints.push_back((*scePoints)[i].point);
		}
		CalcHomography(&refSamplePoints, &sceSamplePoints, homography);
		double localError = CalcHomographyError(&refSamplePoints, &sceSamplePoints, homography);
		localError /= (double)count;


		// stochastic method (global optimization)
		// Exploration
		int maxIteration = log(1-0.8) / log(1-pow(0.8, 4)) + 10;
		for(int j=0; j<maxIteration; j++)
		{
			refSamplePoints.clear();
			sceSamplePoints.clear();

			srand((unsigned int)cvGetTickCount());
			for(int i=0; i<4; i++)
			{
				int index = rand() % count;
				refSamplePoints.push_back((*refPoints)[index].point);
				sceSamplePoints.push_back((*scePoints)[index].point);
			}
			CalcHomography(&refSamplePoints, &sceSamplePoints, homography);
			
			int inlierCount = 0;
			for(int i=0; i<count; i++)
			{
				double error = CalcHomographyError((*refPoints)[i].point, (*scePoints)[i].point, homography);
				if(error < 5*5)
				{
					inlier[i] = true;
					inlierCount++;
				}
				else
				{
					inlier[i] = false;
				}
			}

			if(inlierCount > maxInlierCount)
			{
				maxInlierCount = inlierCount;
				for(int i=0; i<count; i++)
				{
					maxInlier[i] = inlier[i];
				}
			}
		}

		// Refinement
		refSamplePoints.clear();
		sceSamplePoints.clear();
		for(int i=0; i<count; i++)
		{
			if(maxInlier[i])
			{
				refSamplePoints.push_back((*refPoints)[i].point);
				sceSamplePoints.push_back((*scePoints)[i].point);
			}
		}

		CalcHomography(&refSamplePoints, &sceSamplePoints, homography);
		double globalError1 = CalcHomographyError(&refSamplePoints, &sceSamplePoints, homography);
		globalError1 /= (double)refSamplePoints.size();

		RefineHomography(&refSamplePoints, &sceSamplePoints, homography, 20);
		double globalError2 = CalcHomographyError(&refSamplePoints, &sceSamplePoints, homography);
		globalError2 /= (double)refSamplePoints.size();

		refSamplePoints.clear();
		sceSamplePoints.clear();
		for(int i=0; i<count; i++)
		{
			refSamplePoints.push_back((*refPoints)[i].point);
			sceSamplePoints.push_back((*scePoints)[i].point);
		}
		double globalError3 = CalcHomographyError(&refSamplePoints, &sceSamplePoints, homography);
		globalError3 /= (double)count;

		std::cout << localError << " : " << globalError2 << " (" << globalError3 << ") " << std::endl;

		// log
		trace->log("localError", localError);
		trace->log("globalError", globalError3);
		trace->log("inlier", globalError1);
		trace->log("refinement", globalError2);
		trace->logNewLine();
	}

}


void DrawBoundary(IplImage* image, double* homography)
{
	double x, y;
	double X, Y, Z;

	x = -WIDTH/2; y = -HEIGHT/2;
	Z = homography[6]*x + homography[7]*y + homography[8];
	X = (homography[0]*x + homography[1]*y + homography[2]) / Z;
	Y = (homography[3]*x + homography[4]*y + homography[5]) / Z;
	CvPoint point1 = cvPoint(X, Y);

	x = +WIDTH/2; y = -HEIGHT/2;
	Z = homography[6]*x + homography[7]*y + homography[8];
	X = (homography[0]*x + homography[1]*y + homography[2]) / Z;
	Y = (homography[3]*x + homography[4]*y + homography[5]) / Z;
	CvPoint point2 = cvPoint(X, Y);

	x = +WIDTH/2; y = +HEIGHT/2;
	Z = homography[6]*x + homography[7]*y + homography[8];
	X = (homography[0]*x + homography[1]*y + homography[2]) / Z;
	Y = (homography[3]*x + homography[4]*y + homography[5]) / Z;
	CvPoint point3 = cvPoint(X, Y);

	x = -WIDTH/2; y = +HEIGHT/2;
	Z = homography[6]*x + homography[7]*y + homography[8];
	X = (homography[0]*x + homography[1]*y + homography[2]) / Z;
	Y = (homography[3]*x + homography[4]*y + homography[5]) / Z;
	CvPoint point4 = cvPoint(X, Y);

	cvLine(image, point1, point2, CV_RGB(255, 255, 255), 5);
	cvLine(image, point2, point3, CV_RGB(255, 255, 255), 5);
	cvLine(image, point3, point4, CV_RGB(255, 255, 255), 5);
	cvLine(image, point4, point1, CV_RGB(255, 255, 255), 5);

	cvLine(image, point1, point3, CV_RGB(255, 255, 255), 5);
	cvLine(image, point2, point4, CV_RGB(255, 255, 255), 5);

	cvLine(image, point1, point2, CV_RGB(255, 255, 0), 3);
	cvLine(image, point2, point3, CV_RGB(255, 255, 0), 3);
	cvLine(image, point3, point4, CV_RGB(255, 255, 0), 3);
	cvLine(image, point4, point1, CV_RGB(255, 255, 0), 3);

	cvLine(image, point1, point3, CV_RGB(255, 255, 0), 3);
	cvLine(image, point2, point4, CV_RGB(255, 255, 0), 3);
}

void main()
{
	trace = new windage::Logger("Log/optimizationTrace.txt", true);
	windage::Logger* log = new windage::Logger(&std::cout);

	// connect camera
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	char message[100];
	IplImage* inputImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* inputImage2 = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* tempImage = cvCreateImage(cvSize(WIDTH, HEIGHT), IPL_DEPTH_8U, 3);
	IplImage* grayImage = cvCreateImage(cvGetSize(inputImage), IPL_DEPTH_8U, 1);
	
	// Tracker Initialize
	IplImage* trainingImage = cvLoadImage("reference1_320.png", 0);
	IplImage* referenceImage = cvLoadImage("reference1.png");
	IplImage* resultImage = cvCreateImage(cvSize(WIDTH, HEIGHT*2), IPL_DEPTH_8U, 3);
	windage::ModifiedSURFTracker* tracker = CreateTracker(trainingImage, 0);

	// for undistortion
	windage::Calibration* calibration = new windage::Calibration();
	calibration->Initialize(intrinsicValues[0], intrinsicValues[1], intrinsicValues[2], intrinsicValues[3], intrinsicValues[4], intrinsicValues[5], intrinsicValues[6], intrinsicValues[7]);
	calibration->InitUndistortionMap(WIDTH, HEIGHT);

	// adaptive threshold
	int fastThreshold = 30;
	IplImage* grabFrame = NULL;

	bool saving = false;
	bool processing = true;
	cvNamedWindow("result");
	cvNamedWindow("result2");
	cvNamedWindow("matching");
	while(processing)
	{
		// camera frame grabbing and convert to gray and undistortion
		log->updateTickCount();
		grabFrame = cvQueryFrame(capture);
		cvFlip(grabFrame, grabFrame);
		cvResize(grabFrame, tempImage);
		calibration->Undistortion(tempImage, inputImage);
		cvCvtColor(inputImage, grayImage, CV_BGRA2GRAY);
		log->log("capture", log->calculateProcessTime());

		// call tracking algorithm
		log->updateTickCount();

		tracker->SetFeatureExtractThreshold(fastThreshold);
		tracker->UpdateCameraPose(grayImage);
		int featureCount = tracker->GetFeatureCount();
		int matchingCount = tracker->GetMatchedCount();

		double trackingTime = log->calculateProcessTime();
		log->log("tracking", trackingTime);
		log->logNewLine();
			
		// update fast threshold for Adaptive threshold
#ifdef ADAPTIVE_THRESHOLD
		if(featureCount > ADAPTIVE_THRESHOLD_VALUE )	fastThreshold = MIN(MAX_FAST_THRESHOLD, fastThreshold+THRESHOLD_STEP);
		else											fastThreshold = MAX(MIN_FAST_THRESHOLD, fastThreshold-THRESHOLD_STEP);
#endif

		cvZero(resultImage);
		cvSetImageROI(resultImage, cvRect(0, 0, WIDTH, HEIGHT));
		cvCopy(referenceImage, resultImage);
		cvSetImageROI(resultImage, cvRect(0, HEIGHT, WIDTH, HEIGHT));
		cvCopy(inputImage, resultImage);
		cvResetImageROI(resultImage);
		tracker->DrawDebugInfo2(resultImage);
		
		// draw tracking result
		if(featureCount > FIND_FEATURE_COUNT)
		{
			cvCopyImage(inputImage, inputImage2);

			std::vector<SURFDesciription>* refPoints = tracker->GetMatchedReferencePoints();
			std::vector<SURFDesciription>* scePoints = tracker->GetMatchedScenePoints();

			// stochastic
			double homography[9];
			memset(homography, 0, sizeof(double)*9);
			CalcHomography(refPoints, scePoints, homography);

			DrawBoundary(inputImage, homography);


			std::vector<CvPoint2D32f> refSamplePoints;
			std::vector<CvPoint2D32f> sceSamplePoints;

			// local optimization
			refSamplePoints.clear();
			sceSamplePoints.clear();
			for(int i=0; i<refPoints->size(); i++)
			{
				refSamplePoints.push_back((*refPoints)[i].point);
				sceSamplePoints.push_back((*scePoints)[i].point);
			}
			CalcHomography(&refSamplePoints, &sceSamplePoints, homography);
			
			DrawBoundary(inputImage2, homography);

//			tracker->DrawOutLine(inputImage, true);
			tracker->DrawInfomation(inputImage, 100.0);
//			tracker->DrawDebugInfo(inputImage);
		}
/*
		sprintf(message, "Tracking Time : %.2f(ms)", trackingTime);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 30), message);
		sprintf(message, "FAST feature count : %d, threashold : %d", featureCount, fastThreshold);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 50), message);
		sprintf(message, "Match count : %d", matchingCount);
		windage::Utils::DrawTextToImage(inputImage, cvPoint(20, 70), message);
*/
		char ch = cvWaitKey(1);
		switch(ch)
		{
		case 's':
		case 'S':
			saving = true;
			break;
		case 'q':
		case 'Q':
			processing = false;
			break;
		}

		cvShowImage("result", inputImage);
		cvShowImage("result2", inputImage2);
		cvShowImage("matching", resultImage);

		if(saving)
		{
			cvSaveImage("matching.jpg", resultImage);
			cvSaveImage("result1.jpg", inputImage);
			cvSaveImage("result2.jpg", inputImage2);
			saving = false;
		}
	}

	cvReleaseCapture(&capture);
	cvDestroyAllWindows();
}
