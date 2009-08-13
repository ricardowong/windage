#include "ModifiedSURFTracker.h"
using namespace windage;

#include "FAST/fast.h"
#include "FAST/wsurf.h"

ModifiedSURFTracker::ModifiedSURFTracker()
{
	cameraParameter = NULL;
	referenceImage = NULL;
	featureExtractThreshold = 50;

	referenceFeatureStorage = NULL;
	referenceFeatureTree = NULL;

	opticalflow = NULL;
	runOpticalflow = false;
	step = 0;
}

ModifiedSURFTracker::~ModifiedSURFTracker()
{
	this->Release();
}

void ModifiedSURFTracker::Release()
{
	if(cameraParameter) delete cameraParameter;
	cameraParameter = NULL;
	if(referenceImage) cvReleaseImage(&referenceImage);
	referenceImage = NULL;

	if(referenceFeatureStorage) cvReleaseMat(&referenceFeatureStorage);
	referenceFeatureStorage = NULL;
	if(referenceFeatureTree) cvReleaseFeatureTree(referenceFeatureTree);
	referenceFeatureTree = NULL;

	if(opticalflow) delete opticalflow;
	opticalflow = NULL;
}

void ModifiedSURFTracker::Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4, IplImage* referenceImage, double realWidth, double realHeight, int featureExtractThreshold)
{
	this->Release();
	cameraParameter = new Calibration();
	cameraParameter->Initialize(fx, fy, cx, cy, d1, d2, d3, d4);

	this->referenceImage = cvCloneImage(referenceImage);
//	cvFlip(this->referenceImage, this->referenceImage);

	this->realWidth = realWidth;
	this->realHeight = realHeight;
	this->SetFeatureExtractTreshold(featureExtractThreshold);

	GenerateReferenceFeatureTree();
}

int ModifiedSURFTracker::ExtractFASTCorner(std::vector<CvPoint>* corners, IplImage* grayImage, int threshold, int n)
{
	int cornerCount = 0;
	xy* cornertemp = NULL;

	bool isProcessed = true;
	switch(n)
	{
	case 9:
		cornertemp = fast_corner_detect_9((const byte *)grayImage->imageData, grayImage->width, grayImage->height, threshold, &cornerCount);
		break;
	case 10:
		cornertemp = fast_corner_detect_10((const byte *)grayImage->imageData, grayImage->width, grayImage->height, threshold, &cornerCount);
		break;
	case 11:
		cornertemp = fast_corner_detect_11((const byte *)grayImage->imageData, grayImage->width, grayImage->height, threshold, &cornerCount);
		break;
	case 12 :
		cornertemp = fast_corner_detect_12((const byte *)grayImage->imageData, grayImage->width, grayImage->height, threshold, &cornerCount);
		break;
	default:
		isProcessed = false;
		break;
	}

	if(isProcessed)
	{
		int count = 0;
		xy* nonmax = NULL;
		nonmax = fast_nonmax((const byte *)grayImage->imageData, grayImage->width, grayImage->height, cornertemp, cornerCount, threshold, &count);

		for(int i=0; i<count; ++i)
		{
			corners->push_back(cvPoint(nonmax[i].x, nonmax[i].y));
		}

		if(nonmax) delete nonmax;
		if(cornertemp) delete cornertemp;
	}

	return (int)corners->size();
}

int ModifiedSURFTracker::ExtractModifiedSURF(IplImage* grayImage, std::vector<SURFDesciription>* descriptions, int thresholdFAST)
{
	CvSURFParams params = cvSURFParams(500, 0);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq *referenceKeypoints = 0, *referenceDescriptors = 0;
//	cvExtractSURF(image, 0, &referenceKeypoints, &referenceDescriptors, storage, params );

	std::vector<CvPoint> fastCorners;

	ExtractFASTCorner(&fastCorners, grayImage, thresholdFAST, 9);
	referenceKeypoints = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSURFPoint), storage );
	for(int i=0; i<fastCorners.size(); i++)
	{
		CvSURFPoint point = cvSURFPoint( cvPoint2D32f(fastCorners[i].x, fastCorners[i].y), 0, 15, 0, 0);
		cvSeqPush(referenceKeypoints, &point);
	}
	wExtractSURF(grayImage, 0, &referenceKeypoints, &referenceDescriptors, storage, params, 1);
//	cvExtractSURF(grayImage, 0, &referenceKeypoints, &referenceDescriptors, storage, params, 1);

	CvSeqReader reader;
	cvStartReadSeq( referenceDescriptors, &reader, 0 );
	int length = (int)(referenceDescriptors->elem_size/sizeof(float));
	int count = referenceKeypoints->total;
	for(int i = 0; i<count; i++ )
	{
		SURFDesciription description;

		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( referenceKeypoints, i );
		description.point = r->pt;
		description.size = r->size;

		float* vec = (float*)reader.ptr;
		for(int i=0; i<length; i++)
		{
			description.descriptor[i] = vec[i];
		}

		descriptions->push_back(description);
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
	}

	cvReleaseMemStorage(&storage);
	return count;
}


CvFeatureTree* ModifiedSURFTracker::CreateReferenceTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage)
{
	int count = (int)referenceSURF->size();
	referenceFeatureStorage = cvCreateMat(count, DESCRIPTOR_DIEMNSION, CV_32FC1);
	for(int y=0; y<count; y++)
	{
		for(int x=0; x<DESCRIPTOR_DIEMNSION; x++)
		{
			cvSetReal2D(referenceFeatureStorage, y, x, (*referenceSURF)[y].descriptor[x]);
		}
	}
	CvFeatureTree* tree = cvCreateFeatureTree(referenceFeatureStorage);
	return tree;
}

int ModifiedSURFTracker::FindPairs(SURFDesciription description, CvFeatureTree* tree)
{
	CvMat* currentFeature = cvCreateMat(1, DESCRIPTOR_DIEMNSION, CV_32FC1);
	CvMat* result = cvCreateMat(1, 2, CV_32SC1);
	CvMat* distance = cvCreateMat(1, 2, CV_64FC1);
	for(int x=0; x<DESCRIPTOR_DIEMNSION; x++)
	{
		cvSetReal2D(currentFeature, 0, x, description.descriptor[x]);
	}

	cvFindFeatures(tree, currentFeature, result, distance, 2, 20);

	double min2 = cvGetReal2D(distance, 0, 0);
	double min1 = cvGetReal2D(distance, 0, 1);
	int index2 = (int)cvGetReal2D(result, 0, 0);
	int index1 = (int)cvGetReal2D(result, 0, 1);
	int index = index1;
//*
	double temp;
	if(min2 < min1)
	{
		temp = min1;
		min1 = min2;
		min2 = temp;
		index = index2;
	}
//*/
	cvReleaseMat(&currentFeature);
	cvReleaseMat(&result);
	cvReleaseMat(&distance);

	if ( min1 < 0.65*min2 )
		return index;
    return -1;
}

int ModifiedSURFTracker::FindPairs(SURFDesciription description, std::vector<SURFDesciription>* descriptions)
{
	double min1 = 1e6;
	double min2 = 1e6;
	int index = -1;

	for(unsigned int i=0; i<descriptions->size(); i++)
	{
		double d = description.distance((*descriptions)[i]);
		if(d < min1)
		{
			min2 = min1;
			min1 = d;
			index = i;
		}
		else if(d < min2)
		{
			min2 = d;
		}
	}
	
	if ( min1 < 0.65*min2 )
        return index;
    return -1;
}


int ModifiedSURFTracker::GenerateReferenceFeatureTree()
{
	int count = 8;
	double scale = 4.0;
	int width = referenceImage->width/scale;
	int height = referenceImage->height/scale;

	IplImage* tempReference;
	for(int y=1; y<=count; y++)
	{
		for(int x=1; x<=count; x++)
		{
			tempReference = cvCreateImage(cvSize(width*x, height*y), IPL_DEPTH_8U, 1);
			cvResize(referenceImage, tempReference);

			std::vector<SURFDesciription> tempSurf;
			tempSurf.clear();

			this->ExtractModifiedSURF(tempReference, &tempSurf, featureExtractThreshold);
			float xScaleFactor = (float)this->realWidth / (float)tempReference->width;
			float yScaleFactor = (float)this->realHeight / (float)tempReference->height;

			for(int i=0; i<tempSurf.size(); i++)
			{
				tempSurf[i].point.x *= xScaleFactor;
				tempSurf[i].point.y *= yScaleFactor;
				tempSurf[i].point.y = this->realHeight - tempSurf[i].point.y;
				referenceSURF.push_back(tempSurf[i]);
			}

			cvReleaseImage(&tempReference);
		}
	}

	this->referenceFeatureTree = CreateReferenceTree(&this->referenceSURF, referenceFeatureStorage);

	return 0;
}


/** borrowed code from bazAR */
void DecomposeHomographyToRT(CvMat *intrinsic, CvMat *Homography, CvMat *RT)
{
	int i, j;

	if(Homography != NULL)
	{
		CvMat *invIntrinsic = cvCloneMat(intrinsic);
		cvInv(intrinsic, invIntrinsic);

		// Vectors holding columns of H and R:
		float a_H1[3];
		CvMat  m_H1 = cvMat( 3, 1, CV_32FC1, a_H1 );
		for( i = 0; i < 3; i++ ) cvmSet( &m_H1, i, 0, cvmGet( Homography, i, 0 ) );

		float a_H2[3];
		CvMat  m_H2 = cvMat( 3, 1, CV_32FC1, a_H2 );
		for( i = 0; i < 3; i++ ) cvmSet( &m_H2, i, 0, cvmGet( Homography, i, 1 ) );

		float a_H3[3];
		CvMat  m_H3 = cvMat( 3, 1, CV_32FC1, a_H3 );
		for( i = 0; i < 3; i++ ) cvmSet( &m_H3, i, 0, cvmGet( Homography, i, 2 ) );

		float a_CinvH1[3];
		CvMat  m_CinvH1 = cvMat( 3, 1, CV_32FC1, a_CinvH1 );

		float a_R1[3];
		CvMat  m_R1 = cvMat( 3, 1, CV_32FC1, a_R1 );

		float a_R2[3];
		CvMat  m_R2 = cvMat( 3, 1, CV_32FC1, a_R2 );

		float a_R3[3];
		CvMat  m_R3 = cvMat( 3, 1, CV_32FC1, a_R3 );

		// The rotation matrix:
		float a_R[9];
		CvMat  m_R = cvMat( 3, 3, CV_32FC1, a_R );

		// The translation vector:
		float a_T[3];
		CvMat  m_T = cvMat( 3, 1, CV_32FC1, a_T );

		////////////////////////////////////////////////////////
		// Create norming factor lambda:
		cvGEMM(invIntrinsic, &m_H1, 1, NULL, 0, &m_CinvH1, 0 );

		// Search next orthonormal matrix:
		if( cvNorm( &m_CinvH1, NULL, CV_L2, NULL ) != 0 )
		{
			float lambda = 1.00/cvNorm( &m_CinvH1, NULL, CV_L2, NULL );

			// Create normalized R1 & R2:
			cvGEMM( invIntrinsic, &m_H1, lambda, NULL, 0, &m_R1, 0 );
			cvGEMM( invIntrinsic, &m_H2, lambda, NULL, 0, &m_R2, 0 );

			// Get R3 orthonormal to R1 and R2:
			cvCrossProduct( &m_R1, &m_R2, &m_R3 );

			// Put the rotation column vectors in the rotation matrix:
			for( i = 0; i < 3; i++ )
			{
				cvmSet( &m_R, i, 0,  cvmGet( &m_R1, i, 0 ) );
				cvmSet( &m_R, i, 1,  cvmGet( &m_R2, i, 0 ) );
				cvmSet( &m_R, i, 2,  cvmGet( &m_R3, i, 0 ) );
			}

			// Calculate Translation Vector T (- because of its definition):
			cvGEMM( invIntrinsic, &m_H3, lambda, NULL, 0, &m_T, 0 );

			// Transformation of R into - in Frobenius sense - next orthonormal matrix:
			float a_W[9]; CvMat  m_W  = cvMat( 3, 3, CV_32FC1, a_W  );
			float a_U[9]; CvMat  m_U  = cvMat( 3, 3, CV_32FC1, a_U  );
			float a_Vt[9]; CvMat  m_Vt = cvMat( 3, 3, CV_32FC1, a_Vt );
			cvSVD( &m_R, &m_W, &m_U, &m_Vt, CV_SVD_MODIFY_A | CV_SVD_V_T );
			cvMatMul( &m_U, &m_Vt, &m_R );

			cvReleaseMat(&invIntrinsic);

			cvSetIdentity(RT);
			for(i=0; i<3; i++)
			{
			for(j=0; j<3; j++)
			{
				cvmSet(RT, i, j, cvmGet(&m_R, i, j));
			}
			cvmSet(RT, i, 3, cvmGet(&m_T, i, 0));
			}
		}

	}
}

double ModifiedSURFTracker::CalculatePose()
{
	float homography[9];
	float intrinsicMatrix[9];
	float extrinsicMatrix[12];
	float extrinsicOutMatrix[16];

	CvMat* intrinsic = cameraParameter->GetIntrinsicMatrix();
	for(int y=0; y<3; y++)
	{
		for(int x=0; x<3; x++)
		{
			intrinsicMatrix[y*3+x] = (float)cvGetReal2D(intrinsic, y, x);
		}
	}


	// HACK! for coordinate
/*
	std::vector<CvPoint2D32f> temp;
	for(int i=0; i<pt2->size(); i++)
	{
		CvPoint2D32f point = (*pt2)[i];
		point.y = this->imageHeight - point.y;
		temp.push_back(point);
	}
*/
	float homographyError = 0;
	int n = (int)matchedReferencePoints.size();
	if(n >= 4)
	{
		CvMat _h = cvMat(3, 3, CV_32F, homography);

		CvMat _intrinsic = cvMat(3, 3, CV_32F, intrinsicMatrix);
		CvMat _extrinsic = cvMat(3, 4, CV_32FC1, extrinsicMatrix);

		CvMat _pt1 = cvMat(1, n, CV_32FC2, &(matchedReferencePoints[0]) );
		CvMat _pt2 = cvMat(1, n, CV_32FC2, &(matchedScenePoints[0]) );

		homographyError = 0.0;
		if(cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 2.0 ))
		{
//*
			// calculate homography error & remove outlier
			float difference = 0;
			int count = 0;
			for(unsigned int i=0; i<matchedReferencePoints.size(); i++)
			{

				float point[3];
				point[0] = matchedReferencePoints[i].x;
				point[1] = matchedReferencePoints[i].y;
				point[2] = 1.0;

				float projectionPointX = homography[0] * point[0] + homography[1] * point[1] + homography[2] * point[2];
				float projectionPointY = homography[3] * point[0] + homography[4] * point[1] + homography[5] * point[2];
				float projectionPointZ = homography[6] * point[0] + homography[7] * point[1] + homography[8] * point[2];
				projectionPointX /= projectionPointZ;
				projectionPointY /= projectionPointZ;

				std::vector<CvPoint2D32f>::iterator it1 = this->matchedReferencePoints.begin();
				std::vector<CvPoint2D32f>::iterator it2 = this->matchedScenePoints.begin();

				if(abs(matchedScenePoints[i].x - projectionPointX) + abs(matchedScenePoints[i].y - projectionPointY) < 2.0)
				{
					difference += abs(matchedScenePoints[i].x - projectionPointX);
					difference += abs(matchedScenePoints[i].y - projectionPointY);
					count++;
				}
				else // outlier
				{
					this->matchedReferencePoints.erase(this->matchedReferencePoints.begin() + count);
					this->matchedScenePoints.erase(this->matchedScenePoints.begin() + count);

					this->matchedReference.erase(this->matchedReference.begin() + count);
					this->matchedScene.erase(this->matchedScene.begin() + count);
				}
			}
			homographyError = (difference / 2.) / (float)count;
//*/
			DecomposeHomographyToRT(&_intrinsic, &_h, &_extrinsic);

//			memset(extrinsicOutMatrix, 0, sizeof(double)*16);
			for(int y=0; y<3; y++)
			{
				for(int x=0; x<4; x++)
				{
					extrinsicOutMatrix[y*4 + x] = extrinsicMatrix[y*4 + x];
				}
			}
			extrinsicOutMatrix[3*4 + 0] = extrinsicOutMatrix[3*4 + 1] = extrinsicOutMatrix[3*4 + 2] = 0;
			extrinsicOutMatrix[3*4 + 3] = 1;

			cameraParameter->SetExtrinsicMatrix(extrinsicOutMatrix);
		}
	}
	else
	{
		homographyError = 1.0;
	}

	return homographyError;
}

int ModifiedSURFTracker::UpdateCameraPose(IplImage* grayImage)
{
	if(runOpticalflow)
	{
		if(step > stepSize)
		{
			std::vector<CvPoint2D32f> matchedTempPoints;
			opticalflow->TrackFeature(grayImage, &matchedScenePoints, &matchedTempPoints);

			int index = 0;
			for(unsigned int i=0; i<matchedTempPoints.size(); i++)
			{
				if(matchedTempPoints[i].x >= 0 && matchedTempPoints[i].y >= 0)
				{
					matchedScene[index].point = matchedTempPoints[i];
					index++;
				}
				else
				{
					matchedReference.erase(matchedReference.begin() + index);
					matchedScene.erase(matchedScene.begin() + index);
				}
			}

			// add feature
			sceneSURF.clear();
			int featureCount = ExtractModifiedSURF(grayImage, &sceneSURF, featureExtractThreshold);

			std::vector<SURFDesciription> tempReferenceSURF;
			std::vector<SURFDesciription> tempSceneSURF;

			for(unsigned int i=0; i<sceneSURF.size(); i++)
			{
				int index = FindPairs(sceneSURF[i], referenceFeatureTree);
				if(index > 0)
				{
					tempReferenceSURF.push_back(referenceSURF[index]);
					tempSceneSURF.push_back(sceneSURF[i]);
				}
			}

			for(unsigned int i=0; i<tempReferenceSURF.size(); i++)
			{
				bool isFound = false;
				for(unsigned int j=0; j<matchedReference.size()&&!isFound; j++)
				{
					if( tempReferenceSURF[i].point.x == matchedReference[j].point.x &&
						tempReferenceSURF[i].point.y == matchedReference[j].point.y)
					{
						isFound = true;
					}
				}

				if(!isFound)
				{
					matchedReference.push_back(tempReferenceSURF[i]);
					matchedScene.push_back(tempSceneSURF[i]);
				}
			}

			step = 0;
		}
		else
		{
			std::vector<CvPoint2D32f> matchedTempPoints;
			opticalflow->TrackFeature(grayImage, &matchedScenePoints, &matchedTempPoints);

			int index = 0;
			for(unsigned int i=0; i<matchedTempPoints.size(); i++)
			{
				if(matchedTempPoints[i].x >= 0 && matchedTempPoints[i].y >= 0)
				{
					matchedScene[index].point = matchedTempPoints[i];
					index++;
				}
				else
				{
					matchedReference.erase(matchedReference.begin() + index);
					matchedScene.erase(matchedScene.begin() + index);
				}
			}
		}
		step++;
	}
	else
	{
		sceneSURF.clear();
		int featureCount = ExtractModifiedSURF(grayImage, &sceneSURF, featureExtractThreshold);

		matchedReference.clear();
		matchedScene.clear();
		for(unsigned int i=0; i<sceneSURF.size(); i++)
		{
			int index = FindPairs(sceneSURF[i], referenceFeatureTree);
			if(index > 0)
			{
				matchedReference.push_back(referenceSURF[index]);
				matchedScene.push_back(sceneSURF[i]);
			}
		}
	}

	matchedReferencePoints.clear();
	matchedScenePoints.clear();
	for(unsigned int i=0; i<matchedScene.size(); i++)
	{
		matchedReferencePoints.push_back(matchedReference[i].point);
		matchedScenePoints.push_back(matchedScene[i].point);
	}
	this->CalculatePose();

	return (int)matchedScenePoints.size();
}

void ModifiedSURFTracker::DrawDebugInfo(IplImage* colorImage)
{
	int pointCount = (int)sceneSURF.size();
	int r = 255;
	int g = 0;
	int b = 0;

	int size = 4;
//*
	for(int i=0; i<pointCount; i++)
	{
		r = 255 - 255 * (i/(double)pointCount);
		b = 255 * (i/(double)pointCount);
		cvCircle(colorImage, cvPoint(sceneSURF[i].point.x, sceneSURF[i].point.y), size, CV_RGB(r, g, b), 1);
	}
//*/
	for(unsigned int i=0; i<matchedScene.size(); i++)
	{
		cvCircle(colorImage, cvPoint(matchedReference[i].point.x * colorImage->width/realWidth, (colorImage->height - matchedReference[i].point.y * colorImage->height/realHeight)), size, CV_RGB(0, 255, 255), CV_FILLED);
		cvCircle(colorImage, cvPoint(matchedScene[i].point.x, matchedScene[i].point.y), size, CV_RGB(255, 255, 0), CV_FILLED);
		cvLine(colorImage, cvPoint(matchedReference[i].point.x * colorImage->width/realWidth, (colorImage->height - matchedReference[i].point.y * colorImage->height/realHeight)), cvPoint(matchedScene[i].point.x, matchedScene[i].point.y), CV_RGB(255, 0, 0));
	}

	CvScalar color = CV_RGB(255, 0, 255);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0),							cameraParameter->ConvertWorld2Image(this->realWidth, 0.0, 0.0),					color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(this->realWidth, 0.0, 0.0),				cameraParameter->ConvertWorld2Image(this->realWidth, this->realHeight, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(this->realWidth, this->realHeight, 0.0), cameraParameter->ConvertWorld2Image(0.0, this->realHeight, 0.0),				color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, this->realHeight, 0.0),				cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0),								color, 2);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(0.0, 0.0, 0.0),							cameraParameter->ConvertWorld2Image(this->realWidth, this->realHeight, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(this->realWidth, 0.0, 0.0),				cameraParameter->ConvertWorld2Image(0.0, this->realHeight, 0.0),				color, 2);
}


void ModifiedSURFTracker::InitializeOpticalFlow(int width, int height, int stepSize, CvSize windowSize, int pyramidLevel)
{
	if(opticalflow) delete opticalflow;
	opticalflow = new OpticalFlow();
	opticalflow->Initialize(width, height, windowSize, pyramidLevel);

	runOpticalflow = true;
	this->stepSize = stepSize;
	this->step = stepSize + 1;
}