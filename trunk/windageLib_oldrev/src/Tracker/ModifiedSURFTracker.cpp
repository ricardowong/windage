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

#include "Tracker/ModifiedSURFTracker.h"
using namespace windage;

#include "Tracker/PoseEstimation/FindPROSACHomography.h"
#include "Tracker/PoseEstimation/Find3DPoseEstimation.h"

//#define USING_KDTREE
#define USING_FLANN

const double ERROR_BOUND = 10.0;
const int POSE_POINTS_COUNT = 10;
const double COMPAIR_RATE = 0.50;
const int EMAX = 20;

ModifiedSURFTracker::ModifiedSURFTracker()
{
	this->cameraParameter = NULL;
	this->prevImage = NULL;
	this->referenceImage = NULL;
	this->featureExtractThreshold = 50;

	this->referenceFeatureStorage = NULL;
	this->referenceFeatureTree = NULL;
	this->flannIndex = NULL;

	this->opticalflow = NULL;
	this->runOpticalflow = false;
	this->step = 0;

	this->poseEstimationMethod = windage::RANSAC;
	this->outlinerRemove = true;
	this->refinement = false;

//	this->log = new Logger("Log\\modifiedSURF_Performance", true);
	log = NULL;
}

ModifiedSURFTracker::~ModifiedSURFTracker()
{
	this->Release();
}

void ModifiedSURFTracker::Release()
{
	if(cameraParameter) delete cameraParameter;
	cameraParameter = NULL;
	if(prevImage) cvReleaseImage(&prevImage);
	prevImage = NULL;
	if(referenceImage) cvReleaseImage(&referenceImage);
	referenceImage = NULL;

	if(referenceFeatureStorage) cvReleaseMat(&referenceFeatureStorage);
	referenceFeatureStorage = NULL;
	if(referenceFeatureTree) cvReleaseFeatureTree(referenceFeatureTree);
	referenceFeatureTree = NULL;

	if(opticalflow) delete opticalflow;
	opticalflow = NULL;
}

void ModifiedSURFTracker::Initialize(double fx, double fy, double cx, double cy, double d1, double d2, double d3, double d4, int featureExtractThreshold)
{
	this->Release();
	cameraParameter = new Calibration();
	cameraParameter->Initialize(fx, fy, cx, cy, d1, d2, d3, d4);

	this->SetFeatureExtractThreshold(featureExtractThreshold);

}

void ModifiedSURFTracker::RegistReferenceImage(IplImage* referenceImage, double realWidth, double realHeight, double scaleFactor, int scaleStep)
{
	// delete previous reference data
	if(this->referenceImage) cvReleaseImage(&this->referenceImage);
	this->referenceImage = NULL;

	if(this->referenceFeatureStorage) cvReleaseMat(&this->referenceFeatureStorage);
	this->referenceFeatureStorage = NULL;
	if(this->referenceFeatureTree) cvReleaseFeatureTree(this->referenceFeatureTree);
	this->referenceFeatureTree = NULL;

	// registrate
	this->referenceImage = cvCloneImage(referenceImage);

	this->realWidth = realWidth;
	this->realHeight = realHeight;
	this->SetFeatureExtractThreshold(featureExtractThreshold);

	GenerateReferenceFeatureTree(scaleFactor, scaleStep);
}

int ModifiedSURFTracker::ExtractFASTCorner(std::vector<CvPoint>* corners, IplImage* grayImage, int threshold, int n)
{
	int cornerCount = 0;
	xy* cornertemp = NULL;

	bool isProcessed = true;
	switch(n)
	{
	case 9:
		cornertemp = fast9_detect((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	case 10:
		cornertemp = fast10_detect_nonmax((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	case 11:
		cornertemp = fast11_detect_nonmax((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	case 12 :
		cornertemp = fast12_detect_nonmax((const byte *)grayImage->imageData, grayImage->width, grayImage->height, grayImage->widthStep, threshold, &cornerCount);
		break;
	default:
		isProcessed = false;
		break;
	}

	if(isProcessed)
	{
		corners->resize(cornerCount);
		for(int i=0; i<cornerCount; ++i)
		{
			(*corners)[i] = cvPoint(cornertemp[i].x, cornertemp[i].y);
		}
		if(cornertemp) delete cornertemp;
	}

	return (int)corners->size();
}

int ModifiedSURFTracker::ExtractModifiedSURF(IplImage* grayImage, std::vector<CvPoint>* corners, std::vector<SURFDesciription>* descriptions)
{
	CvSURFParams params = cvSURFParams(500, 0);

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq *referenceKeypoints = 0, *referenceDescriptors = 0;
//	cvExtractSURF(image, 0, &referenceKeypoints, &referenceDescriptors, storage, params );

//	std::vector<CvPoint> fastCorners;
//	ExtractFASTCorner(&fastCorners, grayImage, thresholdFAST);

	referenceKeypoints = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSURFPoint), storage );
	for(int i=0; i<(int)(*corners).size(); i++)
	{
		CvSURFPoint point = cvSURFPoint( cvPoint2D32f((*corners)[i].x, (*corners)[i].y), 0, 15, 0, 0);
		cvSeqPush(referenceKeypoints, &point);
	}
//	wExtractSURF(grayImage, 0, &referenceKeypoints, &referenceDescriptors, storage, params, 1);
	wExtractFASTSURF(grayImage, 0, &referenceKeypoints, &referenceDescriptors, storage, params, 1);
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
		description.dir = r->dir;
		description.objectID = -1;

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

#include <iostream>
CvFeatureTree* ModifiedSURFTracker::CreateReferenceTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage)
{
	int count = (int)referenceSURF->size();
	std::cout << count << std::endl;

	if(!referenceFeatureStorage) cvReleaseMat(&referenceFeatureStorage);
	referenceFeatureStorage = cvCreateMat(count, SURF_DESCRIPTOR_DIMENSION, SURF_DESCRIPTOR_TYPE);

	for(int y=0; y<count; y++)
	{
		for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
		{
			cvSetReal2D(referenceFeatureStorage, y, x, (*referenceSURF)[y].descriptor[x]);
		}
	}
	
	CvFeatureTree* tree = cvCreateKDTree(referenceFeatureStorage);
	return tree;
}

bool ModifiedSURFTracker::CreateFlannTree(std::vector<SURFDesciription>* referenceSURF, CvMat* referenceFeatureStorage)
{
	int count = (int)referenceSURF->size();

	if(referenceFeatureStorage != NULL) cvReleaseMat(&referenceFeatureStorage);
	referenceFeatureStorage = cvCreateMat(count, SURF_DESCRIPTOR_DIMENSION, SURF_DESCRIPTOR_TYPE);

	for(int y=0; y<count; y++)
	{
		for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
		{
			cvmSet(referenceFeatureStorage, y, x, (*referenceSURF)[y].descriptor[x]);
		}
	}

	cv::Mat refer(referenceFeatureStorage, false);
	flannFeatureTree = refer;

	if(flannIndex) delete flannIndex;
	flannIndex = new cv::flann::Index(flannFeatureTree, cv::flann::KDTreeIndexParams(2));
	return true;
}

int ModifiedSURFTracker::FindPairs(SURFDesciription description, CvFeatureTree* tree, double distanceRate, float* outDistance)
{
	CvMat* currentFeature = cvCreateMat(1, SURF_DESCRIPTOR_DIMENSION, SURF_DESCRIPTOR_TYPE);
	CvMat* result = cvCreateMat(1, 2, CV_32S);
	CvMat* distance = cvCreateMat(1, 2, CV_64FC1);
	for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
	{
		cvSetReal2D(currentFeature, 0, x, description.descriptor[x]);
	}

	cvFindFeatures(tree, currentFeature, result, distance, 2, EMAX);

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

	if(outDistance)
		(*outDistance) = (float)min2;

	if ( min1 < distanceRate*min2 )
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
		double d = description.getDistance((*descriptions)[i]);
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

int ModifiedSURFTracker::FindPairs(SURFDesciription description, cv::flann::Index* treeIndex, float distanceRate, float* outDistance)
{
	CvMat* currentFeature = cvCreateMat(1, SURF_DESCRIPTOR_DIMENSION, SURF_DESCRIPTOR_TYPE);
	cv::Mat result(1, 2, CV_32S);
	cv::Mat distance(1, 2, SURF_DESCRIPTOR_TYPE);

	for(int x=0; x<SURF_DESCRIPTOR_DIMENSION; x++)
	{
		cvmSet(currentFeature, 0, x, description.descriptor[x]);
	}

	cv::Mat object(currentFeature, false);
	treeIndex->knnSearch(object, result, distance, 2, cv::flann::SearchParams(EMAX));

	float min2 = distance.ptr<float>(0)[0];
	float min1 = distance.ptr<float>(0)[1];
	int index2 = result.ptr<int>(0)[0];
	int index1 = result.ptr<int>(0)[1];
	int index = index1;
//*
	float temp;
	if(min2 < min1)
	{
		temp = min1;
		min1 = min2;
		min2 = temp;
		index = index2;
	}
//*/
	cvReleaseMat(&currentFeature);

	if(outDistance)
		(*outDistance) = (float)min2;

	if ( min1 < distanceRate*min2 )
		return index;
    return -1;
}

void Rotate(IplImage* src, IplImage* dst, float angle)
{  
   CvPoint2D32f centre;
   CvMat *translate = cvCreateMat(2, 3, CV_32FC1);
   cvSetZero(translate);
   centre.x = src->width/2.0f;
   centre.y = src->height/2.0f;
   cv2DRotationMatrix(centre, angle, 1.0, translate);
   cvWarpAffine(src, dst, translate, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
   cvReleaseMat(&translate);
}

int ModifiedSURFTracker::GenerateReferenceFeatureTree(double scaleFactor, int scaleStep)
{
	int width = (int)((double)referenceImage->width/scaleFactor);
	int height = (int)((double)referenceImage->height/scaleFactor);

	referenceSURF.clear();

	IplImage* tempReference = NULL;
	for(int y=1; y<=scaleStep; y++)
	{
		for(int x=1; x<=scaleStep; x++)
		{
			tempReference = cvCreateImage(cvSize(width*x, height*y), IPL_DEPTH_8U, 1);
			cvResize(this->referenceImage, tempReference, CV_INTER_LINEAR);
			cvSmooth(tempReference, tempReference, CV_GAUSSIAN, 3, 3);

			std::vector<SURFDesciription> tempSurf;
			tempSurf.clear();

			std::vector<CvPoint> fastCorners;
			ModifiedSURFTracker::ExtractFASTCorner(&fastCorners, tempReference, featureExtractThreshold);
			ModifiedSURFTracker::ExtractModifiedSURF(tempReference, &fastCorners, &tempSurf);
			float xScaleFactor = (float)this->realWidth / (float)tempReference->width;
			float yScaleFactor = (float)this->realHeight / (float)tempReference->height;

			for(int i=0; i<(int)tempSurf.size(); i++)
			{
				tempSurf[i].point.x *= xScaleFactor;
				tempSurf[i].point.y *= yScaleFactor;
				tempSurf[i].point.y = (float)this->realHeight - tempSurf[i].point.y;

				tempSurf[i].point.x = tempSurf[i].point.x - (float)this->realWidth/2;
				tempSurf[i].point.y = tempSurf[i].point.y - (float)this->realHeight/2;

				referenceSURF.push_back(tempSurf[i]);
			}

			cvReleaseImage(&tempReference);
		}
	}

#ifdef USING_KDTREE
	this->referenceFeatureTree = CreateReferenceTree(&this->referenceSURF, referenceFeatureStorage);
#endif
#ifdef USING_FLANN
	this->CreateFlannTree(&this->referenceSURF, referenceFeatureStorage);
#endif

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
			float lambda = 1.0f/cvNorm( &m_CinvH1, NULL, CV_L2, NULL );

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

double ModifiedSURFTracker::CalculatePose(bool update)
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

	float homographyError = 0;
	int n = (int)matchedReferencePoints.size();
	if(n >= POSE_POINTS_COUNT)
	{
		int inlierCount = 0;
		int outlierCount = 0;

		CvMat _h = cvMat(3, 3, CV_32F, homography);

		CvMat _intrinsic = cvMat(3, 3, CV_32F, intrinsicMatrix);
		CvMat _extrinsic = cvMat(3, 4, CV_32FC1, extrinsicMatrix);

		homographyError = 0.0;

		CvMat _pt1 = cvMat(1, n, CV_32FC2, &(matchedReferencePoints[0]) );
		CvMat _pt2 = cvMat(1, n, CV_32FC2, &(matchedScenePoints[0]) );

		bool isCalculate = false;
		switch(this->poseEstimationMethod)
		{
		case windage::PROSAC:
			{
				FindPROSACHomography prosac;
				std::vector<MatchedPoint> prosacMatchedPoints;
				for(int i=0; i<(int)matchedScene.size(); i++)
				{
					MatchedPoint referenceTemp(matchedScene[i].point, matchedReference[i].point, matchedScene[i].distance);
					prosacMatchedPoints.push_back(referenceTemp);
				}
				prosac.SetReprojectionThreshold((float)ERROR_BOUND);
				prosac.AttatchMatchedPoints(&prosacMatchedPoints);

				isCalculate = prosac.Calculate();
				if(isCalculate)
				{
					float* tempHomography = prosac.GetHomography();
					for(int i=0; i<9; i++) homography[i] = tempHomography[i];
				}
			}
			break;
		case windage::RANSAC:
			{				
				if(cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, ERROR_BOUND) > 0)
					isCalculate = true;
				else
					isCalculate = false;
			}
			break;
		case windage::LMEDS:
			{
				if(cvFindHomography( &_pt1, &_pt2, &_h, CV_LMEDS) > 0)
					isCalculate = true;
				else
					isCalculate = false;
			}
			break;
		case windage::POSE_3D:
			{
				Find3DPoseEstimation poseEstimator;
				std::vector<Matched3DPoint> poseEstimatorPoints;
				for(int i=0; i<(int)matchedScene.size(); i++)
				{
					Matched3DPoint referenceTemp(matchedScene[i].point, cvPoint3D32f(matchedReference[i].point.x, matchedReference[i].point.y, 0.0));
					poseEstimatorPoints.push_back(referenceTemp);
				}
				poseEstimator.AttatchMatchedPoints(&poseEstimatorPoints);
				poseEstimator.AttatchCalibration(this->cameraParameter);
				poseEstimator.SetReprojectionThreshold(ERROR_BOUND);
				poseEstimator.SetUseRANSAC(true);
				poseEstimator.SetItrationTime(300);
				if(poseEstimator.Calculate() > 0.0)
					isCalculate = true;
				else
					isCalculate = false;
			}
			break;
		}


		if(outlinerRemove && isCalculate)
		{
			// calculate homography error & remove outlier
			float difference = 0;
			int count = 0;
			for(unsigned int i=0; i<matchedReferencePoints.size(); i++)
			{
				float error = 0.0f;
				if(this->poseEstimationMethod == windage::POSE_3D)
				{
					error = Find3DPoseEstimation::ComputeReprojError(matchedScenePoints[i], cvPoint3D32f(matchedReferencePoints[i].x, matchedReferencePoints[i].y, 0.0), cameraParameter->GetIntrinsicMatrix(), cameraParameter->GetExtrinsicMatrix());
				}
				else
				{
					error = FindHomography::ComputeReprojError(matchedReferencePoints[i], matchedScenePoints[i], homography);
				}

				if(error <= ERROR_BOUND)
				{
					this->matchedReference[count].distance /= 10.0;
					this->matchedScene[count].distance /= 10.0;

					count++;
					inlierCount++;
				}
				else // outlier
				{
					this->matchedReferencePoints.erase(this->matchedReferencePoints.begin() + count);
					this->matchedScenePoints.erase(this->matchedScenePoints.begin() + count);

					this->matchedReference.erase(this->matchedReference.begin() + count);
					this->matchedScene.erase(this->matchedScene.begin() + count);

					outlierCount++;
					i--;
				}

				difference += (error);
			}
			homographyError = difference / (float)matchedReferencePoints.size();
		}

		if(this->poseEstimationMethod == windage::POSE_3D)
		{
			if(isCalculate)
			{
			}
			else
			{
				// remove all points
				if(outlinerRemove)
				{
					for(unsigned int i=0; i<matchedReferencePoints.size(); i++)
					{
						this->matchedReferencePoints.clear();
						this->matchedScenePoints.clear();
						this->matchedReference.clear();
						this->matchedScene.clear();
					}
				}

				homographyError = -1.0;
			}
		}
		else
		{
			if(isCalculate)
			{
				if(this->refinement && matchedScene.size() > 4)
				{
					Find3DPoseEstimation poseEstimator;
					std::vector<Matched3DPoint> poseEstimatorPoints;
					for(int i=0; i<(int)matchedScene.size(); i++)
					{
						Matched3DPoint referenceTemp(matchedScene[i].point, cvPoint3D32f(matchedReference[i].point.x, matchedReference[i].point.y, 0.0));
						poseEstimatorPoints.push_back(referenceTemp);
					}
					poseEstimator.AttatchMatchedPoints(&poseEstimatorPoints);
					poseEstimator.AttatchCalibration(this->cameraParameter);
					poseEstimator.SetReprojectionThreshold(ERROR_BOUND);
					poseEstimator.SetItrationTime(5);
					poseEstimator.SetUseRANSAC(true);
					poseEstimator.Calculate();
				}
				else
				{
					DecomposeHomographyToRT(&_intrinsic, &_h, &_extrinsic);
					for(int y=0; y<3; y++)
					{
						for(int x=0; x<4; x++)
						{
							extrinsicOutMatrix[y*4 + x] = extrinsicMatrix[y*4 + x];
						}
					}
					extrinsicOutMatrix[3*4 + 0] = extrinsicOutMatrix[3*4 + 1] = extrinsicOutMatrix[3*4 + 2] = 0;
					extrinsicOutMatrix[3*4 + 3] = 1;

					if(update)
						cameraParameter->SetExtrinsicMatrix(extrinsicOutMatrix);
				}

			}
			else
			{
				// remove all points
				if(outlinerRemove)
				{
					for(unsigned int i=0; i<matchedReferencePoints.size(); i++)
					{
						this->matchedReferencePoints.clear();
						this->matchedScenePoints.clear();
						this->matchedReference.clear();
						this->matchedScene.clear();
					}
				}

				homographyError = -1.0;
			}
		}
	}
	else
	{
		// remove all points
		if(outlinerRemove && n < POSE_POINTS_COUNT/2)
		{
			for(unsigned int i=0; i<matchedReferencePoints.size(); i++)
			{
				this->matchedReferencePoints.clear();
				this->matchedScenePoints.clear();
				this->matchedReference.clear();
				this->matchedScene.clear();
			}
		}

		homographyError = -1.0;
	}	

	return homographyError;
}

double ModifiedSURFTracker::UpdateCameraPose(IplImage* grayImage)
{
	bool update = true;
	if(runOpticalflow)
	{
		if(prevImage)
		{
			if(step > stepSize)
			{
	//			update = false;
				std::vector<CvPoint2D32f> matchedTempPoints;

				if(log) log->updateTickCount();
				opticalflow->TrackFeature(prevImage, grayImage, &matchedScenePoints, &matchedTempPoints);
				if(log) log->log("opticalflow", log->calculateProcessTime());

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
				std::vector<CvPoint> fastCorners;

				if(log) log->updateTickCount();
				ModifiedSURFTracker::ExtractFASTCorner(&fastCorners, grayImage, featureExtractThreshold, 9);
				if(log) log->log("FAST", log->calculateProcessTime());

				if(log) log->updateTickCount();
				int featureCount = ModifiedSURFTracker::ExtractModifiedSURF(grayImage, &fastCorners, &sceneSURF);
				if(log) log->log("Descriptor", log->calculateProcessTime());
				if(log) log->log("COUNT", featureCount);
				

				std::vector<SURFDesciription> tempReferenceSURF;
				std::vector<SURFDesciription> tempSceneSURF;

				if(log) log->updateTickCount();
				for(int i=0; i<(int)sceneSURF.size(); i++)
				{
					float distance = 0;
#ifdef USING_KDTREE
					int index = FindPairs(sceneSURF[i], referenceFeatureTree, COMPAIR_RATE, &distance);
#endif
#ifdef USING_FLANN
					int index = FindPairs(sceneSURF[i], this->flannIndex, COMPAIR_RATE, &distance);
#endif
					if(index > 0)
					{
						sceneSURF[i].distance = distance;
						tempReferenceSURF.push_back(referenceSURF[index]);
						tempSceneSURF.push_back(sceneSURF[i]);
					}
				}

				for(unsigned int i=0; i<tempReferenceSURF.size(); i++)
				{
					bool isFound = false;
					for(unsigned int j=0; j<matchedReference.size()&&!isFound; j++)
					{
						if( abs(tempReferenceSURF[i].point.x - matchedReference[j].point.x) +
							abs(tempReferenceSURF[i].point.y - matchedReference[j].point.y) < 0.5)
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

				if(log) log->log("Matching", log->calculateProcessTime());
				if(log) log->log("COUNT", (int)matchedScene.size());

				step = 0;

				
			}
			else
			{
				std::vector<CvPoint2D32f> matchedTempPoints;

				if(log) log->updateTickCount();
				opticalflow->TrackFeature(prevImage, grayImage, &matchedScenePoints, &matchedTempPoints);
				if(log) log->log("opticalflow", log->calculateProcessTime());
				if(log) log->log("FAST", 0.0);
				if(log) log->log("Descriptor", 0.0);
				if(log) log->log("COUNT", (int)matchedScenePoints.size());
				if(log) log->log("Matching", 0.0);
				if(log) log->log("COUNT", (int)matchedScenePoints.size());

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

		}
		else
		{
			prevImage = cvCreateImage(cvGetSize(grayImage), IPL_DEPTH_8U, 1);
		}

		step++;

		// for optical flow
		cvCopyImage(grayImage, prevImage);
	}
	else
	{
		sceneSURF.clear();
		std::vector<CvPoint> fastCorners;

		if(log) log->updateTickCount();
		ModifiedSURFTracker::ExtractFASTCorner(&fastCorners, grayImage, featureExtractThreshold);
		if(log) log->log("FAST", log->calculateProcessTime());

		int featureCount = ModifiedSURFTracker::ExtractModifiedSURF(grayImage, &fastCorners, &sceneSURF);
		if(log) log->log("Descriptor", log->calculateProcessTime());
		if(log) log->log("COUNT", featureCount);

		matchedReference.clear();
		matchedScene.clear();

		if(log) log->updateTickCount();
		for(int i=0; i<(int)sceneSURF.size(); i++)
		{
			float distance = 0;
#ifdef USING_KDTREE
			int index = FindPairs(sceneSURF[i], referenceFeatureTree, COMPAIR_RATE, &distance);
#endif
#ifdef USING_FLANN
			int index = FindPairs(sceneSURF[i], this->flannIndex, COMPAIR_RATE, &distance);
#endif
			if(index > 0)
			{
				sceneSURF[i].distance = distance;
				matchedReference.push_back(referenceSURF[index]);
				matchedScene.push_back(sceneSURF[i]);
			}
		}
		if(log) log->log("Matching", log->calculateProcessTime());
		if(log) log->log("COUNT", (int)matchedScene.size());
	}

	matchedReferencePoints.clear();
	matchedScenePoints.clear();
	for(unsigned int i=0; i<matchedScene.size(); i++)
	{
		matchedReferencePoints.push_back(matchedReference[i].point);
		matchedScenePoints.push_back(matchedScene[i].point);
	}

	if(log) log->updateTickCount();
	double homograpyError = this->CalculatePose(update);
	if(log) log->log("PoseEstimation", log->calculateProcessTime());
	if(log) log->logNewLine();

	return homograpyError;//(int)matchedScenePoints.size();
}

void ModifiedSURFTracker::DrawDebugInfo(IplImage* colorImage)
{
	int pointCount = (int)sceneSURF.size();
	int r = 255;
	int g = 0;
	int b = 0;

	int size = 4;
	for(unsigned int i=0; i<matchedScene.size(); i++)
	{
		CvPoint referencePoint = cvPoint((int)matchedReference[i].point.x * colorImage->width/realWidth + colorImage->width/2,
									(int)(colorImage->height - matchedReference[i].point.y * colorImage->height/realHeight - colorImage->height/2));
		CvPoint imagePoint = cvPoint((int)matchedScene[i].point.x, (int)matchedScene[i].point.y);

		cvCircle(colorImage, referencePoint, size, CV_RGB(0, 255, 255), CV_FILLED);
		cvCircle(colorImage, imagePoint, size, CV_RGB(255, 255, 0), CV_FILLED);

		cvLine(colorImage, referencePoint, imagePoint, CV_RGB(255, 0, 0));
	}
}


void ModifiedSURFTracker::DrawDebugInfo2(IplImage* colorImage)
{
	int pointCount = (int)sceneSURF.size();
	int r = 255;
	int g = 0;
	int b = 0;

	int size = 6;

	for(unsigned int i=0; i<matchedScene.size(); i++)
	{
		CvPoint referencePoint = cvPoint((int)matchedReference[i].point.x * colorImage->width/realWidth + colorImage->width/2,
									(int)(colorImage->height/2 - matchedReference[i].point.y * (colorImage->height/2)/realHeight - colorImage->height/4));
		CvPoint imagePoint = cvPoint((int)matchedScene[i].point.x, (int)matchedScene[i].point.y + colorImage->height/2);

		cvCircle(colorImage, referencePoint, size, CV_RGB(0, 255, 255), CV_FILLED);
		cvCircle(colorImage, imagePoint, size, CV_RGB(255, 255, 0), CV_FILLED);
		cvLine(colorImage, referencePoint, imagePoint, CV_RGB(255, 0, 0), 1);
	}
}


void ModifiedSURFTracker::DrawOutLine(IplImage* colorImage, bool drawCross)
{
	int pointCount = (int)sceneSURF.size();
	int r = 255;
	int g = 0;
	int b = 0;

	int size = 4;

	CvScalar color = CV_RGB(255, 0, 255);
	CvScalar color2 = CV_RGB(255, 255, 255);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	color2, 6);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	color2, 6);

	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	color, 2);
	cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	color, 2);

	if(drawCross)
	{
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color2, 6);
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color2, 6);

		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, -this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, +this->realHeight/2, 0.0),	color, 2);
		cvLine(colorImage, cameraParameter->ConvertWorld2Image(-this->realWidth/2, +this->realHeight/2, 0.0),	cameraParameter->ConvertWorld2Image(+this->realWidth/2, -this->realHeight/2, 0.0),	color, 2);
	}
}


void ModifiedSURFTracker::InitializeOpticalFlow(int width, int height, int stepSize, CvSize windowSize, int pyramidLevel)
{
	if(opticalflow) delete opticalflow;
	opticalflow = new OpticalFlow();
	opticalflow->Initialize(width, height, windowSize, pyramidLevel);
	opticalflow->SetRemovePrevPoints(false);

	runOpticalflow = true;
	this->stepSize = stepSize;
	this->step = stepSize + 1;
}