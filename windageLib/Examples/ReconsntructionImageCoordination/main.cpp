/* ========================================================================
 * PROJECT: windage Library
 * ========================================================================
 * This work is based on the original windage Library developed by
 *   Woonhyuk Baek (wbaek@gist.ac.kr / windage@live.com)
 *   Woontack Woo (wwoo@gist.ac.kr)
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
#include <direct.h>

#include <cv.h>
#include <highgui.h>

#include <windage.h>

const int WIDTH = 640;
const int HEIGHT = (WIDTH * 3) / 4;

const double REPROJECTION_ERROR = 2.0;
const double INTRINSIC[] = {1, 1, 0, 0, 0, 0, 0, 0};
//const double INTRINSIC[] = {1033.93, 1033.84, 319.044, 228.858,-0.206477, 0.306424, 0.000728208, 0.0011338};

const char* RECONSTRUCTION_PATH_TEMPLATE = "data/reconstruction-%s";
const char* RECONSTRUCTION_FILENAME = "%s/reconstruction";
const char* IMAGE_FILE_NAME_TEMPLATE = "%s/image%03d.png";

const double real_width = 180;

//const char* FILE_NAME = "data/reconstruction-2010-03-29_18_28_38/reconstruction.txt";
//const char* FILE_NAME = "data/reconstruction-2010-03-29_09_33_01/reconstruction.txt";
//const char* FILE_NAME = "data/reconstruction-2010-03-29_09_33_01/reconstruction.txt";
const char* FILE_NAME = "data/reconstruction-2010-11-28_16_13_16/reconstruction.txt";
const char* REFE_FILE = "data/reconstruction-2010-11-28_16_13_16/reference.png";

void main()
{
	windage::Logger logger(&std::cout);

	cvNamedWindow("result");

	// create and initialize tracker
	windage::Calibration* calibration					= new windage::Calibration();
	windage::Algorithms::FeatureDetector* detector		= new windage::Algorithms::SIFTGPUdetector();
	windage::Algorithms::SearchTree* searchtree			= new windage::Algorithms::FLANNtree(30);
	windage::Algorithms::OpenCVRANSACestimator* estimator	= new windage::Algorithms::OpenCVRANSACestimator();
	windage::Algorithms::PoseRefiner* refiner			= new windage::Algorithms::PoseLMmethod();
	
	calibration->Initialize(INTRINSIC[0], INTRINSIC[1], INTRINSIC[2], INTRINSIC[3], INTRINSIC[4], INTRINSIC[5], INTRINSIC[6], INTRINSIC[7]);
	searchtree->SetRatio(0.3);
	estimator->SetReprojectionError(REPROJECTION_ERROR);
	estimator->SetConfidence(0.90);
	estimator->SetMaxIteration(100);
	refiner->SetMaxIteration(20);

	// load coordination image
	IplImage* inputImage = cvLoadImage(REFE_FILE);
	IplImage* grayImage = cvLoadImage(REFE_FILE, 0);

	// load reconstruction data
	std::vector<windage::Calibration*> calibrationList;
	std::vector<std::string> filenameList;
	std::vector<windage::ReconstructionPoint> reconstructionPoints;

	windage::Reconstruction::Loader* loader = new windage::Reconstruction::Loader();
	loader->AttatchCalibration(&calibrationList);
	loader->AttatchFilename(&filenameList);
	loader->AttatchReconstructionPoints(&reconstructionPoints);
	loader->DoLoad(FILE_NAME);

	// set repository
	std::vector<windage::FeaturePoint> referenceRepository;
	int index = 0;
	for(unsigned int i=0; i<reconstructionPoints.size(); i++)
	{
		std::vector<windage::FeaturePoint>* featureList = reconstructionPoints[i].GetFeatureList();

		// remove less matched feature points
		if(featureList->size() > 1)
		{
			windage::FeaturePoint feature = (*featureList)[0];
			windage::Vector4 point = reconstructionPoints[i].GetPoint();
			feature.SetPoint(windage::Vector3(point.x, point.y, point.z));
			feature.SetObjectID(0);
			feature.SetRepositoryID(index);

			// normalize descriptor
			int featureCount = (int)featureList->size();
			for(int j=1; j<featureCount; j++)
			{
				for(int k=0; k<(*featureList)[j].DESCRIPTOR_DIMENSION; k++)
					feature.descriptor[k] += (*featureList)[j].descriptor[k];
			}

			for(int k=0; k<feature.DESCRIPTOR_DIMENSION; k++)
				feature.descriptor[k] /= (double)featureCount;

			referenceRepository.push_back(feature);
			index++;
		}
	}
	std::cout << "reference repository size : " << referenceRepository.size() << std::endl;

	// match
	searchtree->Training(&referenceRepository);
	detector->DoExtractKeypointsDescriptor(grayImage);
	detector->DrawKeypoints(inputImage);

	calibration->DrawInfomation(inputImage, 50);
	cvShowImage("result", inputImage);
	cvWaitKey();
	
	std::vector<windage::FeaturePoint>* keypoints = detector->GetKeypoints();
	for(int i=0; i<keypoints->size(); i++)
	{
		windage::Vector3 pt = (*keypoints)[i].GetPoint();
		pt.x -= (grayImage->width/2.0f);
		pt.y = (grayImage->height/2.0f) - pt.y;
		pt.z = 1.0;

		(*keypoints)[i].SetPoint(pt);
	}

	std::vector<windage::FeaturePoint> refMatchedKeypoints;
	std::vector<windage::FeaturePoint> sceMatchedKeypoints;
	std::vector<windage::FeaturePoint>* sceneKeypoints = detector->GetKeypoints();
	std::cout << "secene size : " << sceneKeypoints->size() << std::endl;

	for(unsigned int i=0; i<sceneKeypoints->size(); i++)
	{
		int count = 0;
		int index = searchtree->Matching((*sceneKeypoints)[i]);
		if(0 <= index && index < (int)referenceRepository.size())
		{
			(*sceneKeypoints)[i].SetRepositoryID(index);

			refMatchedKeypoints.push_back(referenceRepository[index]);
			sceMatchedKeypoints.push_back((*sceneKeypoints)[i]);
		}
	}

	std::cout << "matching count : " << sceMatchedKeypoints.size() << std::endl; 

	// pose estimation
	estimator->AttatchCameraParameter(calibration);
	estimator->AttatchReferencePoint(&refMatchedKeypoints);
	estimator->AttatchScenePoint(&sceMatchedKeypoints);
	estimator->Calculate();

	// outlier remove
	for(int i=0; i<(int)refMatchedKeypoints.size(); i++)
	{
		if(refMatchedKeypoints[i].IsOutlier() == true)
		{
			referenceRepository[refMatchedKeypoints[i].GetRepositoryID()].SetTracked(false);

			refMatchedKeypoints.erase(refMatchedKeypoints.begin() + i);
			sceMatchedKeypoints.erase(sceMatchedKeypoints.begin() + i);
			i--;
		}
	}

	// refinement
	if(refiner)
	{
		refiner->AttatchCalibration(calibration);
		refiner->AttatchReferencePoint(&refMatchedKeypoints);
		refiner->AttatchScenePoint(&sceMatchedKeypoints);
		refiner->Calculate();
	}


	windage::Vector3 scePt1 = sceMatchedKeypoints[0].GetPoint();
	windage::Vector3 scePt2 = sceMatchedKeypoints[10].GetPoint();

	windage::Vector3 refPt1 = refMatchedKeypoints[0].GetPoint();
	windage::Vector3 refPt2 = refMatchedKeypoints[10].GetPoint();

	calibration->DrawInfomation(inputImage, 50);
	
	// update
	{
		double scale = (real_width / (double)inputImage->width) * scePt1.getDistance(scePt2) / refPt1.getDistance(refPt2) ;
		std::cout << "scale : " << scale << std::endl;

		windage::Vector4 center;
		int count = (unsigned)reconstructionPoints.size();
		for(int i=0; i<count; i++)
		{
			center += reconstructionPoints[i].GetPoint();
		}
		center /= (double)count;

		double distance = 0.0;
		for(int i=0; i<count; i++)
		{
			distance += center.getDistance(reconstructionPoints[i].GetPoint());
		}
		distance /= (double)count;
//		scale = scale / distance;

		// points translation
		for(int i=0; i<count; i++)
		{
			windage::Vector4 tempPoint = reconstructionPoints[i].GetPoint();
			tempPoint = (tempPoint - center) * scale;
			tempPoint.w = 1.0;
			reconstructionPoints[i].SetPoint(tempPoint);
		}
	}

	cvShowImage("result", inputImage);
	char ch = cvWaitKey();

	// save
	if(ch == 's' || ch == 'S')
	{
		std::string timeString = windage::Logger::getTimeString();

		// create folder
		char message[100];
		char path[100];
		sprintf_s(path, RECONSTRUCTION_PATH_TEMPLATE, timeString.c_str());
		_mkdir(path);

		// save data
		std::cout << "save reconstruction datas" << std::endl;
		windage::Reconstruction::Exportor exportor;

		sprintf_s(message, RECONSTRUCTION_FILENAME, path);
		windage::Logger* reconstructionLogger = new windage::Logger(message, "txt");

		exportor.SetFunctionName(detector->GetFunctionName());
		exportor.AttatchLogger(reconstructionLogger);
		exportor.SetReconstructionPoints(&reconstructionPoints);
		for(int i=0; i<filenameList.size(); i++)
		{
			sprintf_s(message, IMAGE_FILE_NAME_TEMPLATE, path, i);

			exportor.PushCalibration(calibrationList[0]);
			exportor.PushImageFile(message);
		}
		exportor.DoExport();
		delete reconstructionLogger;
		reconstructionLogger = NULL;
	}

	cvDestroyAllWindows();
}
