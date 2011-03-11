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

#include <time.h>

#include "Reconstruction/Utilities/Exportor.h"
using namespace windage;
using namespace Reconstruction;

bool Exportor::DoExport()
{
	if(reconstructionPoints == NULL)
		return false;
	if(calibrationList.size() <= 0)
		return false;
	if(calibrationList.size() != imageFileList.size())
		return false;

	this->logger->log("###############################################################################");
	this->logger->logNewLine();
	this->logger->log("# windage Library Reconstruction datas - generate date :");
	this->logger->log(this->logger->getTimeString().c_str());
	this->logger->log(" #");
	this->logger->logNewLine();
	this->logger->log("#\t\t\t\t\t function : ");
	this->logger->log(this->function_name.c_str());
	this->logger->logNewLine();
	this->logger->log("###############################################################################");
	this->logger->logNewLine();
	this->logger->logNewLine();

	this->logger->log("#####################");
	this->logger->logNewLine();
	this->logger->log("# Calibration datas #");
	this->logger->logNewLine();
	this->logger->log("#####################");
	this->logger->logNewLine();
	this->logger->logNewLine();
	int count = (int)MAX(this->calibrationList.size(), this->imageFileList.size());
	this->logger->log(count);
	this->logger->logNewLine();
	this->logger->logNewLine();
	for(int i=0; i<count; i++)
	{
		this->logger->log("# Camera ");
		this->logger->log(i);
		this->logger->logNewLine();

		this->logger->log("# Intrinsic");
		this->logger->logNewLine();
		this->logger->log(this->calibrationList[i]->GetIntrinsicMatrix());
		this->logger->logNewLine();

		this->logger->log("# Extrinsic");
		this->logger->logNewLine();
		this->logger->log(this->calibrationList[i]->GetExtrinsicMatrix());
		this->logger->logNewLine();

		this->logger->log("# Image file");
		this->logger->logNewLine();
		this->logger->log(this->imageFileList[i].c_str());
		this->logger->logNewLine();
		this->logger->logNewLine();
	}

	this->logger->log("########################");
	this->logger->logNewLine();
	this->logger->log("# Reconstruction datas #");
	this->logger->logNewLine();
	this->logger->log("########################");
	this->logger->logNewLine();
	this->logger->logNewLine();
	count = (int)this->reconstructionPoints->size();
	this->logger->log(count);
	this->logger->logNewLine();
	this->logger->logNewLine();
	for(int i=0; i<count; i++)
	{
		this->logger->log("#############################");
		this->logger->logNewLine();
		this->logger->log("# Reconstruction point :");
		this->logger->log(i);
		this->logger->logNewLine();
		this->logger->log("#############################");
		this->logger->logNewLine();

		this->logger->log((*reconstructionPoints)[i].GetPoint());
//		this->logger->log("\t#point");
		this->logger->logNewLine();
		this->logger->log((*reconstructionPoints)[i].GetObjectID());
//		this->logger->log("\t#object id");
		this->logger->logNewLine();
		this->logger->log((*reconstructionPoints)[i].GetColor());
//		this->logger->log("\t#color");
		this->logger->logNewLine();
		this->logger->logNewLine();

		this->logger->log("# feature point datas #");
		this->logger->logNewLine();
		std::vector<windage::FeaturePoint>* featurePoints = (*reconstructionPoints)[i].GetFeatureList();
		int featureCount = featurePoints->size();
		this->logger->log(featureCount);
//		this->logger->log("\t#feature count");
		this->logger->logNewLine();
		for(int j=0; j<featureCount; j++)
		{
			this->logger->log("# feature point : ");
			this->logger->log(j);
			this->logger->logNewLine();

			this->logger->log((*featurePoints)[j].GetPoint());
			this->logger->logNewLine();
			this->logger->log((*featurePoints)[j].GetObjectID());
			this->logger->logNewLine();
			this->logger->log((*featurePoints)[j].GetColor());
			this->logger->logNewLine();
			this->logger->log((*featurePoints)[j].GetSize());
			this->logger->logNewLine();
			this->logger->log((*featurePoints)[j].GetDir());
			this->logger->logNewLine();
			this->logger->log((*featurePoints)[j].GetDistance());
			this->logger->logNewLine();

			this->logger->log("# descriptor datas");
			this->logger->logNewLine();
			int descriptorCount = (*featurePoints)[j].DESCRIPTOR_DIMENSION;
			this->logger->log(descriptorCount);
			this->logger->logNewLine();

			for(int k=0; k<descriptorCount; k++)
			{
				this->logger->log((*featurePoints)[j].descriptor[k]);
			}
			this->logger->logNewLine();
		}

		this->logger->logNewLine();
	}

	return true;
}