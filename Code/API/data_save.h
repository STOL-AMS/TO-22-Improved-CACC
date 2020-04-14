#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <numeric>
#include <comdef.h>  
//#include "dmd_modify.h"

//#include "parameters.h"
#define len_str 1280
#define CONTR_SW 0

#define MERGE_SECTION 332
#define QUIT -1
#define STABLE_THRD 100

//char data_saving_dir[len_str]="a";

char data_saving_detector[len_str] = "a";
bool READ_DETECTOR_AGGREGATED = true;

int *detIds, *sectIds, *nodeIds;

FILE *dfp, *Vehfp, *Vehfp2, *Vehfp3, *HwyVehfp, *HwyVehfp2, *HwyVehTimeSpacefp, *sfp, *systfp, *difp, *sifp, *sigfp, *metfp, *prtsystfp, *merge_fp;


int open_detector(char* data_saving, unsigned int replic, int acc_percent, int cacc_percent);

int init_data_saving(unsigned int replica, int acc_percent, int cacc_percent);
int get_detIds(void);
int save_detector_info();

void DisableCertainBehavior()
{
	int exp_id = ANGConnGetExperimentId();
	const unsigned short *twolanecfString = AKIConvertFromAsciiString(
		"GKExperiment::applyTwoLanesAtt");
	//ANGConnSetAttributeValueBool(ANGConnGetAttribute( twolanecfString ), exp_id, false);
	AKIDeleteUNICODEString(twolanecfString);

	const unsigned short *overtakeString = AKIConvertFromAsciiString(
		"GKExperiment::overtakeAtt");
	//ANGConnSetAttributeValueDouble(ANGConnGetAttribute( overtakeString ), exp_id, 1e8);
	//delete[] overtakeString;
	AKIDeleteUNICODEString(overtakeString);

	const unsigned short *recoverString = AKIConvertFromAsciiString(
		"GKExperiment::recoverAtt");
	//ANGConnSetAttributeValueDouble(ANGConnGetAttribute( recoverString ), exp_id, 1e8);
	//delete[] recoverString;
	AKIDeleteUNICODEString(recoverString);
}


//////////////////////////////////////////////////////////////////////////
// based on the experimental id tell if the simulation file should be run  
// in batch mode
//////////////////////////////////////////////////////////////////////////
bool IsBatchMode()
{
	int expid = ANGConnGetExperimentId();
	const unsigned short *batch_string =
		AKIConvertFromAsciiString(
		"run_batch");
	bool ret_val = ANGConnGetAttributeValueBool
		(ANGConnGetAttribute(batch_string), expid);
	AKIDeleteUNICODEString(batch_string);
	return ret_val;
}


int open_detector(char *data_saving, unsigned int replic, int acc_percent, int cacc_percent)
{

	const int N_files = 50;
	errno_t err;
	FILE* fname;

	if (use_RM == 0)
	{
		std::ifstream infile("C:\\CACC_Simu_Data\\ParameterSet.txt");
		std::string line = "";
		if (infile.is_open() == true)
		{
			std::getline(infile, line);
			std::getline(infile, line);//ignore the first line
		}
		infile.close();


		sprintf_s(data_saving, len_str, "C:\\CACC_Simu_Data\\acc%u_cacc%u\\%u\\detector\\", acc_percent, cacc_percent, replic);

		//if (IsBatchMode() == false)
		//{
		//	sprintf_s(data_saving, len_str, "C:\\CACC_Simu_Data\\acc%u_cacc%u\\%u\\detector\\", acc_percent, cacc_percent, replic);
		//}
		//else
		//{
		//	int total_through = 0;
		//	int off_ramp = 0;
		//	int on_ramp = 0;
		//	read_volume(total_through, on_ramp, off_ramp);
		//	sprintf_s(data_saving, len_str, "C:\\CACC_Simu_Data\\acc%u_cacc%u\\%u\\detector\\detector_run_%u_%u_%u.txt"
		//		, acc_percent, cacc_percent, replic, on_ramp, total_through, off_ramp);
		//}

		// put extra path to indicate the parameters
		if (line != "")
		{
			int total_through = 0;
			//off_ramp from mainlane
			int off_ramp = 0; //off_ramp vehicles can only be manual driven
			//on ramp to mainlane and exit from mainlane
			int on_ramp = 0;//on_ramp vehicles can only be manual driven
			//read_volume(total_through, on_ramp, off_ramp);

			char buffer[10];
			itoa(total_through, buffer, 10);

			char buffer2[10];
			itoa(on_ramp, buffer2, 10);

			sprintf_s(data_saving, len_str, "%s%s_%s_%s", data_saving, line.c_str(), buffer, buffer2);
			sprintf_s(data_saving, len_str, "%s\\detector_run.txt", data_saving);
		}
	}

	err = fopen_s(&fname, data_saving, "w+");
	if (err == 0)
		fclose(fname);
	else
	{
		fprintf(stderr, "File detector_run not open!");

	}

	return 1;
}





//************************************************
//read ramp type attribute from section
//************************************************
int GetRampType(int IdSection)
{
	const unsigned short *increase_DLC_close_ramp_str =
		AKIConvertFromAsciiString("section_ramp_type");
	int sec_type = ((ANGConnGetAttributeValueInt(
		ANGConnGetAttribute(increase_DLC_close_ramp_str), IdSection)));
	AKIDeleteUNICODEString(increase_DLC_close_ramp_str);
	return sec_type;
}

//////////////////////////////////////////////////////////////////////////
// Get relative position of a section
//////////////////////////////////////////////////////////////////////////
double GetSectionRelativePos(int IdSection)
{
	const unsigned short *tempstr =
		AKIConvertFromAsciiString("relative_position");
	double pos = ((ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(tempstr), IdSection)));
	AKIDeleteUNICODEString(tempstr);
	return pos;
}

//*************************************************************
//save detector information such as location, section_id, and lane_id for 
//post-processing in matlab
//***************************************************************
int save_detector_info()
{
	FILE *fp;
	errno_t err;
	char str_tmp[len_str] = "a";

	char detector_folder[len_str] = "a";
	sprintf_s(detector_folder, len_str, "C:\\CACC_Simu_Data\\detector_info.txt");
	err = fopen_s(&fp, detector_folder, "w+");

	if (err != 0)
	{
		AKIPrintString("can not open detector info ");
		return 1;
	}

	for (int i = 0; i < detectorNum; i++)
	{
		int detector_id = AKIDetGetIdDetector(i);
		structA2KDetector info =
			AKIDetGetPropertiesDetectorById(detector_id);

		const unsigned short *tempstr =
			AKIConvertFromAsciiString("GKObject::nameAtt");
		const unsigned short *valuetemp = ANGConnGetAttributeValueString(
			ANGConnGetAttribute(tempstr), detector_id);

		tempstr =
			AKIConvertFromAsciiString("contour_position");
		double position = ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(tempstr), detector_id);
		AKIDeleteUNICODEString(tempstr);

		//use a calculated position
		position = info.InitialPosition + GetSectionRelativePos(info.IdSection);

		std::wstring str((wchar_t*)valuetemp);
		std::string s(str.begin(), str.end());
		int real_detID = atoi(s.c_str());

		int ramptype = GetRampType(info.IdSection); //ramp type
		int number_lane = info.IdLastLane - info.IdFirstLane + 1; //number of lanes

		//write info 
		fprintf(fp, "%d,%d,%d,%d,%d,%d,%.1f,%.1f\n",
			detector_id,
			real_detID,
			ramptype,
			number_lane,
			info.IdSection,
			info.IdFirstLane,
			info.InitialPosition,
			position); //position for contour plot
	}

	fflush(fp);
	fclose(fp);
	return 0;
}


int read_detector(double time)
{
	int i;

	for (i = 0; i < detectorNum; i++)
	{
		int id = AKIDetGetIdDetector(i);
		double speed = AKIDetGetSpeedAggregatedbyId(id, 0);
		if (speed >= 0 && speed < 200)
			fprintf(dfp, "%.1lf,%d,%d,%d,%.1lf,%.1lf,%.1lf\n",
			time,
			id,
			AKIDetGetPropertiesDetectorById(id).IdFirstLane,
			AKIDetGetCounterAggregatedbyId(id, 0),
			speed,
			AKIDetGetTimeOccupedAggregatedbyId(id, 0),
			AKIDetGetDensityAggregatedbyId(id, 0));
		else
			fprintf(dfp, "%.1lf,%d,%d,%d,%.1lf,%.1lf,%.1lf\n",
			time,
			id,
			AKIDetGetPropertiesDetectorById(id).IdFirstLane,
			0,
			0,
			0,
			0);

	}

	return 0;
}

/****************************************
Read sensor data and save to  files
******************************************/
int save_data(double absolute_time)
{
	if (READ_DETECTOR_AGGREGATED)
	{
		if (fabs(absolute_time - last_det_readtime - detInterval) < 1e-3)
		{
			read_detector(absolute_time);   //read and save the data into the file.

			//meanwhile record the space mean speed and density by averaging over all the vehicles on 
			//inner and outer lanes
			//read_space_mean_speed_density(absolute_time, MERGE_SECTION);

			last_det_readtime = absolute_time;
		}
	}

	return 0;
}

int get_detIds()
{
	int i;
	detIds = new int[detectorNum];
	for (i = 0; i < detectorNum; i++)
		detIds[i] = AKIDetGetIdDetector(i);
	return 0;
}


/********************************************
Open files for writing
*********************************************/
int init_data_saving(unsigned int replica, int acc_percent, int cacc_percent)
{
	errno_t err;
	//save_networkinfo(data_saving_networkinfo, replica, acc_percent, cacc_percent);
	save_detector_info();

	if (READ_DETECTOR_AGGREGATED)
	{
		open_detector(data_saving_detector, replica, acc_percent, cacc_percent);     // changed on 07/24/12 XYLu
		err = fopen_s(&dfp, data_saving_detector, "w+");
		if (err != 0)
			READ_DETECTOR_AGGREGATED = false;
	}

	last_det_readtime = 0.0;
	get_detIds();
	return 0;
}






/****************************************
   Closing files after data saving
******************************************/

int finish_data_saving()
{
	delete detIds;
	delete sectIds;
	delete nodeIds;

	if (READ_DETECTOR_AGGREGATED)
	{
		fflush(dfp);
		fclose(dfp);
	}

	//delete all pointers


	return 0;
}


