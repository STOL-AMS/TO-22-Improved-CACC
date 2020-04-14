/********************************************************
* Aimsun API for ACC/CACC Simulation in Aimsun		   *
*                                                      *
* The program is developed by the PATH research team   *
* at U.C. Berkeley under the FHWA EARP Project "Using  *
* CACC to Form High-Performance Vehicle Streams"	   *
*                                                      *
* Author: Dr. Hao Liu, Dr. Xiao-Yun Lu, David Kan,     *
* Fang-Chieh Chou, Dr. Dali Wei                        *
*                                                      *
* Usage: Compile the program and it will generate a    *
* DLL and a ILK file. Copy the DLL and ILK file to	   *
* C:\CACC_Simu_Data									   *
*													   *
* 06/19/2017										   *
********************************************************/

#include "AKIProxie.h"

int count = 1;

#include "CIProxie.h"
#include "ANGConProxie.h"
#include "AAPI.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <time.h>       /* time_t, struct tm, time, localtime */
#include <string>
#include <vector>
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find

#include "parameters.h"
#include "sim_data_io.h"
#include "data_save.h"
#include "dmd_modify.h"
//#include "RM.h"
#include "CRM.h"
#include "OptSolver.h"

#define _CRT_SECURE_NO_DEPRECATE



#define ORIGIN_SECTION 11960

char str[len_str];


int AAPILoad()
{
	AKIPrintString("LOAD");
	return 0;
}

//////////////////////////////////////////////////////////////////////////
//set mainline speed limit for all mainline from experimental attribute setting
// this is to prevent we missed setting speed limit for some of the section
//////////////////////////////////////////////////////////////////////////
void SetSpeedLimit()
{
	int expriment_id = ANGConnGetExperimentId();
	const unsigned short *name = AKIConvertFromAsciiString(
		"mainlane_speedlimit");
	int limit = ANGConnGetAttributeValueInt(ANGConnGetAttribute(name), expriment_id);
	AKIDeleteUNICODEString(name);
	if (limit <= 0)
		return;
	else
	{
		int num_sec = AKIInfNetNbSectionsANG();
		for (int i = 0; i < num_sec; i++)
		{
			int id = AKIInfNetGetSectionANGId(i);
			const unsigned short *typestr = AKIConvertFromAsciiString(
				"section_ramp_type");
			int type = ANGConnGetAttributeValueInt(ANGConnGetAttribute(typestr), id);
			AKIDeleteUNICODEString(typestr);
			if (type != 3 && type != 4)
			{
				const unsigned short *speedatt = AKIConvertFromAsciiString(
					"GKSection::speedAtt");
				ANGConnSetAttributeValueDouble(ANGConnGetAttribute(speedatt), id, ((double)limit)*1.6);
				AKIDeleteUNICODEString(speedatt);

			}
			else if (type == 3) // true on ramp
			{
				const unsigned short *speedatt = AKIConvertFromAsciiString(
					"GKSection::speedAtt");
				ANGConnSetAttributeValueDouble(ANGConnGetAttribute(speedatt), id, ((double)50)*1.6);
				AKIDeleteUNICODEString(speedatt);
			}

		}
	}
}

//////////////////////////////////////////////////////////////////////////
// set ramp types for sections to ensure that sections without acceleration lane is 
// not set to on-ramp or off-ramp mainlines
//////////////////////////////////////////////////////////////////////////
void SetRampTypes()
{
	int num_sec = AKIInfNetNbSectionsANG();
	for (int i = 0; i < num_sec; i++)
	{
		int id = AKIInfNetGetSectionANGId(i);
		const unsigned short *typestr = AKIConvertFromAsciiString(
			"section_ramp_type");
		int type = ANGConnGetAttributeValueInt(ANGConnGetAttribute(typestr), id);
		AKIDeleteUNICODEString(typestr);
		if (type == 1 || type == 2)
		{
			if (AKIInfNetGetSectionANGInf(id).nbSideLanes == 0)
			{
				ANGConnSetAttributeValueInt(ANGConnGetAttribute(typestr), id, 0);
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// Set relative position of a section w.r.s. to the beginning
//////////////////////////////////////////////////////////////////////////
void SetRelativePos(int des_id, double pos)
{
	const unsigned short *tempstr =
		AKIConvertFromAsciiString("relative_position");
	((ANGConnSetAttributeValueDouble(
		ANGConnGetAttribute(tempstr), des_id, pos)));
	AKIDeleteUNICODEString(tempstr);
}

//////////////////////////////////////////////////////////////////////////
//Set up the initial relative position for each section
//////////////////////////////////////////////////////////////////////////
void SetSectionInitialPos()
{
	A2KSectionInf inf = AKIInfNetGetSectionANGInf(ORIGIN_SECTION);
	int sec_id = ORIGIN_SECTION;
	//the relative position of the origin section is 0
	double pos = inf.length;
	while (inf.nbTurnings > 0)
	{
		for (int i = 0; i < inf.nbTurnings; i++)
		{
			int des_id = AKIInfNetGetIdSectionANGDestinationofTurning(sec_id, i);
			A2KSectionInf des_inf = AKIInfNetGetSectionANGInf(des_id);
			if (des_inf.nbCentralLanes > 2
				&&
				GetRampType(des_id) != 3
				&&
				GetRampType(des_id) != 4)
			{
				sec_id = des_id;
				inf = des_inf;
				SetRelativePos(des_id, pos);
				pos += inf.length;
				break;
			}
		}
	}
}



//////////////////////////////////////////////////////////////////////////
// determine if this network is specifically for merging networks
// if so we use the merging demand setting method
//////////////////////////////////////////////////////////////////////////
bool UseMergingDemandReading()
{
	int expid = ANGConnGetExperimentId();
	const unsigned short *temp_str = AKIConvertFromAsciiString("read_matrix_demand");
	bool ret = ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(temp_str), expid);
	AKIDeleteUNICODEString(temp_str);
	return ret;
}

int AAPIInit()
{
	AKIPrintString("\tInit");
	ANGConnEnableVehiclesInBatch(true);

	Init();
	//SetExternalParameters(); //No need to call setexternalparamters here because the behaviormodel was created first; 
	//this step has already been done there

	ReadExternalParameters();

	//Read percentages and volume
	double acc_percent = 0;
	double cacc_percent = 0;
	read_precentage(acc_percent, cacc_percent);
	ACC_percent = acc_percent;
	CACC_percent = cacc_percent;


	Init_sim_data_out(acc_percent, cacc_percent);

	if (UseMergingDemandReading())
	{
		//this reading method is for merging network
		//otherwise using the dmd_create_pmes method
		ModifyMatrixDemand(acc_percent, cacc_percent);
	}

	init_data_saving(replication, (int)(acc_percent * 100), (int)(cacc_percent * 100));   //setup and open the folder and file for later saving data.

	//disable the two-lane car following model and overtaking
	DisableCertainBehavior();

	//set section speed limit if attribute available
	SetSpeedLimit();

	//set ramp type for section to ensure that no property of on-ramp(not true on-ramp) or off-ramp is set to sections without side lanes
	SetRampTypes();

	//set position of each section so that we can calculate the relative position of each detector w.r.s. to the begining of the network
	SetSectionInitialPos();

	//initialize c++ random number generator
	int rep_id = ANGConnGetReplicationId();
	srand(rep_id);

	// Initialize energy and mode switch output
	// Initialize the energy output matrices
	int num_secs = AKIInfNetNbSectionsANG();
	int num_rows = 0;
	for (int p = 0; p < num_secs; p++)
	{
		int secID = AKIInfNetGetSectionANGId(p);
		A2KSectionInf secInfo = AKIInfNetGetSectionANGInf(secID);
		double length = secInfo.length;
		int rowNum = ceil(length / Space_Resolution);
		num_rows = num_rows + rowNum;
		section_rows.insert(std::pair<int, int>(secID, num_rows - rowNum));
	}
	num_rows_in_one_step = num_rows;
	double sim_time = AKIGetEndSimTime() - AKIGetIniSimTime();
	num_rows = num_rows*ceil(sim_time / Time_Resolution);
	fuelMatrix.resize(num_rows);
	for (int i = 0; i < num_rows; ++i)
	{
		//Grow Columns by n
		fuelMatrix[i].resize(Column_Numbers);
	}

	// Initialize matrices for the speed advisory algorithm
	double total_dist = 0;
	int total_row_num = 0;
	//std::vector<double> temp_test;

	for (int p = 0; p < linkCount; p++)
	{
		int secID = freewayLinks[p];
		A2KSectionInf secInfo = AKIInfNetGetSectionANGInf(secID);
		double length = secInfo.length;
		int row_num = ceil(length / Space_Resolution_Speed_Advisory);
		for (int q = 1; q <= row_num; q++)
		{

			double dist = 1;
			if (q != row_num)
			{
				dist = Space_Resolution_Speed_Advisory;
			}
			else
			{
				dist = secInfo.length - (q - 1)*Space_Resolution_Speed_Advisory;
			}
			total_dist = total_dist + dist;
			std::vector<double> temp;
			temp.push_back(secID);
			temp.push_back(total_dist);
			temp.push_back(0);
			temp.push_back(0);
			temp.push_back(0);
			speedAdvData.push_back(temp);
			advisorySpeed.push_back(v_max);
			advisorySpeedPrevious.push_back(v_max);

			//temp_test.push_back(secID);
		}

		linkRows.insert(std::pair<int, int>(secID, total_row_num));
		total_row_num = total_row_num + row_num;
	}
	Section_ID_Previous = -1;

	return 0;
}

void RemoveOverflowVehicle()
{
	int no_secs = AKIInfNetNbSectionsANG();
	for (int i = 0; i < no_secs; i++)
	{
		int sid = AKIInfNetGetSectionANGId(i);
		int nveh = AKIVehStateGetNbVehiclesSection(sid, true);
		for (int j = nveh - 1; j >= 0; j--)
		{
			StaticInfVeh s_inf = AKIVehGetVehicleStaticInfSection(sid, j);
			if (s_inf.width == 1)
				//if(AKIVehStateGetVehicleInfSection(sid,j).CurrentSpeed <0 )
			{
				AKIRemoveVehicle(sid, j);
				break;
			}
		}
	}
}

void removeCrashVehicle()
{
	int no_secs = AKIInfNetNbSectionsANG();
	for (int i = 0; i < no_secs; i++)
	{
		int sid = AKIInfNetGetSectionANGId(i);
		int nveh = AKIVehStateGetNbVehiclesSection(sid, true);
		for (int j = 0; j < nveh; j++)
		{
			InfVeh veh_inf = AKIVehStateGetVehicleInfSection(sid, j);
			//A2KSectionInf s_inf = AKIInfNetGetSectionANGInf(sid);

			int VehANGId = ANGConnVehGetGKSimVehicleId(veh_inf.idVeh);
			const unsigned short *temp1 = AKIConvertFromAsciiString("isCrashed");
			bool isCrashed = ANGConnGetAttributeValueBool(ANGConnGetAttribute(temp1), VehANGId);   //the number of the vehicle ahead in the platoon.
			AKIDeleteUNICODEString(temp1);
			if (isCrashed)
			{
				AKIVehSetAsTracked(veh_inf.idVeh);
				int removed = AKIVehTrackedRemove(veh_inf.idVeh);
				int test = removed;
			}
			else if (veh_inf.distance2End < -1)
			{
				AKIVehSetAsTracked(veh_inf.idVeh);
				int removed = AKIVehTrackedRemove(veh_inf.idVeh);
				int test = removed;
			}
			//else if (veh_inf.report != 0)
			//{
			//	int removed = AKIVehTrackedRemove(veh_inf.idVeh);
			//}

			//if (veh_inf.report != 0)
			//{
			//	int removed = AKIVehTrackedRemove(veh_inf.idVeh);
			//}
		}
	}

}

int AAPIManage(double time,
	double timeSta, double timTrans,
	double acicle)
{

	if (UseMergingDemandReading())
		dmd_generate_matrix(time,
		timeSta, timTrans,
		acicle);
	//else
	///*dmd_generate_section(time, 
	//	timeSta, timTrans, 
	//	acicle);*/
	//	dmd_generate_turn_2_OD(time, 
	//		timeSta, timTrans, 
	//		acicle);

	// Configure signal heads==============================================================
	int num_nodes = ECIGetNumberJunctions();
	for (int p = 0; p < num_nodes; p++)
	{
		int node_ID = ECIGetJunctionId(p);

		// read update time
		const unsigned short *update_string = AKIConvertFromAsciiString("Signal_Update_Time");
		double update_time = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(update_string), node_ID);
		AKIDeleteUNICODEString(update_string);



		if (time > update_time)
		{
			if (read_new_SPaT)
			{
				const unsigned short *times_string = AKIConvertFromAsciiString("Signal_Times");
				double rec = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(times_string), node_ID);
				AKIDeleteUNICODEString(times_string);
				int pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
				pos1 = rec / 1e14;
				rec = rec - pos1*1e14;
				pos2 = rec / 1e12;
				rec = rec - pos2*1e12;
				pos3 = rec / 1e10;
				rec = rec - pos3*1e10;
				pos4 = rec / 1e8;
				rec = rec - pos4*1e8;
				pos5 = rec / 1e6;
				rec = rec - pos5*1e6;
				pos6 = rec / 1e4;
				rec = rec - pos6*1e4;
				pos7 = rec / 1e2;
				rec = rec - pos7*1e2;
				pos8 = rec;

				if (signal_optimization_approach == 1)
				{
					pos1 = MAX(0, pos1 - yellow_time - all_red);
					pos2 = MAX(0, pos2 - yellow_time - all_red);
					pos3 = MAX(0, pos3 - yellow_time - all_red);
					pos4 = MAX(0, pos4 - yellow_time - all_red);
					pos5 = MAX(0, pos5 - yellow_time - all_red);
					pos6 = MAX(0, pos6 - yellow_time - all_red);
					pos7 = MAX(0, pos7 - yellow_time - all_red);
					pos8 = MAX(0, pos8 - yellow_time - all_red);
				}

				optimal_signal_times[0] = pos1;
				optimal_signal_times[1] = pos2;
				optimal_signal_times[2] = pos3;
				optimal_signal_times[3] = pos4;
				optimal_signal_times[4] = pos5;
				optimal_signal_times[5] = pos6;
				optimal_signal_times[6] = pos7;
				optimal_signal_times[7] = pos8;
				read_new_SPaT = false;
			}



			//ConfigureSignalHeads(node_ID, time, false);
			int event_flat = ECIIsEventsEnabled(node_ID);
			if (event_flat == 1)
			{
				ECIDisableEvents(node_ID);
			}
			ConfigureSignalHeadsExternalController(node_ID, time, update_time, optimal_signal_times, timeSta, acicle);
		}


	}

	// Configure signal heads==============================================================

	return 0;
}

int AAPIPostManage(double time, double timeSta, double timTrans, double acicle)
{
	// CRM algorithm================================================================================================================================================
	if (time > timTrans && (time - timTrans) == count*detInterval)
	{
		if ((time > dmd_up_t) && (time < dmd_dwn_t))
		{
			if (dmd_modify(time - timTrans) < 1)
				AKIPrintString("Demand change failed!");
		}

		readSection(time - timTrans);
		readDetector_s(time - timTrans);  // read only those for control: each cell with one detector; detector ID assigned

		save_data(time - timTrans);

		if (CRM_On == 1)
		{
			//control
			optControl(time - timTrans, time, timeSta);
		}


		count++;
	}
	// CRM algorithm================================================================================================================================================

	//if(time>timTrans && (time-timTrans)==count*detInterval)
	//{
	//	//readDetector_s(time-timTrans);  
	//	/*if(IsBatchMode()==true)
	//	{*/
	//		/*if(save_data(time-timTrans) == QUIT)
	//		{
	//			AKISetEndSimTime(AKIGetCurrentSimulationTime()+1);
	//		}*/
	//	//}
	//	//else
	//	{
	//		save_data(time-timTrans);
	//		//read_detector(time - timTrans);
	//	}
	//	count++;
	//}
	//RecordVehInf(time-timTrans);  
	//RecordVehInf2(time-timTrans);  

	//if (time>timTrans &&  (int)(time * 10) % 2 != 1)
	//{
	//	//RecordHWYVehInf(time - timTrans);   //for recording vehicles on the cordon line.
	//	//RecordHWYSectionVehInf(time-timTrans);
	//	//RecordHWYVehDrivingMode(time - timTrans);
	//}

	// Configure signal heads==============================================================
	//int num_nodes = ECIGetNumberJunctions();
	//for (int p = 0; p < num_nodes; p++)
	//{
	//	int node_ID = ECIGetJunctionId(p);

	//	// read update time
	//	const unsigned short *update_string = AKIConvertFromAsciiString("Signal_Update_Time");
	//	double update_time = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(update_string), node_ID);
	//	AKIDeleteUNICODEString(update_string);

	//	if (abs(update_time - acicle - time) < 0.00000001)
	//	{
	//		// Configure all signal heads, except for the last two
	//		// The last two need to be configured in the next update interval
	//		ConfigureSignalHeads(node_ID, time, true);
	//	}
	//	//else if (abs(update_time - time) < 0.00000001)
	//	//{
	//	//	ConfigureSignalHeads(node_ID, time, false);
	//	//}

	//}

	// Configure signal heads==============================================================

	//remove vehicles that are outside of section
	RemoveOverflowVehicle();
	removeCrashVehicle();

	if (SPD_ADV_On == 1)
	{
		// Determine speed advisory
		if (time - updateTime < 0.0001)
		{
			CollectSpeedAdvisoryData();
		}
		else
		{
			updateTime = updateTime + Time_Resolution_Speed_Advisory;
			DetermineAdvisorySpeed();

			// Reset speedAdvData
			for (int p = 0; p < speedAdvData.size(); p++)
			{
				speedAdvData[p][2] = 0;
				speedAdvData[p][3] = 0;
				speedAdvData[p][4] = 0;
			}
			CollectSpeedAdvisoryData();


		}

		// Setup desired speed for vehicles based on the advisory speed
		SetAdvisorySpeed();
	}


	//Record energy and mode switch data for vehicles in sections
	int no_secs = AKIInfNetNbSectionsANG();
	for (int i = 0; i < no_secs; i++)
	{
		int sid = AKIInfNetGetSectionANGId(i);
		int nveh = AKIVehStateGetNbVehiclesSection(sid, true);
		A2KSectionInf sec_inf = AKIInfNetGetSectionANGInf(sid);
		for (int j = 0; j < nveh; j++)
		{
			// Get vehicle operation information
			InfVeh veh_inf = AKIVehStateGetVehicleInfSection(sid, j);

			StaticInfVeh veh_static_inf = AKIVehGetVehicleStaticInfSection(sid, j);
			double vsa = veh_static_inf.giveWayTime / 3.6;

			bool truck_flag = false;
			if (veh_inf.type > 1)
			{
				double veh_length = AKIVehGetMinLengthVehType(veh_inf.type);
				if (veh_length > 10)
				{
					truck_flag = true;
				}
			}


			double v = veh_inf.CurrentSpeed / 3.6;
			double a = (veh_inf.PreviousSpeed - veh_inf.CurrentSpeed) / 3.6 / AKIGetSimulationStepTime();
			int veh_ID = veh_inf.idVeh;
			int vehSecID = veh_inf.idSection;

			// VSA compliance observed
			double compliance_free = 0;
			double total_free = 0;
			double compliance_obs = 0;
			if (vsa < 1000)
			{
				if (abs(v - vsa) <= vsa*0.05)
				{
					// Consider vehicle complied if speed within plus/minus 5% of the VSA
					compliance_obs = AKIGetSimulationStepTime();
				}

				// Preceding vehicle
				if (j > 0)
				{
					InfVeh veh_inf_pre = AKIVehStateGetVehicleInfSection(sid, j - 1);
					double v_pre = veh_inf_pre.CurrentSpeed / 3.6;
					double pos = veh_inf.CurrentPos;
					double pos_pre = veh_inf_pre.CurrentPos;
					double lane = veh_inf.numberLane;
					double lane_pre = veh_inf_pre.numberLane;

					if (lane == lane_pre && pos_pre > pos + 20 && v_pre >= v)
					{
						// Vehicle is considered complied freely if it is far from the preceding vehicle
						total_free = AKIGetSimulationStepTime();
						if (abs(v - vsa) <= vsa*0.05)
						{
							// Consider vehicle complied if speed within plus/minus 5% of the VSA
							compliance_free = AKIGetSimulationStepTime();
						}

					}

				}
			}





			// Specify row and column IDs for results recording
			double timeID = ceil(AKIGetCurrentSimulationTime() / Time_Resolution);
			double distID = MAX(1, ceil(veh_inf.CurrentPos / Space_Resolution));
			int rowID = (timeID - 1)*num_rows_in_one_step + section_rows.at(vehSecID) + distID - 1;
			timeID = timeID * Time_Resolution;
			if (distID == ceil(sec_inf.length / Space_Resolution))
			{
				distID = ceil(sec_inf.length);
			}
			else
			{
				distID = distID * Space_Resolution;
			}
			// Get the energy and mode switch data to be stored
			int veh_Aimsun_ID = ANGConnVehGetGKSimVehicleId(veh_ID);
			const unsigned short *string_info = AKIConvertFromAsciiString("CACC_String_Info");
			int rec = ANGConnGetAttributeValueInt(ANGConnGetAttribute(string_info), veh_Aimsun_ID);
			AKIDeleteUNICODEString(string_info);
			int pos1, pos2, pos3, pos4;
			pos1 = rec / 1000;
			pos2 = (rec - pos1 * 1000) / 100;
			pos3 = (rec - pos1 * 1000 - pos2 * 100) / 10;
			pos4 = rec - pos1 * 1000 - pos2 * 100 - pos3 * 10;

			fuelVMT outFuelVMT = computeEnergy(v, a, truck_flag);

			std::vector<double> tempVec = fuelMatrix[rowID];
			tempVec.at(0) = timeID;
			tempVec.at(1) = vehSecID;
			tempVec.at(2) = distID;
			tempVec.at(3) = fuelMatrix[rowID][3] + outFuelVMT.fuel;
			tempVec.at(4) = fuelMatrix[rowID][4] + outFuelVMT.VMT;
			tempVec.at(5) = fuelMatrix[rowID][5] + outFuelVMT.VHT;
			tempVec.at(6) = fuelMatrix[rowID][6] + outFuelVMT.fuel_VT;
			tempVec.at(7) = fuelMatrix[rowID][7] + 1;
			tempVec.at(8) = fuelMatrix[rowID][8] + pos1;
			tempVec.at(9) = fuelMatrix[rowID][9] + pos2;
			tempVec.at(10) = fuelMatrix[rowID][10] + outFuelVMT.VMT * pos2;
			tempVec.at(11) = fuelMatrix[rowID][11] + pos3;
			tempVec.at(12) = fuelMatrix[rowID][12] + pos4;
			tempVec.at(13) = fuelMatrix[rowID][13] + compliance_obs;
			tempVec.at(14) = fuelMatrix[rowID][14] + compliance_free;
			tempVec.at(15) = fuelMatrix[rowID][15] + total_free;

			fuelMatrix[rowID] = tempVec;

			//double d = tempVec.at(5);
			//double b = tempVec.at(13);
			//double tt = 0;
		}
	}

	// For vehicles in the nodes
	int no_nodes = AKIInfNetNbJunctions();
	for (int i = 0; i < no_nodes; i++)
	{
		int nid = AKIInfNetGetJunctionId(i);
		int nveh = AKIVehStateGetNbVehiclesJunction(nid);

		for (int j = 0; j < nveh; j++)
		{
			// Get vehicle operation information
			InfVeh veh_inf = AKIVehStateGetVehicleInfJunction(nid, j);

			bool truck_flag = false;
			if (veh_inf.type > 1)
			{
				double veh_length = AKIVehGetMinLengthVehType(veh_inf.type);
				if (veh_length > 10)
				{
					truck_flag = true;
				}
			}

			double v = veh_inf.CurrentSpeed / 3.6;
			double a = (veh_inf.PreviousSpeed - veh_inf.CurrentSpeed) / 3.6 / AKIGetSimulationStepTime();
			int veh_ID = veh_inf.idVeh;
			int vehSecID = veh_inf.idSectionTo;
			// Specify row and column IDs for results recording
			double timeID = ceil(AKIGetCurrentSimulationTime() / Time_Resolution);
			double distID = 1;
			int rowID = (timeID - 1)*num_rows_in_one_step + section_rows.at(vehSecID) + distID - 1;
			timeID = timeID * Time_Resolution;
			distID = distID * Space_Resolution;

			// Get the energy and mode switch data to be stored
			int veh_Aimsun_ID = ANGConnVehGetGKSimVehicleId(veh_ID);
			const unsigned short *string_info = AKIConvertFromAsciiString("CACC_String_Info");
			int rec = ANGConnGetAttributeValueInt(ANGConnGetAttribute(string_info), veh_Aimsun_ID);
			AKIDeleteUNICODEString(string_info);
			int pos1, pos2, pos3, pos4;
			pos1 = rec / 1000;
			pos2 = (rec - pos1 * 1000) / 100;
			pos3 = (rec - pos1 * 1000 - pos2 * 100) / 10;
			pos4 = rec - pos1 * 1000 - pos2 * 100 - pos3 * 10;

			fuelVMT outFuelVMT = computeEnergy(v, a, truck_flag);

			std::vector<double> tempVec = fuelMatrix[rowID];
			tempVec.at(0) = timeID;
			tempVec.at(1) = vehSecID;
			tempVec.at(2) = distID;
			tempVec.at(3) = fuelMatrix[rowID][3] + outFuelVMT.fuel;
			tempVec.at(4) = fuelMatrix[rowID][4] + outFuelVMT.VMT;
			tempVec.at(5) = fuelMatrix[rowID][5] + outFuelVMT.VHT;
			tempVec.at(6) = fuelMatrix[rowID][6] + outFuelVMT.fuel_VT;
			tempVec.at(7) = fuelMatrix[rowID][7] + 1;
			tempVec.at(8) = fuelMatrix[rowID][8] + pos1;
			tempVec.at(9) = fuelMatrix[rowID][9] + pos2;
			tempVec.at(10) = fuelMatrix[rowID][10] + outFuelVMT.VMT * pos2;
			tempVec.at(11) = fuelMatrix[rowID][11] + pos3;
			tempVec.at(12) = fuelMatrix[rowID][12] + pos4;

			fuelMatrix[rowID] = tempVec;

		}
	}

	return 0;
}


int AAPIFinish()
{

	Finish();
	finish_data_saving();
	Finish_sim_data_out();
	/*AKIPrintString("\tFinish");*/

	outputEnergy();

	return 0;
}

// Collect vehicle VMT and VHT data for determining the advisory speed
void CollectSpeedAdvisoryData()
{
	for (int p = 0; p < linkCount; p++)
	{
		int sid = freewayLinks[p];
		int nveh = AKIVehStateGetNbVehiclesSection(sid, true);
		A2KSectionInf sec_inf = AKIInfNetGetSectionANGInf(sid);
		int row_id = linkRows[sid];
		for (int j = 0; j < nveh; j++)
		{
			// Get vehicle operation information
			InfVeh veh_inf = AKIVehStateGetVehicleInfSection(sid, j);

			int veh_type = AKIVehTypeGetIdVehTypeANG(veh_inf.type);

			int VehANGId = ANGConnVehGetGKSimVehicleId(veh_inf.idVeh);
			const unsigned short *temp1 = AKIConvertFromAsciiString("Connected_Vehicle");
			int connected_flag = ANGConnGetAttributeValueInt(ANGConnGetAttribute(temp1), VehANGId);
			AKIDeleteUNICODEString(temp1);

			int id = row_id + MAX(0, floor(veh_inf.CurrentPos / Space_Resolution_Speed_Advisory));
			double vht = AKIGetSimulationStepTime();
			if (veh_type == CV_TYPE || veh_type == CAV_TYPE || connected_flag == 1)
			{
				double v = veh_inf.CurrentSpeed / 3.6;
				double a = (veh_inf.PreviousSpeed - veh_inf.CurrentSpeed) / 3.6 / AKIGetSimulationStepTime();
				// Specify row and column IDs for results recording			
				double vmt = v * AKIGetSimulationStepTime() + 0.5 * a * AKIGetSimulationStepTime() *AKIGetSimulationStepTime();
				
				speedAdvData[id][2] = speedAdvData[id][2] + vmt;
				speedAdvData[id][3] = speedAdvData[id][3] + vht;			
				//double sec_len = speedAdvData[id][1];
				//if (id > 0)
				//{
				//	sec_len = speedAdvData[id][1] - speedAdvData[id - 1][1];
				//}				
				//occ = occ / sec_len*4.5; // assume detector length of 4.5 m

			}

			StaticInfVeh veh_static_inf = AKIVehGetVehicleStaticInfSection(sid, j);
			double detector_loc = MAX(0, floor(veh_inf.CurrentPos / Space_Resolution_Speed_Advisory))*Space_Resolution_Speed_Advisory + 4.5 + veh_static_inf.length;
			double occ = 0;
			if (veh_inf.CurrentPos <= detector_loc && veh_inf.numberLane > sec_inf.nbSideLanes)
			{
				occ = vht / Time_Resolution_Speed_Advisory;
			}
			speedAdvData[id][4] = speedAdvData[id][4] + occ / sec_inf.nbCentralLanes;
		}
	}
}

void DetermineAdvisorySpeed()
{
	std::vector<double> temp_speed_adv(speedAdvData.size(), { 0 });
	std::vector<double> temp_speed;

	// Model parameters
	double occ_critical = 0.5;
	double sigma_o1 = 6; // 6
	double sigma_o2 = 6;
	double alpha = 0.6;
	double pho1 = 0.65;
	double pho2 = 0.2;
	double pho3 = 0.15;

	int upstream_sec_index = -1;
	double speed_pre = v_max * 2;
	double speed_congested_region = 0;

	// Test for short network=================================
	std::vector<int>::iterator it = bottleneckLinks.begin();
	while (it != bottleneckLinks.end())
	{
		int sec_id = *it;
		int p = linkRows[sec_id];
		double speed_low = v_max;
		double occ = 0;
		double speed_pre = 0;

		if (speedAdvData[p][3] != 0)
		{
			speed_low = speedAdvData[p][2] / speedAdvData[p][3];
			occ = speedAdvData[p][4];
		}

		double sec_len = Space_Resolution_Speed_Advisory;
		if (p > 0)
		{
			sec_len = speedAdvData[p][1] - speedAdvData[p - 1][1];
		}
		else
		{
			sec_len = speedAdvData[p][1];
		}

		if (sec_len < 70 && p < speedAdvData.size() - 1)
		{
			// Get a smoothed speed for short sections (at the end of a section)
			speed_low = (speedAdvData[p][2] / speedAdvData[p][3] + speedAdvData[p - 1][2] / speedAdvData[p - 1][3]
				+ speedAdvData[p + 1][2] / speedAdvData[p + 1][3]) / 3;
		}

		if (speed_low < Threshold_Speed)
		{
			// The section with the local mimimum speed is identified
			// This is the downstream section where the speed advisory is implemented
			int	q = p;
			upstream_sec_index = p;
			while (q >= 0)
			{
				int coef = floor((upstream_sec_index - q) / num_secs);
				double adv_spd = v_max;
				if (q >= upstream_sec_index - 4)
				{
					// bottleneck area
					adv_spd = v_max;
				}
				// Occupancy-based feedback control
				else 
				{
					speed_pre = advisorySpeed[q];

					if (coef == 0)
					{
						double occ_weighted = 0;
						double occ1 = 0, occ2 = 0, occ3 = 0, spd1 = 0;

						if (coef == 0)
						{
							occ1 = speedAdvData[upstream_sec_index - coef*num_secs][4];
							spd1 = speedAdvData[upstream_sec_index - 5][2] / speedAdvData[upstream_sec_index - 5][3];
							occ2 = occ1;
							occ3 = occ1;
						}
						else if (coef == 1)
						{
							occ1 = speedAdvData[upstream_sec_index - coef*num_secs][4];
							occ2 = speedAdvData[upstream_sec_index - (coef - 1)*num_secs][4];;
							occ3 = occ1;
						}
						else
						{
							occ1 = speedAdvData[upstream_sec_index - coef*num_secs][4];
							occ2 = speedAdvData[upstream_sec_index - (coef - 1)*num_secs][4];
							occ3 = speedAdvData[upstream_sec_index - (coef - 2)*num_secs][4];
						}


						// Compute the average occupancy for the current speed control segment
						//int num_sec = (upstream_sec_index - coef*num_secs) - MAX(0, upstream_sec_index - (coef + 1)*num_secs + 1) + 1;
						//for (int n = MAX(0, upstream_sec_index - (coef + 1)*num_secs + 1); n <= upstream_sec_index - coef*num_secs; n++)
						//{
						//	occ1 = occ1 + speedAdvData[n][4] / num_sec;
						//}
						//// Compute the average occupancy for the next speed control segment
						//if (coef > 0)
						//{
						//	num_sec = (upstream_sec_index - (coef - 1)*num_secs) - MAX(0, upstream_sec_index - coef*num_secs + 1) + 1;
						//	for (int n = MAX(0, upstream_sec_index - (coef + 0)*num_secs + 1); n <= upstream_sec_index - (coef - 1)*num_secs; n++)
						//	{
						//		occ2 = occ2 + speedAdvData[n][4] / num_sec;
						//	}
						//}
						//else
						//{
						//	occ2 = occ1;
						//}

						//// Compute the average occupancy for the third downstream speed control segment
						//if (coef > 1)
						//{
						//	num_sec = (upstream_sec_index - (coef - 2)*num_secs) - MAX(0, upstream_sec_index - (coef - 1)*num_secs + 1) + 1;
						//	for (int n = MAX(0, upstream_sec_index - (coef - 1)*num_secs + 1); n <= upstream_sec_index - (coef - 2)*num_secs; n++)
						//	{
						//		occ3 = occ3 + speedAdvData[n][4] / num_sec;
						//	}
						//}
						//else
						//{
						//	occ3 = occ1;
						//}

						occ_weighted = pho1*occ1 + pho2*occ2 + pho3*occ3;

						if (occ_weighted < occ_critical)
						{
							adv_spd = MIN(v_max, MAX(v_min, spd1 + sigma_o1*(occ_critical - occ_weighted)));
						}
						else
						{
							adv_spd = MIN(v_max, MAX(v_min, spd1 + sigma_o2*(occ_critical - occ_weighted)));
						}
					}
					else
					{
						double speed_step = MIN(v_max, MAX(v_min, temp_speed_adv[upstream_sec_index - 5] + coef*Speed_Increase_Step));
						adv_spd = speed_step;
					}

					// Spatial bound 
					//if (adv_spd - temp_speed_adv[q + 1] > Speed_Increase_Step)
					//{
					//	adv_spd = MIN(v_max, MAX(v_min, temp_speed_adv[q + 1] + Speed_Increase_Step));
					//}
					//else if (adv_spd - temp_speed_adv[q + 1] < -Speed_Increase_Step)
					//{
					//	adv_spd = MIN(v_max, MAX(v_min, temp_speed_adv[q + 1] - Speed_Increase_Step));
					//}

					

					// Temporal bound
					if (adv_spd - speed_pre > Speed_Increase_Step)
					{
						adv_spd = MIN(v_max, MAX(v_min, speed_pre + Speed_Increase_Step));
					}
					else if (adv_spd - speed_pre < -Speed_Increase_Step)
					{
						adv_spd = MIN(v_max, MAX(v_min, speed_pre - Speed_Increase_Step));
					}
				}
				//else
				//{
				//	double speed_step = MIN(v_max, MAX(v_min, temp_speed_adv[upstream_sec_index - 3] + coef*Speed_Increase_Step));
				//	double occ_weighted = 0;
				//	double occ1 = 0, occ2 = 0, occ3 = 0;
				//	speed_pre = advisorySpeed[q];

				//	// Compute the average occupancy for the current speed control segment
				//	int num_sec = (upstream_sec_index - coef*num_secs) - MAX(0, upstream_sec_index - (coef + 1)*num_secs + 1) +1;
				//	for (int n = MAX(0, upstream_sec_index - (coef + 1)*num_secs + 1); n <= upstream_sec_index - coef*num_secs; n++)
				//	{
				//		occ1 = occ1 + speedAdvData[n][4] / num_sec;
				//	}
				//	// Compute the average occupancy for the next speed control segment
				//	num_sec = (upstream_sec_index - (coef - 1)*num_secs) - MAX(0, upstream_sec_index - coef*num_secs + 1) + 1;
				//	for (int n = MAX(0, upstream_sec_index - (coef + 0)*num_secs + 1); n <= upstream_sec_index - (coef - 1)*num_secs; n++)
				//	{
				//		occ2 = occ2 + speedAdvData[n][4] / num_sec;
				//	}
				//	// Compute the average occupancy for the third downstream speed control segment
				//	if (coef > 1)
				//	{
				//		num_sec = (upstream_sec_index - (coef - 2)*num_secs) - MAX(0, upstream_sec_index - (coef - 1)*num_secs + 1) + 1;
				//		for (int n = MAX(0, upstream_sec_index - (coef - 1)*num_secs + 1); n <= upstream_sec_index - (coef - 2)*num_secs; n++)
				//		{
				//			occ3 = occ3 + speedAdvData[n][4] / num_sec;
				//		}
				//	}
				//	else
				//	{
				//		occ3 = occ1;
				//	}


				//	if (q < speedAdvData.size() - 2)
				//	{
				//		occ_weighted = pho1*occ1 + pho2*occ2 + pho3*occ3;
				//	}
				//	if (occ1 > occ_weighted)
				//	{
				//		adv_spd = MIN(v_max, MAX(v_min, speed_step + alpha*(occ1 - occ_weighted)));
				//	}
				//	else
				//	{
				//		adv_spd = speed_step;
				//	}

				//	//// Spatial bound 
				//	//if (adv_spd - temp_speed_adv[q + 1] > Speed_Increase_Step)
				//	//{
				//	//	adv_spd = MIN(v_max, MAX(v_min, temp_speed_adv[q + 1] + Speed_Increase_Step));
				//	//}
				//	//else if (adv_spd - temp_speed_adv[q + 1] < -Speed_Increase_Step)
				//	//{
				//	//	adv_spd = MIN(v_max, MAX(v_min, temp_speed_adv[q + 1] - Speed_Increase_Step));
				//	//}

				//	// Temporal bound
				//	if (adv_spd - speed_pre > Speed_Increase_Step)
				//	{
				//		adv_spd = MIN(v_max, MAX(v_min, speed_pre + Speed_Increase_Step));
				//	}
				//	else if (adv_spd - speed_pre < -Speed_Increase_Step)
				//	{
				//		adv_spd = MIN(v_max, MAX(v_min, speed_pre - Speed_Increase_Step));
				//	}
				//}

				// Speed-based method============================================================
				//else
				//{
				//	adv_spd = MIN(v_max, MAX(v_min, speed_low*Speed_Increase_Coefficient + coef*Speed_Increase_Step));
				//	// adv_spd = MIN(Free_Flow_Speed, Min_Advisory_Speed + (coef - 1)*Speed_Increase_Step);
				//}
				// Speed-based method============================================================

				if (temp_speed_adv[q] > 0 && temp_speed_adv[q] < adv_spd)
				{
					// Already set by previous steps
					break;
				}

				temp_speed_adv[q] = adv_spd;
				q--;
			}

		}

		// Set the remaining subsections
		int	q = speedAdvData.size() - 1;
		while (q >= 0)
		{
			if (temp_speed_adv[q] > 0)
			{
				// Already set by previous steps
				break;
			}
			temp_speed_adv[q] = v_max;
			q--;
		}

		it++;
	}



	//========================================================*/

	//for (int p = speedAdvData.size() - 1; p >= 0; p--)
	//{
	//	double speed = Free_Flow_Speed;
	//	if (speedAdvData[p][3] != 0)
	//	{
	//		speed = speedAdvData[p][2] / speedAdvData[p][3];
	//	}
	//	
	//	double sec_len = 100;
	//	if (p > 0)
	//	{
	//		sec_len = speedAdvData[p][1] - speedAdvData[p-1][1];
	//	}

	//	if (sec_len < 70 && p < speedAdvData.size() - 1)
	//	{
	//		// Get a smoothed speed for short sections (at the end of a section)
	//		speed = (speedAdvData[p][2] / speedAdvData[p][3] + speedAdvData[p-1][2] / speedAdvData[p-1][3]
	//			+ speedAdvData[p+1][2] / speedAdvData[p+1][3]) / 3;
	//	}

	//	if (advisorySpeedPrevious[p] > Free_Flow_Speed)
	//	{
	//		advisorySpeedPrevious[p] = speed;
	//	}
	//	double v_pre = advisorySpeedPrevious[p];

	//	if (p == speedAdvData.size() - 1)
	//	{
	//		temp_speed_adv[p] = Free_Flow_Speed;
	//		continue;
	//	}

	//	double occ = speedAdvData[p][4];
	//	double occ_down = speedAdvData[p + 1][4];
	//	double adv_spd = v_pre - Speed_Gain*(occ_down - occ);
	//	if (adv_spd - v_pre > Speed_Increase_Step)
	//	{
	//		adv_spd = MIN(Free_Flow_Speed, v_pre + Speed_Increase_Step);
	//	}
	//	else if (adv_spd - v_pre < -Speed_Increase_Step)
	//	{
	//		adv_spd = MAX(Min_Advisory_Speed, v_pre - Speed_Increase_Step);
	//	}
	//	else
	//	{
	//		adv_spd = MAX(Min_Advisory_Speed, MIN(adv_spd, Free_Flow_Speed));
	//	}

	//	if (p < speedAdvData.size() - 2)
	//	{
	//		double v_down = temp_speed_adv[p + 1];
	//		if (adv_spd - v_down > Speed_Increase_Step)
	//		{
	//			adv_spd = MIN(Free_Flow_Speed, v_down + Speed_Increase_Step);
	//		}
	//		else if (adv_spd - v_down < -Speed_Increase_Step)
	//		{
	//			adv_spd = MAX(Min_Advisory_Speed, v_down - Speed_Increase_Step);
	//		}
	//		else
	//		{
	//			adv_spd = MAX(Min_Advisory_Speed, MIN(adv_spd, Free_Flow_Speed));
	//		}
	//	}

	//	temp_speed_adv[p] = adv_spd;

	//	temp_speed.push_back(speed);

	//}

	advisorySpeedPrevious = advisorySpeed;
	advisorySpeed = temp_speed_adv;

}

void SetAdvisorySpeed()
{
	for (int p = 0; p < linkCount; p++)
	{
		int sid = freewayLinks[p];
		int nveh = AKIVehStateGetNbVehiclesSection(sid, true);
		A2KSectionInf sec_inf = AKIInfNetGetSectionANGInf(sid);
		int row_id = linkRows[sid];
		for (int j = 0; j < nveh; j++)
		{
			// Get vehicle operation information
			InfVeh veh_inf = AKIVehStateGetVehicleInfSection(sid, j);
			int veh_id = veh_inf.idVeh;

			int VehANGId = ANGConnVehGetGKSimVehicleId(veh_inf.idVeh);
			const unsigned short *temp1 = AKIConvertFromAsciiString("Connected_Vehicle");
			int connected_flag = ANGConnGetAttributeValueInt(ANGConnGetAttribute(temp1), VehANGId);
			AKIDeleteUNICODEString(temp1);

			int veh_type = AKIVehTypeGetIdVehTypeANG(veh_inf.type);
			// Specify row and column IDs for results recording
			int id = row_id + MAX(0, floor(veh_inf.CurrentPos / Space_Resolution_Speed_Advisory));
			double adv_speed = advisorySpeed[id] * 3.6;
			double vsa = adv_speed;
			StaticInfVeh veh_static_inf = AKIVehGetVehicleStaticInfSection(sid, j);
			veh_static_inf.giveWayTime = vsa; // Used to record the VSA level
			unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			std::default_random_engine generator(seed);
			if (veh_type == CV_TYPE || veh_type == CAV_TYPE || connected_flag == 1)
			{

				if (connected_flag == 1 && veh_type != CAV_TYPE) // random_compliance between 0 and 1
				{
					if (adv_speed < 15.65 * 3.6) // 35 mph
					{
						// Log-normal distribution under low speed cases					
						std::uniform_real_distribution<double> distribution(0.0, 1.0);
						int row_id = floor(distribution(generator) / 0.01);
						double speed_error = VSA_Compliance_Low[row_id] * random_compliance*3.6;

						// Dispersion of the desired speed
						//std::default_random_engine generator1;
						std::normal_distribution<double> distribution1(0, 10); // Same as the normal human drivers
						double std = 0;
						std = distribution1(generator); // truncated normal
						if (std > 10)
						{
							std = 10;
						}
						else if (std < -10)
						{
							std = -10;
						}
						adv_speed = MIN(sec_inf.speedLimit, MAX(v_min*3.6, adv_speed + speed_error + std));

					}
					else
					{
						//std::default_random_engine generator;
						std::uniform_real_distribution<double> distribution(0.0, 1.0);
						int row_id = floor(distribution(generator) / 0.01);
						double speed_error = VSA_Compliance_High[row_id] * random_compliance*3.6;

						// Dispersion of the desired speed
						//std::default_random_engine generator1;
						std::normal_distribution<double> distribution1(0, 10); // Same as the normal human drivers
						double std = 0;
						std = distribution1(generator); // truncated normal
						if (std > 10)
						{
							std = 10;
						}
						else if (std < -10)
						{
							std = -10;
						}
						adv_speed = MIN(sec_inf.speedLimit, MAX(v_min*3.6, adv_speed + speed_error + std));
					}

				}
				
				// A driver cannot decrease the desired speed too much from her original desired speed
				// Given a 0.1s update interval, this constraint makes sure that the deceleration will 
				// not be larger than 1.5 m/s2 when the speed is 30 m/s
				double ori_desired_speed = veh_static_inf.maxDesiredSpeed;
				if (adv_speed - ori_desired_speed > ori_desired_speed*0.01)
				{
					adv_speed = ori_desired_speed + ori_desired_speed*0.01;
				}
				else if (adv_speed - ori_desired_speed < -ori_desired_speed*0.005)
				{
					adv_speed = ori_desired_speed - ori_desired_speed*0.005;
				}
				veh_static_inf.maxDesiredSpeed = adv_speed; // Desired speed affected by the VSA							
			}
			int modify_flag = AKIVehSetVehicleStaticInfSection(sid, j, veh_static_inf);
			int tt = 0;
		}
	}
}

// Configure signal heads using the external signal controller
bool ConfigureSignalHeadsExternalController(int node_ID, double current_time, double cycle_start_time, int optimal_signal_times[], double time_sta, double sim_step)
{
	bool signal_set = true;

	// read signal times
	int pos1 = optimal_signal_times[0], pos2 = optimal_signal_times[1],
		pos3 = optimal_signal_times[2], pos4 = optimal_signal_times[3],
		pos5 = optimal_signal_times[4], pos6 = optimal_signal_times[5],
		pos7 = optimal_signal_times[6], pos8 = optimal_signal_times[7];


	// Configure signal heads
	int totalSignalGroups = ECIGetNumberSignalGroups(node_ID);
	int green_num = 0;
	for (int p = 1; p <= totalSignalGroups; p++)
	{
		// Assign signal times
		const unsigned short *signal_ID_pt = ECIGetExternalIdofSignalGroup(node_ID, p);
		bool nonChar;
		const char *signal_name_char = AKIConvertToAsciiString(signal_ID_pt, true, &nonChar);
		std::string signal_name(signal_name_char);
		double phase_start = 0;
		double phase_end = 0;
		int change_flag = -1;
		if (signal_name == "Signal 1")
		{
			phase_start = cycle_start_time + pos3 + (pos3 > 0)*(yellow_time + all_red) + pos4 + (pos4 > 0)*(yellow_time + all_red);
			phase_end = phase_start + pos1 + (pos1 > 0)*(yellow_time + all_red);
			green_num = 0;
		}
		else if (signal_name == "Signal 2")
		{
			phase_start = cycle_start_time + pos3 + (pos3 > 0)*(yellow_time + all_red) + pos4 + (pos4 > 0)*(yellow_time + all_red) + pos1 + (pos1 > 0)*(yellow_time + all_red);
			phase_end = phase_start + pos2 + (pos2 > 0)*(yellow_time + all_red);
			green_num = 1;
		}
		else if (signal_name == "Signal 3")
		{
			phase_start = cycle_start_time;
			phase_end = phase_start + pos3 + (pos3 > 0)*(yellow_time + all_red);
			green_num = 2;
		}
		else if (signal_name == "Signal 4")
		{
			phase_start = cycle_start_time + pos3 + (pos3 > 0)*(yellow_time + all_red);
			phase_end = phase_start + pos4 + (pos4 > 0)*(yellow_time + all_red);
			green_num = 3;
		}
		else if (signal_name == "Signal 5")
		{
			phase_start = cycle_start_time + pos7 + (pos7 > 0)*(yellow_time + all_red) + pos8 + (pos8 > 0)*(yellow_time + all_red);
			phase_end = phase_start + pos5 + (pos5 > 0)*(yellow_time + all_red);
			green_num = 4;
		}
		else if (signal_name == "Signal 6")
		{
			phase_start = cycle_start_time + pos7 + (pos7 > 0)*(yellow_time + all_red) + pos8 + (pos8 > 0)*(yellow_time + all_red) + pos5 + (pos5 > 0)*(yellow_time + all_red);
			phase_end = phase_start + pos6 + (pos6 > 0)*(yellow_time + all_red);
			green_num = 5;
		}
		else if (signal_name == "Signal 7")
		{
			phase_start = cycle_start_time;
			phase_end = phase_start + pos7 + (pos7 > 0)*(yellow_time + all_red);
			green_num = 6;
		}
		else if (signal_name == "Signal 8")
		{
			phase_start = cycle_start_time + pos7 + (pos7 > 0)*(yellow_time + all_red);
			phase_end = phase_start + pos8 + (pos8 > 0)*(yellow_time + all_red);
			green_num = 7;
		}

		// Determine signal states
		if (current_time > phase_start && current_time <= phase_end)
		{
			double dur = current_time - phase_start;

			if (dur <= optimal_signal_times[green_num])
			{
				// Green
				change_flag = ECIChangeSignalGroupState(node_ID, p, 1, time_sta, current_time, sim_step);
			}
			else if (dur <= optimal_signal_times[green_num] + yellow_time)
			{
				// Yellow
				change_flag = ECIChangeSignalGroupState(node_ID, p, 2, time_sta, current_time, sim_step);
			}
			else
			{
				// Red
				change_flag = ECIChangeSignalGroupState(node_ID, p, 0, time_sta, current_time, sim_step);
			}

		}
		else
		{
			// Red
			change_flag = ECIChangeSignalGroupState(node_ID, p, 0, time_sta, current_time, sim_step);
		}

	}

	// Set next update time
	double new_cycle_length = optimal_signal_times[0] + (optimal_signal_times[0] > 0)*(yellow_time + all_red) +
		optimal_signal_times[1] + (optimal_signal_times[1] > 0)*(yellow_time + all_red) +
		optimal_signal_times[2] + (optimal_signal_times[2] > 0)*(yellow_time + all_red) +
		optimal_signal_times[3] + (optimal_signal_times[3] > 0)*(yellow_time + all_red);
	if (abs(current_time - cycle_start_time - new_cycle_length) < 0.000001)
	{
		const unsigned short *intersection_Info = AKIConvertFromAsciiString("Signal_Update_Time");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(intersection_Info), node_ID, current_time);
		AKIDeleteUNICODEString(intersection_Info);
		read_new_SPaT = true;
	}


	return signal_set;
}

// Configure signal heads
bool ConfigureSignalHeads(int node_ID, double time, bool skip_last)
{
	bool signal_set = true;

	// read signal times
	const unsigned short *times_string = AKIConvertFromAsciiString("Signal_Times");
	double rec = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(times_string), node_ID);
	AKIDeleteUNICODEString(times_string);
	int pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
	pos1 = rec / 1e14;
	rec = rec - pos1*1e14;
	pos2 = rec / 1e12;
	rec = rec - pos2*1e12;
	pos3 = rec / 1e10;
	rec = rec - pos3*1e10;
	pos4 = rec / 1e8;
	rec = rec - pos4*1e8;
	pos5 = rec / 1e6;
	rec = rec - pos5*1e6;
	pos6 = rec / 1e4;
	rec = rec - pos6*1e4;
	pos7 = rec / 1e2;
	rec = rec - pos7*1e2;
	pos8 = rec;
	int optimal_signal_times[] = { pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8 };

	// Configure signal heads
	int totalSignalGroups = ECIGetNumberSignalGroups(node_ID);

	for (int p = 1; p <= totalSignalGroups; p++)
	{
		// Identify the current phase associated with the signal group
		int phaseNum = ECIGetNumberPhases(node_ID);
		int phaseID = -1;
		for (int r = 1; r <= phaseNum; r++)
		{
			int signal = ECIGetSignalGroupPhaseofJunction(node_ID, r, 0, AKIGetCurrentSimulationTime());
			if (signal == p)
			{
				phaseID = r;
				break;
			}
		}

		// Identify the element ID of the node
		int elem_node = 0;
		if (phaseID < 0 || elem_node < 0)
		{
			continue;
		}

		// Assign signal times
		const unsigned short *signal_ID_pt = ECIGetExternalIdofSignalGroup(node_ID, p);
		bool nonChar;
		const char *signal_name_char = AKIConvertToAsciiString(signal_ID_pt, true, &nonChar);
		std::string signal_name(signal_name_char);
		int green_num = 0;
		if (signal_name == "Signal 1")
		{
			green_num = 0;
		}
		else if (signal_name == "Signal 2")
		{
			green_num = 1;
		}
		else if (signal_name == "Signal 3")
		{
			green_num = 2;
		}
		else if (signal_name == "Signal 4")
		{
			green_num = 3;
		}
		else if (signal_name == "Signal 5")
		{
			green_num = 4;
		}
		else if (signal_name == "Signal 6")
		{
			green_num = 5;
		}
		else if (signal_name == "Signal 7")
		{
			green_num = 6;
		}
		else if (signal_name == "Signal 8")
		{
			green_num = 7;
		}

		if (skip_last)
		{
			if (green_num == ring1_last_phase || green_num == ring2_last_phase)
			{
				continue;
			}

		}
		//else if (!skip_last &&
		//	(green_num != 1 && green_num != 5))
		//{
		//	continue;
		//}

		double green = 0;
		if (optimal_signal_times[green_num] > 0 && signal_optimization_approach == 1)
		{
			green = optimal_signal_times[green_num] - yellow_time + all_red;
		}
		else
		{
			green = optimal_signal_times[green_num];
		}
		int total_duration_flag = ECIChangeTimingPhase(node_ID, phaseID, green, time);
		int max_time_flag = ECISetActuatedParamsMaxGreen(elem_node, node_ID, phaseID, green);
		int min_time_flag = ECISetActuatedParamsMinimumGreen(elem_node, node_ID, phaseID, green, green, 0);

		// If this phase has been passed, set cancel the following interphase as well
		if (green == 0)
		{
			ECIChangeTimingPhase(node_ID, phaseID + 1, 0, time);
			int yellow_flag = ECISetYellowTimePhaseofJunction(elem_node, node_ID, phaseID + 1, 0);

			if (total_duration_flag < 0 || max_time_flag < 0 || min_time_flag < 0)
			{
				signal_set = false;
			}
		}
		else
		{
			int yellow_flag;
			ECIChangeTimingPhase(node_ID, phaseID + 1, yellow_time + all_red, time);
			yellow_flag = ECISetYellowTimePhaseofJunction(elem_node, node_ID, phaseID + 1, yellow_time);
			if (total_duration_flag < 0 || max_time_flag < 0 || min_time_flag < 0 || yellow_flag < 0)
			{
				signal_set = false;
			}
		}

	}

	if (!skip_last)
	{
		// Set next update time
		double new_cycle_length = optimal_signal_times[0] + (optimal_signal_times[0] > 0)*(yellow_time + all_red) +
			optimal_signal_times[1] + (optimal_signal_times[1] > 0)*(yellow_time + all_red) +
			optimal_signal_times[2] + (optimal_signal_times[2] > 0)*(yellow_time + all_red) +
			optimal_signal_times[3] + (optimal_signal_times[3] > 0)*(yellow_time + all_red);
		const unsigned short *intersection_Info = AKIConvertFromAsciiString("Signal_Update_Time");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(intersection_Info), node_ID, time + new_cycle_length);
		AKIDeleteUNICODEString(intersection_Info);
	}




	return signal_set;
}

//read parameters from external txt file for parameters
void ReadExternalParameters()
{
	std::ifstream infile("C:\\CACC_Simu_Data\\ParameterSet.txt");
	std::string line;
	if (infile.is_open() == false)
		return;
	while (std::getline(infile, line))
	{
		while (true)
		{
			int index = (int)line.find(",");
			if (index == std::string::npos)
			{
				break;
			}
			std::string str_interval = line.substr(0, index);
			int tempindex = (int)str_interval.find_last_of(":");
			std::string key_value = line.substr(0, tempindex);
			std::string value_string = line.substr(tempindex + 1, index - tempindex - 1);
			double param_value = atof(value_string.c_str());

			// Setup signal control related parameters here:===========================================
			if (key_value.find("exp_Yellow_Time") != std::string::npos)
			{
				yellow_time = param_value;
			}
			else if (key_value.find("exp_All_Red") != std::string::npos)
			{
				all_red = param_value;
			}
			else if (key_value.find("exp_Signal_Coordination_On") != std::string::npos)
			{
				signal_optimization_approach = param_value;
			}
			else if (key_value.find("exp_CRM_On") != std::string::npos)
			{
				CRM_On = param_value;
			}
			else if (key_value.find("exp_SPD_ADV_On") != std::string::npos)
			{
				SPD_ADV_On = param_value;
			}
			else if (key_value.find("exp_ADV_Vmax") != std::string::npos)
			{
				v_max = param_value;
			}
			else if (key_value.find("exp_ADV_Vmin") != std::string::npos)
			{
				v_min = param_value;
			}
			else if (key_value.find("exp_ADV_Random_Compliance") != std::string::npos)
			{
				random_compliance = param_value;
			}
			else if (key_value.find("exp_Number_Sections") != std::string::npos)
			{
				num_secs = param_value;
			}
			//================================================================================================================

			line = line.substr(index + 1, line.length() - index - 1);
		}
		break; //only read one line
	}
	infile.close();
}

fuelVMT computeEnergy(double v, double a, bool truck_flag)
{
	fuelVMT outFuelVMT;

	outFuelVMT.VMT = v * AKIGetSimulationStepTime() + 0.5 * a * AKIGetSimulationStepTime() *AKIGetSimulationStepTime();
	outFuelVMT.VHT = AKIGetSimulationStepTime();

	int opMode = 0;
	double vsp = (0.156461*v + 0.002002*pow(v, 2) + 0.000493*pow(v, 3)) / 1.4788 + a*v;

	if (a <= -1.0)
	{
		opMode = 0;
	}
	else if (v < 1 * 0.44704)
	{
		opMode = 1;
	}
	else if (v <= 25 * 0.44704 && vsp <= 0)
	{
		opMode = 2;
	}
	else if (v <= 25 * 0.44704 && vsp <= 3)
	{
		opMode = 3;
	}
	else if (v <= 25 * 0.44704 && vsp <= 6)
	{
		opMode = 4;
	}
	else if (v <= 25 * 0.44704 && vsp <= 9)
	{
		opMode = 5;
	}
	else if (v <= 25 * 0.44704 && vsp <= 12)
	{
		opMode = 6;
	}
	else if (v <= 25 * 0.44704 && vsp > 12)
	{
		opMode = 7;
	}
	else if (v <= 50 * 0.44704 && vsp <= 0)
	{
		opMode = 8;
	}
	else if (v <= 50 * 0.44704 && vsp <= 3)
	{
		opMode = 9;
	}
	else if (v <= 50 * 0.44704 && vsp <= 6)
	{
		opMode = 10;
	}
	else if (v <= 50 * 0.44704 && vsp <= 9)
	{
		opMode = 11;
	}
	else if (v <= 50 * 0.44704 && vsp <= 12)
	{
		opMode = 12;
	}
	else if (v <= 50 * 0.44704 && vsp <= 18)
	{
		opMode = 13;
	}
	else if (v <= 50 * 0.44704 && vsp <= 24)
	{
		opMode = 14;
	}
	else if (v <= 50 * 0.44704 && vsp <= 30)
	{
		opMode = 15;
	}
	else if (v <= 50 * 0.44704 && vsp > 30)
	{
		opMode = 16;
	}
	else if (v > 50 * 0.44704 && vsp <= 6)
	{
		opMode = 17;
	}
	else if (v > 50 * 0.44704 && vsp <= 12)
	{
		opMode = 18;
	}
	else if (v > 50 * 0.44704 && vsp <= 18)
	{
		opMode = 19;
	}
	else if (v > 50 * 0.44704 && vsp <= 24)
	{
		opMode = 20;
	}
	else if (v > 50 * 0.44704 && vsp <= 30)
	{
		opMode = 21;
	}
	else if (v > 50 * 0.44704 && vsp > 30)
	{
		opMode = 22;
	}

	outFuelVMT.fuel = energyRates[opMode] * AKIGetSimulationStepTime() / 3600;

	// Fuel consumption computed based on the VT-CPFM model

	double alpha0 = 0.00059217;
	double alpha1 = 4.2378e-5;
	double alpha2 = 1e-6;
	double m = 1453;
	double eta_vt = 0.92;
	double rho = 1.2256;
	double Cd = 0.30;
	double Ch = 1;
	double Af = 2.32;
	double Cr = 1.75;
	double C1 = 0.0328;
	double C2 = 4.575;
	double G = 0;

	if (truck_flag)
	{
		double alpha0 = 1.56e-4;
		double alpha1 = 4.12e-5;
		double alpha2 = 3.57e-8;
		double m = 29400;
		double eta_vt = 0.94;
		double rho = 1.2256;
		double Cd = 0.57;
		double Ch = 1;
		double Af = 10.7;
		double Cr = 1.75;
		double C1 = 0.0328;
		double C2 = 4.575;
		double G = 0;
	}

	double Rt = 0;
	v = v*3.6;
	Rt = rho / 25.92*Cd*Ch*Af*v*v + 9.8066*m*Cr / 1000 * (C1*v + C2) + 9.8066*m*G;
	double Pt = 0;
	Pt = (Rt + 1.04*m*a) / 3600 / eta_vt*v;
	double fuel_rate = 0; //Gal/s

	if (Pt < 0)
	{
		fuel_rate = alpha0*0.264172;
	}
	else
	{
		fuel_rate = (alpha0 + alpha1*Pt + alpha2*Pt*Pt)*0.264172;
	}

	outFuelVMT.fuel_VT = fuel_rate*AKIGetSimulationStepTime();

	return outFuelVMT;
}

void outputEnergy()
{
	int exp_id = ANGConnGetExperimentId();
	int rep_id = ANGConnGetReplicationId();
	const unsigned short *CACC_PercentString = AKIConvertFromAsciiString(
		"CACC_Percent");
	double cacc_percent = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(CACC_PercentString), exp_id);
	const unsigned short *ACC_PercentString = AKIConvertFromAsciiString(
		"ACC_Percent");
	double acc_percent = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(ACC_PercentString), exp_id);
	std::ifstream infile("C:\\CACC_Simu_Data\\ParameterSet.txt");
	std::string parastr;
	if (infile.is_open() == true)
	{
		std::getline(infile, parastr);
		std::getline(infile, parastr);//ignore the first line
	}
	infile.close();


	char str_tmp[1024] = "a";

	// Energy
	sprintf_s(str_tmp, 1024,
		"C:\\CACC_Simu_Data\\acc%u_cacc%u\\%s\\fuelConsumption_%u.txt",
		(int)(acc_percent * 100), (int)(cacc_percent * 100), parastr.c_str(), rep_id);

	//FILE* file = fopen(str_tmp, "w");

	std::ofstream file(str_tmp);
	std::vector<std::vector<double>>::size_type sz = fuelMatrix.size();
	for (int p = 0; p < sz; p++)
	{
		std::vector<double> tempVec = fuelMatrix[p];
		std::ostringstream oss;
		// Convert all but the last element to avoid a trailing ","
		std::copy(tempVec.begin(), tempVec.end(), std::ostream_iterator<double>(oss, "	"));
		file << oss.str() << '\n';
	}

	file.close();
}

int AAPIUnLoad()
{
	AKIPrintString("UNLOAD");
	return 0;
}

int AAPIEnterVehicle(int idveh, int idsection)
{
	return 0;
}

int AAPIExitVehicle(int idveh, int idsection)
{
	return 0;
}



int AAPIPreRouteChoiceCalculation(double time, double timeSta)
{
	AKIPrintString("\tPreRouteChoice Calculation");
	return 0;
}
