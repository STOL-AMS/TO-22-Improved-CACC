/********************************************************
* Simulating Impacts of ACC/CACC in Aimsun			   *
*                                                      *
* The program is developed by the PATH research team   *
* at U.C. Berkeley under the FHWA EARP Project "Using  *
* CACC to Form High-Performance Vehicle Streams"	   *
*                                                      *
* Author: Dr. Hao Liu, Dr. Xiao-Yun Lu, David Kan,     *
* Fang-Chieh Chou, Dr. Dali Wei                        *
*                                                      *
* Usage: Compile the program and it will generate a    *
* DLL file named myBehaviorModel.dll. Copy the DLL	   *
* file to <Aimsun Folder>\plugins\aimsun\models		   *
*													   *
* 06/19/2017										   *
********************************************************/

#include "mybehavioralModel.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <math.h>
#include <map>

#include "myVehicleDef.h"
#include <time.h>       /* time_t, struct tm, time, localtime */
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <iostream>

// Socket programming
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>

#include <stdlib.h> 
#include <stdio.h> 
#include <chrono>
#include <thread>
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

#define MAX(a,b)    (((a)>(b)) ? (a) : (b))
#define MIN(a,b)    (((a)<(b)) ? (a) : (b))

#define SAVE_SECTION 23551

#define Reaction_Time_ACC 1.2
#define Reaction_Time_CACC 0.2

#define Reaction_Time_Intersection 2.0
//#define _DEBUG

// For energy consumption matrices
#define Time_Resolution 180 // sceonds
#define Space_Resolution 200 // meters

// For cooperative signal control
#define Signal_Detection_Range 500.0
#define Cycle_Length 90
#define Cycle_Length_Min 60.0
#define Yellow_All_Red_Time 5.0
#define All_Red_Time 2.0
#define Estimated_Acceleration_CACC 0.5 // Acceleration used in predicting vehicle movements in the future
#define Estimated_Acceleration_Manual 0.5 // Acceleration used in predicting vehicle movements in the future
#define Estimated_Deceleration -2.5 // Deceleration used in predicting vehicle movements in the future
#define Start_Reaction_Time 1.5 // Reaction time for human drivers when a signal turns green from red
#define Moving_Reaction_Time 1.2
#define Threshold_Speed 2.0 // A vehicle is considered to be in the queue if its speed is less than the threshold
#define Equivalent_Vehicle_Factor 1

// For intersection-in-the-loop test
#define Traffic_Scenario_Start_Time 30 // in seconds, start prepare the last three vehicles in the simulated model before syncrhonizing with the real data
#define Server_Address "169.229.202.17"
//#define Server_Address "192.168.0.33"
//#define Server_Address "10.142.40.37"
#define Server_Port 8800
#define Server_Port_Signal 8900
#define Local_Port 8000 
#define Local_Port_Signal 8100  


#include <stdlib.h>
#include <crtdbg.h>

//#define OUTPUT

static int file_init = 1;
errno_t err;
#define len_str 256
char str_tmp[len_str] = "a";


FILE *OutFile;														// Output file pointer 
FILE *VehStatFile;													// summary output file pointer 			
FILE *StatFile;													    // summary output file pointer 	
FILE *CF_Data;														//add by DW to tracking CF data
FILE *errp;

std::map<int, std::map<int, int>> s_vids; //the last arrival of vehicle ids

// passenger car fuel consumption rate in gallons per hour by operation mode
double energyRates[23] = {
	0.439924998,
	0.403028208,
	0.620998679,
	0.835079895,
	1.170144664,
	1.485700785,
	1.78510614,
	2.177848651,
	0.821099529,
	0.940306847,
	1.157147214,
	1.486324463,
	1.949577513,
	2.545464421,
	3.430942488,
	4.696629632,
	5.883428994,
	1.181570445,
	1.867266985,
	2.418363834,
	3.149873266,
	4.191181023,
	5.334442671 };
// section-distance converter for the simple network. It is used to convert the 
// position of a subject vehicle in a section to a global distance, measuring from
// the starting point of section 3616
double linkDistanceConverter_Simple[8][2] = {
	{ 3616, 0 },
	{ 2919, 1005.97 },
	{ 4953, 1259.27 },
	{ 4956, 1614.14 },
	{ 4162, 2019.79 },
	{ 4818, 2667.59 },
	{ 4948, 2771.17 },
	{ 4740, 3414.9 }
};

//// fuel and VMT marices for the simple freeway network
//double fuelMatrix_Simple[413][360] = { { 0, 0 } };
//double VMT_Matrix_Simple[413][360] = { { 0, 0 } };
//double VHT_Matrix_Simple[413][360] = { { 0, 0 } };


// This is for the single intersection network
//double linkDistanceConverter_Simple_Intersection[3][2] = {
//	{ 6482, 0 },
//	{ 6529, 543.261 },
//	{ 6523, 543.261+16.13 }
//};
//
//// fuel and VMT marices for the single intersection
//double fuelMatrix_Simple[114][360] = { { 0, 0 } };
//double VMT_Matrix_Simple[114][360] = { { 0, 0 } };
//double VHT_Matrix_Simple[114][360] = { { 0, 0 } };


// This is for the 2-intersection network
//double linkDistanceConverter_Simple_Intersection[5][2] = {
//	{ 9166, 0 },
//	{ 6219, 551.809 },
//	{ 6220, 551.809 + 30.72 },
//	{ 6316, 551.809 + 30.72 + 388.569 },
//	{ 6286, 551.809 + 30.72 + 388.569 + 25.90 }
//};
//// fuel and VMT marices for the 2-intersection intersection
//double fuelMatrix_Simple[124][360] = { { 0, 0 } };
//double VMT_Matrix_Simple[124][360] = { { 0, 0 } };
//double VHT_Matrix_Simple[124][360] = { { 0, 0 } };




char energyStr[1024 * 1024 * 10] = "\0";
char VMTStr[1024 * 1024 * 10] = "\0";
char VHTStr[1024 * 1024 * 10] = "\0";


/* Methods required by A2BehavioralModel class+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
mybehavioralModel::mybehavioralModel()
{

	ReadExternalParameters();
	SetExternalParameters();

	int exp_id = ANGConnGetExperimentId();
	int rep_id = ANGConnGetReplicationId();
	const unsigned short *CACC_PercentString = AKIConvertFromAsciiString(
		"CACC_Percent");
	cacc_percent = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(CACC_PercentString), exp_id);
	const unsigned short *ACC_PercentString = AKIConvertFromAsciiString(
		"ACC_Percent");
	acc_percent = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(ACC_PercentString), exp_id);
	std::ifstream infile("C:\\CACC_Simu_Data\\ParameterSet.txt");
	if (infile.is_open() == true)
	{
		std::getline(infile, parastr);
		std::getline(infile, parastr);//ignore the first line
	}
	infile.close();

	ReadHOVSetting();


	// Set vehicle type data
	int num_veh_types = AKIVehGetNbVehTypes();
	for (int p = 1; p <= num_veh_types; p++)
	{
		int veh_type_id = AKIVehTypeGetIdVehTypeANG(p);
		readVehTypeData(veh_type_id);
	}

}

mybehavioralModel::~mybehavioralModel()
{

}


myVehicleDef * mybehavioralModel::arrivalNewVehicle(void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh){
	myVehicleDef  * res = new myVehicleDef(handlerVehicle, idHandler, isFictitiousVeh);
	res->setNewArrivalAdjust(true);

	return res;
}

void mybehavioralModel::removedVehicle(void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh)
{

}

double mybehavioralModel::computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, bool ImprudentCase, bool VehicleIspVehDw, int time)
{
	return -1;
}

bool mybehavioralModel::avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre)
{
	return false;
}

bool mybehavioralModel::evaluateLaneChanging(A2SimVehicle *vehicle, int threadId)
{
	myVehicleDef* veh = (myVehicleDef*)vehicle;

	if (vehicle->isCurrentLaneInNode())
	{
		if (veh->getMode() != 9 &&
			veh->getMode() != 10 &&
			veh->getMode() != 11 &&
			veh->getMode() != 91 &&
			veh->getMode() != 101 &&
			veh->getMode() != 102 &&
			veh->getMode() != 103 &&
			veh->getMode() != 104 &&
			veh->getMode() != 105 )
		{
			// If the vehicle is not in the CACC/ACC mode, switches to the CF mode
			veh->setMode(0);
		}
		return true;
	}


	if (!veh->getNewArrivalAdjust() && !veh->isCurrentLaneInNode())
	{

		if (veh->getMode() == 8 && !veh->isLaneChangingPossible(veh->getTargetLane()))
		{
			// In case the vehicle has already missed the turn during the process of LC,
			// abort LC and return to normal CF_Hao 11/26
			veh->setMode(0);
			return true;
		}

		if (veh->getMode() == 8) // LC mode
		{

			veh->getAroundLeaderFollowers();

			double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), veh->getTargetLane());
			double ShiftUp = 0, ShiftDw = 0;
			const A2SimVehicle *vehUp = NULL;
			const A2SimVehicle *vehDown = NULL;
			double upDist;
			double downDist;
			if (veh->getTargetLane() == LEFT)
			{
				vehUp = veh->left_follower;
				vehDown = veh->left_leader;
				upDist = veh->leftLagHeadway;
				downDist = veh->leftLeadHeadway;
			}
			else
			{
				vehUp = veh->right_follower;
				vehDown = veh->right_leader;
				upDist = veh->rightLagHeadway;
				downDist = veh->rightLeadHeadway;
			}

			if (vehDown != NULL
				&& downDist < vehDown->getLength() + ((myVehicleDef*)vehDown)->getJamGap())
			{
				veh->setMode(0);
				return true;
			}

			if (vehUp != NULL
				&& upDist < veh->getLength() + veh->getJamGap())
			{
				veh->setMode(0);
				return true;
			}

			vehicle->assignAcceptedGap(veh->getTargetLane(), XPosTargetlane, vehUp, ShiftUp, vehDown, ShiftDw, threadId);
			//vehicle->applyLaneChanging(veh->getTargetLane(), threadId);
		}
	}
	return true;
}

bool mybehavioralModel::evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed)
{


	myVehicleDef* veh = (myVehicleDef*)vehicle;
	myVehicleDef::PositionSpeed pos_speed;

	// For new vehicles ====================================================================================== 
	if (veh->getNewArrivalAdjust())
	{
		//New vehicles are placed at equilibrium spacing regarding its leader	
		this->setPara4NewVeh(veh);
		veh->getSectionInfo();
		veh->leader = veh->getLeader();
		pos_speed = veh->AdjustArrivalVehicle_New(this->UseMergingDemandReading());
		newpos = pos_speed.position;
		newspeed = pos_speed.speed;

		// Prevent vehicle from entering the network at the very beginning of the source link.
		// Otherwise, Vehicles will be deleted if there is not enough space at the beginning. 
		// The human driven vehicles have larger probability to be deleted than the CACC vehicles because their larger desired headway.
		// Hao_08/31/17
		// It is recommended that the source link is at least 500 m
		int sec = veh->getIdCurrentSection();
		double dist_left = veh->sec_inf.length - newpos;
		if (newpos < 20)
		{
			if (this->source_links_release_switch.find(sec) != this->source_links_release_switch.end())
			{
				this->source_links_release_switch[sec] = true;
			}
			else
			{
				this->source_links_release_switch.insert(std::pair<int, bool>(sec, true));
			}
			
		}
		else if (dist_left < 150)
		{
			if (this->source_links_release_switch.find(sec) != this->source_links_release_switch.end())
			{
				this->source_links_release_switch[sec] = false;
			}
			else
			{
				this->source_links_release_switch.insert(std::pair<int, bool>(sec, false));
			}
		}

		if (this->source_links_release_switch.find(sec) != this->source_links_release_switch.end() && this->source_links_release_switch[sec])
		{
			veh->SetReportError();
		}

		return true;
	}

	// For vehicles already in network========================================================================
	veh->getSectionInfo();

	//set active status of hov
	veh->setHOVActive();

	// Get front vehicle info
	veh->leader = veh->getLeader();

	// The following two parameters should be global parameters. 
	// It might affect computation speed if we obtain time step and simulation time for individual vehicles
	veh->delta_t = getSimStep();
	veh->current_time = AKIGetCurrentSimulationTime();

	// Get average speed info for left, current and right lane
	veh->getAroundSpeed();

	// Get pointers of the leaders and followers on the adjacent lanes
	veh->getAroundLeaderFollowers();


	if (veh->getVehType() == NGSIM_CACC_ACC_V2V || veh->getVehType() == CACC_HOV)  //CACC+ACC+V2V merging vehicles   4369
	{
		pos_speed = veh->NGSIMPlusACC_CACC_V2VAHM(false);
		newpos = pos_speed.position;
		newspeed = pos_speed.speed;
	}
	else if (veh->getVehType() == NGSIM_ACC || veh->getVehType() == ACC_HOV)  //ACC only vehicle
	{
		pos_speed = veh->NGSIMPlusACC(false);
		newpos = pos_speed.position;
		newspeed = pos_speed.speed;
	}
	//run NGSIM model
	else if (veh->ApplyNGSIMModel()
		||
		veh->acc_2_manual)
	{
		//passing false indicates that the driving mode has not been determined 
		//and needs to be determined in the function
		//Read_leader_follower_data();
		pos_speed = veh->RunNGSIM(false);

		// Do not use when there is no need to study the automated merging assistance system_Hao 01/29/2019
		//veh->MergeSecPosition();

		newpos = pos_speed.position;
		newspeed = pos_speed.speed;
	}
	else
	{
		//ACC and CACC behavior
		//they are only applied to these vehicles without lane-changing intent
		//the lane change intent is identified when the vehicle is on the rightmost lane for the specific testing scenario	
		pos_speed = pos_speed = veh->RunNGSIM(false);
		newpos = pos_speed.position;
		newspeed = pos_speed.speed;
	}

	veh->update_Following_Veh_ID(); //Pass subject vehicle's information to the Leader. The vehicles will have access to following vehicle, then. 


	// Only use this function when debugging
	// It slows down the simulation_Hao 01/28/2019
	veh->update_Driving_Mode();  //Update an attribute "Driving_Mode" in Aimsun.
	
	//Delete vehicles that have infinity position
	if (isnan(newspeed) || isnan(newpos))
	{
		newspeed = 0;
		newpos = veh->getPosition(0);

		int VehANGId = ANGConnVehGetGKSimVehicleId(veh->getId());
		const unsigned short *temp1 = AKIConvertFromAsciiString("isCrashed");
		ANGConnSetAttributeValueBool(ANGConnGetAttribute(temp1), VehANGId, true);

		char str[1280];
		AKIPrintString(str);
		sprintf_s(str,
			"Strange section time: %.1f, Veh ID: %d, New Arrival: %d, Sec ID: %d, Lane ID: %d, Pos: %.1f, Speed: %.1f\n",
			AKIGetCurrentSimulationTime(), veh->getId(), veh->getNewArrivalAdjust(),
			veh->getIdCurrentSection(), veh->getIdCurrentLane(),
			veh->getPosition(), veh->getSpeed());
		AKIPrintString(str);
		//ANGSetSimulationOrder(4, AKIGetCurrentSimulationTime());
	}



	// Compute energy consumption and CACC vehicle mode switch data for individual vehicles==================================================================
	this->recordEnergyModeSwitch(veh);

	return true;
}

bool mybehavioralModel::isVehicleGivingWay(A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield)
{
	return false;
}

double mybehavioralModel::computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle, double VelActual, double VelDeseada, double RestoCiclo)
{
	return -1;
}

double mybehavioralModel::computeCarFollowingDecelerationComponentSpeed(A2SimVehicle *vehicle, double Shift, A2SimVehicle *vehicleLeader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	return -1;
}

double mybehavioralModel::computeCarFollowingDecelerationComponentSpeedCore(A2SimVehicle *vehicle, double VelAnterior, A2SimVehicle *vehicleLeader, double VelPreAnterior, double GapAnterior, double DecelEstimadaLeader)
{
	return -1;
}

int mybehavioralModel::evaluateLaneSelectionDiscretionary(A2SimVehicle *vehicle, bool LeftLanePossible, bool RightLanePossible)
{
	return -10;
}


int mybehavioralModel::evaluateHasTime2CrossYellowState(A2SimVehicle *vehicle, double distance2StopLine)
{
	return -1;
}
/* Methods required by A2BehavioralModel class+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/* User specifed methods for simulating CACC/ACC traffic+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void mybehavioralModel::recordEnergyModeSwitch(myVehicleDef* veh)
{

	try
	{
		// Generate an integer to represent the vehicle string status
		//Pos 1: CACC or not; Pos 2: string leader or string follower (1), first CACC following a VAD (2)
		//Pos 3: is there a string in front or not; Pos 4: mode switch or not
		int pos1 = 0;
		int pos2 = 0;
		int pos3 = 0;
		int pos4 = 0;
		if (veh->getVehType() == NGSIM_CACC_ACC_V2V || veh->getVehType() == CACC_HOV)
		{
			pos1 = 1;
			// CACC vehicle in string and CACC vehicle VMT
			if (veh->stringPosID == 1)
			{
				// Counting the leader
				if (veh->leader != NULL
					&& (veh->leader->getVehType() == NGSIM_CACC_ACC_V2V || veh->leader->getVehType() == CACC_HOV)
					&& ((myVehicleDef*)veh->leader)->stringPosID == 0)
				{
					pos2 = 2;
				}
				else
				{
					pos2 = 1;
				}

			}
			else if (veh->stringPosID > 1)
			{
				pos2 = 1;
			}
			else
			{
				// Number of strings
				if (veh->leader != NULL
					&& (veh->leader->getVehType() == NGSIM_CACC_ACC_V2V || veh->leader->getVehType() == CACC_HOV)
					&& ((myVehicleDef*)veh->leader)->stringPosID > 0)
				{
					pos3 = 1;
				}
			}

			// number of CACC mode off
			if (veh->CACCModeSwitch > 0)
			{
				pos4 = 1;
			}
		}
		else
		{
			// Number of strings
			if (veh->leader != NULL
				&& (veh->leader->getVehType() == NGSIM_CACC_ACC_V2V || veh->leader->getVehType() == CACC_HOV)
				&& ((myVehicleDef*)veh->leader)->stringPosID > 0)
			{
				pos3 = 1;
			}

		}

		int CACC_string_flag = pos1 * 1000 + pos2 * 100 + pos3 * 10 + pos4;
		int veh_Aimsun_ID = ANGConnVehGetGKSimVehicleId(veh->getId());
		const unsigned short *string_Info = AKIConvertFromAsciiString("CACC_String_Info");
		ANGConnSetAttributeValueInt(ANGConnGetAttribute(string_Info), veh_Aimsun_ID, CACC_string_flag);
		AKIDeleteUNICODEString(string_Info);

	}
	catch (int e)
	{
		char str[1280];
		AKIPrintString(str);
		sprintf_s(str,
			"Energy consumption error: %d\n", e);
		AKIPrintString(str);
	}

}





void mybehavioralModel::setPara4NewVeh(myVehicleDef* res)
{
	int exp_id = ANGConnGetExperimentId();

	//Traffic density data is predetermined in the Aimsun setup.
	//No need to update on-line currently.
	res->desAccL = 5;
	res->virtual_des_AccL = 0;
	res->RampFollowDist = 10;
	res->MergeTargetFound = false;
	res->MergeTargetID = -1;
	res->readyToMergeBehind = false;
	res->spdSync = false;
	res->desAccel = 0;
	res->Active_AHM = true;

	res->old_V_leader = -1;
	res->V_CACC_init_time = 0.0;


	// CACC Parameters
	const unsigned short *CACC_Lane_String =
		AKIConvertFromAsciiString("Activate_CACC_Managed_Lane");
	int CACC_Lane_Flag = ANGConnGetAttributeValueInt(
		ANGConnGetAttribute(CACC_Lane_String), exp_id);
	AKIDeleteUNICODEString(CACC_Lane_String);
	const unsigned short *CACC_Lane_Number_String =
		AKIConvertFromAsciiString("Number_of_CACC_Managed_Lanes");
	int CACC_Lane_Number = ANGConnGetAttributeValueInt(
		ANGConnGetAttribute(CACC_Lane_Number_String), exp_id);
	AKIDeleteUNICODEString(CACC_Lane_Number_String);
	const unsigned short *CACC_Lane_Access_String =
		AKIConvertFromAsciiString("Activate_CACC_Managed_Lane_Restricted_Access");
	int CACC_Lane_Access_Flag = ANGConnGetAttributeValueInt(
		ANGConnGetAttribute(CACC_Lane_Access_String), exp_id);
	AKIDeleteUNICODEString(CACC_Lane_Access_String);
	const unsigned short *CACC_LC_Desire_Str = AKIConvertFromAsciiString("GKExperiment::CACC_LC_Desire_Threshold");
	double LC_Thre = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(CACC_LC_Desire_Str), exp_id);
	AKIDeleteUNICODEString(CACC_LC_Desire_Str);

	res->CACC_ML_Activated = CACC_Lane_Flag;
	res->number_CACC_ML = CACC_Lane_Number;
	res->CACC_ML_RA = CACC_Lane_Access_Flag;
	res->CACC_DLC_Thre = LC_Thre;


	// Determine if a vehicle is VAD vehicle or not
	double vadFlag = AKIGetRandomNumber();
	const unsigned short *VAD_Percent =
		AKIConvertFromAsciiString("GKExperiment::VAD_Percent");
	double vadPercent = ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(VAD_Percent), exp_id);
	AKIDeleteUNICODEString(VAD_Percent);
	if (vadFlag < vadPercent)
	{
		res->ConnectFlag = true;
	}
	else
	{
		res->ConnectFlag = false;
	}

	res->v_leader_flag = false;
	res->v_follower_flag = false;
	res->vir_leader_speed = -1;
	res->on_ramp_spd_syn = false;
	res->virtual_leader_found = false;
	res->ACC_recover_time = 0;

	res->v_vleader = -1;
	res->dis_vleader = -1;
	res->v_vfollower = -1;
	res->dis_vfollower = -1;
	res->v_real_leader = -1;
	res->dis_real_leader = -1;
	res->virtual_ACC_follower_ID = -1;
	res->v_follower_ID = -1;
	res->CACC_recover_time = 11;

	res->set_ACC_CTG();  //set up the ACC following time gap.
	res->set_CACC_CTG();
	res->ACC_state = 91;
	res->CACC_state = 101;
	res->CACC_mode_switch_time = 0;
	res->stringPosID = -1;

	res->rightTurnStartTime = 0;
	res->rightTurnWaitTime = 0;

	int VehANGId = ANGConnVehGetGKSimVehicleId(res->getId());

	const unsigned short *temp1 = AKIConvertFromAsciiString("isCrashed");
	ANGConnSetAttributeValueBool(ANGConnGetAttribute(temp1), VehANGId, false);   //the number of the vehicle ahead in the platoon.
	AKIDeleteUNICODEString(temp1);

	const unsigned short *temp2 = AKIConvertFromAsciiString("Connected_Vehicle");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp2), VehANGId, (int)res->ConnectFlag);   //the number of the vehicle ahead in the platoon.
	AKIDeleteUNICODEString(temp2);

	const unsigned short *temp_string1 =
		AKIConvertFromAsciiString("GKSimVehicle::v_left_leader_spd");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string1), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string1);

	const unsigned short *temp_string2 =
		AKIConvertFromAsciiString("GKSimVehicle::v_left_leader_dis");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string2), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string2);

	const unsigned short *temp_string3 =
		AKIConvertFromAsciiString("GKSimVehicle::v_left_follower_spd");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string3), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string3);

	const unsigned short *temp_string4 =
		AKIConvertFromAsciiString("GKSimVehicle::v_left_follower_dis");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string4), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string4);

	const unsigned short *temp_string5 =
		AKIConvertFromAsciiString("GKSimVehicle::leader_spd");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string5), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string5);

	const unsigned short *temp_string6 =
		AKIConvertFromAsciiString("GKSimVehicle::leader_dis");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string6), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string6);

	const unsigned short *temp_string7 =
		AKIConvertFromAsciiString("GKSimVehicle::ConnectedVehId");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string7), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string7);

	const unsigned short *temp_string8 =
		AKIConvertFromAsciiString("GKSimVehicle::FollowByVehId");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string8), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string8);

	const unsigned short *temp_string9 =
		AKIConvertFromAsciiString("GKSimVehicle::OnRampPlatoonNum1");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string9), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string9);

	const unsigned short *temp_string10 =
		AKIConvertFromAsciiString("GKSimVehicle::Platoon_leader_ID");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string10), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string10);

	const unsigned short *temp_string11 =
		AKIConvertFromAsciiString("GKSimVehicle::CACCPlatoonNum");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string11), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string11);

	const unsigned short *temp_string12 =
		AKIConvertFromAsciiString("GKSimVehicle::Following_Vehicle_ID");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string12), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string12);

	const unsigned short *temp_string13 = AKIConvertFromAsciiString("GKSimVehicle::DrivingMode");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string13), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string13);

	const unsigned short *temp_string14 = AKIConvertFromAsciiString("GKSimVehicle::CACC_PrecedingVehID");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string14), VehANGId, -1);
	AKIDeleteUNICODEString(temp_string14);

	const unsigned short *temp15 = AKIConvertFromAsciiString("GKSimVehicle::PrecdeCACCLength");
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp15), VehANGId, -1);
	AKIDeleteUNICODEString(temp15);


	const unsigned short *temp16 = AKIConvertFromAsciiString("GKExperiment::CACCLengthLimit");
	int CACC_length_limit = ANGConnGetAttributeValueInt(ANGConnGetAttribute(temp16), ANGConnGetExperimentId());
	AKIDeleteUNICODEString(temp16);
	res->setCACC_length_limit(CACC_length_limit);

	const unsigned short *temp17 = AKIConvertFromAsciiString("GKExperiment::CACCPlatoonGAP");
	res->CACC_platoon_CTG = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(temp17), ANGConnGetExperimentId());
	AKIDeleteUNICODEString(temp17);

	if (!res->isFictitious())
	{



		SetGlobalSetParameters(res);

		int type = res->getVehType();
		std::map<int, A2BehavioralVehData>::const_iterator iter = vehTypeData.find(type);
		double jamGap, E, T; //, devE, devT, minT, minE;

		// In case the vehicle type appears at the first time++++++++++++++++++++++++++++++++++++++++++++++++++
		if (iter == vehTypeData.end()){
			readVehTypeData(res->getVehType());
			iter = vehTypeData.find(res->getVehType());
		}
		// In case the vehicle type appears at the first time++++++++++++++++++++++++++++++++++++++++++++++++++

		// Set jam gap, which follows a normal distribution++++++++++++++++++++++++++++++++++++++++++++++++++++
		jamGap = sampleNormalDist((*iter).second.meanJam, (*iter).second.devJam);
		jamGap = (std::min)((*iter).second.maxJam, (std::max)(jamGap, (*iter).second.minJam));
		res->setJamGap(jamGap);
		// Set jam gap, which follows a normal distribution++++++++++++++++++++++++++++++++++++++++++++++++++++

		// Set car-following mode++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		res->setMode(0);
		// Set car-following mode++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		//Set reaction time, which follows a normal distribution+++++++++++++++++++++++++++++++++++++++++++++++
		double reaction_time = sampleNormalDist((*iter).second.avg_reaction_time,
			(*iter).second.dev_reaction_time);
		reaction_time = (std::min)((*iter).second.max_reaction_time,
			(std::max)(reaction_time, (*iter).second.min_reaction_time));
		res->setReactionTime(reaction_time);
		res->reaction_time_ACC = Reaction_Time_ACC;
		res->reaction_time_CACC = Reaction_Time_CACC;
		//Set reaction time, which follows a normal distribution++++++++++++++++++++++++++++++++++++++++++++++++

		//Set headway time, which follows a normal distribution+++++++++++++++++++++++++++++++++++++++++++++++++
		double headway_time = sampleNormalDist((*iter).second.avg_headway_time,
			(*iter).second.dev_headway_time);
		headway_time = (std::min)((*iter).second.max_headway_time,
			(std::max)(headway_time, (*iter).second.min_headway_time));
		res->setHeadwayTime(headway_time);
		//Set headway time, which follows a normal distribution+++++++++++++++++++++++++++++++++++++++++++++++++

		//**********************************************
		//parameters related to NGSIM model


		const unsigned short *gipps_theta_str =
			AKIConvertFromAsciiString("gipps_theta");
		res->setGippsTheta(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(gipps_theta_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(gipps_theta_str);
#else
		delete[] gipps_theta_str;
#endif

		const unsigned short *leader_max_dec_est_coef_str =
			AKIConvertFromAsciiString("leader_max_dec_est_coef");
		res->setEstimateLeaderDecCoeff(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(leader_max_dec_est_coef_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(leader_max_dec_est_coef_str);
#else
		delete[] leader_max_dec_est_coef_str;
#endif

		const unsigned short *acc_smooth_factor_str =
			AKIConvertFromAsciiString("acc_smooth_factor");
		res->setAccSmoothCoef(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(acc_smooth_factor_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(acc_smooth_factor_str);
#else
		delete[] acc_smooth_factor_str;
#endif

		//**********************************************
		//parameters related to lane change
		res->alpha = 1; //jam gap
		res->beta = 0.5; //reaction time
		//steps depending on the delta t
		const unsigned short *relax_str =
			AKIConvertFromAsciiString("relaxation_time");
		double relax_time = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(relax_str), exp_id);
		res->ACF_Steps = relax_time / AKIGetSimulationStepTime();
		res->ACF_Step = 0;
		res->Relaxation = 0.5;

		E = sampleNormalDist((*iter).second.meanE, (*iter).second.devE);
		E = (std::min)((*iter).second.maxE, (std::max)(E, (*iter).second.minE));
		res->setE(E);

		T = sampleNormalDist((*iter).second.meanT, (*iter).second.devT);
		T = (std::min)((*iter).second.maxT, (std::max)(T, (*iter).second.minT));
		res->setT(T);

		res->setMinTimeBtwLcs((*iter).second.min_time_between_lc_);

		res->setPoliteness((*iter).second.politeness_);
		res->setRandomPoliteness(AKIGetRandomNumber());
		res->setPolitenessOptional((*iter).second.politeness_optional);
		res->setRandomPolitenessOptional(AKIGetRandomNumber());

		res->setFrictionCoef((*iter).second.friction);

		res->setGapAcceptanceModel(ReadGapModel(exp_id));
		//look ahead parameters of LC
		const unsigned short *dis_lookahead_str =
			AKIConvertFromAsciiString("look_ahead_dis");
		res->setDLCScanRange(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(dis_lookahead_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(dis_lookahead_str);
#else
		delete[] dis_lookahead_str;
#endif

		const unsigned short *dis_lookahead_cars_str =
			AKIConvertFromAsciiString("look_ahead_cars");
		res->setDLCScanNoCars(ANGConnGetAttributeValueInt(
			ANGConnGetAttribute(dis_lookahead_cars_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(dis_lookahead_cars_str);
#else
		delete[] dis_lookahead_cars_str;
#endif

		const unsigned short *acc_exp_str =
			AKIConvertFromAsciiString("acc_exponent");
		res->setAccExp(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(acc_exp_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(acc_exp_str);
#else
		delete[] acc_exp_str;
#endif

		//lane change desires
		const unsigned short *lane_change_desire_thrd_str =
			AKIConvertFromAsciiString("lane_change_desire_thrd");
		double avg_lc_desire = ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(lane_change_desire_thrd_str), exp_id);
#ifdef _DEBUG	
		AKIDeleteUNICODEString(lane_change_desire_thrd_str);
#else
		delete[] lane_change_desire_thrd_str;
#endif

		const unsigned short *lane_change_desire_thrd_dev_str =
			AKIConvertFromAsciiString("lane_change_desire_thrd_dev");
		double lc_desire_dev = ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(lane_change_desire_thrd_dev_str), exp_id);
#ifdef _DEBUG	
		AKIDeleteUNICODEString(lane_change_desire_thrd_dev_str);
#else
		delete[] lane_change_desire_thrd_dev_str;
#endif

		const unsigned short *lane_change_desire_thrd_min_str =
			AKIConvertFromAsciiString("lane_change_desire_thrd_min");
		double lc_desire_min = ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(lane_change_desire_thrd_min_str), exp_id);
#ifdef _DEBUG	
		AKIDeleteUNICODEString(lane_change_desire_thrd_min_str);
#else
		delete[] lane_change_desire_thrd_min_str;
#endif

		const unsigned short *lane_change_desire_thrd_max_str =
			AKIConvertFromAsciiString("lane_change_desire_thrd_max");
		double lc_desire_max = ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(lane_change_desire_thrd_max_str), exp_id);
#ifdef _DEBUG	
		AKIDeleteUNICODEString(lane_change_desire_thrd_max_str);
#else
		delete[] lane_change_desire_thrd_max_str;
#endif

		double lc_desire = sampleNormalDist
			(avg_lc_desire, lc_desire_dev);
		lc_desire =
			(std::min)(lc_desire_max,
			(std::max)(lc_desire, lc_desire_min));
		res->setLaneChangeDesireThrd(lc_desire);

		const unsigned short *dlc_coeff =
			AKIConvertFromAsciiString("dlc_coeff");
		res->setDLCWeight(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(dlc_coeff), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(dlc_coeff);
#else
		delete[] dlc_coeff;
#endif

		const unsigned short *DLC_forbid_zone_before_exit =
			AKIConvertFromAsciiString("DLC_forbid_zone_before_exit");
		res->setDLCForbidZoneBeforeExit(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(DLC_forbid_zone_before_exit), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(DLC_forbid_zone_before_exit);
#else
		delete[] DLC_forbid_zone_before_exit;
#endif

		const unsigned short *right_DLC_coef =
			AKIConvertFromAsciiString("right_DLC_coef");
		res->setRightDLCCoeff(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(right_DLC_coef), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(right_DLC_coef);
#else
		delete[] right_DLC_coef;
#endif

		const unsigned short *lc_gap_reduction_factor_str =
			AKIConvertFromAsciiString("lc_gap_reduction_factor");
		res->setLCGapReductionFactor(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(lc_gap_reduction_factor_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(lc_gap_reduction_factor_str);
#else
		delete[] lc_gap_reduction_factor_str;
#endif

		const unsigned short *sequential_merging_str =
			AKIConvertFromAsciiString("sequential_merging");
		res->SetUnsequentialMerging((ANGConnGetAttributeValueInt(
			ANGConnGetAttribute(sequential_merging_str), exp_id) > 0 ? true : false));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(sequential_merging_str);
#else
		delete[] sequential_merging_str;
#endif

		const unsigned short *e_off_ramp_str =
			AKIConvertFromAsciiString("e_off_ramp");
		res->setOffRampE((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(e_off_ramp_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(e_off_ramp_str);
#else
		delete[] e_off_ramp_str;
#endif

		const unsigned short *t_off_ramp_str =
			AKIConvertFromAsciiString("t_off_ramp");
		res->setOffRampT((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(t_off_ramp_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(t_off_ramp_str);
#else
		delete[] t_off_ramp_str;
#endif

		const unsigned short *penalty_dlc_no_exit_str =
			AKIConvertFromAsciiString("penalty_dlc_no_exit");
		res->setPenaltyDLCNoExitLane((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(penalty_dlc_no_exit_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(penalty_dlc_no_exit_str);
#else
		delete[] penalty_dlc_no_exit_str;
#endif

		const unsigned short *comf_dlc_str =
			AKIConvertFromAsciiString("comf_dec_dlc");
		res->setComfDecDLC((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(comf_dlc_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(comf_dlc_str);
#else
		delete[] comf_dlc_str;
#endif

		const unsigned short *comf_ramplc_str =
			AKIConvertFromAsciiString("comf_dec_ramplc");
		res->setComfDecRampLC((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(comf_ramplc_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(comf_ramplc_str);
#else
		delete[] comf_ramplc_str;
#endif

		const unsigned short *relax_str1 =
			AKIConvertFromAsciiString("relaxation_time");
		res->setRelaxationTime((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(relax_str1), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(relax_str1);
#else
		delete[] relax_str1;
#endif

		//set gap reduction factors
		//forward
		const unsigned short *forward_gap_reduction_onramp_str =
			AKIConvertFromAsciiString("forward_gap_reduction_onramp");
		res->setForwardGapReductionOnRamp((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(forward_gap_reduction_onramp_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(forward_gap_reduction_onramp_str);
#else
		delete[] forward_gap_reduction_onramp_str;
#endif

		const unsigned short *forward_gap_reduction_dlc_str =
			AKIConvertFromAsciiString("forward_gap_reduction_dlc");
		res->setForwardGapReductionDLC((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(forward_gap_reduction_dlc_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(forward_gap_reduction_dlc_str);
#else
		delete[] forward_gap_reduction_dlc_str;
#endif

		const unsigned short *forward_gap_reduction_offramp_str =
			AKIConvertFromAsciiString("forward_gap_reduction_offramp");
		res->setForwardGapReductionOffRamp(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(forward_gap_reduction_offramp_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(forward_gap_reduction_offramp_str);
#else
		delete[] forward_gap_reduction_offramp_str;
#endif
		//backward
		const unsigned short *backward_gap_reduction_onramp_str =
			AKIConvertFromAsciiString("backward_gap_reduction_onramp");
		res->setBackwardGapReductionOnRamp((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(backward_gap_reduction_onramp_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(backward_gap_reduction_onramp_str);
#else
		delete[] backward_gap_reduction_onramp_str;
#endif

		const unsigned short *backward_gap_reduction_offramp_str =
			AKIConvertFromAsciiString("backward_gap_reduction_offramp");
		res->setBackwardGapReductionOffRamp((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(backward_gap_reduction_offramp_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(backward_gap_reduction_offramp_str);
#else
		delete[] backward_gap_reduction_offramp_str;
#endif

		const unsigned short *backward_gap_reduction_dlc_str =
			AKIConvertFromAsciiString("backward_gap_reduction_dlc");
		res->setBackwardGapReductionDLC((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(backward_gap_reduction_dlc_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(backward_gap_reduction_dlc_str);
#else
		delete[] backward_gap_reduction_dlc_str;
#endif

		//
		const unsigned short *increase_DLC_close_ramp_str =
			AKIConvertFromAsciiString("increase_DLC_close_ramp");
		res->setIncreaseDLCCloseRamp((ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(increase_DLC_close_ramp_str), exp_id)));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(increase_DLC_close_ramp_str);
#else
		delete[] increase_DLC_close_ramp_str;
#endif

		res->setOffRampOverpassAcc(1); //offramp use 1 m/s^2 acc to overpass

		//hov related
		//determine if hov lane is active or intended for simulation
		double current_time = AKIGetCurrentSimulationTime();
		bool hov_active = false;
		if (hov_include) // if hov lane is simulated
		{
			//the current time
			if (current_time >= hov_start_time * 60 * 60
				&&
				current_time <= hov_end_time * 60 * 60)
			{
				hov_active = true;
			}
		}
		res->setHOVStart(hov_start_time);
		res->setHOVEnd(hov_end_time);
		res->setHOVIncluded(hov_include);
		res->setHOV(res->getVehType() == HovTypeID);

		//**********************************************
		//Specify CF and LC model parameters for the new vehicle↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

		res->delta = 4;

		const unsigned short *debug_track_id_str =
			AKIConvertFromAsciiString("debug_track_veh_id");
		res->setDebugTrackID(ANGConnGetAttributeValueInt(
			ANGConnGetAttribute(debug_track_id_str), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(debug_track_id_str);
#else
		delete[] debug_track_id_str;
#endif

		//set early lane keep dis
		const unsigned short *earlystr =
			AKIConvertFromAsciiString("early_lane_keep_dis");
		res->setEarlyLaneKeepDis(ANGConnGetAttributeValueDouble(
			ANGConnGetAttribute(earlystr), exp_id));
#ifdef _DEBUG	
		AKIDeleteUNICODEString(earlystr);
#else
		delete[] earlystr;
#endif

		//AKIVehSetAsNoTracked(res->getId());

		res->SetVehTypeIDs(this->CarTypeID, this->HovTypeID, this->TruckTypeID,
			this->CACCTypeID, this->ACCTypeID);
		res->SetInitialVal();
		res->setVehID(res->getId());
		res->setInitialLeaderId(UpdateLatestArrival(res->getId(),
			res->getIdCurrentSection(), res->getIdCurrentLane()));
		res->setSourceSection(res->getIdCurrentSection());
		res->CoopRequester = NULL;
		res->LastCoopRequester = NULL;
	}
}


void mybehavioralModel::ReadHOVSetting()
{
	int exp_id = ANGConnGetExperimentId();
	const unsigned short *hov_rule_active = AKIConvertFromAsciiString(
		"hov_simulation");
	this->hov_include = ANGConnGetAttributeValueBool(ANGConnGetAttribute(hov_rule_active), exp_id);
	AKIDeleteUNICODEString(hov_rule_active);

	if (this->hov_include)
	{
		const unsigned short *hov_time = AKIConvertFromAsciiString(
			"hov_start_time");
		this->hov_start_time = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(hov_time), exp_id);

		hov_time = AKIConvertFromAsciiString(
			"hov_end_time");
		this->hov_end_time = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(hov_time), exp_id);
		AKIDeleteUNICODEString(hov_time);
	}

	//whether or not the hov rule is included, we always need to create hov vehicles according to a certain percentage
	const unsigned short *hov_percent = AKIConvertFromAsciiString(
		"hov_percentage");
	this->hov_percentage = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(hov_percent), exp_id);
	AKIDeleteUNICODEString(hov_percent);
}


/* Obtain CF, LC and behavior parameters for individual vehicle types------------------------------------*/
void mybehavioralModel::readVehTypeData(int vehTypeId)
{
	int delegate_vehtype = vehTypeId;

	//acc and cacc vehicle share some of the common setting of car type
	if (vehTypeId == ACCTypeID
		||
		vehTypeId == CACCTypeID)
	{
		vehTypeId = CarTypeID;
	}

	A2BehavioralVehData data;
	const unsigned short *meanJamString = AKIConvertFromAsciiString(
		"GDrivingSimPluging::GKVehicle::Jam Gap Mean");
	data.meanJam = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(meanJamString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(meanJamString);
#else
	delete[] meanJamString;
#endif	

	const unsigned short *devJamString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Jam Gap Deviation");
	data.devJam = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(devJamString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(devJamString);
#else
	delete[] devJamString;
#endif	

	const unsigned short *maxJamString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Jam Gap Max");
	data.maxJam = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(maxJamString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(maxJamString);
#else
	delete[] maxJamString;
#endif	

	const unsigned short *minJamString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Jam Gap Minimum");
	data.minJam = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(minJamString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(minJamString);
#else
	delete[] minJamString;
#endif	

	const unsigned short *meanEString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Mean E");
	data.meanE = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(meanEString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(meanEString);
#else
	//delete[] meanJamString;
	AKIDeleteUNICODEString(meanEString);
#endif	

	const unsigned short *devEString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::dev E");
	data.devE = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(devEString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(devEString);
#else
	delete[] devEString;
#endif	

	const unsigned short *maxEString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Max E");
	data.maxE = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(maxEString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(maxEString);
#else
	delete[] maxEString;
#endif	

	const unsigned short *minEString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Min E");
	data.minE = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(minEString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(minEString);
#else
	delete[] minEString;
#endif	

	const unsigned short *meanTString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Mean T");
	data.meanT = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(meanTString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(meanTString);
#else
	delete[] meanTString;
#endif	

	const unsigned short *devTString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::dev T");
	data.devT = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(devTString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(devTString);
#else
	delete[] devTString;
#endif	

	const unsigned short *maxTString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Max T");
	data.maxT = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(maxTString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(maxTString);
#else
	delete[] maxTString;
#endif

	const unsigned short *minTString = AKIConvertFromAsciiString("GDrivingSimPluging::GKVehicle::Min T");
	data.minT = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(minTString), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(minTString);
#else
	delete[] minTString;
#endif

	//const unsigned short *distConflictString = AKIConvertFromAsciiString( "GKVehicle::distConflict" );
	//   data.distConflict = ANGConnGetAttributeValueInt( ANGConnGetAttribute( distConflictString ), vehTypeId );

	//set maximum give way time as maximum
	const unsigned short *giveWayDev = AKIConvertFromAsciiString("GKVehicle::giveWayDev");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(giveWayDev), vehTypeId, 0.0);
#ifdef _DEBUG
	AKIDeleteUNICODEString(giveWayDev);
#else
	delete[] giveWayDev;
#endif
	const unsigned short *giveWayMax = AKIConvertFromAsciiString("GKVehicle::giveWayMax");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(giveWayMax), vehTypeId, 60000000.0);
#ifdef _DEBUG
	AKIDeleteUNICODEString(giveWayMax);
#else
	delete[] giveWayMax;
#endif
	const unsigned short *giveWayMin = AKIConvertFromAsciiString("GKVehicle::giveWayMin");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(giveWayMin), vehTypeId, 60000000.0);
#ifdef _DEBUG
	AKIDeleteUNICODEString(giveWayMin);
#else
	delete[] giveWayMin;
#endif
	const unsigned short *giveWayMean = AKIConvertFromAsciiString("GKVehicle::giveWayMean");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(giveWayMean), vehTypeId, 60000000.0);
#ifdef _DEBUG
	AKIDeleteUNICODEString(giveWayMean);
#else
	delete[] giveWayMean;
#endif

	//setup reaction time
	const unsigned short *min_reaction_String =
		AKIConvertFromAsciiString("reaction_time_min_");
	data.min_reaction_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(min_reaction_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(min_reaction_String);
#else
	delete[] min_reaction_String;
#endif

	const unsigned short *max_reaction_String =
		AKIConvertFromAsciiString("reaction_time_max_");
	data.max_reaction_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(max_reaction_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(max_reaction_String);
#else
	delete[] max_reaction_String;
#endif

	const unsigned short *dev_reaction_String =
		AKIConvertFromAsciiString("reaction_time_dev_");
	data.dev_reaction_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(dev_reaction_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(dev_reaction_String);
#else
	delete[] dev_reaction_String;
#endif

	const unsigned short *avg_reaction_String =
		AKIConvertFromAsciiString("reaction_time_avg_");
	data.avg_reaction_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(avg_reaction_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(avg_reaction_String);
#else
	delete[] avg_reaction_String;
#endif

	//minimum time between lane changes
	const unsigned short *min_time_lcs_str =
		AKIConvertFromAsciiString("min_time_between_lc_");
	data.min_time_between_lc_ =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(min_time_lcs_str), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(min_time_lcs_str);
#else
	delete[] min_time_lcs_str;
#endif

	data.min_time_between_lc_ =
		MAX(1, data.min_time_between_lc_);

	//politeness to lane changers
	const unsigned short *politeness_str =
		AKIConvertFromAsciiString("polite_");
	data.politeness_ =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(politeness_str), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(politeness_str);
#else
	delete[] politeness_str;
#endif

	const unsigned short *politeness_optional_str =
		AKIConvertFromAsciiString("polite_optional");
	data.politeness_optional =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(politeness_optional_str), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(politeness_optional_str);
#else
	delete[] politeness_optional_str;
#endif

	data.politeness_ =
		MIN(MAX(0, data.politeness_), 1);
	data.politeness_optional =
		MIN(MAX(0, data.politeness_optional), 1);

	//cross-lane friction
	const unsigned short *friction_String =
		AKIConvertFromAsciiString("lane_friction");
	data.friction =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(friction_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(friction_String);
#else
	delete[] friction_String;
#endif

	//////////////////////////////////////////////////////////////////////////
	//the following parameter is specific to the type given the input
	//////////////////////////////////////////////////////////////////////////
	vehTypeId = delegate_vehtype;
	const unsigned short *min_headway_String =
		AKIConvertFromAsciiString("headway_min");
	data.min_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(min_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(min_headway_String);
#else
	delete[] min_headway_String;
#endif

	const unsigned short *max_headway_String =
		AKIConvertFromAsciiString("headway_max");
	data.max_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(max_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(max_headway_String);
#else
	delete[] max_headway_String;
#endif

	const unsigned short *dev_headway_String =
		AKIConvertFromAsciiString("headway_dev");
	data.dev_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(dev_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(dev_headway_String);
#else
	delete[] dev_headway_String;
#endif

	const unsigned short *avg_headway_String =
		AKIConvertFromAsciiString("headway_mean");
	data.avg_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(avg_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(avg_headway_String);
#else
	delete[] avg_headway_String;
#endif

	vehTypeData[vehTypeId] = data;
}
/* Obtain CF, LC and behavior parameters for individual vehicle types---------------------------------------*/



//----------------------------------------------------------------------------------------------------------
//			Miscellaneous functions
//----------------------------------------------------------------------------------------------------------

// Function :sampleNormalDist
// Generate a normal(mean, std) random number according to the polar method
//
double mybehavioralModel::sampleNormalDist(double mean, double std)
{
	double W = 0.0;
	double V1 = 0.0;
	double V2 = 0.0;
	double X = 0.0;
	double Y = 0.0;
	double val = 0.0;
	double U1 = 0.0;
	double U2 = 0.0;

	do{
		U1 = AKIGetRandomNumber();
		U2 = AKIGetRandomNumber();
		V1 = 2 * U1 - 1;
		V2 = 2 * U2 - 1;
		W = V1*V1 + V2*V2;
	} while (W >= 1);

	Y = sqrt(MAX(0.0, -2 * log(W) / MAX(1.0, W)));
	X = Y*V1;
	val = mean + X*std;

	return val;
}

int mybehavioralModel::ReadGapModel(int exp_id)
{
	const unsigned short *gap_model_code = AKIConvertFromAsciiString(
		"gap_model");
	int temp = ANGConnGetAttributeValueInt(ANGConnGetAttribute(gap_model_code), exp_id);

#ifdef _DEBUG
	AKIDeleteUNICODEString(gap_model_code);
#else
	delete[] gap_model_code;
#endif
	return temp;
}

/* Return the ID of the first vehicle on a lane of a section+++++++++++++++++++++++++++++++++++++++++++++*/
// This function helps identify the leader of a new arrival vehicle
int mybehavioralModel::UpdateLatestArrival(int vid, int secid, int lane_id)
{
	std::map<int, std::map<int, int>>::iterator it;
	it = s_vids.find(secid);
	if (it == s_vids.end())
	{
		std::map<int, int> tempmap;
		tempmap.insert(std::pair<int, int>(lane_id, vid));
		s_vids.insert(std::pair<int, std::map<int, int>>(secid, tempmap));
		return -1;
	}
	else
	{
		std::map<int, int> tempmap = it->second;
		std::map<int, int>::iterator it2 = tempmap.find(lane_id);
		if (it2 == tempmap.end())
		{
			it->second.insert(std::pair<int, int>(lane_id, vid));
			return -1;
		}
		else
		{
			int tempid = it2->second;
			it->second[lane_id] = vid;
			return tempid;
		}
	}
}
/* Store the last vehicle of each section on each lane+++++++++++++++++++++++++++++++++++++++++++++*/


//read parameters from external txt file for parameters
void mybehavioralModel::ReadExternalParameters()
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
			PrintString(key_value);

			std::string value_string = line.substr(tempindex + 1, index - tempindex - 1);
			PrintString(value_string);

			double param_value = atof(value_string.c_str());
			hashmap.insert(std::pair<std::string, double>(key_value, param_value));

			line = line.substr(index + 1, line.length() - index - 1);
		}
		break; //only read one line
	}
	infile.close();
}



void mybehavioralModel::PrintString(std::string key_value)
{
	char *cstr = new char[key_value.length() + 1];
	strcpy_s(cstr, key_value.length() + 1, key_value.c_str());
	AKIPrintString(cstr);
	delete(cstr);
}

void mybehavioralModel::SetExternalParameters()
{
	if (this->hashmap.size() <= 0)
		return;
	else
	{
		int scenario_id = ANGConnGetScenarioId();
		int exp_id = ANGConnGetExperimentId();
		typedef std::map<std::string, double>::iterator it_type;
		for (it_type iterator = hashmap.begin(); iterator != hashmap.end(); iterator++)
		{
			std::string name = iterator->first;
			double param_value = iterator->second;

			//find a set value
			int id = 0;
			if (name.find("sce_") != std::string::npos)
			{
				id = scenario_id;
			}
			else if (name.find("exp_") != std::string::npos)
			{
				id = exp_id;
			}
			else if (name.find("car_") != std::string::npos)
			{
				id = CarTypeID;
			}
			else if (name.find("sec_") != std::string::npos)
			{
				int num_sec = AKIInfNetNbSectionsANG();
				name.erase(0, 4);
				const unsigned short *temp_str = AKIConvertFromAsciiString(name.c_str());
				for (int i = 0; i < num_sec; i++)
				{
					int id = AKIInfNetGetSectionANGId(i);
					double ext_value = ANGConnGetAttributeValueDouble(
						ANGConnGetAttribute(temp_str), id);
					if (ext_value != 0)
						ANGConnSetAttributeValueDouble(
						ANGConnGetAttribute(temp_str), id, param_value);

				}
#ifdef _DEBUG
				AKIDeleteUNICODEString(temp_str);
#else
				delete[] temp_str;
#endif
				return;
			}
			else
			{
				return;
			}
			name.erase(0, 4);
			if (name.find("flow") != std::string::npos) //times 100 for acc/cacc; times 1000 for volumes
			{
				param_value = param_value * 1000;
			}
			else if (name.find("ACC") != std::string::npos
				|| name.find("CACC") != std::string::npos)
			{
				if (DisableExternalACCCACC())
					continue;
			}
			const unsigned short *temp_str = AKIConvertFromAsciiString(name.c_str());
			ANGConnSetAttributeValueDouble(
				ANGConnGetAttribute(temp_str), id, param_value); //set friction for all types of vehicles if this is a car parameter!
#ifdef _DEBUG
			AKIDeleteUNICODEString(temp_str);
#else
			delete[] temp_str;
#endif

		}
	}
}

bool mybehavioralModel::DisableExternalACCCACC()
{
	int expid = ANGConnGetExperimentId();
	const unsigned short *temp_str = AKIConvertFromAsciiString("disable_external_caccacc");
	bool ret = (bool)ANGConnGetAttributeValueDouble(ANGConnGetAttribute(temp_str), expid);
	AKIDeleteUNICODEString(temp_str);
	return ret;
}




void mybehavioralModel::SetGlobalSetParameters(myVehicleDef * res)
{
	res->cacc_percent = this->cacc_percent;
	res->acc_percent = this->acc_percent;
	res->parastr = this->parastr;
}



//////////////////////////////////////////////////////////////////////////
// determine if this network is specifically for merging networks
//////////////////////////////////////////////////////////////////////////
bool mybehavioralModel::UseMergingDemandReading()
{
	int expid = ANGConnGetExperimentId();
	const unsigned short *temp_str = AKIConvertFromAsciiString("read_matrix_demand");
	bool ret = (bool)ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(temp_str), expid);
	AKIDeleteUNICODEString(temp_str);
	return ret;
}


