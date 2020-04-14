/****************************************************************

	 To modify the demand at the beginning of the simulation run
	 It will not work this way!!!

	 ****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>
#include <numeric>
#include <map>
#include <fstream>      // std::ifstream
#include <algorithm>    // std::find
#include <string>
#include <sstream>
#include "AKIProxie.h"

#define Car_Type  53  
#define HOVveh_Type  17859
#define HIAveh_Type  21350  // 811
#define Truck_Type  56  
//#define ACCveh_Type  344  // 812
//#define CACCveh_Type 346  // 813

#define ACCveh_Type  344  // 812
#define CACCveh_Type 346  // 813
#define AHMveh_Type 3369
#define IVeh_Type 3668
#define HWYV2V_veh_Type 3934
#define RAMPV2V_veh_Type 3935
#define HWYV2V_CACC 4369
#define NAGISM_ACC 4476
int Exp_Veh_Type;
int Mainline_Exp_Veh_Type;

#define RAMP_ID 23554
#define MAINLANE 23552

#define CONST_OFFRAMP 12270
#define CONST_LASTSEC 12266
#define CONST_BEFORELASTSEC 12256
#define SPLIT_RATIO 0.1

#define ACC_LENGTH 800 //in feet  243.84 meters
#define FREE_FLOW_SPEED 30.0 //[MPH]
#define ORIGIN_MAINLINE 11960 //SOURCE MAINLINE IN SR99 NETWORK FILE

std::map<int, std::vector<int>> time_next; //the absolute time in seconds that the next vehicle is coming from an origin 
std::vector<int> num_veh_entering;
std::map<int, std::vector<double>> avg_headway_origin; //the time period (secs) between the vehicles coming from an origin centroid
std::map<int, std::vector<double>> min_headway_origin; //required minimum time period (secs) between the vehicles coming from an origin centroid


std::map<int, std::vector<std::vector<double>>> flows;
std::map<int, std::vector<std::vector<double>>> car_flows;
std::map<int, std::vector<std::vector<double>>> truck_flows;
std::map<int, std::vector<std::vector<double>>> hov_flows;
std::map<int, std::vector<std::vector<double>>> truck_portions;
std::map<int, std::vector<double>> turning_portions; //turning proportions
std::map<int, std::vector<std::vector<double>>> turning_portions_bytime; //turning proportions
std::map<int, std::vector<std::vector<double>>> Truck_turning_portions_bytime; //turning proportions
std::map<int, std::vector<std::vector<double>>> HOV_turning_portions_bytime; //turning proportions
std::map<int, std::map<int, double>> dest; // the dest of a origin and its flow proportion
std::map<int, int> orgin_section; // the section that connects the origin
std::vector<std::map<int, std::map<int, double>>> od_maps; //the first dimension is time; and then each map element is a od in that time period
std::vector<std::map<int, std::map<int, double>>> truck_od_maps; //the first dimension is time; and then each map element is a od in that time period
std::vector<std::map<int, std::map<int, double>>> hov_od_maps; //the first dimension is time; and then each map element is a od in that time period
double global_acc = 0;
double global_cacc = 0;
int global_interval = 5; //minutes
//int interval_shift = 7;//in hours
int interval_shift = 0;//in hours for lin's dataset
int num_periods = 0;
std::map<int, A2KSectionInf> hashmap; //section info hashmap
bool hov_include; //if hov lane is included for simulation
double hov_start_time; // hov start time with respect to the start time of the simulation
double hov_end_time;  // hov end time with respect to the start time of the simulation
double hov_percentage;  // HOV percentage used when hov rule is not active or for general purpose lanes
std::vector<int> source_sec; //source sections
std::vector<int> dest_sec; //source sections
std::map<int, int> D_section; //hashmap of destination section
std::map<int, int> O_section; //hashmap of origin section
std::map<int, std::map<int, double>> manual_turning_ratios; //define manual turning ratios
std::map<int, int> ramptype_map; //ramp type hashmap

int CarTypeID;
int TruckTypeID;
int HovTypeID;
int CACCTypeID;
int ACCTypeID;
int HIATypeID;




int dmd_modify(double T)
{
	//int start_up_slice, start_dwn_slice;
	int i, j, current_slice = 0;
	double input_dmd[NumOnRamp][max_onramp_ln] = { { 2000.0, 2000.0, 2000.0 } };
	double new_dmd[NumOnRamp][max_onramp_ln] = { { 2000.0, 2000.0, 2000.0 } };
	static int init_sw = 1, dmd_up_sw = 0, dmd_dwn_sw = 0;

	if (init_sw == 1)
	{
		for (i = 0; i < NumOnRamp; i++)
		{
			for (j = 0; j < max_onramp_ln; j++)
			{
				input_dmd[i][j] = 0.0;
				new_dmd[i][j] = 0.0;
			}
		}
		init_sw = 0;
	}

	current_slice = (int)floor(T / dmd_slice_len);   // 300[s] per spice

	for (i = 0; i < NumOnRamp; i++)
	{
		for (j = 0; j < max_onramp_ln; j++)
		{
			if (OnRamp_Ln_Id[i][j]>0)
			{
				if (DEMAND_CHANGE == 0)
				{
					input_dmd[i][j] = AKIStateDemandGetDemandSection(OnRamp_Ln_Id[i][j], ALLvehType, current_slice);
					new_dmd[i][j] = (input_dmd[i][j]);
				}
				else if (DEMAND_CHANGE == 1)
				{
					if (i == 0 || i == 5 || i == 6 || i == 7 || i == 8)
					{
						input_dmd[i][j] = AKIStateDemandGetDemandSection(OnRamp_Ln_Id[i][j], ALLvehType, current_slice);
						new_dmd[i][j] = (input_dmd[i][j])*(1.0 + dmd_change);
					}
					else  //if (DEMAND_CHANGE==2)
					{
						input_dmd[i][j] = AKIStateDemandGetDemandSection(OnRamp_Ln_Id[i][j], ALLvehType, current_slice);
						new_dmd[i][j] = (input_dmd[i][j]);
					}

				}
				else
				{
					input_dmd[i][j] = AKIStateDemandGetDemandSection(OnRamp_Ln_Id[i][j], ALLvehType, current_slice);
					new_dmd[i][j] = (input_dmd[i][j])*(1.0 + dmd_change);
				}

				if (AKIStateDemandSetDemandSection(OnRamp_Ln_Id[i][j], CarTypeID, current_slice, new_dmd[i][j]) < 0)
					AKIPrintString("Demand change failed!");
				if (AKIStateDemandSetDemandSection(OnRamp_Ln_Id[i][j], HIATypeID, current_slice, new_dmd[i][j]) < 0)
					AKIPrintString("Demand change failed!");
				if (AKIStateDemandSetDemandSection(OnRamp_Ln_Id[i][j], ACCTypeID, current_slice, new_dmd[i][j]) < 0)
					AKIPrintString("Demand change failed!");
				if (AKIStateDemandSetDemandSection(OnRamp_Ln_Id[i][j], CACCTypeID, current_slice, new_dmd[i][j]) < 0)
					AKIPrintString("Demand change failed!");

	/*			if ((i == 0) && (j == 0))
					fprintf(dmd_f, "%lf\t%d\t%lf\t%lf\t ", T, current_slice, input_dmd[0], new_dmd[0]);
				else
					fprintf(dmd_f, "%lf\t%lf\t", input_dmd[i][j], new_dmd[i][j]);
				if ((i == NumOnRamp - 1) && (j == max_onramp_ln - 1))
					fprintf(dmd_f, "%lf\t%lf\n", input_dmd[i][j], new_dmd[i][j]);*/
			}
		} //j-loop end
	} //i-loop end

	return 1;
}

void read_precentage(double &acc_percent, double &cacc_percent)
{
	int expriment_id = ANGConnGetExperimentId();
	const unsigned short *CACC_PercentString = AKIConvertFromAsciiString(
		"CACC_Percent");
	cacc_percent = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(CACC_PercentString), expriment_id);
	//delete[] CACC_PercentString;
	AKIDeleteUNICODEString(CACC_PercentString);

	const unsigned short *ACC_PercentString = AKIConvertFromAsciiString(
		"ACC_Percent");
	acc_percent = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(ACC_PercentString), expriment_id);
	//delete[] ACC_PercentString;
	AKIDeleteUNICODEString(ACC_PercentString);

	const unsigned short *simStepAtt = AKIConvertFromAsciiString(
		"GKExperiment::simStepAtt");
	ANGConnSetAttributeValueDouble(ANGConnGetAttribute(simStepAtt), expriment_id, 0.1);
	//delete[] simStepAtt;
	AKIDeleteUNICODEString(simStepAtt);

}

//read ramp and main-lane volume
void read_volume(int &mainlane, int &on_ramp, int &off_ramp)
{
	int exp_id = ANGConnGetExperimentId();
	const unsigned short *MAINLANE_PercentString =
		AKIConvertFromAsciiString(
		"through_flow");
	mainlane = (int)ANGConnGetAttributeValueDouble
		(ANGConnGetAttribute(MAINLANE_PercentString), exp_id);
	AKIDeleteUNICODEString(MAINLANE_PercentString);

	const unsigned short *on_RAMP_PercentString =
		AKIConvertFromAsciiString(
		"on_ramp_flow");
	on_ramp = (int)ANGConnGetAttributeValueDouble
		(ANGConnGetAttribute(on_RAMP_PercentString), exp_id);
	AKIDeleteUNICODEString(on_RAMP_PercentString);

	const unsigned short *off_RAMP_PercentString =
		AKIConvertFromAsciiString(
		"off_ramp_flow");
	off_ramp = (int)ANGConnGetAttributeValueDouble
		(ANGConnGetAttribute(off_RAMP_PercentString), exp_id);
	AKIDeleteUNICODEString(off_RAMP_PercentString);

}

//////////////////////////////////////////////////////////////////////////
// lane capacity per hour
//////////////////////////////////////////////////////////////////////////
double Lane_Capacity()
{
	//this only serve as a rough estimation
	const unsigned short *avg_headway_String =
		AKIConvertFromAsciiString("headway_mean");
	double avg_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(avg_headway_String), CarTypeID);
	//delete[] avg_headway_String;
	AKIDeleteUNICODEString(avg_headway_String);

	const unsigned short *meanJamString = AKIConvertFromAsciiString(
		"GDrivingSimPluging::GKVehicle::Jam Gap Mean");
	double meanJam = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(meanJamString), CarTypeID);
	//delete[] meanJamString;
	AKIDeleteUNICODEString(meanJamString);

	return FREE_FLOW_SPEED / (meanJam + 4.0 + FREE_FLOW_SPEED*avg_headway_time) * 3600;
}

std::vector<double> getHwyDist(int onramp_flow, double acc_length, double avg_headway)
{

	std::vector<double> hwys;
	bool Even_distribution = true;

	if (Even_distribution)
	{
		hwys.push_back(avg_headway / (0.5*0.5));  //4th lane 
		hwys.push_back(avg_headway / (0.5*0.5));  //3rd lane
		hwys.push_back(avg_headway / (0.5*0.5));  //2nd lane
		hwys.push_back(avg_headway / (0.5*0.5));  //1st lane  //inner-most
	}
	else
	{
		double total_flow = 3600.0 / avg_headway; //total flow
		/*hwys.push_back(avg_headway*4.0);
		hwys.push_back(avg_headway*4.0);
		hwys.push_back(avg_headway*4.0);
		hwys.push_back(avg_headway*4.0);
		return hwys;*/
		//then check if assigning proportional flow to the inside lane 
		//will exceed capacity

		//onramp_flow = 200;
		double prop = (0.2178 - 0.000125*onramp_flow + 0.01115*acc_length / 50);
		double flow_inside_lane = total_flow*(1 - prop)*0.5;  //inside => 1st 2nd left-most lanes.

		double capacity = Lane_Capacity(); //Maximum capacity per lane.
		if (flow_inside_lane > Lane_Capacity()) //if inside lane flow is over the maximum of the lane capacity (roughly estimation)
		{ //Try to put more vehicle on the outside lane 
			double flow_outer = (total_flow - capacity * 2)*0.6;
			if (flow_outer > capacity)
			{
				hwys.push_back(3600.0 / capacity);
				hwys.push_back(3600.0 / capacity);
			}
			else  //put more vehicle on the outside lane if it doesn't exceed the capacity.
			{
				hwys.push_back(3600.0 / flow_outer / 0.6*0.5);
				hwys.push_back(3600.0 / flow_outer);
			}
			hwys.push_back(3600.0 / capacity);
			hwys.push_back(3600.0 / capacity);
		}
		else // if flow_inside doesn't exceed lane capacity.
		{
			hwys.push_back(avg_headway / (prop*0.4));  //4th lane 
			hwys.push_back(avg_headway / (prop*0.6));  //3rd lane
			//hwys.push_back(avg_headway/(prop*0.5));  //4th lane 
			//hwys.push_back(avg_headway/(prop*0.5));  //3rd lane
			hwys.push_back(avg_headway / ((1 - prop)*0.5));  //2nd lane
			hwys.push_back(avg_headway / ((1 - prop)*0.5));  //1st lane
		}

	}

	return hwys;
}



int mainlane_id;
int on_ramp_id;

//Modify demand by OD matrix
void ModifyMatrixDemand(double acc_percent, double cacc_percent)
{
	//all of the sections
	int slice_num = AKIODDemandGetNumSlicesOD(1);
	int vtp_num = AKIVehGetNbVehTypes();

	//from mainlane to mainlane
	int total_through = 0;
	//off_ramp from mainlane
	int off_ramp = 0; //off_ramp vehicles can only be manual driven
	//on ramp to mainlane and exit from mainlane
	int on_ramp = 0;//on_ramp vehicles can only be manual driven

	//read demand
	/*if(IsBatchMode())
	{
	ResetVolumeFromFile();
	}*/
	read_volume(total_through, on_ramp, off_ramp);

	////determine the result from HCM method

	int ACC_pos = AKIVehGetVehTypeInternalPosition(ACCTypeID);
	int CACC_pos = AKIVehGetVehTypeInternalPosition(CACCTypeID);
	int Car_pos = AKIVehGetVehTypeInternalPosition(CarTypeID);

	int nCentroid = AKIInfNetNbCentroids();

	int mainlane_centroid_origin = 0;
	int on_ramp_centroid = 0;
	int off_ramp_centroid = 0;
	int mainlane_centroid_dest = 0;

	mainlane_id = 0;
	on_ramp_id = 0;

	for (int i = 0; i < nCentroid; i++)
	{
		int iId = AKIInfNetGetCentroidId(i);
		A2KCentroidInf centroid_info =
			AKIInfNetGetCentroidInf(iId);
		if (centroid_info.IsOrigin == true)  //Origin Centroid
		{
			for (int j = 0; j < centroid_info.NumConnecTo; j++) //Iterate over every origin centroid
			{
				bool tempVar;
				int section_id =
					AKIInfNetGetIdObjectofOriginCentroidConnector(iId, j, tempVar);
				if (orgin_section.find(section_id) == orgin_section.end())  //Bug here<----------Should insert the Key element into the find function.
				{
					orgin_section.insert(std::pair<int, int>(iId,
						section_id));
				}
				A2KSectionInf section_info =
					AKIInfNetGetSectionANGInf(section_id);
				//find the on-ramp for the test network
				if (section_info.nbCentralLanes == 1) //1 lane --> Onramp
				{
					on_ramp_id = section_id;
					on_ramp_centroid = iId;
				}
				else//multiple lanes --> Offramp
				{
					mainlane_id = section_id;
					mainlane_centroid_origin = iId;
				}
			}
		}
		else //Destination Centroid
		{
			for (int j = 0; j < centroid_info.NumConnecFrom; j++)
			{
				bool tempVar;
				int section_id =
					AKIInfNetGetIdObjectofDestinationCentroidConnector(iId, j, tempVar);
				A2KSectionInf section_info =
					AKIInfNetGetSectionANGInf(section_id);
				//find the on-ramp for the test network
				if (section_info.nbCentralLanes == 1) //One lane --> offramp destination
				{
					off_ramp_centroid = iId;
				}
				else //Multiple lane --> Highway destination
				{
					mainlane_centroid_dest = iId;

				}
			}
		}
	}

	for (int j = 0; j < slice_num; j++)
	{
		////*******Setup OD pairs
		////*******Mainlane to Mainlane


		//get total demand from the mainlane origin
		int total_mainlane = total_through
			+ off_ramp;
		//int total_mainlane = total_through;

		double avg_headway = 3600.0 / (double)total_mainlane;
		std::vector<double> hwy_dist = getHwyDist(on_ramp, ACC_LENGTH, avg_headway);
		avg_headway_origin.insert(
			std::pair<int, std::vector<double>>(mainlane_centroid_origin, hwy_dist));
		std::vector<double> temp_min;
		double min_hwy = 0.8;   //1.5 -->Dali's setting
		temp_min.push_back(min_hwy); temp_min.push_back(min_hwy); temp_min.push_back(min_hwy); temp_min.push_back(min_hwy);
		min_headway_origin.insert(
			std::pair<int, std::vector<double>>(mainlane_centroid_origin, temp_min));

		//get total demand from the on-ramp
		int onramp_flow = on_ramp;
		avg_headway = 3600.0 / (double)onramp_flow;
		std::vector<double> temp1; temp1.push_back(avg_headway);
		avg_headway_origin.insert(
			std::pair<int, std::vector<double>>(on_ramp_centroid, temp1));
		std::vector<double> temp2; temp2.push_back(1.0);
		min_headway_origin.insert(
			std::pair<int, std::vector<double>>(on_ramp_centroid, temp2));

		// initialize the next time vector
		if (time_next.find(mainlane_centroid_origin) == time_next.end())
		{
			num_veh_entering.push_back(0);
			num_veh_entering.push_back(0);
			num_veh_entering.push_back(0);
			num_veh_entering.push_back(0);

			std::vector<int> mainlane_vector;
			mainlane_vector.push_back(0);
			mainlane_vector.push_back(0);
			mainlane_vector.push_back(0);
			mainlane_vector.push_back(0);
			time_next.insert
				(std::pair<int, std::vector<int>>
				(mainlane_centroid_origin, mainlane_vector));

			std::map<int, double> destinations;
			if (total_through + off_ramp > 0)
			{
				destinations.insert(std::pair<int, double>
					(mainlane_centroid_dest, (double)(total_through) / ((double)total_mainlane)));
				destinations.insert(std::pair<int, double>
					(off_ramp_centroid, 1.0));
				//destinations.insert(std::pair<int, double>
				//	(mainlane_centroid_dest, 1.0));
				//destinations.insert(std::pair<int, double>
				//	(off_ramp_centroid, 1.0));
			}
			dest.insert(std::pair<int, std::map<int, double>>
				(mainlane_centroid_origin, destinations));
		}

		if (time_next.find(on_ramp_centroid) == time_next.end())
		{
			// initialize the next time vector
			std::vector<int> on_ramp;
			on_ramp.push_back(0);
			time_next.insert
				(std::pair<int, std::vector<int>>
				(on_ramp_centroid, on_ramp));

			std::map<int, double> destinations;
			destinations.insert(std::pair<int, double>
				(mainlane_centroid_dest, 1));
			dest.insert(std::pair<int, std::map<int, double>>
				(on_ramp_centroid, destinations));
		}
	}

}


//////////////////////////////////////////////////////////////////////////
// generate a random exponential distributed variable
//////////////////////////////////////////////////////////////////////////
double RandomExpHeadway(double min_hwy, double avg_hwy)
{
	if (avg_hwy > 0)
	{
		double x = AKIGetRandomNumber(); // (double)rand() / RAND_MAX;

		//if (avg_hwy < min_hwy)
		//	return min_hwy;
		//else if (avg_hwy <= 1.1)
		//{//do uniform distribution strategy if the traffic flow is high.
		//	x = (x - 0.5) * 0;
		//	x = x + avg_hwy;
		//	return x;
		//}
		//else
		//{
		//	double laneflow = 3600.0 / avg_hwy;
		//	double lambda = laneflow / 3600.0 / (1 - min_hwy * laneflow / 3600.0);
		//	//double lambda = 1/avg_hwy;
		//	double theta = exp(lambda * min_hwy);
		//	//double x = AKIGetRandomNumber();
		//	x = log((1 - x) / theta) / (-lambda);
		//}

		if (avg_hwy < min_hwy)
			return min_hwy;
		else
		{
				double laneflow = 3600.0 / avg_hwy;
				double lambda = laneflow / 3600.0 / (1 - min_hwy * laneflow / 3600.0);
				//double lambda = 1/avg_hwy;
				double theta = exp(lambda * min_hwy);
				//double x = AKIGetRandomNumber();
				x = log((1 - x) / theta) / (-lambda);
			//x = avg_hwy;


		}

			
		if (x < min_hwy)
			return min_hwy;
		else if (x > 0)
		{
			return x;
		}
		else
		{
			return min_hwy;
		}
		
	}
	else
		return min_hwy;
}

double round(double x)
{
	return floor(x + 0.5);
}






//////////////////////////////////////////////////////////////////////////
// Generate vehicle from Centroid
//////////////////////////////////////////////////////////////////////////
int dmd_generate_matrix(double time, double timeSta, double timTrans, double acicle)
{
	//get simulation steps
	int current_step = (int)(round(time / acicle));
	//iterate of vector
	for (int i = 0; i < time_next.size(); ++i) //map::size, number of the element in the map == number of source centroids
	{
		for (int j = 0; j < AKIInfNetNbCentroids(); ++j)
		{
			int origin = AKIInfNetGetCentroidId(j);

			if (time_next.find(origin) != time_next.end())  //Check if this origin_id is in the map time_next
			{
				std::vector<int> times = time_next[origin];  //return the vector with key==origin 

				for (int k = 0; k < times.size(); ++k)  //iterate each lane from the origin
				{
					int next_time = times.at(k);
					if (next_time <= current_step) //If it's time to generate a vehicle from origin j to lane k
					{
						//generate a new vehicle here
						//first determine its destination
						double rand_num = AKIGetRandomNumber(); //random number between 0 to 1.
						std::map<int, double>::iterator _iterator;

						//go over the destinations index
						for (_iterator = dest[origin].begin(); _iterator != dest[origin].end(); ++_iterator)
						{

							//Pick up one of the destination randomly. 

							if (rand_num <= _iterator->second) //dest[origin][n][1] is the flow proportion 
								// of the specific origin and destination
							{
								//find the destination
								int denstine = _iterator->first;

								//determine the vehicle type
								rand_num = AKIGetRandomNumber();
								int veh_type = 0;

								//check if it is the on-ramp road
								if (times.size() == 1) //This section has only one lane. This means that this origin is on-ramp origin.
								{
									if (Exp_Veh_Type == IVeh_Type)
									{
										veh_type = IVeh_Type;
									}
									else if (Exp_Veh_Type == AHMveh_Type)
									{
										veh_type = AHMveh_Type;
									}
									else if (Exp_Veh_Type == RAMPV2V_veh_Type)
									{
										veh_type = RAMPV2V_veh_Type;
									}
									else if (Exp_Veh_Type == Car_Type)
									{
										veh_type = Car_Type;
									}
									else
									{
										if (rand_num < ACC_percent)
											veh_type = NAGISM_ACC;
										else if (rand_num < ACC_percent +
											CACC_percent)
											veh_type = HWYV2V_CACC;
										else
											veh_type = Car_Type;

										////veh_type = Car_Type;
										//if (rand_num * 100 < CACC_percent)
										//	veh_type = RAMPV2V_veh_Type;
										//else
										//	veh_type = HWYV2V_CACC;
										////veh_type = Car_Type;

									}
								}
								else //Main lane
								{ //Generate either ACC CACC Normal Vehicle on the mainlane													
									if (Mainline_Exp_Veh_Type == HWYV2V_veh_Type)
									{
										veh_type = HWYV2V_veh_Type;
									}
									else if (Mainline_Exp_Veh_Type == HWYV2V_CACC)
									{
										veh_type = HWYV2V_CACC;
									}
									else if (Mainline_Exp_Veh_Type == NAGISM_ACC)
									{
										veh_type = NAGISM_ACC;
									}
									else if (Mainline_Exp_Veh_Type == Car_Type)
									{
										veh_type = Car_Type;
									}
									else
									{//distribute the different kind of the vehicles in a different ratio

										int exp_id = ANGConnGetExperimentId();
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

										if (CACC_Lane_Flag == 1)
										{
											//When CACC managed lane strategy is activated, put CACC vehicles in the managed lane first
											//The rest CACC vehicles have priority to enter the left lanes
											if (k == 3)
											{
												if (rand_num < CACC_percent / 0.25)
													veh_type = HWYV2V_CACC;
												else
													veh_type = Car_Type;
											}
											else if (k == 2)
											{
												if (rand_num < (CACC_percent - 0.25) / 0.25)
													veh_type = HWYV2V_CACC;
												else
													veh_type = Car_Type;
											}
											else if (k == 1)
											{
												if (rand_num < (CACC_percent - 0.5) / 0.25)
													veh_type = HWYV2V_CACC;
												else
													veh_type = Car_Type;
											}
											else
											{
												if (rand_num < (CACC_percent - 0.75) / 0.25)
													veh_type = HWYV2V_CACC;
												else
													veh_type = Car_Type;
											}

											//if (CACC_Lane_Number == 1)
											//{
											//	// Only one CACC managed lane
											//	if (k == 3)
											//	{
											//		//CACC managed lane
											//		if (rand_num < CACC_percent / 0.25)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//	else
											//	{
											//		//The rest CACC and non-CACC vehicles are evenly distributed in the rest lanes
											//		if (rand_num < (CACC_percent - 0.25) / 0.75)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//}
											//else if (CACC_Lane_Number == 2)
											//{
											//	// Only one CACC managed lane
											//	if (k == 3)
											//	{
											//		//CACC managed lane
											//		if (rand_num < CACC_percent / 0.25)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//	if (k == 2)
											//	{
											//		//CACC managed lane
											//		if (rand_num < (CACC_percent - 0.25) / 0.25)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//	else
											//	{
											//		//The rest CACC and non-CACC vehicles are evenly distributed in the rest lanes
											//		if (rand_num < (CACC_percent - 0.5) / 0.5)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//}
											//else if (CACC_Lane_Number == 3)
											//{
											//	// Only one CACC managed lane
											//	if (k == 3)
											//	{
											//		//CACC managed lane
											//		if (rand_num < CACC_percent / 0.25)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//	if (k == 2)
											//	{
											//		//CACC managed lane
											//		if (rand_num < (CACC_percent - 0.25) / 0.25)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//	if (k == 1)
											//	{
											//		//CACC managed lane
											//		if (rand_num < (CACC_percent - 0.5) / 0.25)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//	else
											//	{
											//		//The rest CACC and non-CACC vehicles are evenly distributed in the rest lanes
											//		if (rand_num < (CACC_percent - 0.75) / 0.25)
											//			veh_type = HWYV2V_CACC;
											//		else
											//			veh_type = Car_Type;
											//	}
											//}

										}
										else
										{
											// When CACC managed lane strategy is not activated, distribute vehicles evenly in all lanes
											if (rand_num < ACC_percent)
												veh_type = NAGISM_ACC;
											else if (rand_num < ACC_percent +
												CACC_percent)
												veh_type = HWYV2V_CACC;
											else
												veh_type = Car_Type;
										}


										//if (rand_num * 100 < CACC_percent)
										//	veh_type = RAMPV2V_veh_Type;
										//else
										//	veh_type = HWYV2V_CACC;

									}

								}
								int t1 = AKIVehGetVehTypeInternalPosition(veh_type);
								int t2 = orgin_section[origin];
								int aID = AKIPutVehTrafficOD
									(t2, k + 1,
									t1,
									origin, denstine, 0, 0, 0);			//put a car on the origin centroid
								if (aID < 0)
								{//error
									veh_type = 0;
									AKIPrintString("AKIPutVehTrafficOD Return Error!");
									//char temp_char[150];
									//sprintf(temp_char,"orgin_section = %d, veh_type = %d, origin = %d, dest = %d",orgin_section[origin],veh_type,origin,denstine);
									//AKIPrintString(temp_char);
								}
								else
								{/*AKIPrintString("AKIPutVehTrafficOD Success");*/
									num_veh_entering[k] = num_veh_entering[k] + 1;	//number of the vehicles entering highway by lane.	

								}

								//randomly generate the next time and replace
								//with the value in vector
								if (avg_headway_origin.find(origin) != avg_headway_origin.end()  //if this origin is recorded in the database.
									&&
									avg_headway_origin[origin].at(k)>0) //if the number of the seconds that the vehicle is coming from the origin is positive.
								{
									// dmdCoef is used to adjust the input traffic.
									// The input traffic gradually increases from 0 to the user-specified volume in a 30-min period_Hao 01/24/17
									double dmdCoef = 1;
									double headwayCoef = 200;

									int exp_id = ANGConnGetExperimentId();
									const unsigned short *temp_String =
										AKIConvertFromAsciiString("debug_track_veh_id");
									int time_flag = ANGConnGetAttributeValueInt(
										ANGConnGetAttribute(temp_String), exp_id);
									AKIDeleteUNICODEString(temp_String);


									int new_time = (int)(RandomExpHeadway(min_headway_origin[origin].at(k),
										avg_headway_origin[origin].at(k)) / acicle);
									if (new_time <= 0)
									{
										// new_time = (int)(min_headway_origin[origin].at(k) / acicle);
										new_time = 100000;
									}

									if (time_flag == -1)
									{
										if (origin == 3292)
										{
											if (time < 1200)
											{
												new_time = 12000;
											}
										}
										if (time > 3600)
										{
											new_time = 100000;
										}
									}

									//if (origin == 3292)
									//{
									//	if (time <= 1800)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 4200)
									//	{
									//		dmdCoef = 5;
									//	}
									//	else if (time <= 6600)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 9000)
									//	{
									//		dmdCoef = 2.5;
									//	}
									//	else if (time <= 11400)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 13800)
									//	{
									//		dmdCoef = 1.67;
									//	}
									//	else if (time <= 16200)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 18600)
									//	{
									//		dmdCoef = 1.25;
									//	}
									//	else if (time <= 21000)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 23400)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 25800)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else
									//	{
									//		dmdCoef = 50;
									//	}
									//}
									//else
									//{
									//	if (time <= 1800)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 4200)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 5400)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 6600)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 9000)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 10200)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 11400)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 13800)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 15000)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 16200)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 18600)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 19800)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 21000)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 23400)
									//	{
									//		dmdCoef = 1;
									//	}
									//	else if (time <= 24600)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else if (time <= 25800)
									//	{
									//		dmdCoef = 50;
									//	}
									//	else
									//	{
									//		dmdCoef = 50;
									//	}
									//}

									//double expHwy = RandomExpHeadway(dmdCoef * min_headway_origin[origin].at(k),
									//	dmdCoef * avg_headway_origin[origin].at(k));

									time_next[origin][k] = new_time + current_step;
									char temp_char[150];
									sprintf(temp_char, "Num_Lane %d Num_veh = %d time_next = %d, min_headway %f, avg_headway %f", k, num_veh_entering[k], new_time, min_headway_origin[origin].at(k), avg_headway_origin[origin].at(k));
									//AKIPrintString(temp_char);	
								}

								break;
							}
						}
					}
				}
			}
		}
		//}
	}
	return 0;
}

