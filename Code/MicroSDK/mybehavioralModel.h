//-*-Mode: C++;-*-
#ifndef _mybehavioralModel_h_
#define _mybehavioralModel_h_

#include "A2BehavioralModelUtil.h"
#include "A2BehavioralModel.h"
#include "myVehicleDef.h"
#include <stdio.h>
#include <map>

#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <ctime>

// For parallel computing==========================
#include <omp.h>
// For parallel computing==========================

using namespace std::chrono;


class A2BehavioralVehData
{
public:
	A2BehavioralVehData(){};
	~A2BehavioralVehData(){};

	//Jam corresponds to jam gap
	double  minJam;
	double maxJam; 
	double meanJam; 
	double devJam;
	//T corresponds to the slope parameters T_n in Eq. (18)
	double  minT;
	double maxT;
	double meanT; 
	double devT;
	//E corresponds to the target distance E in Eq. (18)
	double  minE;
	double maxE; 
	double meanE;
	double devE;
	double distConflict; // distance to conflict point (e.g. end of ramp) at which driver starts trying to change lanes
	bool Asym;

	double min_reaction_time;
	double max_reaction_time;
	double avg_reaction_time;
	double dev_reaction_time;
	//headway
	double min_headway_time;
	double max_headway_time;
	double avg_headway_time;
	double dev_headway_time;

	double min_time_between_lc_;
	double politeness_;
	double friction;
	double politeness_optional;
};


class A2BEHAVIORALEXPORT mybehavioralModel: public A2BehavioralModel{
public:
	mybehavioralModel ();
	~ mybehavioralModel ();

	/* Methods required by A2BehaviorlModel class+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	myVehicleDef * arrivalNewVehicle(void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh);
	void removedVehicle(void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh );
	bool evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed);
	bool evaluateLaneChanging(A2SimVehicle *vehicle, int threadId);
	bool isVehicleGivingWay(A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield);
	bool avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre);
	double computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle, double VelActual, double VelDeseada, double RestoCiclo);	
	double computeCarFollowingDecelerationComponentSpeed(A2SimVehicle *vehicle, double Shift, A2SimVehicle *vehicleLeader, double ShiftLeader, bool controlDecelMax = false, bool aside = false, int time = 1);
	double computeCarFollowingDecelerationComponentSpeedCore(A2SimVehicle *vehicle, double VelAnterior, A2SimVehicle *vehicleLeader, double VelPreAnterior, double GapAnterior, double DecelEstimadaLeader);
	//double computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, double Xup, double Vup, double Xdw, double Vdw, double Gap, bool ImprudentCase = false, bool VehicleIspVehDw = false);
	double computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, bool ImprudentCase = false, bool VehicleIspVehDw = false, int time = 1);

	int evaluateLaneSelectionDiscretionary(A2SimVehicle *vehicle, bool LeftLanePossible, bool RightLanePossible);
	int evaluateHasTime2CrossYellowState(A2SimVehicle *vehicle, double distance2StopLine);
	/* Methods required by A2BehaviorlModel class+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	
	/* User-specified methods+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
	//virtual void updateVehicle( A2SimVehicle *vehicle );
	void setPara4NewVeh(myVehicleDef* res);
	bool ApplyNGSIMModel(A2SimVehicle *avehicle, double buffer, double aimsun_activate_speed);
	void readVehTypeData( int vehTypeId );
	static double sampleNormalDist(double mean, double dev);
	std::map<int, A2BehavioralVehData> vehTypeData;
	int ReadGapModel(int exp_id);
	int UpdateLatestArrival(int vid, int secid, int lane_id);
	
	void ReadExternalParameters();
	void PrintString(std::string key_value);
	void SetExternalParameters();

	//flow related to setting turnings
	void ReadHOVSetting();
	void SetGlobalSetParameters(myVehicleDef * res);
	bool UseMergingDemandReading();
	bool DisableExternalACCCACC();


	bool hov_include; //if hov lane is included for simulation
	double hov_start_time; // hov start time with respect to the start time of the simulation
	double hov_end_time;  // hov end time with respect to the start time of the simulation
	double hov_percentage;  // HOV percentage used when hov rule is not active or for general purpose lanes

	//external parameter hashmap
	std::map<std::string, double> hashmap;
	double cacc_percent;
	double acc_percent;
	std::string parastr;

	int CarTypeID;
	int HovTypeID;
	int TruckTypeID;
	int CACCTypeID;
	int ACCTypeID;
	/* User-specified methods+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */


	// For preventing vehicles from blocking the source link
	std::map<int, bool> source_links_release_switch;

	void recordEnergyModeSwitch(myVehicleDef* veh);

};


#endif
