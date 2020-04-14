//DO NOT MODIFY THIS FILE
#ifndef _AAPI_h_
#define _AAPI_h_

#ifdef _WIN32
	#define DLLE __declspec(dllexport)
#else
	#define DLLE 
#endif

#include <map>
#include <chrono>
#include <random>

#define MAX(a,b)    (((a)>(b)) ? (a) : (b))
#define MIN(a,b)    (((a)<(b)) ? (a) : (b))

// For energy consumption
// For energy consumption matrices
#define Time_Resolution 180 // sceonds
#define Space_Resolution 200 // meters
#define Column_Numbers 16

// For speed advisory
//#define Free_Flow_Speed 31.2 // m/s
//#define Min_Advisory_Speed 5 // m/s
#define Threshold_Speed 20 // m/s, for identifying the congested sections
#define Speed_Increase_Coefficient 1.2 // The advisory speed of the congested region is increased by 20%
#define Speed_Increase_Step 3 // m/s, for gradually increasing the advisory speed
#define Advisory_Section_Number 25 // Number of sections that have the same advisory speed
#define Time_Resolution_Speed_Advisory 30 // sceonds
#define Space_Resolution_Speed_Advisory 100 // meters
#define Speed_Gain 60
#define CV_TYPE 62
#define CAV_TYPE 346 //26543 


extern "C" {
	DLLE int AAPILoad();
	DLLE int AAPIInit();
	DLLE int AAPIManage(double time, double timeSta, double timTrans, double acicle);
	DLLE int AAPIPostManage(double time, double timeSta, double timTrans, double acicle);
	DLLE int AAPIFinish();
	DLLE int AAPIUnLoad();

	DLLE int AAPIEnterVehicle(int idveh, int idsection);
	DLLE int AAPIExitVehicle(int idveh, int idsection);
	DLLE int AAPIEnterPedestrian(int idPedestrian, int originCentroid);
	DLLE int AAPIExitPedestrian(int idPedestrian, int destinationCentroid);
	DLLE int AAPIEnterVehicleSection(int idveh, int idsection, double atime);
	DLLE int AAPIExitVehicleSection(int idveh, int idsection, double time);
	
	DLLE int AAPIPreRouteChoiceCalculation(double time, double timeSta);

	// For energy consumption
	// For energy consumption matrices
	std::map<int, int> section_rows;
	void outputEnergy();
	std::vector<std::vector<double>> fuelMatrix;
	int num_rows_in_one_step;
	struct fuelVMT {
		double fuel;
		double fuel_VT;
		double VMT;
		double VHT;
	};
	fuelVMT computeEnergy(double v, double a, bool truck_flag);
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

	// For speed advisory

	// This is the link IDs for the network with CRM====================================
	//int freewayLinks[39] = {
	//	26565, 17570, 17556, 26646, 2383, 2384, 2385,
	//	26649, 2386, 2387, 17234, 26652, 17240, 17248, 17266,
	//	26655, 17276, 17298, 17338, 26658, 17374, 17382, 17398,
	//	17508, 26661, 16687, 16695, 16711, 16733, 26664, 16743,
	//	16783, 16789, 16573, 16823, 16847, 26744, 16873, 16883
	//}; 
	//std::vector<int> bottleneckLinks({ 26652, 26661 });
	// South to north==========================================================

	// This is the link IDs for the network with CRM, reduced network===============================
	//int freewayLinks[17] = {
	//	26565, 17570, 17556, 26646, 2383, 2384, 2385,
	//	26649, 2386, 2387, 17234, 26652, 17240, 17248, 17266,
	//	26655, 17276 
	//}; 
	//std::vector<int> bottleneckLinks({ 26652});
	//================================================================================
	
	// South to north============================================================
	// This is the link IDs for the network without CRM
	//int freewayLinks[37] = { 11960, 13600, 11974, 11986, 11998, 13597, 
	//	12010, 12022, 12030, 13594, 12042, 12050, 12062, 
	//	12072, 12274, 12082, 12096, 13588, 12108, 12116, 
	//	12130, 12142, 13585, 12150, 12162, 12170, 12178, 
	//	13582, 12190, 12200, 12214, 12226, 12234, 12248, 
	//	13579, 12256, 12266
	//};
	//std::vector<int> bottleneckLinks({ 13594, 12072, 13588, 13585, 13582, 13579 });

	// This is the link IDs for the network without CRM, reduced network
	int freewayLinks[12] = { 11960, 13600, 11974, 11986, 11998, 13597,
		12010, 12022, 12030, 13594, 12042, 12050, 
	};
	std::vector<int> bottleneckLinks({ 13594 });
	//==================================================================================

	//==================================================================================
	// Empirical distribution for VSA compliance
	// Number represents (Desired speed - VSA) in m/s
	// Smallest value represents CDF = 0; largest value represents CDF = 1; each value maps to a CDF increment of 0.01
	double VSA_Compliance_Low[] = { -44.70, -3.62, -2.19, -1.70, -1.25, -1.03, -0.80, -0.58, -0.45, -0.27, -0.18, 0.00, 0.09,
		0.22, 0.27, 0.40, 0.49, 0.54, 0.67, 0.76, 0.80, 0.89, 0.98, 1.07, 1.12, 1.21, 1.30, 1.39, 1.48, 1.56,
		1.65, 1.70, 1.79, 1.88, 1.97, 2.06, 2.19, 2.28, 2.37, 2.46, 2.59, 2.68, 2.82, 2.95, 3.08, 3.22, 3.40,
		3.58, 3.80, 4.07, 4.34, 4.69, 5.14, 5.54, 6.04, 6.75, 7.42, 7.91, 8.36, 8.81, 9.25, 9.66, 9.97, 10.19,
		10.51, 10.77, 11.00, 11.27, 11.49, 11.71, 11.89, 12.03, 12.25, 12.47, 12.65, 12.83, 13.05, 13.23,
		13.50, 13.72, 13.95, 14.17, 14.39, 14.71, 14.98, 15.29, 15.51, 15.78, 16.05, 16.36, 16.59, 16.90,
		17.17, 17.48, 17.75, 18.15, 18.55, 18.91, 19.40, 20.03, 22.44 };
	double VSA_Compliance_High[] = { -44.70, -4.02, -2.95, -2.46, -2.15, -1.92, -1.70, -1.52, -1.39, -1.25, -1.16, -1.07, -0.98,
		-0.85, -0.76, -0.72, -0.63, -0.54, -0.45, -0.40, -0.31, -0.27, -0.18, -0.13, -0.09, -0.04,
		0.04, 0.09, 0.13, 0.18, 0.22, 0.27, 0.36, 0.40, 0.45, 0.49, 0.54, 0.58, 0.63, 0.67, 0.72, 0.76,
		0.85, 0.89, 0.94, 0.98, 1.03, 1.07, 1.12, 1.16, 1.21, 1.25, 1.30, 1.34, 1.39, 1.43, 1.48, 1.52,
		1.61, 1.65, 1.70, 1.74, 1.79, 1.88, 1.92, 1.97, 2.01, 2.10, 2.15, 2.24, 2.28, 2.37, 2.41, 2.50,
		2.59, 2.68, 2.77, 2.86, 2.95, 3.04, 3.17, 3.26, 3.40, 3.53, 3.71, 3.84, 4.07, 4.29, 4.56, 4.83,
		5.14, 5.50, 5.95, 6.39, 6.88, 7.38, 7.91, 8.54, 9.30, 10.19, 11.62 };
	//==================================================================================

	std::vector<int> bottleneckRecordID;
	int linkCount = 12; // 37; //39 for the network with CRM 26
	double updateTime = Time_Resolution_Speed_Advisory;
	std::map<int, int> linkRows;
	std::vector<std::vector<double>> speedAdvData; // secID, subsection length, VMT, VHT, occupancy
	std::vector<double> advisorySpeed; // m/s, for each subsection
	std::vector<double> advisorySpeedPrevious; // m/s, for each subsection
	void CollectSpeedAdvisoryData();
	void DetermineAdvisorySpeed();
	void SetAdvisorySpeed();
	int CRM_On = 0;
	int SPD_ADV_On = 0;
	int Section_ID_Previous;
	double v_max = 32;
	double v_min = 5;
	double random_compliance = 0;
	int num_secs = 25;

	// For signal control algorithm
	void ReadExternalParameters();
	bool ConfigureSignalHeads(int node_id, double time, bool skip_last);
	bool ConfigureSignalHeadsExternalController(int node_id, double current_time, double cycle_start_time, int optimal_signal_times[], 
		double time_sta, double sim_step);

	int yellow_time; // seconds
	int all_red; // seconds
	int signal_optimization_approach;
	int ring1_last_phase;
	int ring2_last_phase;
	int optimal_signal_times[8];
	bool read_new_SPaT = true;

}

#endif
