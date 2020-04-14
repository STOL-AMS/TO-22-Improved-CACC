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

#include "myVehicleDef.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include "mybehavioralModel.h"
#include "ANGConProxie.h"
#include "CIProxie.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <queue>
#include <hash_map>
#include <algorithm>  
#include <Windows.h>
#include <sstream>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <crtdbg.h>
#include <random>

#define MAX(a,b)    (((a)>(b)) ? (a) : (b))
#define MIN(a,b)    (((a)<(b)) ? (a) : (b))

#define CF	0
#define MCF	1
#define BCF	2
#define ECF	3
#define DCF	4
#define ACF	5
#define CCF	6
#define RCF	7
#define LC	8
#define ACC_ON 9
#define CACC_ON 10
#define CC_ON 11
#define V_CACC_ON 12
#define ACC_ON_CC 91
#define CACC_ON_Follower_Speed_Regulation 101
#define CACC_ON_ACC 102
#define CACC_ON_Leader 103
#define CACC_ON_Leader_Speed_Regulation 104
#define CACC_ON_Leader_Gap_Regulation 105

#define CACC_Recover_Time 10
#define ACC_Recover_Time 20
#define CACC_Relaxation_Time 300

#define ACC_Detection_Range_Upper 65
#define ACC_Detection_Range_Lower 50

#define EXIT 1
#define ELC 2
#define DLC 3
#define RAMP 4
#define TURNING 5

#define MANDATORY 4
#define OPTIONAL_LC 3

#define RAMP_DECISION_SLOW_DOWN -1
#define RAMP_DECISION_FOLLOW 1
#define RAMP_LANE_CHANGE_FEASIBLE 0
#define RAMP_DECISION_NORMAL_FOLLOW 2

#define DLC_DECISION_SLOW_DOWN -1
#define DLC_DECISION_FOLLOW 1
#define DLC_LANE_CHANGE_FEASIBLE 0
#define DLC_DECISION_NORMAL_FOLLOW 2

#define EXIT_DECISION_SLOW_DOWN -1
#define EXIT_DECISION_FOLLOW 1
#define EXIT_CHANGE_FEASIBLE 0
#define EXIT_DECISION_NORMAL_FOLLOW 2

#define FeederOnRamp 3280
#define END_SECTION 23551
#define NO_CHANGE_ZONE_LENGTH 1357
#define RAMP_LENGTH 1666.3
#define END_OF_RAMP 1657 //ASSUME THE EFFECTIVE RAMP LENGTH = 300 meters
#define RAMP_SPEED 25
#define CREEP_SPEED 20
#define SYNC_CREEP_SPEED 2.23 // = 5 mph
#define NEC_SPEED_DIFF_OFF_RAMP_SYNC 2.23 //5MPH

#define COMF_LEVEL_DLC 0.8 //the comfortable level of deceleration for DLC
#define COMF_LEVEL 1.0 //the comfortable level of deceleration for on-ramp slow down decision
//#define ACCEPT_LEVEL 0.9 //the accepted level of deceleration for on-ramp sync
#define DANGER_GAP 0.8
#define MIN_REGAIN_AUTO_SPEED 2.3
#define MIN_COOP_SPEED 2.3
#define MIN_DESIRE_COOP 0.8
#define MIN_TIME_OVERPASS 7
#define MAX_WAIT_TIME 10

#define ACC_BASED_MODEL 1
#define GAP_BASED_MODEL 0

#define TRUE_ON_RAMP 3 //on ramp
#define TRUE_OFF_RAMP 4 //off ramp
#define ON_RAMP 1 //mainline with an acceleration lane
#define OFF_RAMP 2 //mainline with an offramp auxiliary lane
#define NO_RAMP 0 //mainline without any aside lane

#define MIN_DLC_SPEED 2.3

#define ACC_LANE_LENGTH 250 //Length of on-ramp acceleration lane[m]
#define FORBID_RAMPCHANGE_ZONE 0 //Length of on-ramp acceleration lane[m]

#define MIN_DLC_SPEED_DIFF 2.34 //5 [mph]

#define START_POSITION 100 //start position is the distance to the end of the starting segment

//#define _CRTDBG_MAP_ALLOC

#define MAX_COOP_TIME 1 //[s]

#define MIN_SPEED_OFF_RAMP_SLOWDOWN 22.0 // = 50[mph]

#define COLLI_IMPOS 0
#define COLLI_SCE2 1
#define COLLI_SCE3 2

#define ACCMODE 1 //CACC or ACC vehicle running in ACC mode
#define CACCMODE 2 //CACC running in CACC mode
#define CACCINMAN 4 //CACC running in manual mode
#define ACCINMAN 3 //ACC running in manual mode

#define LC_Threshold_Transition_Section 3279 //ACC running in manual mode



// This function returns the input value itself if it is within a lower and upper bound.
// Otherwise, the function returns the lower bound/upper bound if the input is lower/higher
// than the lower bound/upper bound.
double bound(double x, double x_high, double x_low)
{
	double b;
	b = MIN(x, x_high);
	b = MAX(b, x_low);
	return b;
}


myVehicleDef::myVehicleDef(void *handlerVehicle, unsigned short idhandler, bool isFictitiousVeh) : A2SimVehicle(handlerVehicle, idhandler, isFictitiousVeh)
{

}

myVehicleDef::~myVehicleDef()
{

}

bool myVehicleDef::getapplyACC()
{
	return applyACC_CACC;
}

// update the vehicle in regular car following mode
myVehicleDef::PositionSpeed myVehicleDef::updateRegularCf()
{
	double x, x_CF;
	PositionSpeed pos_speed;
	x = getPosition(0);
	//if (this->isCurrentLaneInNode())
	//{
	//	// CF and LC algorithms do not work properly when the vehicle is in a node_Hao 11/23/16
	//	x_CF = x + MAX(this->getSpeed(), 3) * this->delta_t;
	//}
	//else
	//{
	//	x_CF = PosCf(leader);
	//}
	x_CF = PosCf(leader);
	pos_speed.position = x_CF;
	pos_speed.speed = 2 * (x_CF - x) / delta_t - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(x_CF, 2*(x_CF - x) / delta_t-this->getSpeed());  // newSpeed*2-oldSpeed?
}

//determine the current mode based on the mode from the last step
//this is used for ACC or CACC vehicles
int myVehicleDef::determineDrivingModeACC()
{
	int currentMode = getMode();

	//do not do any lane changes on node
	if (this->isCurrentLaneInNode() == true)
	{
		switch (currentMode)
		{
		case BCF:
			return setMode(CF);
		case LC:
			return setMode(CF);
		default:
			return getMode();
		}
	}
	switch (currentMode)
	{
	case CF:
		return DetermineLcOrMergeOrCoop();  //decide if lane change /merge /cooperation is needed
	case  ACF:
		return Determine2ndLcAfterLc();  //decide if another lane change follows this lane change
	case CCF:
		return determineCoopOrLc();  //see if it is willing to yield or itself needs a change
	case RCF:
		return DetermineReceiveOrLcOrCoop();  //see if it is needs a change or Coop or doing RCF
	case BCF:
		return determineGapOrGiveup();
	default:
		return currentMode;
	}
}

//determine the current mode based on the mode from the last step
//This function reorganized Hwasoo's code as the basic car following function 
//In this function, lane changing conditions are tested, and the corresponding modes are set.
int myVehicleDef::determineDrivingMode()
{

	int currentMode = getMode();

	//do not do any lane changes on node
	if (this->isCurrentLaneInNode() == true)
	{
		switch (currentMode)
		{
		case CF:
			return getMode();  //decide if lane change /merge /cooperation is needed
		case  ACF:
			return getMode();  //decide if another lane change followers this lane change
		case CCF:
			return getMode();  //see if it is willing to yield or itself needs a change
		case BCF:
			return setMode(CF); // No LC motivation on nodes
		case LC:
			return getMode();
		default:
			return currentMode;
		}
	}

	switch (currentMode)
	{
	case CF:
		return DetermineLcOrMergeOrCoop();  //decide if lane change /merge /cooperation is needed
	case ACF:
		return Determine2ndLcAfterLc();  //decide if another lane change followers this lane change
	case CCF:
		return determineCoopOrLc();  //see if it is willing to yield or itself needs a change
	case RCF:
		return DetermineReceiveOrLcOrCoop();  //see if it is needs a change or Coop or doing RCF
	case BCF:
		return determineGapOrGiveup();  // see if it needs a Coop
	default:
		return currentMode;
	}
}

//run Yeo Model
// each update function must do three things:
// calculate the next position, call set new position, and set the mode for the next step
myVehicleDef::PositionSpeed myVehicleDef::RunNGSIM(bool mode_predetermined)
{
	PositionSpeed pos_speed;

	//Driving mode is updated every 0.2 second
	int mode = 0;
	if (mode_predetermined == true || (int)(current_time * 10) % 2 != 1)
		mode = getMode();
	else
		mode = determineDrivingMode();

	switch (mode)
	{
		//MCF mode was disabled. Try to avoid this type of behavior. And we try to make the connection one to one
		//using side lane to model merge type of behavior
	case CF:
		pos_speed = updateRegularCf();
		return pos_speed;
	case BCF:
		// LC maneuver is also updated here
		pos_speed = UpdateBeforeLaneChangeCf();
		return pos_speed;
	case ACF:
		pos_speed = UpdateAfterLaneChangeCf();
		return pos_speed;
	case CCF:
		pos_speed = updateCoopCf();
		return pos_speed;
	case RCF:
		pos_speed = UpdateReceiveCf();
		return pos_speed;
	case LC:
		pos_speed = UpdateLc();
		return pos_speed;
	default:
		pos_speed.position = -1;
		pos_speed.speed = -1;
		return pos_speed;
	}
}


//#pragma optimize("",off) // It is problematic!
bool myVehicleDef::ApplyNGSIMModel()
{
	//int sectionid = getIdCurrentSection(); 		
	A2KSectionInf sectioninfo = this->sec_inf;
	if (getapplyACC() == false //this is not a cacc or acc vehicle
		|| IsLaneOnRamp(0) //this vehicle is on ramp
		|| this->GetRampType(sec_inf.id) == TRUE_ON_RAMP //this vehicle is on ramp
		)
	{
		return true;
	}
	return false;
}
//#pragma optimize("",on)





double myVehicleDef::getMAXacc()
{
	/*double MaxDec = 0.0;
	StaticInfVeh vInfo;

	vInfo = AKIVehGetStaticInf (this->getId());
	MaxDec = vInfo.maxAcceleration;
	return MaxDec; */
	double acc = this->getAcceleration();

	int mode = this->getMode();
	if ((mode == BCF && this->LCType != DLC) ||
		(mode == ACF && this->last_lc_type != DLC) ||
		(mode == RCF && this->leader && ((myVehicleDef*) this->leader)->last_lc_type != DLC) ||
		mode == CCF)
	{
		// Acceleration increased when performing the mandatory lane changes
		acc = acc * 2;
	}

	return acc;
	//return MAX(1.5, this->staticinfo.maxAcceleration); // Sometimes Aimsum gives very small and unrealistic max acc_Hao 03/18/17

}

double myVehicleDef::getReactionTime()
{
	double rec_t = this->reaction_time_;
	int mode = this->getMode();
	if ((mode == BCF && this->LCType != DLC) ||
		(mode == ACF && this->last_lc_type != DLC) ||
		(mode == RCF && this->leader && ((myVehicleDef*) this->leader)->last_lc_type != DLC) ||
		mode == CCF)
	{
		// Reaction time reduced when performing the mandatory lane changes
		rec_t = rec_t * 0.2;
	}
	return rec_t;
};

double myVehicleDef::getMAXdec()
{
	//double MaxDec = 0.0;
	//StaticInfVeh vInfo; 

	//vInfo = AKIVehGetStaticInf (this->getId()); 
	//// MaxDec = vInfo.maxDeceleration;     // default sign
	//MaxDec = (vInfo.maxDeceleration);     // sign changed by XYLu 11_07_14
	double maxDec = this->getDecelerationMax();

	int mode = this->getMode();
	if ((mode == BCF && this->LCType != DLC) ||
		(mode == ACF && this->last_lc_type != DLC) ||
		(mode == RCF && this->leader && ((myVehicleDef*) this->leader)->last_lc_type != DLC) ||
		mode == CCF)
	{
		// Acceleration increased when performing the mandatory lane changes
		maxDec = maxDec * 2;
	}

	//MaxDec = MIN(-1, staticinfo.maxDeceleration); // Sometimes Aimsum gives very small and unrealistic max dec_Hao 03/18/17
	return maxDec;
}

// This gap acceptance method judges the front gap first. If the gap is unacceptable, the driver
// will try to sync its speed and spacing regarding the front vehicle on the target lane by decelerating. 
// After the front gap is satisfied, the driver will judge the lag gap by estimating the possible deceleration 
// the new follower would apply. If the maximum intensity leads to collision, the driver gives up this lane changing
// and slow down to pass this gap.
int myVehicleDef::GapAcceptDecision_Sync_First()
{
	if (VehID == this->debug_track_id)
		VehID = debug_track_id;
	//the intensity of the 
	//desire determines some parameters 
	//in the gap acceptance process
	double desire = this->getLaneChangeDesire();
	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	//no up and down vehicles, gap accepted
	if ((vehDown == NULL || vehDown == this) &&
		(vehUp == this || vehUp == NULL))
	{
		return RAMP_LANE_CHANGE_FEASIBLE;
	}

	//this gap ac threshold defines when the driver feels the gap is acceptable
	//this should be the comfortable deceleration
	double Gap_AC_Thrd = 0;
	//acceleration for the follower on the target lane after inserted
	//the vehicle assumes that the follower is a manual driving car
	double tau = this->getReactionTime();
	double headway = 0;
	double d_leader = 0;
	//relaxation defines how much deceleration would be applied to calculate the safe distance in NGSIM model
	//1 is the neutral one meaning no affect.
	double min_headway = this->getDesireHeadway()*Relaxation;

	//headway w.r.s to the upstream vehicle
	double up_headway = 0;

	double acc_self = 0;
	double acc_up = 0;

	Gap_AC_Thrd = this->getMAXdec()*0.9; // Why?

	// boolean indicators to see if upstream and downstream gaps are satisfied
	bool Ok_upstream_gap = true;
	bool Ok_downstream_gap = true;
	if (vehDown != NULL && this != vehDown)
	{
		// use the subject vehicle's max dec
		d_leader = -
			(((myVehicleDef *)vehDown)->getSpeed()
			*((myVehicleDef *)vehDown)->getSpeed()) /
			(2 * getMAXdec())
			*this->Relaxation;
		//get the headway using the current vehicle as the reference car
		headway = ((myVehicleDef *)vehDown)
			->getPositionReferenceVeh(this);
		// to ensure consistence for fictitious vehicle
		// the position of the leader is replaced with current my pos
		// + current headway
		double current_pos_down =
			GetPositionRelativeFake(((myVehicleDef *)vehDown), 0, true);
		double past_pos_down = 0; // GetPositionRelativeFake(((myVehicleDef *)vehDown), tau*beta, true);

		if (headway - ((myVehicleDef *)vehDown)->getLength() <= 0)
		{
			// Spacing is less than leader's length
			Ok_downstream_gap = false;
		}
		else
		{
			if (this->getGapAcceptanceModel() == ACC_BASED_MODEL)
			{
				if (this->AccGapAccepted(
					getMAXdec(),
					getMAXacc(),
					tau*beta,
					headway,
					this->alpha*getJamGap(),
					d_leader,
					((myVehicleDef *)vehDown)->getLength(),
					freeflowspeed,
					getSpeed(),
					getPosition(),
					current_pos_down,
					past_pos_down,
					((myVehicleDef *)vehDown)->getSpeed(),
					min_headway,
					Gap_AC_Thrd,
					desire)
					== false)
				{
					Ok_downstream_gap = false;
				}
			}
			else
			{
				if (this->DisGapAccepted(getMAXdec(),
					getMAXacc(),
					tau*beta,
					headway,
					this->alpha*getJamGap(),
					d_leader,
					((myVehicleDef *)vehDown)->getLength(),
					freeflowspeed,
					getSpeed(),
					getPosition(),
					current_pos_down,
					past_pos_down,
					((myVehicleDef *)vehDown)->getSpeed(),
					min_headway,
					Gap_AC_Thrd,
					desire,
					IsLaneOnRamp(0),
					true, acc_self) == false)
				{
					Ok_downstream_gap = false;
				}
				else
				{
					acc_self = AnticipatedAcc(
						getMAXdec(),
						getMAXacc(),
						tau*beta,
						headway,
						this->alpha*getJamGap(),
						d_leader,
						((myVehicleDef *)vehDown)->getLength(),
						freeflowspeed,
						getSpeed(),
						getPosition(),
						current_pos_down,
						past_pos_down,
						((myVehicleDef *)vehDown)->getSpeed(),
						min_headway,
						Gap_AC_Thrd,
						desire
						);  // get the acceleration of the current vehicle

					if (getLCType() == OPTIONAL_LC)
					{
						if (acc_self < this->getComfDecDLC())
						{
							Ok_downstream_gap = false;
						}
					}



				}
			}
		}
	}

	if (vehUp != NULL && this != vehUp)
	{
		d_leader = -
			(this->getSpeed() *this->getSpeed())
			/ (2 * this->getMAXdec())*this->Relaxation;


		up_headway = this->getPositionReferenceVeh(((myVehicleDef *)vehUp));


		//the lane changer has no information about the min-headway of 
		//the follower so assume it is the same
		min_headway = this->getDesireHeadway()*Relaxation;

		//means the car is traveling in the opposite direction)_Incorrect statement_Hao 12/4/16
		//if (up_headway < 0) 
		//{
		//	Ok_upstream_gap = true;
		//}
		//else 
		if (up_headway - getLength() <= 0)
		{
			Ok_upstream_gap = false;
		}
		else if (((myVehicleDef *)vehUp)->isFictitious() == false
			&& ((myVehicleDef *)vehUp)->getSpeed() <= 0
			)
		{
			Ok_upstream_gap = true;
		}
		else
		{
			//use shorter gap mode
			//note: use the follower's free flow speed on the target lane
			double early_pos = 0; // this->GetPastPos(tau*beta);
			//the subject car is the leader
			//the other car is the follower
			double current_pos_up =
				GetPositionRelativeFake(((myVehicleDef *)vehUp), 0, false);

			if (this->getGapAcceptanceModel() == ACC_BASED_MODEL)
			{
				if (this->AccGapAccepted(
					getMAXdec(),
					getMAXacc(),
					getReactionTime()*beta,
					up_headway,
					getJamGap()*alpha,
					d_leader,
					this->getLength(),
					((myVehicleDef *)vehUp)->getFreeFlowSpeed(),
					((myVehicleDef *)vehUp)->getSpeed(),
					current_pos_up,
					this->getPosition(),
					early_pos,
					this->getSpeed(),
					min_headway,
					Gap_AC_Thrd,
					desire) == false)
				{
					Ok_upstream_gap = false;
				}
				else
				{
					Ok_upstream_gap = true;
				}
			}
			else
			{
				bool gapAccepted = this->DisGapAccepted(
					getMAXdec(),
					getMAXacc(),
					getReactionTime()*beta,
					up_headway,
					getJamGap()*alpha,
					d_leader,
					this->getLength(),
					((myVehicleDef *)vehUp)->getFreeFlowSpeed(),
					((myVehicleDef *)vehUp)->getSpeed(),
					current_pos_up,
					this->getPosition(),
					early_pos,
					this->getSpeed(),
					min_headway,
					Gap_AC_Thrd,
					desire,
					IsLaneOnRamp(0),
					false,
					acc_self);
				double follower_dec = 0;
				if (gapAccepted == false)
				{
					Ok_upstream_gap = false;
				}
				else
				{
					follower_dec = this->AnticipatedAcc(
						getMAXdec(),
						getMAXacc(),
						getReactionTime()*beta,
						up_headway,
						getJamGap()*alpha,
						d_leader,
						this->getLength(),
						((myVehicleDef *)vehUp)->getFreeFlowSpeed(),
						((myVehicleDef *)vehUp)->getSpeed(),
						current_pos_up,
						this->getPosition(),
						early_pos,
						this->getSpeed(),
						min_headway,
						Gap_AC_Thrd,
						desire
						);

					if (getLCType() == OPTIONAL_LC)
					{
						if (follower_dec < this->getComfDecDLC()*0.5)
						{
							Ok_upstream_gap = false;
						}
					}


				}
			}
		}
	}



	if (Ok_upstream_gap == true)
	{
		if (Ok_downstream_gap == true)
		{
			return RAMP_LANE_CHANGE_FEASIBLE;
		}
		else
		{

			return RAMP_DECISION_FOLLOW;
		}
	}
	else
	{
		return RAMP_DECISION_SLOW_DOWN;
	}
}

//////////////////////////////////////////////////////////////////////////
//skip gap behavior if the forward gap is accepted
//this differentiate for on-ramp, off-ramp and DLC
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::PosCfSkipGap(const A2SimVehicle* potential_leader,
	bool apply_creep_speed)
{
	if (potential_leader != NULL)
	{
		double a_L = getComfDecRampLC(); 		//comfortable dec
		if (this->getLCType() == OPTIONAL_LC)       //optional dec
			a_L = getComfDecDLC();
		double speed = MAX(0, this->getSpeed() + a_L*delta_t);
		if (apply_creep_speed
			&&
			this->getLCType() == RAMP) //this is for on-ramp LC
		{
			speed = MAX(speed, MIN(this->getSpeed(), CREEP_SPEED));
		}
		// regarding turning. if the drive intends to slow down to skip the current gap. 
		//the minimum speed the drive could reduce is a certain amount below the potential leader
		else if (this->getLCType() == TURNING)
		{


			// If the lag vehicle in the target lane is yielding, the subject vehicle will try to synchronize the speed with the lag vehicle_Hao 03/15/17
			if (this->getTargetLane() == RIGHT
				&& this->right_follower != NULL
				&& ((myVehicleDef*)this->right_follower)->CoopRequester == this
				&&
				(this->getPositionReferenceVeh((myVehicleDef*)this->right_follower) >= getJamGap() + this->getLength()
				|| this->right_follower->getSpeed(0) <= 10
				)
				)
			{
				// Calculate the acceleration needed to merge in front of the right follower
				double desired_acc = 0;
				if (this->getPositionReferenceVeh((myVehicleDef*)this->right_follower) <= getJamGap() + this->getLength())
				{
					desired_acc = (getJamGap() + this->getLength() - this->getPositionReferenceVeh((myVehicleDef*)this->right_follower)
						+ this->right_follower->getSpeed(0)*delta_t - this->getSpeed(0)*delta_t) * 2 / (delta_t * delta_t);
					if (desired_acc > this->getMAXacc())
					{
						desired_acc = this->getMAXacc();
					}
					else if (desired_acc < this->getMAXdec())
					{
						desired_acc = this->getMAXdec();
					}
				}
				double desired_speed = this->getSpeed() + desired_acc * delta_t;
				double pos_to_exit = BaseCfModel(this->getMAXdec(),
					this->getMAXacc(),
					this->getReactionTime()*beta,
					this->getDis2NextTurning(), //the distance to the end of the ramp
					this->getJamGap()*alpha,
					0, 4,
					freeflowspeed, this->getSpeed(), this->getPosition(),
					this->getPosition(0) + this->getDis2NextTurning(),//assume there is a car at the end of the ramp with speed of zero
					this->getPosition(0) + this->getDis2NextTurning(), //the position of the car steps earlier
					0,
					this->getDesireHeadway());
				double exit_speed = (pos_to_exit - this->getPosition(0)) / delta_t * 2 - this->getSpeed();
				desired_speed = MIN(desired_speed, exit_speed);
				//if (lag_veh_speed <= 5)
				//{
				//	// To avoid completely stop of the subject vehicle
				//	lag_veh_speed = lag_veh_speed + 3;
				//}
				speed = MAX(speed, desired_speed);
			}
			//else 
			//{
			//	speed = MAX(speed, 3);
			//}
		}

		return this->getPosition() +
			(this->getSpeed() + speed)*delta_t / 2;
	}
	return this->getPosition() + MAX(3, -1 * delta_t + this->getSpeed())*delta_t;
}

//this function returns the position of the vehicle if it slow down with a comfortable deceleration
//double myVehicleDef::PosCfSkipGap(const A2SimVehicle* potential_leader)
//{
//	//apply a comfortable deceleration to a speed that is slightly lower than the the potential leader
//	if(potential_leader!=NULL)
//	{
//		double a_L = this->getMAXdec()*COMF_LEVEL; 		//comfortable dec
//		double speed = this->getSpeed()+a_L*delta_t; 
//
//		//double target_speed = MAX(0, ((myVehicleDef*)potential_leader)->getSpeed()-5); 
//		//speed = MAX(target_speed, speed); 
//		
//		speed = MAX(CREEP_SPEED, speed); 
//
//		return this->getPosition() + (this->getSpeed()+speed)*delta_t/2; 
//	}
//	return this->getPosition(); 
//}

/***************************************************************************************/
//	cntract() Used in PosCf
/***************************************************************************************/
// Nobody uses it!
void cntract(double sample_t, double mag_rate, double e_rate, double a_val, double b_val,
	double *int_val, double *int_val_d)
{
	double temp = 0.0, temp1 = 0.0, rate = 0.0;

	double SIGN1(double);

	temp = a_val - b_val;
	if (mag_rate <= fabs(temp))
		rate = mag_rate;
	else
		rate = (double)fabs(temp);
	temp1 = SIGN1(temp);
	*int_val = a_val - temp1*rate*sample_t*
		(exp(e_rate*temp) - exp(-e_rate*temp)) / (exp(e_rate*temp) + exp(-e_rate*temp));
	*int_val_d = -temp1*rate*sample_t*4.0*
		e_rate / ((exp(e_rate*temp) + exp(-e_rate*temp))*(exp(e_rate*temp) + exp(-e_rate*temp)));
}


double SIGN1(double x)
{
	static double tmp = 0.0000001;
	if (x >= tmp)
		return 1.0;
	else if (x<tmp && x>-tmp)
		return 0.0;
	else
		return -1.0;
}

// base car following model
double myVehicleDef::BaseCfModel
(double a_L, double a_U, double tau,
double headway, double jamGap, double d_leader,
double l_leader, double vf, double v, double x,
double x_leader, double x_leader_steps_early,
double lead_v, double min_headway
)
{
	double target_pos = 0;
	return this->BaseCfModel
		(a_L, a_U, tau,
		headway, jamGap, d_leader,
		l_leader, vf, v, x,
		x_leader, x_leader_steps_early,
		lead_v, min_headway, target_pos
		);
}

// base car following model
double myVehicleDef::BaseCfModel
(double a_L, double a_U, double tau,
double headway, double jamGap, double d_leader,
double l_leader, double vf, double v, double x,
double x_leader, double x_leader_steps_early,
double lead_v, double min_headway, double &target_pos
)
{

	double x_CF = 0;
	double ths = 5;
	target_pos = 0;

	double maxDec = a_L;
	double maxAcc = a_U;
	double reaction_time = tau;
	//this coefficient is suggested to be larger than 
	//the maximum dec of the current vehicle for stability purpose
	double b_estimate = getEstimateLeaderDecCoeff()
		*maxDec;
	if (this->leader != NULL && leader->isFictitious() == false)
	{
		// Assume the subject driver knows the max deceleration of the leader
		b_estimate = MIN(((myVehicleDef*)leader)->getMAXdec()*getEstimateLeaderDecCoeff(),
			b_estimate);
	}
	double desired_headway = min_headway;
	double theta = this->getGippsTheta()*reaction_time; // Extra reaction time?

	// The maximum speed based on Gipps safety criterion
	double v_after_tau = GippsDecelerationTerm(
		maxDec, reaction_time, theta, x_leader, x, jamGap,
		l_leader, v, lead_v, b_estimate);


	double max_a = MIN(maxAcc,
		(v_after_tau - v) / reaction_time);

	if (this->getVehType() == Truck_Type)
		max_a = MAX(max_a, maxDec); // There might be a crash for the truck.

	// The acceleration based on Newell rule, bound by maxAcc and MaxDec
	double newell_a = MIN(maxAcc,
		MAX(maxDec,
		((headway - l_leader - jamGap) / desired_headway - v)
		/ (desired_headway / 2)));    //Newell original paper Eq. (9)

	// free acc
	//double min_a = maxAcc*(1-v/vf); 
	double min_a = maxAcc;
	if (v > 0)
		min_a = maxAcc*(1 - pow((v / vf), this->getAccExp()));
	//when one reduces from a speed that exceeds free flow speed the deceleration should not be too much
	min_a = MAX(min_a, this->getComfDecDLC());

	double current_acc = (getSpeed(0) - getSpeed(1)) / delta_t;
	double acc_target = MIN(MIN(min_a, newell_a), max_a);
	double acc = acc_target;
	if (this->getFirstCycleAfterAdjust() == true)
	{
		// For new arrival vehicles
		acc = 0;
		this->setFirstCycleAfterAdjust(false);
	}
	else
	{
		//this is the interpolation method
		acc =
			current_acc + (acc_target - current_acc)
			/ this->getAccSmoothCoef();
		//*(this->getAccSmoothCoef()-1); 


	}

	double vel = v + acc*delta_t;
	if (vel < 0)
	{
		vel = 0;
	}
	x_CF = x + (vel + v) / 2 * delta_t;

	double target_v = v + acc_target*delta_t;
	target_pos = x + (vel + target_v) / 2 * delta_t;

	return x_CF;
}


double myVehicleDef::NGSIM_Speed(double x, double v)
{
	double maxDec = getMAXdec();
	double maxAcc = getMAXacc();
	double reaction_time = getReactionTime();
	double v_leader = ((myVehicleDef*)leader)->getSpeed();
	double x_leader = ((myVehicleDef*)leader)->getPosition();
	double l_leader = leader->getLength();
	double vf = this->freeflowspeed;
	double jamGap = getJamGap();
	double headway = x_leader - x;
	double delta_t = AKIGetSimulationStepTime();

	//this coefficient is suggested to be larger than 
	//the maximum dec of the current vehicle for stability purpose
	double b_estimate = getEstimateLeaderDecCoeff()
		*maxDec;
	if (this->leader != NULL && leader->isFictitious() == false)
	{
		// Assume the subject driver knows the max deceleration of the leader
		b_estimate = MIN(((myVehicleDef*)leader)->getMAXdec()*getEstimateLeaderDecCoeff(),
			b_estimate);
	}

	double theta = this->getGippsTheta()*reaction_time; // Extra reaction time?

	// The maximum speed based on Gipps safety criterion
	double adjFactor = 0.1;
	double v_after_tau = GippsDecelerationTerm(
		maxDec, reaction_time * adjFactor, 0, x_leader, x, jamGap,
		l_leader, v, v_leader, b_estimate * adjFactor);


	double max_a = MIN(maxAcc,
		(v_after_tau - v) / reaction_time);

	if (this->getVehType() == Truck_Type)
		max_a = MAX(max_a, maxDec); // There might be a crash for the truck.

	// The acceleration based on Newell rule, bound by maxAcc and MaxDec
	double desired_headway = getDesireHeadway();
	double newell_a = MIN(maxAcc,
		MAX(maxDec,
		((headway - l_leader - jamGap) / desired_headway - v)
		/ (desired_headway / 2)));    //Newell original paper Eq. (9)

	// free acc
	//double min_a = maxAcc*(1-v/vf); 
	double min_a = maxAcc;
	if (v > 0)
		min_a = maxAcc*(1 - pow((v / vf), this->getAccExp()));
	//when one reduces from a speed that exceeds free flow speed the deceleration should not be too much
	min_a = MAX(min_a, this->getComfDecDLC());

	double current_acc = (getSpeed(0) - getSpeed(1)) / delta_t;
	double acc_target = MIN(MIN(min_a, newell_a), max_a);
	double acc = acc_target;

	acc =
		current_acc + (acc_target - current_acc)
		/ this->getAccSmoothCoef();

	double vel = v + acc*delta_t;
	if (vel < 0)
	{
		vel = 0;
	}
	return (vel + v) / 2;


}

void myVehicleDef::getGapHeadwayLeader(
	double& gap,
	double& headway,
	double& l_leader,
	double& ref_pos_front)
{
	if (leader->isFictitious() == false)
		ref_pos_front = this->followingDistance;
	else
		ref_pos_front = this->followingDistance + leader->getLength();




	l_leader = leader->getLength();
	headway = ref_pos_front;
	gap = headway - l_leader;
}

// veh is the preceding vehicle
double myVehicleDef::getDistanceHeadway(int stateSV, myVehicleDef* refVeh, int stateRefVeh)
{
	double pos = this->getPosition(stateSV);;
	double posUp = 0;
	double headway = A2SimVehicle::getPositionReferenceVeh(stateSV, refVeh, stateRefVeh);

	if (headway > 0)
	{
		return headway;
	}

	// Aimsun may return negative headway in a curve road
	if (refVeh != NULL)
	{
		posUp = refVeh->getPosition(stateRefVeh);

		// Get headway where there is a leader
		if (refVeh->getIdCurrentSection() == this->getIdCurrentSection())
		{
			// Same section
			headway = pos - posUp;
		}
		else if (refVeh->nextsec == this->getIdCurrentSection())
		{
			// The leader is in the next section
			headway = pos + refVeh->sec_inf.length - posUp;
		}
		else if (refVeh->nextnextsec == this->getIdCurrentSection())
		{
			headway = pos + refVeh->sec_inf.length - posUp + AKIInfNetGetSectionANGInf(refVeh->nextsec).length;
		}
		else if (refVeh->thirdsec == this->getIdCurrentSection())
		{
			headway = pos + refVeh->sec_inf.length - posUp + AKIInfNetGetSectionANGInf(refVeh->nextsec).length
				+ AKIInfNetGetSectionANGInf(refVeh->nextnextsec).length;
		}
		else
		{
			headway = 500;
		}
	}
	else
	{
		headway = 500; // Returns a large headway when there is no leader
	}

	return headway;
}

double myVehicleDef::getPositionReferenceVeh(int stateSV, A2SimVehicle* refVeh, int stateRefVeh)
{
	double headway = this->getDistanceHeadway(stateSV, (myVehicleDef*)refVeh, stateRefVeh);
	return headway;
}

double myVehicleDef::getPositionReferenceVeh(int stateSV, myVehicleDef* refVeh, int stateRefVeh)
{
	double headway = this->getDistanceHeadway(stateSV, refVeh, stateRefVeh);
	return headway;
}

// override the base class to automatically include state
// this can be used only for the current states
double myVehicleDef::getPositionReferenceVeh(myVehicleDef* ref_veh)
{
	int ref_state = ref_veh->isUpdated() ? 1 : 0;
	int this_state = isUpdated() ? 1 : 0;

	double headway = this->getDistanceHeadway(this_state, ref_veh, ref_state);
	return headway;

	//return A2SimVehicle::getPositionReferenceVeh(this_state, ref_veh,
	//	ref_state);

	//return this->getDistanceHeadway();
}

//----------------------------------------------------------------------------------------------------------
// Car-following functions
//----------------------------------------------------------------------------------------------------------

// Function PosCf
// returns base car-following distance
//
double myVehicleDef::PosCf
(const A2SimVehicle* leader, int shortGap, double beta, double alpha, double relaxation)
{
	double	x_CF = 0.0;
	double a_U = getMAXacc();  			//a_U>0 
	double a_L = getMAXdec(); 			//a_L<0
	double jamGap = getJamGap();
	double vf = this->freeflowspeed;
	double v = getSpeed(0);
	double x = getPosition(0);
	double reaction = getReactionTime();
	double l_leader = 0.0;
	double a_L_leader = 0.0;
	double d_leader = 0.0;
	double v_leader = 0.0;
	double x_leader = 0.0;
	double headway = 0.0; // space headway

	if (leader != NULL && leader != this)
	{
		//Has_Leader = 1;
		//if the leader has updated, then use its previous step
		//if (leader->isFictitious() == false)
		//	ref_pos_front = ((myVehicleDef*)leader)->getPositionReferenceVeh(1, this, 0);
		//else
		//	ref_pos_front = leader->getPositionReferenceVeh(0, this, 0) + leader->getLength();

		if (leader->isFictitious() == false)
		{
			x_leader = x + this->followingDistance;
			headway = this->followingDistance;
		}
		else
		{
			// This is used for the signalized intersections. Aimsun assumes the signal has a length of 1.5m.
			// We don't have to consider the length of the signal in the CF model_Hao 12/05/17
			x_leader = x + this->followingDistance + leader->getLength();
			headway = this->followingDistance + leader->getLength();
		}


		l_leader = leader->getLength();

		// the update judgment has been put input the reloaded getspeed
		v_leader = ((myVehicleDef*)leader)->getSpeed();

		double leader_past_pos = 0; //This parameter is not used by any mode_Hao 11/30
		//leader_past_pos = ((myVehicleDef*)leader)->GetPastPos(getReactionTime()*beta);

		a_L_leader = ((myVehicleDef *)leader)->getMAXdec();
		d_leader = -(v_leader * v_leader)
			/ (2 * a_L_leader)*(shortGap == 1 ? relaxation : 1);

		if (shortGap == 1)
			x_CF = BaseCfModel(a_L, a_U, reaction*beta,
			headway, jamGap*alpha, d_leader,
			l_leader, vf, v, x, x_leader,
			leader_past_pos,
			((myVehicleDef*)leader)->getSpeed(),
			this->getDesireHeadway()*relaxation);
		else
			x_CF = BaseCfModel(a_L, a_U, reaction,
			headway, jamGap, d_leader,
			l_leader, vf, v, x, x_leader,
			leader_past_pos,
			((myVehicleDef*)leader)->getSpeed(),
			this->getDesireHeadway());
	}
	else
	{
		// IDM model: No leader and accelerates to the free flow speed mildly
		double var1 = delta_t*v;
		double var2 = 0.5*a_U*(1 - pow(v / vf, delta));
		double var3 = pow(delta_t, 2);
		double var4 = var2*var3;
		x_CF = x + delta_t*v +
			0.5*a_U*(1 - pow(v / vf, delta))*pow(delta_t, 2);
	}
	return x_CF;
}

//////////////////////////////////////////////////////////////////////////
// overload of poscf without shortgap mode
double myVehicleDef::PosCf(const A2SimVehicle* leader)
{
	return PosCf(leader, 0, 1, 1, 1);
}

//this function determines if the driver decides to do a lane-changing
//what is the target lane, and how strong is the desire.
//both discretionary and mandatory are combined together
//using a weight coefficient
bool myVehicleDef::NeedLC()
{
	ResetDesires();

	if (this->isCurrentLaneInNode())
	{
		return false;
	}

	//if (this->isCurrentLaneInNode())
	//	return false;
	if (this->GetRampType(this->sec_inf.angId) == TRUE_ON_RAMP
		|| this->sec_inf.id == source_section) //do not allow lc on source section
		return CombineLCDesires();

	//discretionary 
	bool dlcFlag = NeedDlc();

	//mandatory options are exclusive
	bool rampLcFlag = NeedRampLc();
	bool turningLcFlag = false;
	if (!rampLcFlag)
		turningLcFlag = NeedLc4Turning();

	//combine all options
	//within combine desires, we set lc type
	bool combinedDesireFlag = CombineLCDesires();
	return combinedDesireFlag;
}

double myVehicleDef::getPosition(int state)
{
	double x = 0.0;
	x = A2SimVehicle::getPosition(state);
	//this is probably because the Aimsum did not save that many states for use
	//so we must use our own list
	//if (state > 1 && this->ModelApplied.compare("NGSIM") == 0)
	//{
	//	if (!this->position_queue.empty() && this->position_queue.size() >= state)
	//	{
	//		//std::list<double>::iterator it = this->position_queue.begin();

	//		double pos = this->position_queue.at(this->position_queue.size() - state);
	//		return pos;
	//	}
	//	else
	//	{
	//		if (this->position_queue.size() > 0)
	//			return this->position_queue.front();
	//		else
	//			return this->getPosition();
	//	}
	//}
	return x;
}

myVehicleDef::PositionSpeed myVehicleDef::setNewPosition(double pos, double velocity)
{
	PositionSpeed pos_speed;

	//if (this->ModelApplied.compare("NGSIM") == 0)
	//{
	//	position_queue.push_back(this->getPosition());
	//	if (this->position_queue.size() > this->getReactionTime() / delta_t + 10)
	//		this->position_queue.pop_front();
	//}
	if (velocity < 0)
	{
		velocity = 0;
		pos = this->getPosition() + this->getSpeed() / 2 * delta_t;
	}

	if (this->leader != NULL)
	{
		// Subject vehicle stops if a crash will happen after update
		CrashAvoidancePosition(velocity, pos);
	}

	pos_speed.position = pos;
	pos_speed.speed = velocity;
	return pos_speed;
}

void myVehicleDef::CrashAvoidancePosition(double& velocity,
	double& pos)
{
	if (this->leader != NULL
		&&
		this->getNewArrivalAdjust() == false)
	{
		double gap = 0;
		double headway = 0;
		double l_leader = 0;
		double ref_pos_front = 0;
		getGapHeadwayLeader(
			gap,
			headway,
			l_leader,
			ref_pos_front
			);
		if (pos >= ref_pos_front + this->getPosition())
		{
			char* ch = "Avoid Crash!!!!!";
			if (this->getVehType() == CARveh)
			{
				ch = "Avoid Crash for human driving vehicle";
			}
			else if (this->getVehType() == CACCveh ||
				this->getVehType() == ACCveh)
			{
				if (this->adaptive_mode == CACCINMAN)
				{
					ch = "Avoid Crash for CACC vehicle in manual driven mode";
				}
				else if (this->adaptive_mode == ACCMODE)
				{
					if (this->getVehType() == ACCveh)
						ch = "Avoid Crash for ACC vehicle in ACC mode";
					else
						ch = "Avoid Crash for CACC vehicle in ACC mode";
				}
				else if (this->adaptive_mode == ACCINMAN)
				{
					if (this->getVehType() == ACCveh)
						ch = "Avoid Crash for ACC vehicle in MAN mode";
					else
						ch = "Avoid Crash for CACC vehicle in MAN mode";
				}
				else if (this->adaptive_mode == CACCMODE)
				{
					ch = "Avoid Crash for CACC vehicle in CACC mode";
				}
			}

			RecordCrashInformation();

			if (gap < 0)
			{
				AKIPrintString(ch);
				char str[1280];
				sprintf_s(str,
					"Crash gap: %.1f, headway: %f, leader length: %f\n",
					gap, headway, l_leader);
				AKIPrintString(str);
				sprintf_s(str,
					"Crash time: %.1f, Veh ID: %d, Driving Mode: %d, Sec ID: %d, Lane ID: %d, Pos: %.1f, Speed: %.1f, LC Type: %d, LC Time: %.1f, Target Lane: %d\n",
					AKIGetCurrentSimulationTime(), this->getId(), this->getMode(),
					this->getIdCurrentSection(), this->getIdCurrentLane(),
					this->getPosition(), this->getSpeed(), this->getLastLCType(), this->getLastLCTime(), this->getLastLCTarget());
				AKIPrintString(str);
				myVehicleDef* temp = (myVehicleDef*)leader;
				sprintf_s(str,
					"Leader Driving Mode: %d, Leader ID: %d, Sec ID: %d, Lane ID: %d, Pos: %.1f, Speed: %.1f, LC Type: %d, LC Time: %.1f, Target Lane: %d\n",
					temp->getMode(), temp->getId(), temp->getIdCurrentSection(),
					temp->getIdCurrentLane(),
					temp->getPosition(), temp->getSpeed(),
					temp->getLastLCType(), temp->getLastLCTime(), temp->getLastLCTarget());
				AKIPrintString(str);

				//ANGSetSimulationOrder(3, AKIGetCurrentSimulationTime());

				int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());
				const unsigned short *temp1 = AKIConvertFromAsciiString("isCrashed");
				ANGConnSetAttributeValueBool(ANGConnGetAttribute(temp1), VehANGId, true);   //the number of the vehicle ahead in the platoon.
				VehANGId = ANGConnVehGetGKSimVehicleId(temp->getId());
				ANGConnSetAttributeValueBool(ANGConnGetAttribute(temp1), VehANGId, true);
				AKIDeleteUNICODEString(temp1);
			}


			////stop when crash happens
			pos = this->getPosition(0);
			velocity = 0;
			this->isCrashed = true;
		}
		else
		{
			this->isCrashed = false;
		}
	}
}


double myVehicleDef::getPosition()
{
	int state = 0;
	double v = 0.0;
	state = isUpdated() ? 1 : 0;
	v = A2SimVehicle::getPosition(state);
	return v;
}
// get updated speed for a 
double myVehicleDef::getSpeed()
{
	int state = 0;
	double v = 0.0;
	state = isUpdated() ? 1 : 0;
	v = A2SimVehicle::getSpeed(state);
	return MAX(v, 0);
}

double myVehicleDef::getSpeed(int state)
{
	return MAX(A2SimVehicle::getSpeed(state), 0);
}


const A2SimVehicle* myVehicleDef::getLeader()
{
	//For test
	A2SimVehicle* upVeh;
	A2SimVehicle* downVeh;
	double upShift = 0;
	double downShift = 0;
	getRealUpDown(0, this->getPosition(0), upVeh, upShift, downVeh, downShift);

	A2SimVehicle* downFic;
	double shiftFic;
	getUpDown(0, this->getPosition(0), upVeh, upShift, downFic, shiftFic);

	if (downVeh && downFic)
	{
		double posDown = downVeh->getPosition(0);
		double posFic = downFic->getPosition(0);
		double posSub = this->getPosition(0);
		double distDown = posDown + downShift - posSub;
		double distFic = posFic + shiftFic - posSub;

		if (distDown <= distFic)
		{
			this->followingDistance = distDown;
			return downVeh;
		}
		else
		{
			this->followingDistance = distFic;
			return downFic;
		}
	}
	else if (downFic)
	{
		double posFic = downFic->getPosition(0);
		double posSub = this->getPosition(0);
		double distFic = posFic + shiftFic - posSub;
		this->followingDistance = distFic;
		return downFic;
	}
	else if (downVeh)
	{
		double posDown = downVeh->getPosition(0);
		double posSub = this->getPosition(0);
		double distDown = posDown + downShift - posSub;
		this->followingDistance = distDown;
		return downVeh;
	}
	else
	{
		this->followingDistance = 1000;
		return NULL;
	}

}

double myVehicleDef::createFreeFlowSpeed(bool ACCCACC)
{
	double single_free = A2SimVehicle::getFreeFlowSpeed();
	single_free = MIN(sec_inf.speedLimit / 3.6, single_free);

	// Variation of free flow speed across lanes++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Empirical data is needed to determine the speed distribution across lanes! _Hao 11/28
	//int laneID = this->getIdCurrentLane();
	//// Do nothing for the leftmost lane
	//if (laneID == this->getNumberOfLanesInCurrentSection() - 1)
	//{
	//	// 2nd Leftmost lane
	//	single_free = single_free * 0.95;
	//}
	//else if (laneID < this->getNumberOfLanesInCurrentSection() - 1)
	//{
	//	// lanes to the right of the 2nd Leftmost lane
	//	single_free = single_free * 0.9;
	//}
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	if (ACCCACC)
		return single_free;

	//add lane specified speed limit distribution

	//for friction we only look a few distance away and a few cars ahead
	double v_left = this->left_avg_speed_ahead; //single_free;  
	double v_right = this->right_avg_speed_ahead;
	if (v_left <= 0)
	{
		v_left = v_right;
	}
	if (v_right <= 0)
	{
		v_right = v_left;
	}

	//For vehicles trying to exit the freeway or merge, the freeway flow speed is determined based on the average speed of the right/left lane Hao 03/06/17
	if (this->getMode() == BCF
		&& this->getTargetLane() == RIGHT
		&& this->getLCType() == TURNING)
	{
		return MIN(v_right + 3, single_free);
	}
	else if (this->getMode() == BCF
		&& this->getTargetLane() == LEFT
		&& this->getLCType() == RAMP)
	{
		return MIN(v_left + 3, single_free);
	}

	//consider friction due to the adjacent lanes
	double v_friction = MIN(v_right, v_left);
	if (this->GetRampType(sec_inf.id) != ON_RAMP)
		v_friction = v_right;
	double v_sigma = 5.0; // threshold of speed difference between lanes
	if (v_friction > 0 && single_free > v_friction + v_sigma)
		//only if the friction speed is very slow otherwise still keep the current lane's free flow speed
	{
		std::default_random_engine generator;
		std::uniform_real_distribution<double> distribution(0.5, 1.0);
		double v = MIN(single_free, v_friction + v_sigma + 5.0 * distribution(generator));
		return v;
		return v_friction + (single_free - v_friction)*this->getFrictionCoef();
		//this actually is not correct because free flow speed is not the same 
		//as how fast this vehicle can actually go

		//return MIN(single_free, v_friction + MPH2MPS(10));  //maximum 20 above the friction speed
	}

	return single_free;
}

double myVehicleDef::getFreeFlowSpeed()
{
	return this->freeflowspeed;
}

//***********************************************************************
//extension to myVehicleDef::DetermineLcOrMergeOrCoop()
//add the decision to turn on ACC 
//By Jerry
//***********************************************************************/
int myVehicleDef::DetermineLcOrMergeOrCoopOrACC()
{

	double v_now = this->getSpeed();
	//first decide if lane change is necessary
	if (NeedLC())
	{
		ACC_recover_time = 0;
		return this->setMode(BCF);

	}
	//if cooperation is needed
	else if (NeedCoop())
	{
		ACC_recover_time = 0;
		return this->setMode(CCF);

	}
	else if (this->leader != NULL
		&&
		this->ACC_Manual_TakeOver_Check_CAMP(v_now)
		)
	{
		//fail the CAMP safety criterion
		//keep manual driving mode.
		//setLaneChangeDesireThrd(0.5);
		ACC_recover_time = 0;
		return this->setMode(CF);

	}
	else
	{
		//Turn on ACC mode if driver keeps following in the same lane.
		if (ACC_recover_time < ACC_Recover_Time)
		{
			ACC_recover_time++;
			return this->setMode(CF);
		}
		else
		{
			//return this->setMode(CF);
			return this->setMode(ACC_ON);  //special_test_0706
		}
	}

}


int myVehicleDef::DetermineLcOrMergeOrCoop()
{
	//first decide if lane change is necessary
	if (NeedCoop())
	{
		return this->setMode(CCF);
	}
	else if (NeedLC())
	{
		return this->setMode(BCF);
	}
	else
		return this->getMode();

}

int myVehicleDef::Determine2ndLcAfterLc()
{
	//first decide if lane change is necessary
	if (NeedCoop())
	{
		return this->setMode(CCF);
	}
	else if (NeedLC())
	{
		return this->setMode(BCF);
	}
	else
	{
		if (isAfterLaneChangeFinish())
			return this->setMode(CF);
		else
			return getMode();
	}
}

int myVehicleDef::determineCoopOrLc()
{
	if (NeedCoop())
	{
		return setMode(CCF);
	}
	else if (NeedLC())
	{
		return setMode(BCF);
	}
	else
	{
		return setMode(CF);
	}
	//else
	//{
	//	return getMode();
	//}
}



//////////////////////////////////////////////////////////////////////////
//this function accelerates vehicle so that it can overpass the leader on the target lane
//////////////////////////////////////////////////////////////////////////
myVehicleDef::PositionSpeed myVehicleDef::BeforeExitorTurningOverPass()
{
	PositionSpeed pos_speed;

	double acc = this->getOffRampOverpassAcc(); //get overpass acc
	//apply this acc
	double speed = this->getSpeed() + acc*delta_t;  //get the speed in the next time interval
	speed = MIN(speed, A2SimVehicle::getFreeFlowSpeed()*1.2);  // even for overpass;  it cannot surpass the speed limit by 20%
	double posOverpass = (this->getSpeed() + speed) / 2.0*delta_t + this->getPosition();  //get the position of next time interval

	double posDisFollowCurrentLeader = PosCf2EndofExitTurning();
	if (this->leader != NULL)
	{
		posDisFollowCurrentLeader =
			MinCollisionAvoidancePos(this->leader, 1, //this is in short-gap mode
			beta, alpha, Relaxation); //this is the distance to avoid collision
	}

	pos_speed.position = MIN(posOverpass, posDisFollowCurrentLeader);
	pos_speed.speed = (MIN(posOverpass, posDisFollowCurrentLeader) - this->getPosition()) / delta_t * 2 - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(MIN(posOverpass, posDisFollowCurrentLeader),
	//	(MIN(posOverpass,posDisFollowCurrentLeader) - this->getPosition()) / delta_t*2-this->getSpeed()); 
}

// This function models the gap seeking behaviors for a HOV vehicle that tries to merge into the HOV lane
myVehicleDef::PositionSpeed myVehicleDef::BeforeLeftLaneChangingMove4HOV()
{
	PositionSpeed pos_speed;

	double acc = this->getOffRampOverpassAcc(); //get overpass acc
	//apply overpass acc
	double speed = this->getSpeed() + acc*delta_t;  //get the speed in the next time interval

	speed = MIN(speed, this->freeflowspeed);  // even for overpass 
	double posOverpass = (this->getSpeed() + speed) / 2.0*delta_t + this->getPosition();  //get the position of next time interval

	double posDisFollowCurrentLeader = 0;
	if (this->leader != NULL)
	{
		posDisFollowCurrentLeader =
			MinCollisionAvoidancePos(this->leader, 1, //this is in short-gap mode
			beta, alpha, Relaxation); //this is the distance to avoid collision
	}
	else
	{
		posDisFollowCurrentLeader = PosCf(this->leader, 1, beta, alpha, Relaxation);
	}

	pos_speed.position = MIN(posOverpass, posDisFollowCurrentLeader);
	pos_speed.speed = (MIN(posOverpass, posDisFollowCurrentLeader) - this->getPosition()) / delta_t * 2 - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
}

// Behavior function about before a lane change after the decision is made
// 1) either try to sync its speed with respect to the leader on the target lane
//    or slow down to skip the current gap
myVehicleDef::PositionSpeed myVehicleDef::UpdateBeforeLaneChangeCf()
{
	PositionSpeed pos_speed;

	if (VehID == debug_track_id)
		VehID = VehID;

	if (this->getSpeed() == 0)
	{
		// The driver will take aggressive lane change if waiting for too long.
		this->waiting_time += 1;
		if (this->IsLaneOnRamp(0) == false && this->getLCType() != DLC)
		{
			if (this->waiting_time > MAX_WAIT_TIME / this->delta_t)
				//become risky
				SetRiskyRelax();
		}
	}
	else
	{
		this->waiting_time = 0;
	}

	int CACC_Lane_Flag = this->CACC_ML_Activated;
	int CACC_Lane_Number = this->number_CACC_ML;

	const A2SimVehicle *templeader = this->getTargetLane() == RIGHT ? this->right_leader : this->left_leader;
	const A2SimVehicle *tempfollower = this->getTargetLane() == RIGHT ? this->right_follower : this->left_follower;

	if (this->LCType == EXIT || this->LCType == TURNING) //type 1: exit or turning
	{
		int decision = ExitCfDecision();
		int OppLaneLC = this->getTargetLane() == LEFT ? RIGHT : LEFT;

		if (this->getHOV() //this car is a hov car
			&&
			this->isHOVActive() // hov is active
			&&
			this->GetSectionHOVLane() > 0 // this section has a HOV lane
			&&
			this->getTargetLane() == LEFT) // the vehicle is merging to the left lane
		{
			// Model the LC behavior toward the HOV lane
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
		}
		else if (this->getHOV() //this car is a hov car
			&&
			this->isHOVActive() // hov is active
			&&
			this->GetSectionHOVLane() > 0 // this section has a HOV lane
			&&
			this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection()// the vehicle is in the HOV lane
			&&
			this->getTargetLane() == RIGHT) // the vehicle wants to exit the HOV lane
		{
			// Model the LC behavior of exiting the HOV lane
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
		}
		else if (
			(GetRampType(this->getIdCurrentSection()) == ON_RAMP || GetRampType(this->nextsec) == ON_RAMP) // On-ramp section
			&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection() // Mainstream vehicle
			&& this->getIdCurrentLane() < this->getNumberOfLanesInCurrentSection() // Not in the leftmost lane
			&& this->getTargetLane() == LEFT)
		{
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
		}
		else if (// The subject vehicles try to make LC toward the CACC managed lane Hao 3/8/17
			this->getTargetLane() == LEFT
			&& CACC_Lane_Flag == 1 // CACC managed lane is activated
			&& (this->getVehType() == NGSIM_CACC_ACC_V2V || (this->getVehType() == CAR_Type && this->ConnectFlag) || this->getVehType() == CACC_HOV) // CACC vehicle or VAD vehicle
			&& this->getDistanceToOffRamp() < 0 // The subject vehicle is not going to exit the freeway
			&& this->getIdCurrentLane() <= this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number // The subject vehicle is not in the managed lane
			&& this->getNumberOfLanesInCurrentSection() > 2 // The subject vehicle is in the freeway mainline
			&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection() // The subject vehicle is not in the acceleration lane
			)
		{
			if (this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number)
			{
				//We do not want create large disturbance to the managed lane
				if (this->desireLC >= 0.25
					&& this->left_leader != NULL
					&& ((myVehicleDef*)this->left_leader)->getPositionReferenceVeh(0, this, 0) - this->getSpeed(0) * delta_t + this->left_leader->getSpeed(0) * delta_t > this->left_leader->getLength() + this->getSpeed(0) * 0.8
					&& this->left_follower != NULL
					&& this->getPositionReferenceVeh((myVehicleDef*)this->left_follower) - this->left_follower->getSpeed(0) * delta_t + this->getSpeed(0) * delta_t > this->getLength() + this->left_follower->getSpeed(0) * 0.8
					)
				{
					pos_speed = this->UpdateLc();
				}
				else
				{
					pos_speed = this->BeforeLeftLaneChangingMove4HOV();
				}
			}
			else if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
		}
		else if (this->LCType == EXIT)
		{
			// For vehicles that want to exit the freeway but it is far from the off-ramp 3/6/17 Hao
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
		}
		else if (
			this->getTargetLane() == RIGHT
			&& this->desireLC_force_right < 1
			//&& this->getEqvDis2NextTurning() <= this->getE4OffRamp()
			//&& this->getEqvDis2NextTurning() > this->getMinE4OffRamp()
			)
		{
			// The distance to the off-ramp is less than E or the time to the off-ramp is less than T 3/6/17 Hao
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{

				if (this->right_follower != NULL
					&& ((myVehicleDef*)this->right_follower)->CoopRequester == this
					&& this->rightLagHeadway >= getJamGap() + this->getLength()
					&& this->getSpeed(0) > this->right_follower->getSpeed(0)
					)
				{
					// The subject vehicle has a cooperative lag vehicle. Synchronize the speed with the lag vehicle.
					pos_speed = this->BeforeOffRampLcSlowDown();
				}
				else
				{
					pos_speed = this->BeforeLeftLaneChangingMove4HOV();
				}
			}
		}
		else if (
			this->getTargetLane() == RIGHT
			&& this->desireLC_force_right >= 1)
		{
			// When x <= minE or T <= minT 3/6/17 Hao
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else if (this->right_leader != NULL
				&& this->rightLeadHeadway - this->getSpeed(0) * delta_t + this->right_leader->getSpeed(0) * delta_t > this->right_leader->getLength() + 1.5//this->getSpeed(0) * 0.2
				&& this->right_follower != NULL
				&& this->rightLagHeadway - this->right_follower->getSpeed(0) * delta_t + this->getSpeed(0) * delta_t > this->getLength() + 1.5//this->right_follower->getSpeed(0) * 0.2
				)
			{
				// aggresive LC at the end of the off-ramp
				pos_speed = this->UpdateLc();
			}
			else if (this->right_leader == NULL)
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
			else if (this->right_follower != NULL
				&& ((myVehicleDef*)this->right_follower)->desireLC_force_left >= 1
				)
			{
				// Overpass the leader that is not very far ahead of the subject vehicle
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
			else
			{
				// Slow down and try to merge in the following gap. The on-ramp CF model is used here.
				pos_speed = this->BeforeExitorTurningLcSync();
			}
		}
		else if (// The subject vehicles try to make LC to exit the CACC managed lane Hao 3/9/17
			this->getTargetLane() == RIGHT
			&& CACC_Lane_Flag == 1 // CACC managed lane is activated
			&& (this->getVehType() == NGSIM_ACC || (this->getVehType() == CAR_Type && !this->ConnectFlag) || this->getVehType() == ACC_HOV) // CACC vehicle or VAD vehicle
			&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number // The subject vehicle is in the managed lane
			&& this->getNumberOfLanesInCurrentSection() > 2 // The subject vehicle is in the freeway mainline
			&& this->desireLC_force_right < 1
			)
		{
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
		}
		else if (
			this->getTargetLane() == LEFT
			&& this->desireLC_force_left < 1)
		{
			// The distance to the off-ramp is less than E or the time to the off-ramp is less than T 3/6/17 Hao
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else
			{

				if (this->left_follower != NULL
					&& ((myVehicleDef*)this->left_follower)->CoopRequester == this
					&& this->leftLagHeadway >= getJamGap() + this->getLength()
					&& this->getSpeed(0) > this->left_follower->getSpeed(0)
					)
				{
					// The subject vehicle has a cooperative lag vehicle. Synchronize the speed with the lag vehicle.
					pos_speed = this->BeforeExitorTurningLcSync();
				}
				else
				{
					pos_speed = this->BeforeLeftLaneChangingMove4HOV();
				}
			}
		}
		else if (
			this->getTargetLane() == LEFT
			&& this->desireLC_force_left >= 1)
		{
			// When x <= minE or T <= minT 3/6/17 Hao
			if (decision == EXIT_CHANGE_FEASIBLE)
			{
				// LC feasible
				pos_speed = this->UpdateLc();
			}
			else if (this->left_leader != NULL
				&& this->leftLeadHeadway - this->getSpeed(0) * delta_t + this->left_leader->getSpeed(0) * delta_t > this->left_leader->getLength() + 1.5//this->getSpeed(0) * 0.2
				&& this->left_follower != NULL
				&& this->leftLagHeadway - this->left_follower->getSpeed(0) * delta_t + this->getSpeed(0) * delta_t > this->getLength() + 1.5//this->right_follower->getSpeed(0) * 0.2
				)
			{
				// aggresive LC at the end of the off-ramp
				pos_speed = this->UpdateLc();
			}
			else if (this->left_leader == NULL)
			{
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
			else if (this->left_follower != NULL
				&& ((myVehicleDef*)this->left_follower)->desireLC_force_right >= 1
				)
			{
				// Overpass the leader that is not very far ahead of the subject vehicle
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
			else
			{
				// Slow down and try to merge in the following gap. The on-ramp CF model is used here.
				pos_speed = this->BeforeExitorTurningLcSync();
			}
		}
		else
		{
			//feasible 
			pos_speed = this->UpdateLc();
		}
	}
	else if (this->LCType == DLC) //type 3: optional 4 improving speed
	{

		//vehicles choose to sync with the leader on the target lane or slow to skip this gap
		int decision = DLCCFDecision();  //get CF options before lane changing
		//if (GetRampType(this->getIdCurrentSection()) == ON_RAMP && this->getTargetLane() == LEFT)
		//{
		//	// Vehicles in the mainstream change to the left lane to create gaps for on-ramp traffic
		//	pos_speed = this->BeforeLeftLaneChangingMove4HOV();
		//	return pos_speed;
		//}
		//else 

		if (AKIGetRandomNumber() >= 0.01 // 99% chance to make such an aggressive DLC in 5s
			&& this->getTargetLane() == LEFT
			&& this->desireLC >= 0.25
			&& this->left_leader != NULL
			&& ((myVehicleDef*)this->left_leader)->getPositionReferenceVeh(0, this, 0) - this->getSpeed(0) * delta_t + this->left_leader->getSpeed(0) * delta_t > this->left_leader->getLength() + this->getSpeed(0) * 0.5
			&& this->left_follower != NULL
			&& this->getPositionReferenceVeh((myVehicleDef*)this->left_follower) - this->left_follower->getSpeed(0) * delta_t + this->getSpeed(0) * delta_t > this->getLength() + this->left_follower->getSpeed(0) * 0.5
			)
		{
			// For very strong DLC motivation. It implies that the speed difference between two lanes is really large.
			pos_speed = this->UpdateLc();
		}
		else if (AKIGetRandomNumber() >= 0.01 // 99% chance to make such an aggresive DLC in 5s
			&& this->getTargetLane() == RIGHT
			&& this->desireLC >= 0.25
			&& this->right_leader != NULL
			&& ((myVehicleDef*)this->right_leader)->getPositionReferenceVeh(0, this, 0) + this->right_leader->getSpeed(0) * delta_t - this->getSpeed(0) * delta_t > this->right_leader->getLength() + this->getSpeed(0) * 0.5
			&& this->right_follower != NULL
			&& this->getPositionReferenceVeh((myVehicleDef*)this->right_follower) - this->right_follower->getSpeed(0) * delta_t + this->getSpeed(0) * delta_t > this->getLength() + this->getSpeed(0) * 0.5
			)
		{
			// LC feasible
			pos_speed = this->UpdateLc();
		}
		else if (decision == DLC_DECISION_SLOW_DOWN)
		{
			// for DLC, we do not slow down instead we just follow the leader normally
			// in this case, we would naturally let the current gap skip
			//pos_speed = this->updateRegularCf();
			pos_speed = this->BeforeLeftLaneChangingMove4HOV();
		}
		else if (decision == DLC_DECISION_FOLLOW)
		{
			//pos_speed = this->BeforeDLcSync();
			pos_speed = this->BeforeLeftLaneChangingMove4HOV();
		}
		else
		{
			pos_speed = this->UpdateLc();
		}
	}
	else if (this->LCType == RAMP) //type 4: on-ramp merging
	{
		//vehicles choose to sync with the leader on the target lane or slow to skip this gap
		int ramp_lc_decision = RampCfDecision();
		this->setRampDecision(ramp_lc_decision);

		if (ramp_lc_decision == RAMP_LANE_CHANGE_FEASIBLE)
		{
			pos_speed = this->UpdateLc();
		}
		else if (this->getDistance2Obstacle() > 30 && this->getSpeed(0) < 10)
		{
			pos_speed = this->BeforeLeftLaneChangingMove4HOV(); // Accelerate to the end of the acceleration lane before finding a gap at congested condition
		}
		else if (this->getTargetLane() == LEFT
			&& this->desireLC_force_left >= 1
			&& this->getDistance2Obstacle() < 30
			&& this->left_leader != NULL
			&& ((myVehicleDef*)this->left_leader)->getPositionReferenceVeh(0, this, 0) - this->getSpeed(0) * delta_t + this->left_leader->getSpeed(0) * delta_t > this->left_leader->getLength() + MAX(this->getSpeed(0) * 0.4, getJamGap())
			&& this->left_follower != NULL
			&& this->getPositionReferenceVeh((myVehicleDef*)this->left_follower) - this->left_follower->getSpeed(0) * delta_t + this->getSpeed(0) * delta_t > this->getLength() + MAX(this->left_follower->getSpeed(0) * 0.4, getJamGap())
			)
		{
			// aggressive LC at the end of the on-ramp
			pos_speed = this->UpdateLc();
		}
		else
		{
			if (ramp_lc_decision == RAMP_DECISION_SLOW_DOWN)
			{
				if (this->getLaneChangeDesire() > this->getRampLCSlowDownDesire())
				{
					pos_speed = this->BeforeOnRampLcSlowDown();
					//pos_speed = this->BeforeOnRampLcSync();
				}
				else
				{
					//pos_speed = this->updateRegularCf();
					pos_speed = this->BeforeLeftLaneChangingMove4HOV();
				}
			}
			else if (ramp_lc_decision == RAMP_DECISION_FOLLOW)
			{
				pos_speed = this->BeforeOnRampLcSync();
			}
			else if (ramp_lc_decision == RAMP_DECISION_NORMAL_FOLLOW
				|| PreventSimultaneousLC() == true)
			{
				//pos_speed = this->updateRegularCf();
				pos_speed = this->BeforeLeftLaneChangingMove4HOV();
			}
			else
			{
				pos_speed = this->UpdateLc();
			}
		}


	}
	return pos_speed;
}


// Update the position after lane changing
myVehicleDef::PositionSpeed myVehicleDef::UpdateAfterLaneChangeCf()
{
	PositionSpeed pos_speed;

	double new_beta =
		(1 - beta) / (double)ACF_Steps*(double)ACF_Step + beta;
	double new_alpha =
		(1 - alpha) / (double)ACF_Steps*(double)ACF_Step + alpha;
	double new_relaxation =
		(1 - Relaxation) / (double)ACF_Steps*(double)ACF_Step + Relaxation;
	//if (getLastLCType() == OPTIONAL_LC)
	//{
	//	ACF_Step = ACF_Step;
	//}
	double x_CF = PosCf
		(this->leader, 1, new_alpha, new_beta, new_relaxation);

	ACF_Step++;

	//setNewPosition(x_CF,
	//	2 * (x_CF - this->getPosition()) / delta_t - this->getSpeed()); 
	if (ACF_Step >= ACF_Steps)
	{
		ACF_Step = 0;
		setMode(CF);
	}
	pos_speed.position = x_CF;
	pos_speed.speed = 2 * (x_CF - this->getPosition()) / delta_t - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
}

// Apply cooperative car following model to give gap for lane changers
// as if it has two leaders
// but it only applies a comfortable deceleration
myVehicleDef::PositionSpeed myVehicleDef::updateCoopCf()
{
	PositionSpeed pos_speed;

	//follow the current leader in a normal way
	double nextPos = this->PosCf(leader);

	//CoopRequester is a diverging vehicle
	if (this->CoopRequester != NULL &&
		this->CoopRequester->VehID > 0
		&& this->CoopRequester->getTargetLane() == RIGHT)
	{
		double nextPosCoopVeh = this->PosCf(CoopRequester);// -this->getJamGap() * delta_t / this->getReactionTime();
		double v = 2 * (nextPosCoopVeh - this->getPosition()) / delta_t - this->getSpeed();

		//// In case the LC vehicle runs very slow
		//if (CoopRequester->getSpeed() < 5)
		//{
		//	v = MIN(v, CoopRequester->getSpeed());
		//}

		double acc = (v - this->getSpeed()) / delta_t;
		double min_dec = this->getComfDecDLC();
		if (acc < min_dec)
		{
			v = MAX(0, this->getSpeed() + delta_t*min_dec);
		}
		nextPosCoopVeh = this->getPosition() + (v + this->getSpeed())*delta_t / 2;
		nextPos = nextPosCoopVeh < nextPos ? nextPosCoopVeh : nextPos;
	}
	//CoopRequester is a merging vehicle
	else if (this->CoopRequester != NULL &&
		this->CoopRequester->VehID > 0
		&& this->CoopRequester->getDistance2Obstacle() > 30)
	{
		double nextPosCoopVeh = this->PosCf(CoopRequester);
		//but it will apply a deceleration that is beyond the comfortable
		double v = 2 * (nextPosCoopVeh - this->getPosition()) / delta_t - this->getSpeed();
		double acc = (v - this->getSpeed()) / delta_t;
		double min_dec = CoopRequester->getLCType() == OPTIONAL_LC ? this->getComfDecDLC() :
			this->getComfDecRampLC();
		if (acc < min_dec)
		{
			v = MAX(0, this->getSpeed() + delta_t*min_dec);
		}
		nextPosCoopVeh = this->getPosition() + (v + this->getSpeed())*delta_t / 2;
		nextPos = nextPosCoopVeh < nextPos ? nextPosCoopVeh : nextPos;
	}
	// no requester 
	else if (this->CoopRequester != NULL
		&& this->CoopRequester->VehID > 0)
	{
		double v = MAX(0, this->getSpeed() + delta_t*this->getMAXdec()*COMF_LEVEL);
		nextPos = this->getPosition() + (v + this->getSpeed())*delta_t / 2;
	}

	pos_speed.position = nextPos;
	pos_speed.speed = 2 * (nextPos - this->getPosition()) / delta_t - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(nextPos, 
	//	2*(nextPos - this->getPosition()) / delta_t-this->getSpeed()); 

}

// A vehicle cuts in and this function defines how the new follower follows this new leader
myVehicleDef::PositionSpeed myVehicleDef::UpdateReceiveCf()
{
	PositionSpeed pos_speed;

	double new_beta =
		(1 - beta) / (double)ACF_Steps*(double)ACF_Step + beta;
	double new_alpha =
		(1 - alpha) / (double)ACF_Steps*(double)ACF_Step + alpha;
	double new_relaxation =
		(1 - Relaxation) / (double)ACF_Steps*(double)ACF_Step + Relaxation;

	double x_temp = 0;
	myVehicleDef* temp_left = (myVehicleDef*)this->left_leader;
	myVehicleDef* temp_right = (myVehicleDef*)this->right_leader;
	if (temp_right != NULL && temp_right->getMode() == LC && temp_right->getTargetLane() == LEFT)
	{
		x_temp = PosCf
			(this->right_leader, 1, new_alpha, new_beta, new_relaxation);
	}
	else if (temp_left != NULL && temp_left->getMode() == LC && temp_left->getTargetLane() == RIGHT)
	{
		x_temp = PosCf
			(this->left_leader, 1, new_alpha, new_beta, new_relaxation);
	}
	else
	{
		x_temp = PosCf
			(this->leader, 1, new_alpha, new_beta, new_relaxation);
	}
	double x_CF = MIN(x_temp, PosCf(this->leader, 1, new_alpha, new_beta, new_relaxation));

	ACF_Step++;
	//setNewPosition(x_CF, 
	//	2*(x_CF - this->getPosition()) / delta_t-this->getSpeed()); 
	if (ACF_Step >= ACF_Steps)
	{
		ACF_Step = 0;
		setMode(CF);
	}
	pos_speed.position = x_CF;
	pos_speed.speed = 2 * (x_CF - this->getPosition()) / delta_t - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;

}

bool myVehicleDef::isAfterLaneChangeFinish()
{
	if (this->leader == NULL) // || this->leader->isFictitious())
	{
		this->ACF_Step = ACF_Steps;
		return true;
	}
	if (this->ACF_Steps == this->ACF_Step)
	{
		return true;
	}
	return false;
}

//determine if the car still wants to lane change or return to car following
int myVehicleDef::determineGapOrGiveup()
{
	if (NeedCoop())
	{
		// Only cooperate to mandatory LC to the off-ramp, even if the subject vehicle needs to make a LC 
		// Otherwise the left leader will miss the turn
		return this->setMode(CCF);
	}
	else if (NeedLC() == true)
	{
		return setMode(BCF);
	}
	else
	{
		return setMode(CF);
	}
}


int myVehicleDef::RampCfDecision()
{
	//if this vehicle has a direct leader than 
	// just let it follow the leader instead of seeking gaps

	if (this->leader != NULL
		&& this->leader->isFictitious() == false)
	{
		if (this->AllowUnsequentialMerging() == false)
			return RAMP_DECISION_NORMAL_FOLLOW;
	}
	return GapAcceptDecision_Sync_First();
}


//before off-ramp cruising to seek a gap
myVehicleDef::PositionSpeed myVehicleDef::BeforeOffRampLcCruise()
{
	PositionSpeed pos_speed;

	//if the speed is less than the cruising speed then speed up to the cruise speed when the leader allows
	//run synchronization based lane change
	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	double posCruise = 0;
	// for offramp slow down its speed will not lower than a certain speed [50 mph]
	if (this->getSpeed() < MIN_SPEED_OFF_RAMP_SLOWDOWN)
	{
		posCruise = this->getPosition() +
			(this->getSpeed() + this->getSpeed() + delta_t*getOffRampOverpassAcc())*delta_t / 2;

		posCruise = MIN(posCruise, this->getPosition() + MIN_SPEED_OFF_RAMP_SLOWDOWN*delta_t);
	}
	else
	{
		posCruise = this->getPosition() +
			(this->getSpeed() + this->getSpeed() + delta_t*this->getComfDecRampLC())*delta_t / 2;

		posCruise = MAX(posCruise, this->getPosition() + MIN_SPEED_OFF_RAMP_SLOWDOWN*delta_t);
	}

	//the current leader
	double posFollowLeader = posCruise;
	if (this->leader != NULL &&
		this->leader->isFictitious() == false)
	{
		posFollowLeader = MIN(posCruise,
			PosCf(this->leader, 1, beta, alpha, Relaxation));
	}

	double x = getPosition(0);
	pos_speed.position = posFollowLeader;
	pos_speed.speed = (posFollowLeader - x) / delta_t * 2 - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(posFollowLeader,	(posFollowLeader - x) / delta_t*2-this->getSpeed()); 
}

//before lane change on Ramp slow down to skip the current gap 
myVehicleDef::PositionSpeed myVehicleDef::BeforeOnRampLcSlowDown()
{
	PositionSpeed pos_speed;

	//run synchronization based lane change
	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	//now slow down to catch the next gap with a mild deceleration
	double posSlow = PosCfSkipGap(vehUp, true);
	//the current leader
	double posFollowCurrentLeader = 0;
	if (this->leader == NULL
		|| this->leader->isFictitious()
		|| this->leader->getIdCurrentSection() != this->current_sec_id
		) //if no leader, then pay attention to the end of the ramp
	{
		posFollowCurrentLeader = PosCf2EndofRamp();
	}
	else //with leader follow leader 
	{
		posFollowCurrentLeader = PosCf(this->leader, 1, beta, alpha, Relaxation);
	}

	////Prevent subject vehicle from moving backward
	double x = getPosition(0);
	//if (posFollowCurrentLeader < x || posSlow < x)
	//	posFollowCurrentLeader = posFollowCurrentLeader;

	pos_speed.position = MIN(posSlow, posFollowCurrentLeader);
	pos_speed.speed = (MIN(posSlow, posFollowCurrentLeader) - x) / delta_t * 2 - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(MIN(posSlow,posFollowCurrentLeader),
	//	(MIN(posSlow,posFollowCurrentLeader) - x) / delta_t*2-this->getSpeed()); 
}

// Before off-ramp LC slow down. It is used when a subject vehicle is close to the end of the off-ramp (i.e., force LC desire = 1)_Hao 03/15/17
myVehicleDef::PositionSpeed myVehicleDef::BeforeOffRampLcSlowDown()
{
	PositionSpeed pos_speed;

	//run synchronization based lane change
	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	//now slow down to catch the next gap with a mild deceleration
	double posSlow = PosCfSkipGap(vehUp, false);
	//the current leader
	double posFollowCurrentLeader = 0;
	if (this->leader == NULL
		|| this->leader->isFictitious()
		|| this->leader->getIdCurrentSection() != this->current_sec_id
		) //if no leader, then pay attention to the end of the ramp
	{
		posFollowCurrentLeader = PosCf2EndofRamp();
	}
	else //with leader follow leader 
	{
		posFollowCurrentLeader = PosCf(this->leader, 1, beta, alpha, Relaxation);
	}

	////Prevent subject vehicle from moving backward
	double x = getPosition(0);
	//if (posFollowCurrentLeader < x || posSlow < x)
	//	posFollowCurrentLeader = posFollowCurrentLeader;

	pos_speed.position = MIN(posSlow, posFollowCurrentLeader);
	pos_speed.speed = (MIN(posSlow, posFollowCurrentLeader) - x) / delta_t * 2 - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(MIN(posSlow,posFollowCurrentLeader),
	//	(MIN(posSlow,posFollowCurrentLeader) - x) / delta_t*2-this->getSpeed()); 
}

//before lane change on ramp sync the speed with respect to the leader on the target lane
myVehicleDef::PositionSpeed myVehicleDef::BeforeOnRampLcSync()
{
	PositionSpeed pos_speed;

	//run synchronization based lane change
	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	double x_CF_Sync = PosCf(vehDown, 1, beta, alpha, Relaxation);

	//for syncing the deceleration is not allowed to beyond the maximum deceleration	
	double max_accept_dec = this->getComfDecRampLC(); //maximum acceptable deceleration at the current desire level
	double speed = MAX(0, this->getSpeed() + max_accept_dec*delta_t);
	if (speed < SYNC_CREEP_SPEED)
	{
		speed = SYNC_CREEP_SPEED;
	}
	double x_CF_Sync_Limit = this->getPosition() + (this->getSpeed() + speed)*delta_t / 2;  //position based on this most acceptable deceleration

	x_CF_Sync = MAX(x_CF_Sync, x_CF_Sync_Limit);

	double x_CF_NoSync = 0;
	if (this->leader == NULL
		|| this->leader->isFictitious()
		|| this->leader->getIdCurrentSection() != this->current_sec_id
		)
	{
		x_CF_NoSync = PosCf2EndofRamp();
	}
	else
	{
		x_CF_NoSync = PosCf(this->leader, 1, beta, alpha, Relaxation);
	}
	double x = getPosition(0);

	pos_speed.position = MIN(x_CF_NoSync, x_CF_Sync);
	pos_speed.speed = (MIN(x_CF_NoSync, x_CF_Sync) - x) / delta_t * 2 - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(MIN(x_CF_NoSync,x_CF_Sync),
	//		(MIN(x_CF_NoSync,x_CF_Sync) - x) / delta_t*2-this->getSpeed()); 
}

// define SYNC behavior before a mandatory exit /turning LC.
myVehicleDef::PositionSpeed myVehicleDef::BeforeExitorTurningLcSync()
{
	PositionSpeed pos_speed;

	//run synchronization based lane change
	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	double x_CF_Sync = PosCf(vehDown, 1, beta, alpha, Relaxation);
	//here for syncing for off-ramp behavior is to sync with the speed of the leader not necessarily deceleration
	//if the current speed is already smaller than the leader on the target lane do not decelerate
	if (vehDown != NULL && vehDown->isFictitious() == false &&
		this->getSpeed() < ((myVehicleDef*)vehDown)->getSpeed() - NEC_SPEED_DIFF_OFF_RAMP_SYNC)
		//already slower than the leader by 5 mph
		x_CF_Sync = MAX(x_CF_Sync, this->getPosition() + this->getSpeed()*delta_t);

	//the current leader	
	//double posFollowEnd = PosCf2EndofExitTurning();
	//double x_CF_NoSync = posFollowEnd;
	double x_CF_NoSync;
	if (this->leader != NULL)
	{
		x_CF_NoSync = PosCf(this->leader, 1, beta, alpha, Relaxation);
	}
	else
	{
		x_CF_NoSync = this->getPosition(0) + this->getSpeed(0)*delta_t;
	}

	double x = getPosition(0);
	//x_CF_NoSync = MIN(x_CF_NoSync, posFollowEnd);//if no leader, then pay attention to the end of the ramp
	pos_speed.position = MIN(x_CF_NoSync, x_CF_Sync);
	pos_speed.speed = (MIN(x_CF_NoSync, x_CF_Sync) - x) / delta_t * 2 - this->getSpeed();
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	return pos_speed;
	//setNewPosition(MIN(x_CF_NoSync,x_CF_Sync),
	//	(MIN(x_CF_NoSync,x_CF_Sync) - x) / delta_t*2-this->getSpeed()); 
}

//Car following behavior when approaching the end of an on-ramp
double myVehicleDef::PosCf2EndofRamp()
{
	//simplify assume that there is a car at the end of the ramp
	return BaseCfModel(this->getMAXdec(),
		this->getMAXacc(),
		this->getReactionTime()*beta,
		this->distance2EndAccLane(), //the distance to the end of the ramp
		this->getJamGap()*alpha,
		0, 4,
		freeflowspeed, this->getSpeed(), this->getPosition(),
		this->posEndAccLane(),//assume there is a car at the end of the ramp with speed of zero
		this->posEndAccLane(), //the position of the car steps earlier
		0,
		this->getDesireHeadway());
}

//Car following behavior when approaching the position for a mandatory turning
//double myVehicleDef::PosCf2EndofTurningPoint()
//{
//	//simplify assume that there is a car at the end of the ramp
//	return BaseCfModel(this->getMAXdec(),
//		this->getMAXacc(),
//		this->getReactionTime()*beta,
//		this->getEqvDis2NextTurning(), //the distance to the end of turning location discounted by the number of lcs
//		this->getJamGap()*alpha,
//		0, 4,
//		freeflowspeed, this->getSpeed(), this->getPosition(),
//		this->getEqvDis2NextTurning(),//assume there is a car at the end of the turning point
//		this->getEqvDis2NextTurning(), //the position of the car steps earlier
//		0,
//		this->getDesireHeadway());
//}

//get the distance to the end of the on ramp
double myVehicleDef::distance2EndAccLane()
{
	if (this->GetRampType(sec_inf.id) == ON_RAMP)
		return this->getDistance2Obstacle();
	else
		return MAX(0.1, this->getDistanceToOffRamp());
}

//get the length of the acceleration lane
double myVehicleDef::posEndAccLane()
{
	/*int secId = this->getIdCurrentSection();
	A2KSectionInf sectionInfo = AKIInfNetGetSectionANGInf(secId); */
	if (this->GetRampType(current_sec_id) != ON_RAMP)
	{
		double dist_to_end_ramp = this->getDistanceToOffRamp();
		return this->getPosition() + this->getDistanceToOffRamp();
	}
	//return this->sec_inf.distance_OnRamp; 
	return this->getPosition() + this->getDistance2Obstacle();
}

double myVehicleDef::getLaneChangeDesire()
{
	return this->desireLC;
}


// lane change is feasible
myVehicleDef::PositionSpeed myVehicleDef::UpdateLc()
{
	PositionSpeed pos_speed;

	int mode = this->getMode();

	if (mode == BCF)
	{
		this->targetLaneID = this->getIdCurrentLane() - this->getTargetLane();
		this->setMode(LC);
	}

	double nextPos = 0;

	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	nextPos = this->PosCf(vehDown, 1, this->beta, this->alpha, this->Relaxation);
	if ((nextPos - getPosition()) / delta_t * 2 - this->getSpeed() > 30)
	{
		nextPos = nextPos;
	}

	if (this->getIdCurrentLane() != this->targetLaneID)
	{
		//A2SimVehicle::applyLaneChanging(-1*this->getTargetLane(), this->threadId);
		//applyLaneChanging(this->getTargetLane());
		//Notify the follower on the target lane to run in RCF mode
		if (vehUp != NULL
			&& vehUp->isFictitious() == false
			&& vehUp->getVehType() != NGSIM_CACC_ACC_V2V
			&& vehUp->getVehType() != CACC_HOV
			&& vehUp->getVehType() != NGSIM_ACC
			&& vehUp->getVehType() != ACC_HOV)
		{
			((myVehicleDef*)vehUp)->setMode(RCF);
		}
	}
	else
	{
		this->setLastLCTarget(this->getTargetLane());
		this->setLastLCTime(AKIGetCurrentSimulationTime());
		setLastLCType(this->getLCType());

		this->setMode(ACF);
		this->waiting_time = 0;
		ResetRelax();

	}

	pos_speed.position = nextPos;
	pos_speed.speed = (nextPos - getPosition()) / delta_t * 2 - this->getSpeed();
	return pos_speed;
}

myVehicleDef::PositionSpeed myVehicleDef::UpdateLc(const A2SimVehicle *vehDown,
	const int targetLane, const double newpos, const double newspeed)
{
	PositionSpeed pos_speed;

	int mode = this->getMode();
	setTargetLane(targetLane); //update target lane using the new input value


	if (mode != LC)
	{
		this->targetLaneID = this->getIdCurrentLane() - targetLane;
		this->setMode(LC);
	}


	const A2SimVehicle *vehUp = NULL;
	if (targetLane == LEFT)
	{
		vehUp = this->left_follower;
	}
	else
	{
		vehUp = this->right_follower;
	}


	if (this->getIdCurrentLane() == this->targetLaneID)
	{
		this->setLastLCTarget(this->getTargetLane());
		this->setLastLCTime(AKIGetCurrentSimulationTime());
		setLastLCType(this->getLCType());
		this->setMode(ACF);
		this->waiting_time = 0;
		ResetRelax();



	}

	pos_speed.position = newpos;
	pos_speed.speed = newspeed;
	return pos_speed;
}

//decide if the car needs a lane change on ramp
//in our model, the on-ramp is modeled as a side lane on the main section
//which is closer to the real case
//so this function tells if the vehicle is on the side lane
bool myVehicleDef::NeedRampLc()
{
	A2KSectionInf sectionInfo = this->sec_inf;

	int next_ramptype = this->GetRampType(this->nextsec);

	if (this->IsLaneOnRamp(0) == true)
	{
		double incentive =
			CalculateDesireForce(1, distance2EndAccLane(),
			this->getSpeed(), true);

		this->setLaneChangeDesireForce(incentive, 0);
		this->setMandatoryType(RAMP);
		//LC desire = 1 if the gap on the left lane is accepted
		BoostOnrampIncentive();
		return true;
	}
	else if (this->getIdCurrentLane() <= 1
		&&
		next_ramptype != TRUE_ON_RAMP
		&&
		next_ramptype != TRUE_OFF_RAMP
		&&
		exit_lane_from > (int)getIdCurrentLane())
	{
		double incentive =
			CalculateDesireForce(1, distance2EndAccLane(),
			this->getSpeed(), true);  //the left length is the section length

		this->setLaneChangeDesireForce(incentive, 0);
		this->setMandatoryType(RAMP);
		BoostOnrampIncentive();
		return true;
	}
	else // to see if the connecting lane on the next section is also a on-ramp
	{
		int next_sec_center_lanes = 0;
		if (getNextSectionRampType(next_sec_center_lanes) != ON_RAMP)
		{
			return false;
		}
		else
		{
			// the current lane is at the rightmost lane except the side lanes
			// and the number of central lanes reduces at next section  
			if ((this->getIdCurrentLane() - sectionInfo.nbSideLanes <= 1)
				&& sectionInfo.nbCentralLanes > next_sec_center_lanes)
			{
				unsigned int next_turning_id = getIdNextTurning();
				A2KSectionInf next_section_info = AKIInfNetGetSectionANGInf(next_turning_id);
				double incentive =
					CalculateDesireForce(1,
					next_section_info.length + this->sec_inf.length - this->getPosition(),
					this->getSpeed(), true);

				this->setLaneChangeDesireForce(incentive, 0);
				this->setMandatoryType(RAMP);
				BoostOnrampIncentive();
				return true;
			}
		}
	}

	return false;
}

double myVehicleDef::DLCDesire(int target_lane)
{
	if (!isLaneChangingPossible(target_lane)
		|| this->IsLaneOnRamp(target_lane))
		return 0;

	// DLC cannot result route departure
	int laneID = this->getIdCurrentLane();
	if (laneID - target_lane < this->exit_lane_from)
	{
		return 0;
	}
	else if (laneID - target_lane > this->exit_lane_to)
	{
		return 0;
	}

	double v = this->avg_speed_ahead;
	double ant_speed = target_lane == LEFT ? this->left_avg_speed_ahead : this->right_avg_speed_ahead;
	if (target_lane == LEFT
		&& this->left_leader != NULL)
	{
		// Only consider leader speed in the left lane
		ant_speed = MIN(((myVehicleDef*)this->left_leader)->getSpeed(), ant_speed);
	}

	//double threshold = target_lane == LEFT ? 4.47 : 0;  //favor left with 20 miles

	////does not favor left-lanes if this section does not need boost for left-lane distribution
	//if (!this->CurrentSectionSuitable4Boost())
	//	threshold = 0;

	//ant_speed += threshold;
	double tempspeed = MAX(v, MIN_DLC_SPEED);
	if (this->getHOV() == false //this car is not hov car
		||
		this->isHOVActive() == false // or hov is not active
		||
		this->GetSectionHOVLane() == 0) //this section has no HOV lane
	{
		//give strong desire to left-turn LC 
		//even if the speed on the left-lane is lower
		if (ant_speed < tempspeed)
		{
			return 0;
		}
		else
		{
			double desire = MIN(1, (ant_speed - tempspeed) / tempspeed);
			desire = MAX(0, desire);
			if (target_lane == RIGHT)
				desire *= this->getRightDLCCoeff(); // desire becomes less for right LC
			return desire;
		}
	}
	else //this is a hov car and hov is active and this section has hov lane
	{
		//the desire is biased to the hov lane 
		if (isLaneChangingPossible(LEFT) == false) //assuming hov is the leftmost lane 
			//if the car is already in the leftmost lane
			//do not give it right lane dlc desire
		{
			return 0;
		}
		else
		{
			if (ant_speed < tempspeed) // with a 10 mph lose will do nothing;  otherwise go generate incentive 
				return 0;
			else
			{
				double desire = MIN(1, (ant_speed - tempspeed) / tempspeed);
				desire = MAX(0, desire);
				if (target_lane == RIGHT)
					desire *= this->getRightDLCCoeff();

				//cheat here make HOV desire 100% for the leftmost lane
				//if (target_lane == RIGHT)
				//	desire = 0;
				//else
				//	desire = 1; //if this is a HOV increase the desire to move the left lane

				return desire;
			}
		}
	}
}

double myVehicleDef::DLCDesire_2(double target_lane)
{
	//if this vehicle is on ramp or this vehicle is 
	//not able to do a lane change return 0
	if (!isLaneChangingPossible(target_lane)
		|| this->IsLaneOnRamp(target_lane))
		return 0;

	double v = this->getSpeed();
	v = this->avg_speed_ahead;
	//double ant_speed = target_lane == LEFT?this->left_avg_speed_ahead:this->right_avg_speed_ahead;

	double ant_speed = 120 / 3.6;

	//the anticipated speed is the minimum between the leader on the 
	if (target_lane == LEFT
		&& this->left_leader != NULL)
	{//target lane is left and there's a vehicle on the left lane.
		ant_speed = MIN(((myVehicleDef*)this->left_leader)->getSpeed(), ant_speed);
	}

	if (target_lane == RIGHT
		&& this->right_leader != NULL)
	{//target lane is left and there's a vehicle on the left lane.
		ant_speed = MIN(((myVehicleDef*)this->right_leader)->getSpeed(), ant_speed);
	}

	double threshold = target_lane == LEFT ? 4.47 : 0; //favor left with 4.47m/s = 16km/hr = 10mph


	//*********Turn off this section 0720 FCC
	////does not favor left-lanes if this section does not need boost for left-lane distribution
	//if(!this->CurrentSectionSuitable4Boost())
	//	threshold = 0;


	ant_speed += threshold;
	double tempspeed = MAX(v, MIN_DLC_SPEED);
	if (this->getHOV() == false //this car is not hov car
		||
		this->isHOVActive() == false // or hov is not active
		||
		this->GetSectionHOVLane() == 0) //this section has no HOV lane
	{
		if (ant_speed < tempspeed)
		{//if the anticipated speed is slower than the tempspeed.
			return 0;
		}
		else
		{
			double desire = MIN(1, (ant_speed - tempspeed) / tempspeed);
			desire = MAX(0, desire);
			//if(target_lane == RIGHT)
			//	desire *= this->getRightDLCCoeff(); //multiply the desire with right LC discourage coefficient
			return desire;
		}
	}
	else //this is a hov car and hov is active and this section has hov lane
	{
		//the desire is biased to the hov lane 
		if (isLaneChangingPossible(LEFT) == false) //assuming hov is the leftmost lane 
			//if the car is already in the leftmost lane
			//do not give it right lane dlc desire
		{
			return 0;
		}
		else
		{
			if (ant_speed < tempspeed)
				return 0;
			else
			{
				double desire = MIN(1, (ant_speed - tempspeed) / tempspeed);
				desire = MAX(0, desire);
				//if(target_lane == RIGHT)
				//	desire *= this->getRightDLCCoeff();

				//make HOV vehicle 100% prefer the leftmost lane
				if (target_lane == RIGHT)
					desire = 0;
				else
					desire = 1;//if this is a HOV increase the desire to move the left lane

				return desire;
			}
		}
	}
}

double myVehicleDef::GetAdditionalDlcDesire(int target_lane)
{
	double vf = this->freeflowspeed;
	double d_scan = getDLCScanRange();    // scan maximum 50m ahead
	int n_scan = getDLCScanNoCars();    // scan maximum 5 vehicles ahead
	double v_target = isLaneChangingPossible(target_lane) ?
		getAverageSpeedAHead(target_lane, d_scan, n_scan) : 0;
	double v_ahead = getAverageSpeedAHead(0, d_scan, n_scan);


	return MAX(0, (v_target - v_ahead) / vf);
}

//////////////////////////////////////////////////////////////////////////
// determine if discretionary lane changing is necessary
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::NeedDlc()
{
	// if this vehicle just made a lane change to this lane,
	// we will not judge its DLC desire for a period of time.
	// this period of time is randomized

	if (this->adaptive_mode == CACCMODE) // No LC motivation for vehicles at CACC mode
		return false;

	if (AKIGetCurrentSimulationTime() - this->getLastLCTime() <= this->getMinTimeBtwLcs4DLC()
		&& this->getLastLCType() == OPTIONAL_LC)
	{
		return false;
	}

	// Do not perform DLC within 10m of a connector. Aimsun cannot return correct target leader and follower when 
	// the target leader or follower is in the connector_Hao 05/23/17
	if (this->getPosition(0) <= 10 || this->sec_inf.length - this->getPosition(0) <= 10)
	{
		return false;
	}



	double lc_prob = 0.0;
	double lc_prob_right = 0.0;
	bool result = false;

	//core equations to calculate DLCDesire
	lc_prob = DLCDesire(LEFT);

	int currentMode = this->getMode();


	int CACC_Lane_Flag = this->CACC_ML_Activated;
	int CACC_Lane_Number = this->number_CACC_ML;
	int CACC_Lane_Access_Flag = this->CACC_ML_RA;

	//CACC vehicles do not make DLC to the ML if the access restriction is on_Hao 5/3/17
	if (
		CACC_Lane_Flag == 1 // CACC managed lane is activated
		&& (this->getVehType() == NGSIM_CACC_ACC_V2V || (this->getVehType() == CAR_Type && this->ConnectFlag) || this->getVehType() == CACC_HOV) // CACC vehicle or VAD vehicle
		&& this->getDistanceToOffRamp() < 0 // The subject vehicle is not going to exit the freeway
		&& this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number // The subject vehicle is left to the managed lane
		&& this->getNumberOfLanesInCurrentSection() > 2 // The subject vehicle is in the freeway mainline
		&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection() // The subject vehicle is not in the acceleration lane
		&& !(currentMode == CACC_ON_Follower_Speed_Regulation
		|| currentMode == CACC_ON_ACC
		|| currentMode == CACC_ON
		|| currentMode == CACC_ON_Leader) // The subject vehicle is not in a CACC string
		)
	{
		if (CACC_Lane_Access_Flag == 1)
		{
			//restricted access
			int ML_type = this->GetCACCManagedLaneType(this->getIdCurrentSection());
			if (ML_type != 1 && ML_type != 3)
			{
				lc_prob = 0;
			}
		}

	}

	// Let non-HOV vehicle exit HOV lane
	if (this->getHOV() == false //this car is not a hov car
		&&
		GetSectionHOVLane() > 0 //this section contains hov lane
		&&
		isHOVActive() //now hov is active in terms of time
		)  // subject vehicle in HOV lane
	{
		if (this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection())
		{
			lc_prob_right = 1;
		}
		else if (this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection() - 1)
		{
			lc_prob = 0;
		}

	}
	else if (//Let CACC vehicles stay in the managed lane Hao 3/8/17
		CACC_Lane_Flag == 1 // CACC managed lane is activated
		&& (this->getVehType() == NGSIM_CACC_ACC_V2V || this->getVehType() == CACC_HOV)// || (this->getVehType() == CAR_Type && this->ConnectFlag)) // not CACC vehicle or VAD vehicle
		&& this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number + 1 // The subject vehicle is not in the rightmost managed lane
		&& this->getNumberOfLanesInCurrentSection() > 2 // The subject vehicle is in the freeway mainline
		)
	{
		lc_prob_right = DLCDesire(RIGHT) / 1.6;
	}
	else if (//Non-CACC vehicles will not enter the managed lane Hao 3/8/17
		CACC_Lane_Flag == 1 // CACC managed lane is activated
		&& (this->getVehType() == NGSIM_ACC || (this->getVehType() == CAR_Type && !this->ConnectFlag) || (this->getVehType() == HOVveh && !this->ConnectFlag) || this->getVehType() == ACC_HOV) // not CACC vehicle or VAD vehicle
		&& this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number // The subject vehicle is not in the managed lane
		&& this->getNumberOfLanesInCurrentSection() > 2 // The subject vehicle is in the freeway mainline
		)
	{
		lc_prob = 0;
		lc_prob_right = DLCDesire(RIGHT);
	}
	else if (GetRampType(this->getIdCurrentSection()) == ON_RAMP // On-ramp section
		&& this->getPosition(0) < 200 // Within the acceleration lane
		&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection()) // Mainstream vehicle
	{
		// Mainstream vehicles do make DLC to the right if on-ramp is congested
		lc_prob_right = 0;
	}
	else if (GetRampType(this->nextsec) == ON_RAMP // Next section has on-ramp
		&& this->sec_inf.length - this->getPosition(0) <= 200 // Within 200 meters from the on-ramp
		&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection()) // Mainstream vehicle
	{
		lc_prob_right = 0;
	}
	else
	{
		lc_prob_right = DLCDesire(RIGHT);
	}

	if (lc_prob_right == 0
		&& lc_prob == 0)
	{
		setTargetLane(NOCHANGE);
		return false;
	}
	else
	{
		if (lc_prob_right > lc_prob)
		{
			result = true;
			setnLC(-1);
			setTargetLane(RIGHT);   // set right lane as target lane
		}
		else
		{
			result = true;
			setnLC(1);
			setTargetLane(LEFT);   // set left lane as target lane
		}
	}

	if (result)
	{
		int curLane = getIdCurrentLane();

		//DLC is also not allowed 
		//when the vehicle is at the original section
		//with the same zone distance
		if (
			this->getIdCurrentSection() == this->getSourceSection()
			&& this->getPosition() < this->getDLCForbidZoneBeforeExit()
			)
		{
			// NO_LANE_CHANGE;
			setTargetLane(NOCHANGE);
			result = false;
			return result;
		}


		if ((this->getVehType() == NGSIM_CACC_ACC_V2V || this->getVehType() == CACC_HOV))
		{
			//setTargetLane(NOCHANGE);
			//return false; //Test, CACC vehicles do not make DLC

			this->setLCType(OPTIONAL_LC);
			int decision = DLCCFDecision();  //get CF options before lane changing
			if (decision != DLC_LANE_CHANGE_FEASIBLE)
			{
				// NO_LANE_CHANGE;
				setTargetLane(NOCHANGE);
				result = false;
				return result;
			}
		}

		this->setLaneChangeDesireOption(lc_prob, lc_prob_right);
	}

	return result;
}



//Thie function returns current distance to the next exit.
//If the subject vehicle is not going to exit the freeway, the function returns -1
double myVehicleDef::getDistanceToOffRamp()
{
	int tempsec = nextsec;
	double totallength = this->sec_inf.length - this->getPosition(0);
	while (tempsec > 0)
	{
		if (GetRampType(tempsec) == TRUE_OFF_RAMP)
		{
			return totallength;
		}
		else
		{
			totallength += AKIInfNetGetSectionANGInf(tempsec).length;
			tempsec = AKIVehInfPathGetNextSection(this->VehID, tempsec);
		}
	}
	return -1.0;
}

//check if lane change is needed for turning
// Looks like this function deals with conditions where more than 1 off-ramp lanes exist.
// Or the off-ramp lanes are on the left side of the freeway. 
// It has nothing to do with the turning movements at intersections_Hao Liu 11/10/16.
bool myVehicleDef::NeedLc4Turning()
{
	if (VehID == debug_track_id)
		VehID = debug_track_id;

	int currentMode = this->getMode();


	//No need to check Exit lane-changing when the subject vehicle is too far from the off-ramp
	if (NotWithinOfframpAwarenessDis())
	{
		this->setDis2NextTurning(this->getEarlyLaneKeepDis());
		this->setEqvDis2NextTurning(this->getEarlyLaneKeepDis());

		int CACC_Lane_Flag = this->CACC_ML_Activated;
		int CACC_Lane_Number = this->number_CACC_ML;
		int CACC_Lane_Access_Flag = this->CACC_ML_RA;

		if (this->getHOV() //this car is a hov car
			&&
			this->isHOVActive() // hov is active
			&&
			this->GetSectionHOVLane() > 0 // this section has a HOV lane
			&&
			this->getIdCurrentLane() <= this->getNumberOfLanesInCurrentSection()) // the vehicle is not in the HOV lane
		{
			// HOV vehicle has a chance to perform mandatory LC to the HOV lane
			if (this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection() - 1)
			{
				if (this->left_avg_speed_ahead > this->avg_speed_ahead)
				{
					setMandatoryType(EXIT);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
			}
			else if (this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection())
			{
				// HOV vehicles have chance to exit the HOV lane if it gets too congested
				if (this->right_avg_speed_ahead > this->avg_speed_ahead)
				{
					setMandatoryType(EXIT);
					setnLC(1);
					setLaneChangeDesireForce(0, AKIGetRandomNumber());
					return true;
				}
			}
			else
			{
				setMandatoryType(EXIT);
				setnLC(1);
				setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
				return true;
			}
		}
		else if (//Let CACC or VAD vehicle make LC towards the managed lane Hao 3/8/17
			CACC_Lane_Flag == 1 // CACC managed lane is activated
			&& (this->getVehType() == NGSIM_CACC_ACC_V2V || this->getVehType() == CACC_HOV)// || (this->getVehType() == CAR_Type && this->ConnectFlag)) // CACC vehicle or VAD vehicle
			&& this->getDistanceToOffRamp() < 0 // The subject vehicle is not going to exit the freeway
			&& this->getIdCurrentLane() <= this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number // The subject vehicle is not in the managed lane
			&& this->getNumberOfLanesInCurrentSection() > 2 // The subject vehicle is in the freeway mainline
			&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection() // The subject vehicle is not in the acceleration lane
			&& !(currentMode == CACC_ON_Follower_Speed_Regulation
			|| currentMode == CACC_ON_ACC
			|| currentMode == CACC_ON
			|| currentMode == CACC_ON_Leader) // The subject vehicle is not in a CACC string
			)
		{
			double d_scan = this->getDLCScanRange();
			double n_scan = this->getDLCScanNoCars();
			double avg_speed_managed_lane = 0;
			for (int lane = this->getNumberOfLanesInCurrentSection(); lane > this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number; lane--)
			{
				// In case there are more than one managed lanes
				double temp_speed = getAverageLaneSpeedAHead(lane, d_scan, n_scan);
				if (temp_speed > avg_speed_managed_lane)
				{
					avg_speed_managed_lane = temp_speed;
				}
			}

			// The subject vehicle will make LC towards the managed lane if the average speed in that lane is larger than its current speed
			if (this->getIdCurrentLane() == this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number)
			{
				// This block of code is left for the future modeling of continous access vs. restricted access Hao 3/8/17
				if (avg_speed_managed_lane > this->avg_speed_ahead)
				{
					if (CACC_Lane_Access_Flag == 1)
					{
						//restricted access
						int ML_type = this->GetCACCManagedLaneType(this->getIdCurrentSection());
						if (ML_type == 1 || ML_type == 3)
						{
							//Continous access
							setMandatoryType(EXIT);
							setnLC(1);
							setLaneChangeDesireForce(DLCDesire(LEFT), 0);//(AKIGetRandomNumber()*0.1512, 0); //90% chance in 30s
							return true;
						}
					}
					else
					{
						setMandatoryType(EXIT);
						setnLC(1);
						setLaneChangeDesireForce(DLCDesire(LEFT), 0);//(AKIGetRandomNumber()*0.1512, 0); //90% chance in 30s
						return true;
					}

				}
			}
			else
			{
				if (avg_speed_managed_lane > this->avg_speed_ahead)
				{
					setMandatoryType(EXIT);
					setnLC(1);
					setLaneChangeDesireForce(DLCDesire(LEFT), 0);//(AKIGetRandomNumber()*0.1512, 0);
					return true;
				}
			}
		}
		else if (//Let non-CACC or non-VAD vehicles exit the managed lane Hao 3/9/17
			CACC_Lane_Flag == 1 // CACC managed lane is activated
			&& (this->getVehType() == NGSIM_ACC || (this->getVehType() == CAR_Type && !this->ConnectFlag) || (this->getVehType() == HOVveh && !this->ConnectFlag) || this->getVehType() == ACC_HOV) // CACC vehicle or VAD vehicle
			&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number // The subject vehicle is in the managed lane
			&& this->getNumberOfLanesInCurrentSection() > 2 // The subject vehicle is in the freeway mainline
			)
		{
			setMandatoryType(EXIT);
			setnLC(1);
			setLaneChangeDesireForce(0, AKIGetRandomNumber()*0.1524);
			return true;
		}
		else if (GetRampType(this->getIdCurrentSection()) == ON_RAMP // On-ramp section
			&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection() // Mainstream vehicle
			&& this->getIdCurrentLane() < this->getNumberOfLanesInCurrentSection()  // Not in the leftmost lane
			&& this->left_avg_speed_ahead > this->avg_speed_ahead // Left lane faster
			&& this->getAverageLaneSpeedAHead(1, 200, 2) < 15 // acceleration lane congested
			&& this->getAverageLaneSpeedAHead(1, 200, 2) > 0 // Within the length of the acceleration lane
			&& (!(currentMode == CACC_ON_Follower_Speed_Regulation
			|| currentMode == CACC_ON_ACC
			|| currentMode == CACC_ON
			|| currentMode == CACC_ON_Leader))) // Not in the CACC string
		{//<=========proactive lane change
			// Mainstream vehicles make gaps for on-ramp traffic
			if (CACC_Lane_Flag == 1
				&& (this->getVehType() == NGSIM_ACC || (this->getVehType() == CAR_Type && !this->ConnectFlag) || this->getVehType() == ACC_HOV)
				&& this->getIdCurrentLane() + 1 <= this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number
				)
			{
				if (this->getHOV() //this car is a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->getHOV() //this car is not a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					&&
					this->getIdCurrentLane() + 1 < this->getNumberOfLanesInCurrentSection()
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->isHOVActive())
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
			}
			else if (CACC_Lane_Flag == 1 // CACC managed lane is activated
				&& (this->getVehType() == NGSIM_CACC_ACC_V2V || (this->getVehType() == CAR_Type && this->ConnectFlag) || this->getVehType() == CACC_HOV)
				)
			{
				if (this->getHOV() //this car is a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->getHOV() //this car is not a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					&&
					this->getIdCurrentLane() + 1 < this->getNumberOfLanesInCurrentSection()
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->isHOVActive())
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
			}
			else if (CACC_Lane_Flag != 1)
			{
				if (this->getHOV() //this car is a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->getHOV() //this car is not a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					&&
					this->getIdCurrentLane() + 1 < this->getNumberOfLanesInCurrentSection()
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->isHOVActive())
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}

			}

		}
		else if (GetRampType(this->nextsec) == ON_RAMP // Next section has on-ramp
			&& (this->sec_inf.length - this->getPosition(0) <= 100) // Within 100 meters from the on-ramp
			&&
			(this->getIdCurrentLane() == 1 + this->getNumberOfLanesInCurrentSection() - this->getNumberOfMainLanesInCurrentSection()) // Mainstream vehicle
			&& this->getIdCurrentLane() < this->getNumberOfLanesInCurrentSection() // Not in the leftmost lane
			&& this->left_avg_speed_ahead > this->avg_speed_ahead + 5 // Left lane faster
			&& (!(currentMode == CACC_ON_Follower_Speed_Regulation
			|| currentMode == CACC_ON_ACC
			|| currentMode == CACC_ON
			|| currentMode == CACC_ON_Leader))) // Not in the CACC string
		{//=============>active lane changing
			// Mainstream vehicles make gaps for on-ramp traffic
			if (CACC_Lane_Flag == 1
				&& (this->getVehType() == NGSIM_ACC || (this->getVehType() == CAR_Type && !this->ConnectFlag) || this->getVehType() == ACC_HOV)
				&& this->getIdCurrentLane() + 1 <= this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number
				)
			{
				if (this->getHOV() //this car is a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->getHOV() //this car is not a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					&&
					this->getIdCurrentLane() + 1 < this->getNumberOfLanesInCurrentSection()
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->isHOVActive())
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
			}
			else if (CACC_Lane_Flag == 1 // CACC managed lane is activated
				&& (this->getVehType() == NGSIM_CACC_ACC_V2V || (this->getVehType() == CAR_Type && this->ConnectFlag) || this->getVehType() == CACC_HOV)
				)
			{
				if (this->getHOV() //this car is a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->getHOV() //this car is not a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					&&
					this->getIdCurrentLane() + 1 < this->getNumberOfLanesInCurrentSection()
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->isHOVActive())
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
			}
			else if (CACC_Lane_Flag != 1)
			{
				if (this->getHOV() //this car is a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->getHOV() //this car is not a hov car
					&&
					this->isHOVActive() // hov is active
					&&
					this->GetSectionHOVLane() > 0 // this section has a HOV lane
					&&
					this->getIdCurrentLane() + 1 < this->getNumberOfLanesInCurrentSection()
					)
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}
				else if (!this->isHOVActive())
				{
					setMandatoryType(TURNING);
					setnLC(1);
					setLaneChangeDesireForce(AKIGetRandomNumber(), 0);
					return true;
				}

			}
		}
		else
		{
			return false;
		}
	}

	bool result = false;
	int targetlane = 0;
	//static double a = 2.0F;
	int n_lc = 0;   // number of lane changes required to have turning
	int lane = 0;
	//int conflictLane = 0;
	//int index = 2;
	double d_exit = 0.0;
	//double req_exit = 0.0;

	lane = getIdCurrentLane();
	//conflictLane = GetConflictLane(this);

	int rightmostlane_feasible = -1;
	int rightmostlane_feasible_lowend = -1;
	if (!this->NotWithinOfframpAwarenessDis()
		&&
		this->exit_section_before_offramp > 0)
		//if the section the second last section for off-ramp turning;  then this vehicle also tries to 
		//change to the rightmost lane (given the condition that this lane 
		//will not stop this vehicle from turning to the next section)
	{
		int numOffRamps = GetSectionOfframpLanes(this->offRampID);
		if (numOffRamps <= 1)
		{
			rightmostlane_feasible = exit_lane_from;
			rightmostlane_feasible_lowend = exit_lane_from;
		}
		else
		{
			//int numsideLanes = MAX(GetSectionOfframpLanes(this->current_sec_id), sec_inf.nbSideLanes);
			rightmostlane_feasible = MIN(exit_lane_to, MAX(exit_lane_from, numOffRamps + sec_inf.nbSideLanes));
			rightmostlane_feasible_lowend = exit_lane_from;
		}
	}

	double eqv_d_exit = 0;
	// check if left lane changing is needed
	if (exit_lane_from > 0 && lane < exit_lane_from)
	{
		d_exit = this->distance_to_off_ramp - this->getPosition(0);
		n_lc = exit_lane_from - lane;
		if (this->GetRampType(this->getIdCurrentSection()) == OFF_RAMP)
		{
			// For the freeway segment with the off-ramp lane, the number of lane changes does not count the last LC from the rightmost lane to the off-ramp lane
			n_lc = n_lc - 1;
		}
		eqv_d_exit = d_exit - (n_lc * 40 + n_lc * 2 * this->getSpeed());
		//use the same criterion as the ramp lane change for this one
		double lc_desire = CalculateDesireForce(n_lc, eqv_d_exit, this->getSpeed(), false);
		setLaneChangeDesireForce(lc_desire, 0);
		result = true;
		targetlane = LEFT;
	}
	// check  if right lane changing is needed
	else if (exit_lane_to > 0 && lane > exit_lane_to)
	{
		d_exit = this->distance_to_off_ramp - this->getPosition(0);
		n_lc = lane - exit_lane_to;
		if (this->GetRampType(this->getIdCurrentSection()) == OFF_RAMP)
		{
			// For the freeway segment with the off-ramp lane, the number of lane changes does not count the last LC from the rightmost lane to the off-ramp lane
			n_lc = n_lc - 1;
		}
		eqv_d_exit = d_exit - (n_lc * 40 + n_lc * 2 * this->getSpeed());
		// same model but different parameter set
		double lc_desire = CalculateDesireForce(n_lc, eqv_d_exit, this->getSpeed(), false);
		setLaneChangeDesireForce(0, lc_desire);
		result = true;
		targetlane = RIGHT;
	}
	else if (lane > rightmostlane_feasible
		&& rightmostlane_feasible >= 1)
	{
		d_exit = this->distance_to_off_ramp - this->getPosition(0);

		n_lc = lane - rightmostlane_feasible;
		if (this->GetRampType(this->getIdCurrentSection()) == OFF_RAMP)
		{
			// For the freeway segment with the off-ramp lane, the number of lane changes does not count the last LC from the rightmost lane to the off-ramp lane
			n_lc = n_lc - 1;
		}
		eqv_d_exit = d_exit - (n_lc * 40 + n_lc * 2 * this->getSpeed());
		// same model but different parameter set
		double lc_desire = CalculateDesireForce(n_lc, eqv_d_exit, this->getSpeed(), false);
		setLaneChangeDesireForce(0, lc_desire);
		result = true;
		targetlane = RIGHT;
	}
	else
	{
		if (NotWithinOfframpAwarenessDis() == false) //within a certain distance to off-ramps 
			//even has optional desire
			//set to zero
		{
			//reset any possible DLCs that are beyond the feasible lanes connecting to the potential offramp
			if (rightmostlane_feasible > 0)
			{
				if (lane - LEFT > rightmostlane_feasible)
					this->desireLC_option_left = 0;
				if (lane - RIGHT < rightmostlane_feasible_lowend)
					this->desireLC_option_right = 0;
			}
			else
			{
				if (lane == this->exit_lane_from)
				{
					desireLC_option_right = 0;
				}
				if (lane == this->exit_lane_to)
				{
					desireLC_option_left = 0;
				}
			}
		}
		// no lane changing is needed
		result = false;
	}


	if (result)
	{
		if (this->distance_to_off_ramp > 0 && d_exit < 0.1)
		{
			// Abort LC when the subject vehicle is too close to the end of the off-ramp_Hao 11/29/16
			this->setDis2NextTurning(d_exit);
			this->setEqvDis2NextTurning(d_exit - (n_lc * 40 + n_lc * 2 * this->getSpeed()));
			setnLC(0);
			setMandatoryType(0);
			setLCType(0);
			ResetDesires();
			return false;
		}

		if (rightmostlane_feasible >= 1)
			EliminateDlcDesireOutSideRouteLanes(rightmostlane_feasible_lowend, rightmostlane_feasible);
		else
			EliminateDlcDesireOutSideRouteLanes(exit_lane_from, exit_lane_to);

		//int type = this->getLCType();
		if (this->getEqvDis2NextTurning() > this->getE())
		{
			// Stay in the CACC mode when there is no gap
			if (this->getVehType() == NGSIM_CACC_ACC_V2V || this->getVehType() == CACC_HOV)
			{
				this->setLCType(EXIT);
				int decision = DLCCFDecision();  //get CF options before lane changing
				if (decision != EXIT_CHANGE_FEASIBLE)
				{
					// NO_LANE_CHANGE;
					setnLC(0);
					setMandatoryType(0);
					setLCType(0);
					ResetDesires();
					return false;
				}
				else
				{
					setLCType(EXIT); //Assume HOV-type LC when far from off-ramp
					setMandatoryType(EXIT);
				}
			}
			else
			{
				setLCType(EXIT); //Assume HOV-type LC when far from off-ramp
				setMandatoryType(EXIT);
			}

		}
		else
		{
			setLCType(TURNING); //Assume mandatory LC when close to off-ramp
			setMandatoryType(TURNING);
		}
		setExtraDesire4FeasibleGapOfframp(targetlane);
		//setLCType(type);
		setnLC(n_lc);


		//Set distance to the exit
		this->setDis2NextTurning(d_exit);
		this->setEqvDis2NextTurning(d_exit - (n_lc * 40 + n_lc * 2 * this->getSpeed()));


		int CACC_Lane_Flag = this->CACC_ML_Activated;
		int CACC_Lane_Number = this->number_CACC_ML;
		int CACC_Lane_Access_Flag = this->CACC_ML_RA;

		if (CACC_Lane_Flag == 1
			&& CACC_Lane_Access_Flag == 1
			&& this->getIdCurrentLane() > this->getNumberOfLanesInCurrentSection() - CACC_Lane_Number
			)
		{
			int ML_type = this->GetCACCManagedLaneType(this->getIdCurrentSection());
			if (ML_type == 2 || ML_type == 3)
			{
				// Within the egress section
				setMandatoryType(EXIT);
				setnLC(1);
				setLaneChangeDesireForce(0, this->getLaneChangeDesireThrd() * 2);//(AKIGetRandomNumber()*0.1512, 0); //90% chance in 30s
				return true;
			}
			else
			{
				setnLC(0);
				setMandatoryType(0);
				setLCType(0);
				ResetDesires();
				return false;
			}
		}
	}
	else
	{
		d_exit = this->distance_to_off_ramp - this->getPosition(0);
		this->setDis2NextTurning(d_exit);
		this->setEqvDis2NextTurning(d_exit - (n_lc * 40 + n_lc * 2 * this->getSpeed()));
	}
	return result;
}




//see if the other vehicles need cooperation
bool myVehicleDef::NeedCoop()
{
	// disable truck's cooperative driving behavior
	if (this->getVehType() == Truck_Type
		|| this->adaptive_mode == ACCMODE
		|| this->adaptive_mode == CACCMODE)
		return false;

	this->CoopRequester = NULL;

	myVehicleDef* frontVeh = (myVehicleDef*)this->leader;

	//if both lanes have cooperation requesters
	//vehicle on the right lane has priority
	const A2SimVehicle *vehDown = this->left_leader;
	int state = 0;
	if (vehDown != NULL && vehDown->isUpdated())
	{
		state = 1;
	}
	if (vehDown != NULL //has a vehicle on the target lane 
		&& ((myVehicleDef*)vehDown)->getPositionReferenceVeh(state, this, 0) > 5
		&& ((myVehicleDef*)vehDown)->getMode() == BCF //the vehicle has a desire for LC
		&& ((myVehicleDef*)vehDown)->getTargetLane() == RIGHT //indicate this is car who wants to lc to the right
		&& ((myVehicleDef*)vehDown)->getLCType() != OPTIONAL_LC // does not cooperate to DLC
		&& ((myVehicleDef*)vehDown)->desireLC_force_right >= 0.8// vehicles in the rightmost lane yield more often
		) // only cooperate to mandatory merging;  dont coopeate to any other type
	{
		if (this->Willing2Coop(((myVehicleDef*)vehDown)))
		{
			this->CoopRequester = (myVehicleDef*)vehDown;
		}
	}

	vehDown = this->right_leader;
	state = 0;
	if (vehDown != NULL && vehDown->isUpdated())
	{
		state = 1;
	}
	if (vehDown != NULL
		&&
		/*((((myVehicleDef*)vehDown)->getPositionReferenceVeh(state, this, 0) > 10 && vehDown->getSpeed(0) >= 9)
		|| vehDown->getSpeed(0) < 9)*/
		((myVehicleDef*)vehDown)->getPositionReferenceVeh(state, this, 0) > 5
		&& ((myVehicleDef*)vehDown)->getMode() == BCF
		&& ((myVehicleDef*)vehDown)->getTargetLane() == LEFT
		&& ((myVehicleDef*)vehDown)->getLCType() != OPTIONAL_LC // yield to on-ramp flow
		&& (
		vehDown->getDistance2Obstacle() < 60
		|| ((myVehicleDef*)vehDown)->desireLC_force_left >= 0.8
		) // only yield to vehicles that close to the end of acceleration lane
		)
	{
		if (this->Willing2Coop((myVehicleDef*)vehDown))
		{
			this->CoopRequester = (myVehicleDef*)vehDown;
		}
	}

	// If the CoopRequester has not completed the LC yet, continue to yield
	if (
		this->CoopRequester == NULL
		&& this->LastCoopRequester != NULL
		&& this->LastCoopRequester->getMode() == BCF
		&& this->LastCoopRequester->getLCType() != OPTIONAL_LC
		&& ((myVehicleDef*)this->LastCoopRequester)->getPositionReferenceVeh(0, this, 0) <= 10
		&& ((myVehicleDef*)this->LastCoopRequester)->getPositionReferenceVeh(0, this, 0) > 5
		)
	{
		if (this->Willing2Coop(this->LastCoopRequester))
		{
			this->CoopRequester = this->LastCoopRequester;
		}
	}

	if (this->CoopRequester != NULL)
	{
		if (this->CoopRequester == LastCoopRequester
			&&
			this->CoopRequester->getTargetLane() == LEFT
			&&
			AKIGetCurrentSimulationTime() - getLastCoopTime() > MAX_COOP_TIME)
			// If the subject vehicle has been trying to yield to the same requester for longer than MAX_COOP_TIME,
			// and its speed is less than the threthold, it will give up cooperation. 
		{
			//this->CoopRequester = NULL;
			//this->LastCoopRequester = NULL;
			return false;
		}
		else if (this->CoopRequester == LastCoopRequester
			&&
			this->CoopRequester->getTargetLane() == RIGHT
			&&
			AKIGetCurrentSimulationTime() - getLastCoopTime() > MAX_COOP_TIME)
			// The yield time for off-ramp lane changer is smaller
		{
			//this->CoopRequester = NULL;
			//this->LastCoopRequester = NULL;
			return false;
		}
		else
		{
			if (this->CoopRequester != LastCoopRequester)
			{
				//if (getLastCoopTime() > 0 && AKIGetCurrentSimulationTime() - getLastCoopTime() < MAX_COOP_TIME * 2)
				//{
				//	//The subject vehicle won't yield to the second vehicle after finishing a cooperation
				//	return false;
				//}
				LastCoopRequester = this->CoopRequester;
				this->setLastCoopTime(AKIGetCurrentSimulationTime());
			}
			return true;
		}
	}
	else
	{
		this->CoopRequester = NULL;
		this->LastCoopRequester = NULL;
		return false;
	}
}

// wiling2Coop defines if the driver is willing to cooperate given the 
// status of the coop-requester
bool myVehicleDef::Willing2Coop(myVehicleDef *coop_veh)
{
	if ((coop_veh->getLCType() != OPTIONAL_LC
		&& this->getPoliteness() > this->getRandomPoliteness())
		||
		(coop_veh->getLCType() == OPTIONAL_LC
		&& this->getPolitenessOptional() > this->getRandomPolitenessOptional()))
	{
		if (!this->IsCoopEffectMuch(coop_veh))
			return true;
	}
	return false;
}

// this function made a decision as to slow down of sync to get lane change 
// when exiting
int myVehicleDef::ExitCfDecision()
{
	return GapAcceptDecision_Sync_First();  // It seems inconsistent with the document describing the simulation models!
}

void myVehicleDef::setLaneChangeDesire(double incentive)
{
	this->desireLC = bound(incentive, 1, 0);
}

//set the desire of the lane change on ramp
void myVehicleDef::setLaneChangeDesireForce(double incentive_left,
	double incentive_right)
{
	this->desireLC_force_left = incentive_left >= 0 ? incentive_left : 0;
	this->desireLC_force_right = incentive_right >= 0 ? incentive_right : 0;
}

//set the desire of the optional lane-change
void myVehicleDef::setLaneChangeDesireOption(double incentive_left,
	double incentive_right)
{
	desireLC_option_left = bound(incentive_left, 1.0, 0.0);
	desireLC_option_right = bound(incentive_right, 1.0, 0.0);
}

//combine all desires of lane change and determine the final decision
bool myVehicleDef::CombineLCDesires()
{
	if (isCurrentLaneInNode() == true)
	{
		// No_Lane_Change;
		setLCType(0);
		setMandatoryType(NOCHANGE);
		setTargetLane(NOCHANGE);
		this->setLaneChangeDesire(0);
		setnLC(0);
		return false;
	}

	//four options: force left/right, optional left/right
	if (desireLC_force_left +
		desireLC_force_right +
		desireLC_option_left +
		desireLC_option_right <= 0.0001)
	{
		// No_Lane_Change;
		setLCType(0);
		setMandatoryType(NOCHANGE);
		setTargetLane(NOCHANGE);
		this->setLaneChangeDesire(0);
		setnLC(0);
		return false;
	}
	else
	{
		double left_desire
			= desireLC_force_left +
			this->discretionary_LC_weight*desireLC_option_left;
		double right_desire
			= desireLC_force_right +
			this->discretionary_LC_weight*desireLC_option_right;

		//prioritize the mandatory lane change
		if (desireLC_force_left > 0)
		{
			right_desire = 0;
			if (desireLC_option_left == 0)
			{
				desireLC_option_left = this->DLCDesire(LEFT);
				left_desire = desireLC_force_left +
					desireLC_option_left*discretionary_LC_weight;
			}
		}
		else if (desireLC_force_right > 0)
		{
			left_desire = 0;
			if (desireLC_option_right == 0)
			{
				desireLC_option_right = this->DLCDesire(RIGHT);
				//GetAdditionalDlcDesire(RIGHT); 
				right_desire = desireLC_force_right +
					desireLC_option_right*discretionary_LC_weight;
			}
		}

		double desire = left_desire > right_desire ? left_desire : right_desire; //(std::max)(left_desire,right_desire); 
		int target_lane =
			left_desire > right_desire ? LEFT : RIGHT;

		//if lane change is impossible return 0
		if (this->isLaneChangingPossible(target_lane == LEFT ? LEFT : RIGHT) == false)
		{
			// No_Lane_Change;
			setLCType(0);
			setMandatoryType(NOCHANGE);
			setTargetLane(NOCHANGE);
			this->setLaneChangeDesire(0);
			setnLC(0);
			return false;
		}


		double desireThreshold = 1;
		int currentMode = this->getMode();
		if ((currentMode == CACC_ON_Follower_Speed_Regulation
			|| currentMode == CACC_ON_ACC
			|| currentMode == CACC_ON
			|| currentMode == CACC_ON_Leader)
			&& desireLC_force_left == 0
			&& desireLC_force_right == 0
			)
		{//If the vehicle is a CACC vehicle and it is in CACC control mode, then it uses lane change CACC lane change desire threshol other than normal driver desire threshold.
			//int exp_id = ANGConnGetExperimentId();
			//const unsigned short *CACC_LC_Desire_Str = AKIConvertFromAsciiString("GKExperiment::CACC_LC_Desire_Threshold");
			//double LC_Thre = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(CACC_LC_Desire_Str), exp_id);
			//AKIDeleteUNICODEString(CACC_LC_Desire_Str);

			double LC_Thre = this->CACC_DLC_Thre;
			desireThreshold = MAX(this->getLaneChangeDesireThrd(), LC_Thre);
		}
		else
		{//Use normal driver desire threshold.
			desireThreshold = this->getLaneChangeDesireThrd();
		}


		if (desire > desireThreshold)//if this desire if larger than the threshold desire
		{
			int type = MANDATORY;
			if (left_desire > right_desire)
				type = desireLC_force_left > 0 ? this->getMandatoryType() : OPTIONAL_LC;
			else
				type = desireLC_force_right > 0 ? this->getMandatoryType() : OPTIONAL_LC;

			//desire is less than meaning it is not that urgent
			// then the time between lc changes must be satisfied
			double punish_time_factor_4_swaping = 3.0;
			if (desire < 0.9
				&&
				(AKIGetCurrentSimulationTime() - this->getLastLCTime() <= this->getMinTimeBtwLcs()
				&& (this->getLastLCTarget() == target_lane || this->getLastLCTarget() == NOCHANGE))
				||
				(AKIGetCurrentSimulationTime() - this->getLastLCTime() <=
				punish_time_factor_4_swaping*this->getMinTimeBtwLcs()
				&& this->getLastLCTarget() == -target_lane) //lane change again to the original lane is not encouraged
				)
			{
				// No_Lane_Change;
				setLCType(0);
				setMandatoryType(NOCHANGE);
				setTargetLane(NOCHANGE);
				this->setLaneChangeDesire(0);
				setnLC(0);
				return false;
			}

			setLCType(type);
			setMandatoryType(type);
			setTargetLane(target_lane);
			this->setLaneChangeDesire(desire);

			//no need to set number lane changes here
			//it has already been set in NeedLc
			return true;
		}
		else
		{
			// No_Lane_Change;
			setLCType(0);
			setMandatoryType(NOCHANGE);
			setTargetLane(NOCHANGE);
			this->setLaneChangeDesire(0);
			setnLC(0);
			return false;
		}
	}
}

double myVehicleDef::getLaneChangeDesireThrd()
{
	// Make the LC desire decrease gradually to the user specified value downstream from the source section
	// The section next to the source section should be more than 1000 meters.
	// The graduate transition only applies to the freeway mainline.
	if (this->getIdCurrentSection() == LC_Threshold_Transition_Section)
	{
		double currentPos = this->getPosition(0);
		if (currentPos <= 1000)
		{
			double threshold = 1;
			double deltaThre = (1 - this->lane_change_prob)*(1000 - currentPos) / 1000;
			threshold = this->lane_change_prob + deltaThre;
			return threshold;
		}
		else
		{
			return this->lane_change_prob;
		}

	}
	else
	{
		return this->lane_change_prob;
	}
}

void myVehicleDef::ResetDesires()
{
	this->desireLC_force_left = 0.0;
	this->desireLC_force_right = 0.0;
	this->desireLC_option_left = 0.0;
	this->desireLC_option_right = 0.0;

}

//////////////////////////////////////////////////////////////////////////
//calculate the desire of mandatory lane change based on the distance left
//the speed, and the number lanes to be crossed
//two set of methods for ramp and off-ramp/turning
//The equivalent distance to the end of on/off ramp should be provided (distance adjusted by the number of lane changes needed_Hao 03/15/17)
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::CalculateDesireForce(int n_lc, double d_exit,
	double speed, bool is_for_on_ramp)
{
	//determine the urgency
	double dis2End = d_exit;
	double time2End = d_exit / speed;
	double para1 = 0;
	double para2 = 0;
	if (is_for_on_ramp)
		DesireEquation(para1, para2, dis2End, time2End, n_lc,
		this->getMinE4OnRamp(), this->getMinT4OnRamp(), this->getE4OnRamp(), this->getT4OnRamp());  // para1 and para2 are references
	else
		DesireEquation(para1, para2, dis2End, time2End, n_lc,
		this->getMinE4OffRamp(), this->getMinT4OffRamp(), this->getE4OffRamp(), this->getT4OffRamp());  // para1 and para2 are references

	return MAX(para2, para1);

	//return Bound_Function(MAX(1-para1, 1-para2)); 
	//return bound(MAX(1-para1, 1-para2), 1,0); 
}

//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::SetInitialVal()
{
	this->setLastLCTime(0);
	this->_smooth_transit_time = 1;

	int veh_Id = this->getId();
	//AKIVehSetAsTracked(veh_Id);
	//this->staticinfo = AKIVehTrackedGetStaticInf(veh_Id);
	//AKIVehSetAsNoTracked(veh_Id);

	this->acc_2_manual = false;
	//setNameByType();

}

// this function get the position of the fictitious vehicle
// using the headway of the ref vehicle 
double myVehicleDef::GetPositionRelativeFake(myVehicleDef* fake_veh,
	double reaction_time_fake,
	bool downstream)
{
	// if the fictitious vehicle is downstream
	if (downstream == true)
	{
		double past_headway = fake_veh->getPastPositionReferenceVehs
			(0, this, reaction_time_fake);
		return past_headway + this->getPosition();
	}
	else //the fake vehicle is upstream and this vehicle is the leader
	{
		double past_headway = getPastPositionReferenceVehs
			(reaction_time_fake, fake_veh, 0);
		return this->getPosition() - past_headway;
	}
}

// this function get the relative distance before the specified reaction times
// taking the reference vehicle's position is zero
// that means the reference vehicle must be downstream
double myVehicleDef::getPastPositionReferenceVehs
(double reaction_time_ref, myVehicleDef* ref_veh,
double reaction_time_this)
{
	int step_ref = (int)(reaction_time_ref / this->delta_t);
	if (ref_veh->isUpdated() == true)
	{
		step_ref
			= (int)(reaction_time_ref / this->delta_t) + 1;
	}

	int step_this = (int)(reaction_time_this / this->delta_t);
	if (isUpdated() == true)
	{
		step_this
			= (int)(reaction_time_this / this->delta_t) + 1;
	}

	return A2SimVehicle::getPositionReferenceVeh(step_this,
		ref_veh, step_ref);
}



//Tell if the distance is too close so cooperation does not make much sense
bool myVehicleDef::IsCoopEffectMuch(myVehicleDef *coop_veh)
{
	double dis
		= coop_veh->getPositionReferenceVeh(this) - coop_veh->getLength();  //current spacing
	/*return false;*/
	if (dis <= 0)
		return true;  //already parallel to the vehicle so does not make sense
	else
	{
		return false;
	}
}

//////////////////////////////////////////////////////////////////////////
// Decide what cf rules should be applied before after a DLC desire is 
// generated. 
//////////////////////////////////////////////////////////////////////////
int myVehicleDef::DLCCFDecision()
{
	return GapAcceptDecision_Sync_First();
}

void myVehicleDef::setLaneChangeDesireThrd(double val)
{
	this->lane_change_prob = bound(val, 1, 0);
}

void myVehicleDef::setRightDLCCoeff(double val)
{
	this->right_dlc_coeff_ = bound(val, 1, 0);
}

void myVehicleDef::setReactionTime(double val)
{
	this->reaction_time_ = MAX(0.01, val);
}

//////////////////////////////////////////////////////////////////////////
// Gipps model based gap acceptance function
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::DisGapAccepted(double a_L, double a_U, double tau,
	double headway, double jamGap,
	double d_leader,
	double l_leader, double vf, double v,
	double x,
	double x_leader,
	double x_leader_steps_early,
	double lead_v, double min_headway,
	double Gap_AC_Thrd,
	double desire,
	bool on_ramp,
	bool forward,
	double acc_self)
{
	double theta = tau*this->getGippsTheta();
	double b_estimate = a_L*this->getEstimateLeaderDecCoeff();
	return GippsGap(a_L, tau, theta, x_leader,
		x, jamGap, l_leader, v, lead_v, b_estimate, on_ramp, forward, acc_self);
}

double myVehicleDef::AnticipatedAcc(double a_L, double a_U, double tau,
	double headway, double jamGap,
	double d_leader,
	double l_leader, double vf, double v,
	double x,
	double x_leader,
	double x_leader_steps_early,
	double lead_v, double min_headway,
	double Gap_AC_Thrd,
	double desire)
{
	double new_pos = 0;
	double pos = BaseCfModel(
		a_L,
		a_U,
		tau,
		headway,
		jamGap,
		d_leader,
		l_leader,
		vf,
		v,
		x,
		x_leader,
		x_leader_steps_early,
		lead_v,
		min_headway,
		new_pos);  //assuming the downstream vehicle has the same reaction time

	double new_v = ((pos - x) * 2 / delta_t) - v;
	double acc = (new_v - v) / delta_t;

	return acc;
}

bool myVehicleDef::AccGapAccepted(double a_L, double a_U, double tau,
	double headway, double jamGap,
	double d_leader,
	double l_leader, double vf, double v,
	double x,
	double x_leader,
	double x_leader_steps_early,
	double lead_v, double min_headway,
	double Gap_AC_Thrd,
	double desire)
{
	double new_pos = 0;
	BaseCfModel(
		a_L,
		a_U,
		tau,
		headway,
		jamGap,
		d_leader,
		l_leader,
		vf,
		v,
		x,
		x_leader,
		x_leader_steps_early,
		lead_v,
		min_headway,
		new_pos);  //assuming the downstream vehicle has the same reaction time

	double new_v = ((new_pos - x) * 2 / delta_t) - v;
	double acc = (new_v - v) / delta_t;

	if (acc < Gap_AC_Thrd*desire)
	{
		return false;  //downstream gap is not satisfied
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////
/////Ciuffo, Biagio, Vincenzo Punzo, 
//and Marcello Montanino. 
//"Thirty Years of Gipps' Car-Following Model: 
//Applications, Developments, and New Features." 
//Transportation Research Record: 
//Journal of the Transportation Research Board 
//2315 (2012): 89-99. (Eq. 3)
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::GippsDecelerationTerm
(double maxDec, double reaction_time, double theta,
double x_leader, double x, double jamGap, double l_leader,
double v, double lead_v, double b_estimate)
{
	double value_in_sqrt =
		pow(maxDec*(reaction_time / 2 + theta), 2) -
		maxDec*(2 * (x_leader - x - jamGap - l_leader) -
		v*reaction_time - lead_v*lead_v / b_estimate);

	double v_after_tau = 0;
	if (value_in_sqrt > 0)
	{
		v_after_tau = maxDec
			*(reaction_time / 2 + theta) +
			sqrt(value_in_sqrt);
	}
	else
	{
		v_after_tau = v_after_tau;
	}
	return v_after_tau;
}

//////////////////////////////////////////////////////////////////////////
//gipps safety gaps
//Gipps, Peter G. "A behavioural car-following model for 
//computer simulation." 
//Transportation Research Part B: Methodological 15, no. 2 (1981): 105-111.
//Eq. and (5)
//without reduction factors
bool myVehicleDef::GippsGap(double maxDec, double reaction_time, double theta,
	double x_leader, double x, double jamGap, double l_leader,
	double v, double lead_v, double b_estimate)
{
	double leader_stop_x
		= x_leader - pow(lead_v, 2) / 2 / b_estimate;

	//assuming no action was taken for reaction time +theta
	double follower_stop_x
		= x - pow(v, 2) / 2 / maxDec + v*(reaction_time + theta);

	double factor = 1;

	if (leader_stop_x - follower_stop_x > (l_leader + jamGap)*factor)
	{
		return true;
	}
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////
//gipps safety gaps
//Gipps, Peter G. "A behavioral car-following model for 
//computer simulation." 
//Transportation Research Part B: Methodological 15, no. 2 (1981): 105-111.
//Eq. and (5)
//with reduction factors
//IN LATER VERSION, THIS IS NOT USED. INSTEAD, WE USE ANTICIPATED GAP
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::GippsGap(double maxDec, double reaction_time, double theta,
	double x_leader, double x, double jamGap, double l_leader,
	double v, double lead_v, double b_estimate,
	bool on_ramp, bool forward, double self_acc)
{

	if (x_leader - x - jamGap - l_leader <= 0)
		return false;

	double factor = 1;
	if (on_ramp == true)
	{
		if (forward == true)
		{
			factor = f_gap_reduction_factor_onramp;
		}
		else
			factor = b_gap_reduction_factor_onramp;
	}
	else if (this->getLCType() == TURNING
		||
		this->getLCType() == EXIT)
	{
		if (forward == true)
		{
			factor = f_gap_reduction_factor_offramp;
		}
		else
			factor = b_gap_reduction_factor_offramp;
	}
	else // reduction factor for DLC
	{
		if (forward == true)
		{
			factor = f_gap_reduction_factor_DLC;
		}
		else
			factor = b_gap_reduction_factor_DLC;
	}

	b_estimate = b_estimate*factor;  //estimation regarding the leader's deceleration

	//if (this->getLCType() != OPTIONAL_LC
	//	&& this->getLaneChangeDesire() >= 0.8)

	//{
	//	self_acc = this->getMAXacc(); 
	//	b_estimate = MAX(self_acc, b_estimate);
	//	//if (forward)
	//	//{
	//	//	b_estimate = MAX(self_acc, b_estimate);
	//	//}
	//	//else
	//	//{
	//	//	b_estimate = this->getMAXdec();			
	//	//	if (b_estimate == 0)
	//	//		b_estimate = -0.1;
	//	//}
	//}
	//else if (
	//	this->getLCType() == OPTIONAL_LC
	//	&&
	//	forward == false)  //backward gap
	//{
	//	//b_estimate = MAX(self_acc, b_estimate); //for backward gap, use the subject vehicle's 
	//	b_estimate = this->getMAXdec();									//anticipated acc as the maximum deceleration
	//	if (b_estimate == 0)
	//		b_estimate = -0.1;// prevent overflow in the later calculations

	//}

	if (b_estimate == 0)
		b_estimate = -0.1;// prevent overflow in the later calculations

	double minimun_time;
	double minimun_gap;
	double delta_v = lead_v - v;  //leader the faster
	double delta_a = b_estimate - maxDec; // if delta_a is larger than zero, then it means that the leader's acceleration is larger than the follower
	double headway = x_leader - x;
	double gap = headway - l_leader;

	double t_l = -lead_v / b_estimate; //time takes for the leader to stop
	double t_f = -v / maxDec; //time takes for the follower to stop
	double t_minGap = -delta_v / delta_a; // time it takes for the follower to reach to the same velocity of the leader

	if (t_l >= 0 && t_f >= 0)
	{
		// Both leader and follower will stop
		if (t_l <= t_f)
		{
			// leader stops first, the minimum gap is reached when the follower stops
			minimun_gap = gap + lead_v*t_l + 0.5*b_estimate*t_l*t_l - v*t_f - 0.5*maxDec*t_f*t_f;
		}
		else
		{
			// follower stops first
			if (t_minGap <= 0)
			{
				// The leader and follower will never reach the same speed
				minimun_gap = gap; // current gap
			}
			else if (t_f >= t_minGap)
			{
				// The leader and follower reach the same speed before the follower stops
				minimun_gap = gap + delta_v*t_minGap + 0.5*delta_a*t_minGap*t_minGap;
			}
			else
			{
				// The follower stops before the leader and follower reach the same speed
				minimun_gap = gap + delta_v*t_f + 0.5*delta_a*t_f*t_f;
			}
		}
	}
	else if (t_l >= 0 && t_f < 0)
	{
		// leader stops but follower does not. Crash will happen
		minimun_gap = -1;
	}
	else if (t_l < 0 && t_f >= 0)
	{
		// follower stops but leader does not
		if (t_minGap <= 0)
		{
			// The leader and follower will never reach the same speed
			minimun_gap = gap; // current gap
		}
		else if (t_f >= t_minGap)
		{
			// The leader and follower reach the same speed before the follower stops
			minimun_gap = gap + delta_v*t_minGap + 0.5*delta_a*t_minGap*t_minGap;
		}
		else
		{
			// The follower stops before the leader and follower reach the same speed
			minimun_gap = gap + delta_v*t_f + 0.5*delta_a*t_f*t_f;
		}
	}
	else
	{
		// both leader and follower does not stop
		if (t_minGap <= 0)
		{
			// The leader and follower will never reach the same speed
			minimun_gap = gap; // current gap
		}
		else
		{
			// The leader and follower reach the same speed before the follower stops
			minimun_gap = gap + delta_v*t_minGap + 0.5*delta_a*t_minGap*t_minGap;
		}
	}

	//double time2stationary = -lead_v / b_estimate; //time takes for the leader to stop

	//minimun_time = MAX(0, -delta_v / delta_a);  // time it takes for the follower to reach to the same velocity of the leader
	//if (delta_a > 0 &&
	//	((time2stationary > minimun_time) || b_estimate >= 0))
	//{
	//	minimun_gap =
	//		x_leader - x - l_leader +
	//		delta_v*minimun_time +
	//		0.5*delta_a*minimun_time*minimun_time;
	//}
	//else
	//{
	//	//the time when the shortest gap occurs is either the time zero
	//	//or the time when the follower stops
	//	double time_follower_stop = -v / maxDec;
	//	minimun_gap =
	//		headway - l_leader +
	//		MIN(0,
	//		delta_v*time_follower_stop +
	//		0.5*delta_a*time_follower_stop*time_follower_stop
	//		);
	//}

	//minimun_gap -= (-delta_v*(reaction_time + theta));//hani 11-17-2016
	//minimun_gap -= (v*(reaction_time + theta));
	minimun_gap -= jamGap;

	if (minimun_gap > 0)
		return true;
	else
		return false;

	////places where the leader stops
	//double leader_stop_x
	//	= x_leader - pow(lead_v, 2) / 2 / b_estimate;

	////assuming no action was taken for reaction time +theta
	//double follower_stop_x
	//	= x - pow(v, 2) / 2 / maxDec + v*(reaction_time + theta);

	double thrd = pow(lead_v, 2) / 2 / b_estimate - pow(v, 2) / 2 / maxDec + v*(reaction_time + theta) + l_leader + jamGap;

	if ((x_leader - x) > thrd)
	{
		return true;
	}
	else
		return false;
}

double myVehicleDef::PosCf2EndofExitTurning()
{
	//simplify assume that there is a car at the edge of exiting or turning
	return BaseCfModel(this->getMAXdec(),
		this->getMAXacc(),
		this->getReactionTime()*beta,
		this->getEqvDis2NextTurning(), //the distance to the end of the ramp
		this->getJamGap()*alpha, 0, 4,
		freeflowspeed, this->getSpeed(), this->getPosition(),
		this->distance_to_off_ramp,//assume there is a car at the end of the ramp with speed of zero
		this->distance_to_off_ramp, //the position of the car steps earlier
		0,
		this->getDesireHeadway());
}

double myVehicleDef::Bound_Function(double param1)
{
	return exp(param1 - 0.5) / (1 + exp(param1 - 0.5));
}

double myVehicleDef::OnRampAddCoef(int num_lane_2_rightmost)
{
	if (IsSectionSource(this->getIdCurrentSection()))
		return 1;
	int next_sec =
		AKIVehInfPathGetNextSection
		(this->VehID, this->getIdCurrentSection());
	//if the next section has side lanes indicating a ramp
	//then we return the additional coefficient to increase
	//the intention of DLC
	unsigned int next_turning_id = getIdNextTurning();
	A2KSectionInf next_section_info = AKIInfNetGetSectionANGInf(next_turning_id);
	if (GetRampType(next_sec) == 1
		&&
		next_section_info.length - this->getPosition() <= 2000
		)
	{
		double ramp_length = 0;
		int num_acc_lane_plus_on_ramp =
			GetOnAccLaneFlow(next_sec)
			+
			getOnRampVehCount(next_sec, &ramp_length);
		double density
			= (double)num_acc_lane_plus_on_ramp
			/ (ramp_length + ACC_LANE_LENGTH)*1000.0;

		//comparing to the jam density 
		double jam_ratio = density / (1000.0 / 6.0);
		return jam_ratio*this->getIncreaseDLCCloseRamp() / num_lane_2_rightmost
			+ 1;
		return this->getIncreaseDLCCloseRamp();
	}
	return 1;
}

double myVehicleDef::getIncreaseDLCCloseRamp()
{
	return this->increase_DLC_close_ramp;
}


void myVehicleDef::setIncreaseDLCCloseRamp(double val)
{
	this->increase_DLC_close_ramp = MAX(0.01, val);
}

int myVehicleDef::GetRampType(int sec_id)
{
	const unsigned short *increase_DLC_close_ramp_str =
		AKIConvertFromAsciiString("section_ramp_type");
	int sec_type = ((ANGConnGetAttributeValueInt(
		ANGConnGetAttribute(increase_DLC_close_ramp_str), sec_id)));
#ifdef _DEBUG
	AKIDeleteUNICODEString(increase_DLC_close_ramp_str);
#else
	delete[] increase_DLC_close_ramp_str;
#endif
	return sec_type;

}

int myVehicleDef::GetCACCManagedLaneType(int sec_id)
{
	const unsigned short *ML_str =
		AKIConvertFromAsciiString("CACC_Managed_Lane_Type");
	int sec_type = ((ANGConnGetAttributeValueInt(
		ANGConnGetAttribute(ML_str), sec_id)));
	AKIDeleteUNICODEString(ML_str);

	return sec_type;

}

//////////////////////////////////////////////////////////////////////////
// find if the section is a source
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::IsSectionSource(int sec_id)
{
	const unsigned short *is_section_source_str =
		AKIConvertFromAsciiString("bool_section_source");
	bool temp = ((ANGConnGetAttributeValueBool(
		ANGConnGetAttribute(is_section_source_str), sec_id)));
#ifdef _DEBUG
	AKIDeleteUNICODEString(is_section_source_str);
#else
	delete[] is_section_source_str;
#endif
	return temp;
}

//////////////////////////////////////////////////////////////////////////
// return num of vehicles on ramp
//////////////////////////////////////////////////////////////////////////
int myVehicleDef::getOnRampVehCount(int next_sec, double *ramp_length)
{
	int num_veh = AKIVehStateGetNbVehiclesSection(next_sec, true);
	int num_sec = AKIInfNetNbSectionsANG();
	for (int i = 0; i < num_sec; i++)
	{
		int id = AKIInfNetGetSectionANGId(i);
		if (GetRampType(id) == TRUE_ON_RAMP){
			if (AKIInfNetGetIdSectionANGDestinationofTurning(id, 0)
				== next_sec)
			{
				double length = AKIInfNetGetSectionANGInf(id).length;
				ramp_length = &length;
				return AKIVehStateGetNbVehiclesSection(id, true);
			}
		}
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////
// return num of vehicles on the acc lane waiting to merge
//////////////////////////////////////////////////////////////////////////
int myVehicleDef::GetOnAccLaneFlow(int next_sec)
{

	int num_veh = AKIVehStateGetNbVehiclesSection(next_sec, true);
	int count = 0;
	double speed = 0;
	for (int i = 0; i < num_veh; i++)
	{
		InfVeh veh_info =
			AKIVehStateGetVehicleInfSection(next_sec, i);
		if (veh_info.numberLane == 1) //on the acc lane
		{
			speed += veh_info.CurrentSpeed / 3.6;
			count += 1;
		}
	}
	if (count > 0){
		speed = speed / (double)count;
		//avg_speed = &speed; 
	}
	else{
		speed = freeflowspeed;
		//avg_speed = &speed; 
	}

	return count;
}




//////////////////////////////////////////////////////////////////////////
// Get vehicle equilibrium-position information given the leader info 
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::GetEquPosition(double leader_pos, double leader_l, double v)
{
	/*int veh_Id= getId();
	AKIVehSetAsTracked(veh_Id);
	this->info = AKIVehTrackedGetInf(veh_Id);
	this->staticinfo = AKIVehTrackedGetStaticInf(veh_Id);
	AKIVehSetAsNoTracked(veh_Id); */

	double desired_headway = this->getDesireHeadway();
	double headway = leader_l + MAX(this->getJamGap(),
		+desired_headway*v - this->getLeader()->getSpeed(0)*AKIGetSimulationStepTime());
	return leader_pos - headway;

}
//////////////////////////////////////////////////////////////////////////
// In this function, we adjust the position of vehicles 
// so that it speed is at its desired speed
// and the spacing with the leader is at the equilibrium spacing
//void myVehicleDef::AdjustArrivalVehicle()
//{
//	//find the last veh on the list of the section with the same lane
//	//and put this vehicle with a equilibrium distance with the leader
//	this->freeflowspeed = A2SimVehicle::getFreeFlowSpeed();
//	if (this->getInitialLeaderId() > 0)
//	{
//		int leaderid = getInitialLeaderId();
//		/*int num_veh_sec =
//		AKIVehStateGetNbVehiclesSection(getIdCurrentSection(), true); */
//
//		double leader_length = 0;
//		/*for (int i=num_veh_sec-1; i>=0; i--)
//		{*/
//
//		//AKIVehSetAsTracked(leaderid);
//		InfVeh info_veh = AKIVehTrackedGetInf(leaderid);
//		if (info_veh.idVeh != getId())
//		{
//			if (getIdCurrentSection() == info_veh.idSection
//				&&
//				getIdCurrentLane() == info_veh.numberLane)
//			{
//				if (info_veh.CurrentPos < this->getPosition())
//				{
//					info_veh = info_veh;
//				}
//				else
//				{
//					StaticInfVeh vehinfo =
//						AKIVehTrackedGetStaticInf(info_veh.idVeh);
//					//AKIVehSetAsNoTracked(info_veh.idVeh);
//					leader_length = vehinfo.length;
//					//determine the equilibrium state regarding the leader
//					//that is when the acceleration equal zero and 
//					//the speed is at its desired speed
//					double v = MIN(info_veh.CurrentSpeed / 3.6, freeflowspeed); // be careful, here info_veh speed is in [km/h]
//					v = freeflowspeed;
//					double eq_pos = GetEquPosition(info_veh.CurrentPos,
//						leader_length, v);
//					if (eq_pos < 0)
//					{
//						setNewPosition(0,
//							0);
//						this->setNewArrivalAdjust(true);
//					}
//					else
//					{
//						double v_after_tau = GippsDecelerationTerm(
//							this->getMAXdec(), this->getReactionTime(), this->getGippsTheta(), info_veh.CurrentPos,
//							0, this->getJamGap(),
//							leader_length, v, info_veh.CurrentSpeed / 3.6,  // be careful, here info_veh speed is in [km/h]
//							getEstimateLeaderDecCoeff()*this->getMAXdec());
//						if (v_after_tau < v)
//						{
//							setNewPosition(0,
//								0);
//							this->setNewArrivalAdjust(true);
//						}
//						else
//						{
//							setNewPosition(this->getPosition() + 0.1,
//								v);
//							this->setNewArrivalAdjust(false);
//							setFirstCycleAfterAdjust(true);
//						}
//					}
//					/*if(GetEquPosition(info_veh.CurrentPos,
//					leader_length)<0)
//					{
//					setNewPosition(0, 0);
//					}*/
//					return;
//				}
//			}
//		}
//		//}
//	}
//
//	//only one car there and it is itself
//	setNewPosition(this->getPosition(),
//		freeflowspeed);
//	setFirstCycleAfterAdjust(true);
//	this->setNewArrivalAdjust(false);
//
//}


//////////////////////////////////////////////////////////////////////////
// A new method for adjusting the position of vehicles
// 1. select a starting point if there is no leader ahead
// 2. if the equilibrium position is larger than the starting point, then make that point as the starting point
//    otherwise, make the position as the equilibrium position
//////////////////////////////////////////////////////////////////////////
myVehicleDef::PositionSpeed myVehicleDef::AdjustArrivalVehicle_New(bool simpleNetwork)
{
	PositionSpeed pos_speed;

	//find the last vehicle on the list of the section with the same lane
	//and put this vehicle with a equilibrium distance with the leader
	this->freeflowspeed = A2SimVehicle::getFreeFlowSpeed();

	double init_start = this->sec_inf.length - START_POSITION;
	//double init_start = START_POSITION;

	// Extra headway in case it is a ACC or CACC vehicle
	SetExtraHeadways();

	this->InitialCACCACCMode();

	int CACC_threshold = getCACC_length_limit();

	if (this->leader != NULL && this->leader->isFictitious() == false)
	{
		int leaderid = this->leader->getId();
		if (leaderid != getInitialLeaderId())
		{
			leaderid = getInitialLeaderId();
		}

		myVehicleDef* my_leader = (myVehicleDef*)leader;
		double leader_length = 0;


		if (leader->getId() != getId())
		{
			if (getIdCurrentSection() == leader->getIdCurrentSection() &&
				getIdCurrentLane() == leader->getIdCurrentLane())
			{
				if (my_leader->getPosition() > this->getPosition())
				{
					leader_length = my_leader->getLength();
					if (this->adaptive_mode == CACCMODE ||
						this->adaptive_mode == ACCMODE ||
						this->getVehType() == NGSIM_CACC_ACC_V2V ||
						this->getVehType() == CACC_HOV)
						leader_length = my_leader->getLengthCACC();
					//determine the equilibrium state regarding the leader
					//that is when the acceleration is equal to zero and the speed is at its desired speed
					double v;
					double freeSpd = A2SimVehicle::getFreeFlowSpeed();
					if (simpleNetwork)
					{
						if (freeSpd > 0)
						{
							v = MIN(my_leader->getSpeed(0), A2SimVehicle::getFreeFlowSpeed(), getAverageSpeedAHead(0, 1000, 20));
						}
						else
						{
							v = MIN(my_leader->getSpeed(0), getAverageSpeedAHead(0, 1000, 20));
						}
					}
					else
					{
						if (freeSpd > 0)
						{
							v = MIN(my_leader->getSpeed(0) + 1, A2SimVehicle::getFreeFlowSpeed());
						}
						else
						{
							v = my_leader->getSpeed(0) + 1;
						}
					}

					//equilibrium position is the position of the leader minus a desired following range (when acc=0)
					double eq_pos = GetEquPosition(my_leader->getPosition(), leader_length, v);

					if (this->getVehType() == NGSIM_CACC_ACC_V2V || this->getVehType() == CACC_HOV)
					{
						//this vehicle is a CACC capable vehicle.

						//int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());

						//check if CACC mode is possible

						//get leader CACC platoon number
						//int VehANGId_temp3 = ANGConnVehGetGKSimVehicleId(my_leader->getId());
						//const unsigned short *temp315152 = AKIConvertFromAsciiString("GKSimVehicle::CACCPlatoonNum");
						//int num_platoon = ANGConnGetAttributeValueInt(ANGConnGetAttribute(temp315152), VehANGId_temp3);   //the virtual leading vehicle of the real leader  

						int num_platoon = my_leader->stringPosID;

						if (my_leader->getVehType() == NGSIM_CACC_ACC_V2V || this->getVehType() == CACC_HOV)  //check 1: leading vehicle is A CACC vehicle 2. leading vehicle platoon is short 
						{//it is CACC capable. do CACC
							if (num_platoon < 0)
							{
								//leading vehicle is not in any platoon.	

								//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp315152), VehANGId_temp3, 0);  //set the leading vehicle as a leader of the platoon
								//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp315152), VehANGId, 1); //set this vehicle as a second of the platoon

								my_leader->stringPosID = 0;
								this->stringPosID = 1;
								this->setMode(CACC_ON);   //Do CACC
							}
							else if (num_platoon >= CACC_threshold - 1)
							{
								//break two consecutive vehicles into two CACC platoons

								//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp315152), VehANGId, 0);
								this->stringPosID = 0;
								this->setMode(CACC_ON_Leader);   //Do CACC_LEADER
							}
							else
							{
								// this platoon is not too long, join this platoon.

								num_platoon++;
								//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp315152), VehANGId, num_platoon);
								this->stringPosID = num_platoon;
								this->setMode(CACC_ON);   //Do CACC
							}

						}
						else if (my_leader->getVehType() == CAR_Type && my_leader->ConnectFlag)
						{
							my_leader->stringPosID = 0;
							this->stringPosID = 1;
							this->setMode(CACC_ON);   //Do CACC
						}
						else
						{
							this->setMode(CACC_ON_ACC);
							ACC_recover_time = 25;
						}

						//AKIDeleteUNICODEString(temp315152);
						eq_pos = GetEquPosition(my_leader->getPosition(),
							leader_length, v); //equ position is the position of the leader minus a desired following range (when acc==0)
						//if that position is larger than the init start position, use init start position
						eq_pos = MIN(init_start, eq_pos);

						//while (true)
						//{
						//	double v_after_tau = v;
						//	PositionSpeed posSpeed = this->Run_CACC();
						//	v_after_tau = posSpeed.speed;
						//	if (v_after_tau < v)
						//	{//if the vehicle is about to decelerate, then increase the head-away distance.
						//		eq_pos -= 3;
						//	}
						//	else
						//	{
						//		break;
						//	}
						//	if (eq_pos <= 0)
						//		break;
						//}
					}
					else if (this->getVehType() == NGSIM_ACC || this->getVehType() == ACC_HOV) //this vehicle is a ACC vehicle.
					{	//this vehicle is a ACC capable vehicle.
						//set vehicle in ACC_ON mode
						this->setMode(ACC_ON);
						ACC_recover_time = 25;
						eq_pos = GetEquPosition(my_leader->getPosition(),
							leader_length, v); //equ position is the position of the leader minus a desired following range (when acc==0)
						//if that position is larger than the init start position, use init start position
						eq_pos = MIN(init_start, eq_pos);

						//while (true)
						//{
						//	double v_after_tau = v;
						//	PositionSpeed posSpeed = this->Run_ACC();
						//	v_after_tau = posSpeed.speed;
						//	if (v_after_tau < v)
						//	{//if the vehicle is about to decelerate, then increase the head-away distance.
						//		eq_pos -= 3;
						//	}
						//	else
						//	{
						//		break;
						//	}
						//	if (eq_pos <= 0)
						//		break;
						//}
					}
					else
					{//otherwise, set this vehicle in CF mode.
						//adjust the equilibrium position if this vehicle is a manual driving vehicle.

						eq_pos = GetEquPosition(my_leader->getPosition(), leader_length, v); //equ position is the position of the leader minus a desired following range (when acc==0)
						//if that position is larger than the init start position, use init start position
						eq_pos = MIN(init_start, eq_pos);

						//the follow while loop is for manual driven vehicle
						//because the equilibrium location from Newell's rule cannot guarantee a zero deceleration from Gipps model
						while (true)
						{
							double v_after_tau = v;
							//v_after_tau = GippsDecelerationTerm(
							//this->getMAXdec(), this->getReactionTime(), this->getGippsTheta() * this->getReactionTime(), my_leader->getPosition(),
							//eq_pos, this->getJamGap(),
							//leader_length, v, my_leader->getSpeed(),  // be careful, here info_veh speed is in [km/h]
							//getEstimateLeaderDecCoeff()*this->getMAXdec());

							v_after_tau = NGSIM_Speed(eq_pos, v);

							double testDist = my_leader->getPosition() - eq_pos;

							// if the desired speed from Gipps model is less than the leader or the free-flow
							// keep moving the position further upstream
							// this increment 10 can be further reduced but that will increase computation time.
							if (v_after_tau < v)
							{//if the vehicle is about to decelerate, then increase the head-away distance.
								eq_pos -= 0.5;
							}
							else
							{
								break;
							}
							if (eq_pos <= 0)
								break;
						}

						if (eq_pos > 0)
						{
							eq_pos = eq_pos + v*AKIGetSimulationStepTime();
						}


						this->setMode(CF);
					}

					//if (eq_pos < init_start)
					//{
					//	if (eq_pos > locationFromHeadway && AKIGetRandomNumber() <= 0.8)
					//	{
					//		// When there is a queue upstream from the virtual starting point, allow half of the 
					//		// vehicles to enter the network based on the equilibrim position. This is to avoid
					//		// distorting the distribution_Hao 10/07/17
					//		eq_pos = locationFromHeadway;
					//	}

					//}

					if (eq_pos <= 0 || v <= 0)
					{
						setNewPosition(0, 0);
						//SetReportError();
						pos_speed.position = eq_pos;
						pos_speed.speed = 0;
						this->setNewArrivalAdjust(true);
						setFirstCycleAfterAdjust(false);
						//AKIVehTrackedRemove(this->getId());
						return pos_speed;
					}
					else
					{
						pos_speed.position = eq_pos;
						pos_speed.speed = v;
						pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
						this->setNewArrivalAdjust(false);
						setFirstCycleAfterAdjust(true);
						return pos_speed;
					}
				}
				else
				{
					SetReportError();
					pos_speed.position = my_leader->getPosition() - getJamGap();
					pos_speed.speed = 0;
					this->setNewArrivalAdjust(true);
					setFirstCycleAfterAdjust(false);
					//AKIVehTrackedRemove(this->getId());
					return pos_speed;
				}
			}
		}
	}

	//only one car there and it is itself
	//set to the starting point
	pos_speed.position = init_start;
	pos_speed.speed = freeflowspeed;
	pos_speed = setNewPosition(pos_speed.position, pos_speed.speed);
	setFirstCycleAfterAdjust(true);
	this->setNewArrivalAdjust(false);
	return pos_speed;
}

double myVehicleDef::getFrictionCoef()
{

	if (friction_coeff == 0)
	{
		friction_coeff = 1;
	}

	double sec_sync_coef =
		this->getSectionSyncCoef();

	if (sec_sync_coef != 0)
		return sec_sync_coef;
	else
		return friction_coeff;
}

void myVehicleDef::setFrictionCoef(double val)
{
	friction_coeff = val;
}

int myVehicleDef::DetermineReceiveOrLcOrCoop()
{
	//first decide if lane change is necessary
	if (NeedCoop())
	{
		return this->setMode(CCF);
	}
	else if (NeedLC())
	{
		return this->setMode(BCF);
	}
	else
		return this->getMode();
}

int myVehicleDef::setMode(int avalue)
{
	mode = avalue;
	if (mode == RCF)
	{
		//reset ACF steps
		ACF_Step = 0;
	}
	else if (mode == ACF)
	{
		ACF_Step = 0;
	}
	return mode;
}

bool myVehicleDef::isLaneChangingPossible(int target_lane)
{
	if (this->getHOV() == false //this car is not a hov car
		&&
		GetSectionHOVLane() > 0 //this section contains hov lane
		&&
		isHOVActive())  //now hov is active in terms of time
	{
		//if the target lane is the left-most lane and hov is active on this lane
		if (this->getIdCurrentLane() - target_lane
			== sec_inf.nbCentralLanes + sec_inf.nbSideLanes)
		{
			return false;  //lane change is impossible if a non-hov car goes to hov lane
		}
	}

	if (this->GetRampType(this->getIdCurrentSection()) == ON_RAMP)
	{
		if (this->getIdCurrentLane() == 1)
		{
			// on the acceleration lane, but in the NO LC zone
			if (this->getPosition() < FORBID_RAMPCHANGE_ZONE)
			{
				return false;
			}
		}
		else if (this->getNumberOfLanesInCurrentSection() > this->getNumberOfMainLanesInCurrentSection()
			&& this->getIdCurrentLane() == 2
			&& target_lane == RIGHT)
		{
			// Do not switch to acceleration lane
			return false;
		}
	}
	return A2SimVehicle::isLaneChangingPossible(target_lane);
}

void myVehicleDef::setInitialLeaderId(int id)
{
	this->initial_leader_id = id;
}

int myVehicleDef::getNextSectionRampType
(int& next_sec_center_lanes)
{
	int sectionid = getIdCurrentSection();
	A2KSectionInf sectioninfo = this->sec_inf;
	//AKIInfNetGetSectionANGInf(sectionid);   

	for (int i = 0; i < sectioninfo.nbTurnings; i++)
	{
		int next_sec_id =
			AKIInfNetGetIdSectionANGDestinationofTurning(sectionid, i);
		if (GetRampType(next_sec_id) != ON_RAMP)
		{
			continue;
		}
		else
		{
			next_sec_center_lanes
				= AKIInfNetGetSectionANGInf(next_sec_id).nbCentralLanes;
			return this->GetRampType(next_sec_id);
		}
	}

	return NO_RAMP;
}

void myVehicleDef::DesireEquation(double& para1, double& para2, double dis2End,
	double time2End, int n_lc, double minE, double minT, double E, double T)
{
	if (dis2End < E //abs(n_lc)*E
		||
		time2End < T) //abs(n_lc)*T)
	{
		if (minT > time2End
			||
			minE > dis2End)
		{
			para2 = 1;
			para1 = 1;
		}
		else
		{
			//para2 = 1 - (time2End - minT)
			//	/ (T - minT)
			//	/ abs(n_lc);
			//para1 = 1 - (dis2End - minE)
			//	/ (E - minE)
			//	/ abs(n_lc);
			para2 = 1 - (time2End - minT)
				/ (T - minT);
			para1 = 1 - (dis2End - minE)
				/ (E - minE);
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// the maximum acceptable deceleration in before lc cf is dependent on the desire
// when the desire approaches one, the maximum deceleration approaches to the maximum deceleration
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::getMaxDecInSync()
{
	return this->getMAXdec()*this->getLaneChangeDesire();
}

//////////////////////////////////////////////////////////////////////////
// get number of vehicles on the acc lane on the next section
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::getNoOfVehsOnNextOnRampAccLane()
{
	int n_turnings = this->sec_inf.nbTurnings;
	for (int i = 0; i < n_turnings; i++)
	{
		int aid = AKIInfNetGetIdSectionANGDestinationofTurning(this->getIdCurrentSection(), i);
		if (GetRampType(aid) == ON_RAMP)
		{
			return (bool)GetOnAccLaneFlow(aid);
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////
// if the leader just performed the lane changing at the same time
// return true
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::PreventSimultaneousLC()
{
	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	if (vehDown != NULL && vehDown->isFictitious() == false)
	{
		if (((myVehicleDef*)(vehDown))->getLastLCTime() >= AKIGetCurrentSimulationTime())
		{
			return true;
		}
	}
	return false;
}

void myVehicleDef::setRampDecision(int ramp_lc_decision)
{
	this->_ramp_lc_decision = ramp_lc_decision;
}

int myVehicleDef::getRampDecision()
{
	return this->_ramp_lc_decision;;
}

int myVehicleDef::getSmoothTransitTime()
{
	return _smooth_transit_time;
}

void myVehicleDef::addOneStepTransitTime()
{
	int temp = this->_smooth_transit_time;
	this->_smooth_transit_time = temp %
		((int)(this->getAccSmoothCoef() / AKIGetSimulationStepTime())) + 1;
}

double myVehicleDef::getPolitenessOptional()
{
	if (this->adaptive_mode == ACCMODE ||
		this->adaptive_mode == CACCMODE)
		return 0;
	else
		return this->politeness_optional;
}

double myVehicleDef::getRandomPolitenessOptional()
{
	return random_politeness_optional;
}

void myVehicleDef::setRandomPolitenessOptional(double param1)
{
	random_politeness_optional = param1;
}

void myVehicleDef::setPolitenessOptional(double param1)
{
	this->politeness_optional = param1;
}

//////////////////////////////////////////////////////////////////////////
// check if the leader or follower is a LCer
bool myVehicleDef::ExistNewLCer(int direction)
{
	double desire = this->getLaneChangeDesire();

	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	if (getTargetLane() == LEFT)
	{
		vehUp = this->left_follower;
		vehDown = this->left_leader;
	}
	else
	{
		vehUp = this->right_follower;
		vehDown = this->right_leader;
	}

	/*A2SimVehicle *vehUp=NULL;
	A2SimVehicle *vehDown=NULL;
	getUpDown((const A2SimVehicle *&)vehUp,
	(const A2SimVehicle *&)vehDown, direction, 0); */
	//no up and down vehicles, gap accepted
	if ((vehDown == NULL || this == vehDown) &&
		(vehUp == this || vehUp == NULL))
		return false;
	if (vehDown != NULL && this != vehDown)
	{
		if (((myVehicleDef*)vehDown)->getLastLCTarget() == direction)
		{
			return true;
		}
	}
	if (vehUp != NULL&&this != vehUp)
	{
		if (((myVehicleDef*)vehUp)->getLastLCTarget() == direction)
		{
			return true;
		}
	}
	return false;
}

void myVehicleDef::getAroundSpeed()
{

	double d_scan = this->getDLCScanRange();
	double n_scan = this->getDLCScanNoCars();
	left_avg_speed_ahead = 0;
	right_avg_speed_ahead = 0;
	left_avg_speed_ahead = isLaneChangingPossible(LEFT) ?
		getAverageSpeedAHead(LEFT, d_scan, n_scan) : 0;
	right_avg_speed_ahead = isLaneChangingPossible(RIGHT) ?
		getAverageSpeedAHead(RIGHT, d_scan, n_scan) : 0;


	//if (this->getIdCurrentLane() - this->sec_inf.nbSideLanes == 1)
	//{
	//	right_avg_speed_ahead = 0;
	//}

	avg_speed_ahead = 0;
	avg_speed_ahead = getAverageSpeedAHead(0, d_scan, n_scan);

	if (avg_speed_ahead < 0)
		avg_speed_ahead = freeflowspeed;
	if (left_avg_speed_ahead < 0)
		left_avg_speed_ahead = freeflowspeed;
	if (right_avg_speed_ahead < 0)
		right_avg_speed_ahead = freeflowspeed;

	this->freeflowspeed = this->createFreeFlowSpeed(false);

	return;



}

void myVehicleDef::getAroundLeaderFollowers()
{
	double ShiftUp = 0, ShiftDw = 0;
	A2SimVehicle* follower = NULL;
	A2SimVehicle* leader = NULL;
	double XPosTargetlane = 0;
	if (this->getIdCurrentLane() > 1 && !this->isCurrentLaneInNode())
	{
		// Vehicle not in the rightmost lane
		int lane = this->getIdCurrentLane();
		XPosTargetlane = this->getPositionInTargetlane(this->getPosition(0), RIGHT);
		getUpDown(RIGHT, XPosTargetlane, follower, ShiftUp, leader, ShiftDw);
		right_follower = (const A2SimVehicle*)follower;
		right_leader = (const A2SimVehicle*)leader;

		if (follower)
		{
			double posFollower = follower->getPosition(0);
			this->rightLagHeadway = XPosTargetlane - ShiftUp - posFollower;
		}
		else
		{
			this->rightLagHeadway = 1000;
		}
		if (leader)
		{
			double posLeader = leader->getPosition(0);
			this->rightLeadHeadway = posLeader + ShiftDw - XPosTargetlane;
		}
		else
		{
			this->rightLeadHeadway = 1000;
		}
	}
	else if (this->isCurrentLaneInNode()
		&& this->getIdCurrentLane() > enter_lane_from)
	{
		// If a sbuject vehicle is in a node, Aimsun can only identify lead/lag vehicles for it in 
		// lanes connect to the upstream section.
		//XPosTargetlane = this->getPositionInTargetlane(this->getPosition(0), RIGHT);
		//getUpDown(RIGHT, XPosTargetlane, follower, ShiftUp, leader, ShiftDw);
		//right_follower = (const A2SimVehicle*)follower;
		//right_leader = (const A2SimVehicle*)leader;
	}
	else
	{
		// Vehicle in the rightmost lane
		right_follower = NULL;
		right_leader = NULL;
	}


	if (this->getIdCurrentLane() < this->getNumberOfLanesInCurrentSection()
		&& !this->isCurrentLaneInNode())
	{
		// Vehicle not in the leftmost lane
		XPosTargetlane = this->getPositionInTargetlane(this->getPosition(0), LEFT);
		getUpDown(LEFT, XPosTargetlane, follower, ShiftUp, leader, ShiftDw);
		left_follower = (const A2SimVehicle*)follower;
		left_leader = (const A2SimVehicle*)leader;


		if (follower)
		{
			double posFollower = follower->getPosition(0);
			this->leftLagHeadway = XPosTargetlane - ShiftUp - posFollower;
		}
		else
		{
			this->leftLagHeadway = 1000;
		}
		if (leader)
		{
			double posLeader = leader->getPosition(0);
			this->leftLeadHeadway = posLeader + ShiftDw - XPosTargetlane;
		}
		else
		{
			this->leftLeadHeadway = 1000;
		}

	}
	else if (this->isCurrentLaneInNode()
		&& this->getIdCurrentLane() < enter_lane_to)
	{
		//int lane = this->getIdCurrentLane();
		//XPosTargetlane = this->getPositionInTargetlane(this->getPosition(0), LEFT);
		//getUpDown(LEFT, XPosTargetlane, follower, ShiftUp, leader, ShiftDw);
		//left_follower = (const A2SimVehicle*)follower;
		//left_leader = (const A2SimVehicle*)leader;
	}
	else
	{
		// Vehicle in the leftmost lane
		left_follower = NULL;
		left_leader = NULL;
	}
}




// get basic section information
// and the next section info.
void myVehicleDef::getSectionInfo()
{

	int sec_id = getIdCurrentSection();

	if (sec_id != current_sec_id && !this->isCurrentLaneInNode())
	{
		//if (this->MissTurns())
		//{
		//	RecordMissTurnLog();
		//}
		sec_inf = (AKIInfNetGetSectionANGInf(sec_id));
		//this->info = AKIVehTrackedGetInf(this->getId());
		nextsec = this->getIdNextSection();
		exit_lane_from = AKIInfNetGetTurningOriginFromLane(sec_id, nextsec);
		exit_lane_to = AKIInfNetGetTurningOriginToLane(sec_id, nextsec);
		enter_lane_from = AKIInfNetGetTurningDestinationFromLane(sec_id, nextsec);
		enter_lane_to = AKIInfNetGetTurningDestinationToLane(sec_id, nextsec);
		A2KSectionInf sec_inf_next = (AKIInfNetGetSectionANGInf(nextsec));
		this->Connect2RightMostLane = false;
		if (nextsec > 0)
		{
			int to_lane = AKIInfNetGetTurningDestinationFromLane(sec_id, nextsec);
			if (to_lane >= 2)
			{
				Connect2RightMostLane = true;
			}
		}

		nextnextsec = AKIVehInfPathGetNextSection(this->VehID, nextsec);
		thirdsec = AKIVehInfPathGetNextSection(this->VehID, nextnextsec);
		CheckCloseness2Offramp(getEarlyLaneKeepDis());  //check if within two miles to offramp
		current_sec_id = sec_id;


	}
}


void myVehicleDef::setLastLCType(int type)
{
	this->last_lc_type = type;
}

int myVehicleDef::getLastLCType()
{
	return this->last_lc_type;
}

double myVehicleDef::getComfDecRampLC()
{
	return this->comf_dec_ramplc;
}

void myVehicleDef::setComfDecRampLC(double param)
{
	this->comf_dec_ramplc = -param;
}

double myVehicleDef::getComfDecDLC()
{
	return this->comf_dec_dlc;
}

void myVehicleDef::setComfDecDLC(double param)
{
	this->comf_dec_dlc = -param;
}

double myVehicleDef::getRampLCSlowDownDesire()
{
	return 0.8;
	return this->ramp_lc_slowdown_desire;
}

double myVehicleDef::getAccExp()
{
	return this->acc_exp;
}

void myVehicleDef::setAccExp(double param1)
{
	param1 = param1 < 1 ? 2 : param1;
	this->acc_exp = param1;
}

void myVehicleDef::setRelaxationTime(double param)
{
	this->ACF_Steps = (int)(param / 0.1);
}


int myVehicleDef::GetSectionHOVLane()
{
	//get the lane id of HOV
	const unsigned short *id_str =
		AKIConvertFromAsciiString("hov_lane");
	int id = ANGConnGetAttributeValueInt(
		ANGConnGetAttribute(id_str), this->getIdCurrentSection());
#ifdef _DEBUG
	AKIDeleteUNICODEString(id_str);
#else
	delete[] id_str;
#endif
	return id;

}

//////////////////////////////////////////////////////////////////////////
// this is the minimum distance to avoid collision
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::MinCollisionAvoidancePos(const A2SimVehicle* leader,
	int shortGap,
	double beta, //reduction factor for reaction time
	double alpha, //reduction factor for jam gap
	double Relaxation)
{
	double ref_pos_front = 0;

	//if the leader has updated, then use its previous step
	if (leader->isFictitious() == false
		&& leader->isUpdated())
		ref_pos_front = ((myVehicleDef*)leader)->getPositionReferenceVeh(1, this, 0);
	else
		ref_pos_front = ((myVehicleDef*)leader)->getPositionReferenceVeh(0, this, 0);

	double l_leader = leader->getLength();
	double headway = ref_pos_front;
	double gap = headway - l_leader;

	// the update judgment has been put input the reloaded getspeed
	double v_leader = ((myVehicleDef*)leader)->getSpeed();
	double x_leader = ((myVehicleDef*)leader)->getPosition();

	//double leader_past_pos
	//	= ((myVehicleDef*)leader)->GetPastPos(getReactionTime()*beta);

	double x = this->getPosition();
	double v = this->getSpeed();
	//if the leader is not on the same section, the x_leader
	//does not equal (x+headway)
	if (abs(x_leader - (ref_pos_front + x)) > 0.1
		|| leader->getIdCurrentSection() != this->getIdCurrentSection())
	{
		x_leader = ref_pos_front + x;
		//the past pos the leader changes to:
		//leader_past_pos = x +
		//	((myVehicleDef*)leader)->
		//	getPastPositionReferenceVehs
		//	(getReactionTime()*beta, this, 0);
	}

	double a_L_leader = ((myVehicleDef *)leader)->getMAXdec();
	double d_leader = -(v_leader * v_leader)
		/ (2 * a_L_leader)*(shortGap == 1 ? Relaxation : 1);

	double theta = this->getGippsTheta()*this->getReactionTime()*beta;

	double v_after_tau = GippsDecelerationTerm(
		this->getMAXdec(), this->getReactionTime(), theta, x_leader, x, this->getJamGap()*alpha,
		l_leader, v, v_leader, this->getMAXdec()*this->getEstimateLeaderDecCoeff());

	return (this->getSpeed() + v_after_tau) / 2 * delta_t + this->getPosition();
}



bool myVehicleDef::isHOVActive()
{
	return _hov_active;
}

void myVehicleDef::setHOVIncluded(bool hov_include)
{
	this->_hov_include = hov_include;
}

bool myVehicleDef::getHOVIncluded()
{
	return this->_hov_include;
}

void myVehicleDef::setHOVActive()
{
	_hov_active = false;
	double current_time = AKIGetCurrentSimulationTime();
	if (this->getHOVIncluded()) // if hov lane is simulated
	{
		//the current time
		if (current_time >= this->getHOVStart() * 60 * 60
			&&
			current_time <= this->getHOVEnd() * 60 * 60)
		{
			_hov_active = true;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// set boolean value to indicate if the current car is within 2 miles to the offramp
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::CheckCloseness2Offramp(double distance)
{
	// Hao's code: only check next section===========================================
	//this->exit_section_before_offramp = 0;
	//this->closeness2Offramp = false;
	//if (GetRampType(nextsec) == TRUE_OFF_RAMP)
	//{
	//	this->exit_section_before_offramp = this->getIdCurrentSection();
	//	this->closeness2Offramp = true;
	//}
	//return;
	//===================================================================================

	this->exit_section_before_offramp = 0;
	//this->closeness2Offramp = false;
	//if (GetRampType(nextsec) == TRUE_OFF_RAMP)
	//{
	//	// Current section is the freeway section with off-ramp
	//	this->exit_section_before_offramp = this->getIdCurrentSection();
	//	this->closeness2Offramp = true;
	//	return;
	//}

	int tempsec = nextsec;
	double totallength = 0;
	while (tempsec > 0)
	{
		if (GetRampType(tempsec) == TRUE_OFF_RAMP)
		{
			this->distance_to_off_ramp = totallength + this->sec_inf.length - this->getPosition(0);
			this->closeness2Offramp = true;
			this->offRampID = tempsec;
			return;
		}
		else
		{
			totallength += AKIInfNetGetSectionANGInf(tempsec).length;
			if (totallength > distance)
			{
				break;
			}
			else
			{
				this->exit_section_before_offramp = tempsec;
				tempsec = AKIVehInfPathGetNextSection(this->VehID, tempsec);
			}
		}
	}

	this->distance_to_off_ramp = -1;
	this->exit_section_before_offramp = 0;
	this->offRampID = 0;
	this->closeness2Offramp = false;
	return;

	//if any of the three following segment is a offramp do not do this
	//std::vector<int> sequent_secs; 
	//sequent_secs.push_back(nextsec); 
	//sequent_secs.push_back(nextnextsec); 
	//sequent_secs.push_back(thirdsec); 
	//double totallength = 0; 
	//for(int i=0; i<sequent_secs.size(); i++)
	//{
	//	if(sequent_secs.at(i)>0)
	//	{
	//		if (GetRampType(sequent_secs[i]) == TRUE_OFF_RAMP)
	//		{	
	//			this->closeness2Offramp = true; 
	//			return; 
	//		}
	//		totallength += AKIInfNetGetSectionANGInf(sequent_secs[i]).length; 
	//		if(totallength>3218.688 &&
	//		   i>=1) // do not consider more than 2 miles but at lease we consider two sequent sections
	//		{
	//			break; 
	//		}
	//	}
	//}
	//this->closeness2Offramp = false; 
}

//////////////////////////////////////////////////////////////////////////
// check if the current car is within three segment to the offramp
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::NotWithinOfframpAwarenessDis()
{
	return !this->closeness2Offramp;
}

//////////////////////////////////////////////////////////////////////////
// Minimun values 
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::getMinTimeBtwLcs4DLC()
{
	//make it random between 10-40 second
	return AKIGetRandomNumber() * 10 + 5;
}

//////////////////////////////////////////////////////////////////////////
// Enhance the off-ramp desire to pass the threshold if the gap is acceptable
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::setExtraDesire4FeasibleGapOfframp(int targetlane)
{
	int target = this->getTargetLane();
	this->setTargetLane(targetlane);
	if (GapAcceptDecision_Sync_First() == EXIT_CHANGE_FEASIBLE)
	{
		if (targetlane == RIGHT)
		{
			double intitialDesire = this->desireLC_option_right;
			double desire = AKIGetRandomNumber();
			this->desireLC_force_left = 0;
			this->desireLC_option_left = 0;
			//this->desireLC_option_right = MAX(intitialDesire, desire);
			this->desireLC_force_right = MAX(this->desireLC_force_right, desire*0.1524); //Make sure there is a 80% chance for a vehicle to make an EXIT LC in 30 s (assume LC desire threshold is 0.15) Hao 03/06/17
		}
		else
		{
			double intitialDesire = this->desireLC_option_left;
			double desire = AKIGetRandomNumber();
			this->desireLC_force_right = 0;
			this->desireLC_option_right = 0;
			//this->desireLC_option_left = MAX(intitialDesire, desire);
			this->desireLC_force_left = MAX(this->desireLC_force_left, desire*0.1524);
		}
	}
	else
	{
		this->setTargetLane(target);
	}
}

//////////////////////////////////////////////////////////////////////////
// Eliminate the optional lc desires if the target lane is outside the feasible lane for exiting
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::EliminateDlcDesireOutSideRouteLanes(int fromlane, int tolane)
{
	if (this->desireLC_option_left > 0)
	{
		int optional_targetlane = this->getIdCurrentLane();
		optional_targetlane -= LEFT;
		if (optional_targetlane > tolane)
		{
			desireLC_option_left = 0;
		}
	}
	if (this->desireLC_option_right > 0)
	{
		int optional_targetlane = this->getIdCurrentLane();
		optional_targetlane -= RIGHT;
		if (optional_targetlane < fromlane)
		{
			desireLC_option_right = 0;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
// if gap is accepted then make the incentive 100%
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::BoostOnrampIncentive()
{
	if (this->desireLC_force_left == 1)
		return;
	int targetlane = this->getTargetLane();
	setTargetLane(LEFT);
	if (this->GapAcceptDecision_Sync_First() == RAMP_LANE_CHANGE_FEASIBLE)
	{
		this->setLaneChangeDesireForce(1, 0);
	}
	this->setTargetLane(targetlane);
}

int myVehicleDef::GetSectionOfframpLanes(int sec_id)
{
	if (sec_id == 0)
		return -1;
	A2KSectionInf secInfo = AKIInfNetGetSectionANGInf(sec_id);
	return secInfo.nbCentralLanes;
}

double myVehicleDef::getEarlyLaneKeepDis()
{
	return _early_lane_keep_dis;
}

void myVehicleDef::setEarlyLaneKeepDis(double param)
{
	_early_lane_keep_dis = param * 1000;
}



//////////////////////////////////////////////////////////////////////////
// set report error so API code could delete it
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::SetReportError()
{
	AKIVehSetAsTracked(this->getId());
	this->staticinfo.width = 1;
	AKIVehSetStaticInf(this->VehID, this->staticinfo);

	AKIVehSetAsNoTracked(this->getId());
}



//////////////////////////////////////////////////////////////////////////
// determine if this network is specifically for merging networks
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::UseMergingDemandReading()
{
	int expid = ANGConnGetExperimentId();
	const unsigned short *temp_str = AKIConvertFromAsciiString("read_matrix_demand");
	bool ret = (bool)ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(temp_str), expid);
	AKIDeleteUNICODEString(temp_str);
	return ret;
}


//////////////////////////////////////////////////////////////////////////
//if overpass is feasible meaning if this car take pass the target in a specified time
// and leave an enough gap
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::OverPassPossbile(myVehicleDef * templeader)
{
	//the time it takes in acceleration mode to reach the end of turning point
	int timelimit = MIN_TIME_OVERPASS;
	if (this->getSpeed() > 0)
		timelimit = (int)((this->getEqvDis2NextTurning() - 20) / this->getSpeed());
	if (timelimit < 0)
		return false;

	//Overpass is feasible if the gap is larger than 2*jamGap after 7s/timelimit.
	timelimit = MAX(timelimit, MIN_TIME_OVERPASS);

	if (templeader != NULL
		&&
		templeader->isFictitious() == false)
	{
		double dis = ((myVehicleDef*)templeader)->getPositionReferenceVeh(this);
		double pos = this->getSpeed()*timelimit + 0.5*getOffRampOverpassAcc()*timelimit*timelimit;
		pos = MIN(pos, (this->getSpeed() + A2SimVehicle::getFreeFlowSpeed()*1.2) / 2.0*timelimit);

		double leader_pos = (templeader->getSpeed(0))*(double)timelimit;
		if (pos > leader_pos + dis + templeader->getLength() + this->getJamGap() * 2)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	return true;
}

void myVehicleDef::SetRiskyRelax()
{

	this->alpha = 0.5;
	this->beta = 0.1;
	this->Relaxation = 0.2;
}

void myVehicleDef::ResetRelax()
{
	alpha = 1;  //jam gap
	beta = 0.5;  //reaction time
	Relaxation = 0.5;
}

//////////////////////////////////////////////////////////////////////////
// Sync coefficient of section
//////////////////////////////////////////////////////////////////////////
double myVehicleDef::getSectionSyncCoef()
{

	const unsigned short *tempstr =
		AKIConvertFromAsciiString("sync_coeff");
	double val = ((ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(tempstr), this->sec_inf.id)));
#ifdef _DEBUG
	AKIDeleteUNICODEString(tempstr);
#else
	delete[] tempstr;
#endif
	return val;
}

//////////////////////////////////////////////////////////////////////////
//Check if ACC mode need to be intervened
//following the rule specified by the 
//////////////////////////////////////////////////////////////////////////
bool myVehicleDef::ACC_Manual_TakeOver_Check_CAMP(double& v_des)
{
	double headway = ((myVehicleDef*)leader)->getPositionReferenceVeh
		(leader->isUpdated() ? 1 : 0, this, 0);
	double spacing = headway - leader->getLength();

	double acc = (v_des - this->getSpeed()) / this->delta_t;  //acc from ACC model
	double leader_acc = 0;


	int current_state = 0;
	if (leader->isUpdated())
		current_state = 1;
	if (leader->getSpeed(current_state + 1) >= 0
		&& leader->getPosition(current_state + 1) > 0)
	{
		leader_acc = (leader->getSpeed(current_state) - leader->getSpeed(current_state + 1)) / delta_t;
	}
	else
	{
		leader_acc = 0;
	}

	//leader_acc = MAX(-2, leader_acc);
	//leader_acc = 0;
	leader_acc = 0; //Do not consider leader acceleration in the following algorithm

	leader_acc = leader_acc / 9.8;
	int POV_moving = 0;
	double leader_speed = ((myVehicleDef*) this->leader)->getSpeed();
	if (((myVehicleDef*) this->leader)->getSpeed() > 0)
		POV_moving = 1;

	if (leader_speed < 3)
	{
		// CAMP algorithm does not apply for the low speed case with the current CACC/ACC model_02/26/18
		return false;
	}

	double decREQ = -0.165 + 0.685*leader_acc + 0.080*POV_moving -
		0.00894776*(this->getSpeed() - leader_speed);  //Old equation according to Kiefer's report_Hao 04/19/17

	//New one:
	//double decREQ = -0.164 + 0.668*leader_acc + 0.078*POV_moving -
	//	0.00368*(this->getSpeed() - leader_speed)*2.23694;  //
	decREQ = decREQ*9.8;

	if (decREQ >= 0)
		return false;

	////The SV can at least decelerate at -4
	//decREQ = MIN(decREQ, -4);



	// Distance traveled by the subject vehicle before stop
	double R = this->getSpeed()*this->getSpeed() / (-2 * decREQ);
	//int contactstate = 0;  //case 1 when the leader is moving
	////first judge if collision is even possible
	//

	//double pos = (leader_speed*leader_speed)/(-2*leader_acc)); 
	//predict range
	leader_acc = leader_acc * 9.8;
	if (POV_moving)
	{
		//leader stationary when contact
		double R2 = MAX(0, (this->getSpeed()*this->getSpeed()) / (-2 * decREQ) - (leader_speed*leader_speed) / (-2 * leader_acc));
		//leader moving when contact
		double R1 = MAX(0, pow(this->getSpeed() - leader_speed, 2) / (-2 * (decREQ - leader_acc)));
		int CSR = CollisionScenario(spacing, leader_speed, this->getSpeed(), leader_acc, decREQ);

		if (CSR == COLLI_IMPOS)
		{
			return false;
		}
		else if (CSR == COLLI_SCE2)
		{
			R = R1;
		}
		else
		{
			R = R2;
		}
	}

	if (spacing < R)
	{
		v_des = this->getSpeed() + decREQ*delta_t;

		return true;
	}
	else
	{
		return false;
	}
}

int myVehicleDef::CollisionScenario(double spacing, double leader_speed,
	double follower_speed, double leader_acc, double decREQ)
{
	//simu in 20 sec
	double leaderpos = spacing;
	double followpos = 0;
	//this->IsSectionSource
	int num_steps = 50;
	if (this->IsSectionSource(this->getIdCurrentSection()))
	{
		num_steps = 200;
	}
	for (int i = 1; i < num_steps; i++)
	{
		double leader_speed_next = MAX(0, leader_speed + leader_acc*0.1);
		leaderpos += (leader_speed_next + leader_speed) / 2 * 0.1;
		leader_speed = leader_speed_next;

		double follower_speed_next = MAX(0, follower_speed + decREQ*0.1);
		followpos += (follower_speed_next + follower_speed) / 2 * 0.1;
		follower_speed = follower_speed_next;

		if (followpos >= leaderpos)
			return leader_speed > 0 ? COLLI_SCE2 : COLLI_SCE3;
		else if (leader_speed > follower_speed
			&&
			leader_acc > decREQ)
			return COLLI_IMPOS;
	}
	return COLLI_IMPOS;
}


void myVehicleDef::SetExtraHeadways()
{
	if (this->getVehType() != ACCveh&&
		this->getVehType() != CACCveh)
		return;
	if (this->getVehType() == ACCveh)
	{
		this->setACCHeadwayTime(this->desire_headway);  //from arrival new function in behavior class
		//then get manual headway
		this->setHeadwayTime(this->GenerateHeadway4Type(CARveh));
	}
	else if (this->getVehType() == CACCveh)
	{
		this->setCACCHeadwayTime(this->desire_headway);  //from arrival new function in behavior class
		//then set manual headway
		this->setHeadwayTime(this->GenerateHeadway4Type(CARveh));
		//then set acc headway
		this->setACCHeadwayTime(this->GenerateHeadway4Type(ACCveh));
	}
}

double myVehicleDef::sampleNormalDist(double mean, double std)
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

//////////////////////////////////////////////////////////////////////////
//set vehicle name attribute by its type
void myVehicleDef::setNameByType()
{
	if (this->getVehType() == CARveh)
	{
		SetNameAttribute("CAR");
		//SetMarkAttribute(0);
	}
	else if (this->getVehType() == ACCveh)
	{
		if (this->acc_2_manual == true)
			SetNameAttribute("ACC2M");
		//SetMarkAttribute(3);
		else
			SetNameAttribute("ACC");
		//SetMarkAttribute(1);

	}
	else if (this->getVehType() == CACCveh)
	{
		SetNameAttribute("CACC");
		//SetMarkAttribute(2);
	}
}

void myVehicleDef::SetNameAttribute(std::string param1)
{
	const unsigned short *tempstr =
		AKIConvertFromAsciiString("GKObject::nameAtt");
	ANGConnSetAttributeValueString(
		ANGConnGetAttribute(tempstr), this->getId(), (const unsigned short *)param1.c_str());
#ifdef _DEBUG
	AKIDeleteUNICODEString(tempstr);
#else
	delete[] tempstr;
#endif
}

void myVehicleDef::SetMarkAttribute(int param1)
{
	const unsigned short *tempstr =
		AKIConvertFromAsciiString("GKGeoObject::markAtt");
	ANGConnSetAttributeValueInt(
		ANGConnGetAttribute(tempstr), this->getId(), param1);
	AKIDeleteUNICODEString(tempstr);
}

double myVehicleDef::getDesireHeadway()
{
	if (this->getVehType() == NGSIM_CACC_ACC_V2V || this->getVehType() == CACC_HOV) //over write the desired headway if the vehicle is the new type vehicle in either ACC or CACC mode.
	{//this vehicle is CACC capable vehicle
		if (this->getMode() == CACC_ON || this->getMode() == CACC_ON_Follower_Speed_Regulation)
		{//if this vehicle is able to do CACC
			return CACC_CTG;
		}
		else if (this->getMode() == CACC_ON_Leader)
		{
			return CACC_platoon_CTG;
		}
		else if (this->getMode() == ACC_ON || this->getMode() == ACC_ON_CC || this->getMode() == CACC_ON_ACC)
		{//if this vehicle is in ACC mode, do ACC
			return ACC_CTG;
		}
		else
		{
			return (this->desire_headway > 0.5 ? this->desire_headway : 0.5);//human headway
		}

	}

	if (this->getVehType() == NGSIM_ACC || this->getVehType() == ACC_HOV)
	{//if this vehicle is a ACC vehicle
		if (this->getMode() == ACC_ON || this->getMode() == ACC_ON_CC)
		{//if this vehicle is in ACC mode, do ACC
			return ACC_CTG;
		}
		else
		{
			return (this->desire_headway > 0.5 ? this->desire_headway : 0.5);//human headway
		}
	}


	//depending on the type return different desired headways
	if (this->getVehType() != ACCveh &&
		this->getVehType() != CACCveh)
	{
		return (this->desire_headway > 0.5 ? this->desire_headway : 0.5);//human headway
	}
	else
	{
		//depending on the leader and running mode
		if (this->getVehType() == ACCveh)
		{
			if (this->adaptive_mode == ACCINMAN
				|| this->acc_2_manual)
			{
				return (this->desire_headway > 0.5 ? this->desire_headway : 0.5);//human headway
			}
			else
			{
				return (this->acc_desire_headway > 0.5 ? this->acc_desire_headway : 0.5);
			}
		}
		else
		{
			if (this->adaptive_mode == CACCINMAN
				|| this->acc_2_manual)
			{
				return (this->desire_headway > 0.5 ? this->desire_headway : 0.5);
			}
			else if (this->adaptive_mode == ACCMODE)
			{
				return (this->acc_desire_headway > 0.5 ? this->acc_desire_headway : 0.5);
			}
			else
			{
				return (this->cacc_desire_headway > 0.5 ? this->cacc_desire_headway : 0.5);
			}
		}
	}
}

double myVehicleDef::GenerateHeadway4Type(int vehTypeId)
{
	const unsigned short *min_headway_String =
		AKIConvertFromAsciiString("headway_min");
	double min_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(min_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(min_headway_String);
#else
	delete[] min_headway_String;
#endif

	const unsigned short *max_headway_String =
		AKIConvertFromAsciiString("headway_max");
	double max_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(max_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(max_headway_String);
#else
	delete[] max_headway_String;
#endif

	const unsigned short *dev_headway_String =
		AKIConvertFromAsciiString("headway_dev");
	double dev_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(dev_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(dev_headway_String);
#else
	delete[] dev_headway_String;
#endif

	const unsigned short *avg_headway_String =
		AKIConvertFromAsciiString("headway_mean");
	double avg_headway_time =
		ANGConnGetAttributeValueDouble(
		ANGConnGetAttribute(avg_headway_String), vehTypeId);
#ifdef _DEBUG
	AKIDeleteUNICODEString(avg_headway_String);
#else
	delete[] avg_headway_String;
#endif

	double headway_time = sampleNormalDist(
		avg_headway_time,
		dev_headway_time
		);
	headway_time = MIN(max_headway_time,
		MAX(headway_time, min_headway_time));
	return headway_time;
}

//////////////////////////////////////////////////////////////////////////
//for set up adaptive mode value when the vehicle is initially release to network
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::InitialCACCACCMode()
{
	if (this->getVehType() != ACCveh
		&&
		this->getVehType() != CACCveh)
		return;

	this->adaptive_mode = ACCMODE;

	if (this->leader != NULL
		&&
		this->leader->isFictitious() == false
		&&
		this->leader->getVehType() == CACCveh
		&&
		this->getVehType() == CACCveh)
		this->adaptive_mode = CACCMODE;
}

void myVehicleDef::SetWidth4Coloring()
{
	double width = 2;
	if (this->getVehType() != ACCveh
		&&
		this->getVehType() != CACCveh)
	{
		width = 2.05;
	}
	else
	{
		if (this->adaptive_mode == ACCMODE)
		{
			this->setLastAdaptiveMode(ACCMODE);
			if (this->getVehType() == CACCveh)
			{
				width = 2.15;
			}
			else
			{
				width = 2.25;
			}
		}
		else if (this->adaptive_mode == CACCMODE)
		{
			this->setLastAdaptiveMode(CACCMODE);
			width = 2.35;
		}
		else if (this->adaptive_mode == ACCINMAN)
		{
			this->setLastAdaptiveMode(ACCINMAN);
			width = 2.45;
		}
		else if (this->adaptive_mode == CACCINMAN)
		{
			this->setLastAdaptiveMode(CACCINMAN);
			width = 2.55;
		}
	}
	if (width != this->staticinfo.width)
		SetVehicleWidth(width);
}

void myVehicleDef::SetVehicleWidth(double width)
{
	//AKIVehSetAsTracked(this->getId());
	this->staticinfo.width = width;
	AKIVehSetStaticInf(this->VehID, this->staticinfo);

	//AKIVehSetAsNoTracked(this->getId());
}

int myVehicleDef::getLastAdaptiveMode()
{
	return this->last_adaptive_mode;
}

void myVehicleDef::setLastAdaptiveMode(int mode)
{
	this->last_adaptive_mode = mode;
}

//////////////////////////////////////////////////////////////////////////
// Record crash information
//////////////////////////////////////////////////////////////////////////
void myVehicleDef::RecordCrashInformation()
{
	int rep_id = ANGConnGetReplicationId();
	char str_tmp[1024] = "a";
	sprintf_s(str_tmp, 1024,
		"C:\\CACC_Simu_Data\\acc%u_cacc%u\\%u\\detector\\%s\\crashinfo.txt",
		(int)(acc_percent * 100), (int)(cacc_percent * 100), rep_id, parastr.c_str());

	FILE* file = fopen(str_tmp, "a");

	char str[1280];
	sprintf_s(str,
		"%.5f, %d, %d, %d, %d, %.5f, %.5f\n",
		AKIGetCurrentSimulationTime(), this->getId(), this->getVehType(),
		this->getIdCurrentSection(), this->getIdCurrentLane(),
		this->getPosition(), this->getSpeed());

	fprintf(file, str);
	fflush(file);
	fclose(file);
}

double myVehicleDef::getLengthCACC()
{
	return A2SimVehicle::getLength();
}

void myVehicleDef::SetVehTypeIDs(int CarTypeID, int HovTypeID, int TruckTypeID,
	int CACCTypeID, int ACCTypeID)
{
	this->CARveh = CarTypeID;
	this->HovTypeID = HovTypeID;
	this->Truck_Type = TruckTypeID;
	this->CACCveh = CACCTypeID;
	this->ACCveh = ACCTypeID;

	if ((getVehType() == ACCveh) || (getVehType() == CACCveh))
	{
		applyACC_CACC = true;
	}
	else{
		applyACC_CACC = false;
	}
}

double myVehicleDef::getPoliteness()
{
	if (this->adaptive_mode == CACCMODE
		||
		this->adaptive_mode == ACCMODE)
		return 0;
	else
		return politeness_;
}

myVehicleDef::PositionSpeed myVehicleDef::Run_ACC()
{
	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	double v_ref = 0;



	double des_a = 0;
	double a_new = 0;
	double v_new = 0;
	double pos_new = 0;

	double modified_ACC_gap = MAX(1.5, ACC_CTG*this->getSpeed());

	// With the current ACC gap regulation model, the CC model seems to be uneccesary.
	// The gap regulation model can give a similar acceleration when the subject vehicle needs to 
	// speed up to reach the free flow speed. Therefore, the CC model is not used here._Hao 03/07/18
	double leader_dis; 
	double leader_spd;
	double leader_length;
	//leader_dis = -1 * leader_dis;



	if (Leader == NULL)
	{
		leader_dis = 10000;
		leader_spd = 10000;
		leader_length = 0;
		v_ref = this->createFreeFlowSpeed(true);

	}
	else
	{
		leader_dis = ((myVehicleDef *)Leader)->getPositionReferenceVeh(this);
		leader_spd = ((myVehicleDef *)Leader)->getSpeed();
		leader_length = leader->getLength();

		// Linear function for the reference speed
		double vf = this->createFreeFlowSpeed(true);
		if (leader_spd > vf)
		{
			v_ref = vf;
		}
		else
		{
			double lowerDist = MAX(3, modified_ACC_gap); // 20;
			double higherDist = MAX(12, 43*ACC_CTG*this->getSpeed()); //120;
			if (leader_dis - leader_length <= lowerDist)
			{
				v_ref = leader_spd;
			}
			else if (leader_dis - leader_length > higherDist)
			{
				v_ref = vf;
			}
			else
			{
				v_ref = leader_spd + (leader_dis - leader_length - lowerDist)*(vf - leader_spd) / (higherDist - lowerDist);
			}
		}
	}

	des_a = 0.07*(leader_spd - this->getSpeed()) + 0.23*(leader_dis - modified_ACC_gap - leader_length);

	setMode(ACC_ON);

	a_new = des_a;
	VehKinematics(a_new, v_new, pos_new);
	if (v_new >= v_ref)
	{
		a_new = MIN(a_new, 0.4*(v_ref - this->getSpeed(0)));
		VehKinematics(a_new, v_new, pos_new);
	}

	myVehicleDef::PositionSpeed pos_speed;

	pos_speed = setNewPosition(pos_new, v_new);



	return pos_speed;

}






void myVehicleDef::VehKinematics(double &Accl, double &Spd, double &Pos)
{
	double Amax = 2;  //vehicle acceleration upper limit, depending on the performance of the vehicle.
	double BrakeMax = -4;   //when 4 wheels are fully brake without slip. Friction coefficient -> 0.9~1


	//double Amax = 1;  //vehicle acceleration upper limit, depending on the performance of the vehicle.
	//double BrakeMax = -2.8;   //when 4 wheels are fully brake without slip. Friction coefficient -> 0.9~1

	double dt = AKIGetSimulationStepTime();

	if (Accl > Amax)
	{
		// acceleration saturated.
		Accl = Amax;
		Spd = this->getSpeed() + Accl*dt;
		Pos = this->getPosition() + this->getSpeed()*dt + 0.5*Accl*dt*dt;
	}
	else if (Accl < 0)
	{
		//deceleration 
		if (this->getSpeed() == 0)
		{
			//**Physical model. Can only decelerate to velocity equals zero.
			Accl = 0;
			Spd = this->getSpeed();
			Pos = this->getPosition();
			//AKIPrintString("Warning! Desired to drive backward");
		}
		else if (Accl <= BrakeMax)
		{
			Accl = BrakeMax;
			Spd = MAX(this->getSpeed() + Accl*dt, 0); //**Physical model. Can only decelerate to velocity equals zero.
			Accl = (Spd - this->getSpeed()) / dt;
			Pos = this->getPosition() + this->getSpeed()*dt + 0.5*Accl*dt*dt;
			//AKIPrintString("Warning! Abnormal braking requirement********!!!!!!!!!");
		}
		else
		{
			Spd = MAX(this->getSpeed() + Accl*dt, 0); //**Physical model. Can only decelerate to velocity equals zero.
			Accl = (Spd - this->getSpeed()) / dt;
			Pos = this->getPosition() + this->getSpeed()*dt + 0.5*Accl*dt*dt;
		}
	}
	else
	{
		Spd = this->getSpeed() + Accl*dt;
		Pos = this->getPosition() + this->getSpeed()*dt + 0.5*Accl*dt*dt;
	}
	return;
}


double myVehicleDef::getIdDistanceToEnd(int VehID)
{
	A2KSectionInf MergeSection = AKIInfNetGetSectionANGInf(2919);
	double junction_length = 0.5;
	double MegeSection_end = 100;

	int MergeSectionID = 2919;
	int	NodeID = 3619;
	int UpSectionID = 3616;
	int RampSectionID = 3280;


	InfVeh Ego_inf = AKIVehGetInf(VehID);
	double Ego_dist = 0;
	if (Ego_inf.idSection == UpSectionID)
	{
		Ego_dist = Ego_inf.distance2End + MergeSection.length + junction_length - MegeSection_end;

	}
	else if (Ego_inf.idSection == RampSectionID)
	{
		Ego_dist = Ego_inf.distance2End + MergeSection.length + junction_length - MegeSection_end;
	}
	else if (Ego_inf.idJunction == NodeID) //On the junction
	{
		Ego_dist = Ego_inf.distance2End + MergeSection.length - MegeSection_end;
	}
	else if (Ego_inf.idSection == MergeSectionID) //On the auxiliary lane
	{
		Ego_dist = Ego_inf.distance2End - MegeSection_end;

	}
	else
	{
		Ego_dist = -1;
	}

	return Ego_dist;
}


void myVehicleDef::Read_leader_follower_data()
{

	int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());

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


	const A2SimVehicle *vehUp = NULL;
	const A2SimVehicle *vehDown = NULL;
	const A2SimVehicle *Leader = NULL;
	vehUp = this->left_follower;
	vehDown = this->left_leader;
	Leader = this->leader;

	if (this->left_leader == NULL || left_leader->isFictitious() == true)
	{
		const unsigned short *temp_string5 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_leader_dis");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string5), VehANGId, -1);
		AKIDeleteUNICODEString(temp_string5);

		const unsigned short *temp_string6 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_leader_spd");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string6), VehANGId, -1);
		AKIDeleteUNICODEString(temp_string6);

	}
	else
	{
		double left_leader_dis = ((myVehicleDef *)vehDown)->getPositionReferenceVeh(this);
		//left_leader_dis = left_leader_dis*-1;
		double left_leader_spd = ((myVehicleDef *)vehDown)->getSpeed();

		const unsigned short *temp_string5 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_leader_dis");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string5), VehANGId, left_leader_dis);
		AKIDeleteUNICODEString(temp_string5);

		const unsigned short *temp_string6 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_leader_spd");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string6), VehANGId, left_leader_spd);
		AKIDeleteUNICODEString(temp_string6);

	}

	if (this->left_follower == NULL || left_follower->isFictitious() == true)
	{
		const unsigned short *temp_string7 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_follower_dis");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string7), VehANGId, -1);
		AKIDeleteUNICODEString(temp_string7);

		const unsigned short *temp_string8 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_follower_spd");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string8), VehANGId, -1);
		AKIDeleteUNICODEString(temp_string8);
	}
	else
	{
		double left_follower_dis = this->getPositionReferenceVeh((myVehicleDef *)vehUp);
		double left_follower_spd = ((myVehicleDef *)vehUp)->getSpeed();

		const unsigned short *temp_string7 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_follower_dis");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string7), VehANGId, left_follower_dis);
		AKIDeleteUNICODEString(temp_string7);

		const unsigned short *temp_string8 =
			AKIConvertFromAsciiString("GKSimVehicle::real_left_follower_spd");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string8), VehANGId, left_follower_spd);
		AKIDeleteUNICODEString(temp_string8);
	}


	if (Leader == NULL || Leader->isFictitious() == true)
	{
		const unsigned short *temp_string9 =
			AKIConvertFromAsciiString("GKSimVehicle::leader_spd");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string9), VehANGId, -1);
		AKIDeleteUNICODEString(temp_string9);

		const unsigned short *temp_string10 =
			AKIConvertFromAsciiString("GKSimVehicle::leader_dis");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string10), VehANGId, -1);
		AKIDeleteUNICODEString(temp_string10);
	}
	else
	{
		double leader_dis = ((myVehicleDef *)Leader)->getPositionReferenceVeh(this);
		double leader_spd = ((myVehicleDef *)Leader)->getSpeed();
		//leader_dis = -1 * leader_dis;

		const unsigned short *temp_string9 =
			AKIConvertFromAsciiString("GKSimVehicle::leader_spd");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string9), VehANGId, leader_spd);
		AKIDeleteUNICODEString(temp_string9);

		const unsigned short *temp_string10 =
			AKIConvertFromAsciiString("GKSimVehicle::leader_dis");
		ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string10), VehANGId, leader_dis);
		AKIDeleteUNICODEString(temp_string10);
	}


	return;
}


void myVehicleDef::set_ACC_CTG()
{

	//ACC_CTG = 1.1; //Three options are 1.1 1.6 2.2
	double rand_num = AKIGetRandomNumber(); //random number between 0 to 1.	
	if (rand_num <= 0.504)
	{ //50.4%
		ACC_CTG = 1.1;
	}
	else if (rand_num <= 0.504 + 0.185)
	{ //18.5%
		ACC_CTG = 1.6;
	}
	else
	{ //30.1%set_CACC_CTG
		ACC_CTG = 2.2;
	}

	return;
}


void myVehicleDef::set_CACC_CTG()
{
	//ACC_CTG = 1.1; //Three options are 1.1 1.6 2.2
	double rand_num = AKIGetRandomNumber(); //random number between 0 to 1.	
	if (rand_num <= 0.57)
	{ //57%
		CACC_CTG = 0.6;
	}
	else if (rand_num <= 0.57 + 0.24)
	{ //24%
		CACC_CTG = 0.7;
	}
	else if (rand_num <= 0.57 + 0.24 + 0.07)
	{ //7%
		CACC_CTG = 0.9;
	}
	else
	{ //12%
		CACC_CTG = 1.1;
	}


	return;
}





//*****************************************************
//Extend the driver intention model (this->determineDrivingMode())
//Include the ACC mode in this driver intention in this model
//*****************************************************
int myVehicleDef::determineDrivingModePlusACC()
{
	int currentMode = getMode();

	//do not do any lane changes on node
	if (this->isCurrentLaneInNode() == true)
	{
		switch (currentMode)
		{//keep current driving mode or reset to CF mode if this vehicle intends to make a lane change.
		case BCF:   //before lane change
			return setMode(CF);
		default:
			return getMode();
		}
	}

	if (currentMode == ACC_ON_CC)
	{
		currentMode = ACC_ON;
	}

	switch (currentMode)
	{
	case CF:
		return DetermineLcOrMergeOrCoopOrACC();//decide if lane change /merge /cooperation/ACC_ON is desired
	case ACF:
		return Determine2ndLcAfterLc(); //decide if another lane change follows this lane change
	case CCF:
		return determineCoopOrLc(); //see if it is willing to yield or itself needs a change
	case RCF:
		return DetermineReceiveOrLcOrCoop(); //see if it is needs a change or Coop or doing RCF
	case BCF:
		return determineGapOrGiveup();
	case ACC_ON:
		return DetermineLcOrMergeOrCoopOrACC(); //decide if lane change /merge /cooperation is desirable
		//case ACC_ON_CC:
		//	return DetermineLcOrMergeOrCoopOrACC(); //decide if lane change /merge /cooperation is desirable
	default:
		return currentMode;
	}

}

int myVehicleDef::determineDrivingModePlusACC_CACC_V2XAHM()
{
	int currentMode = getMode();
	//throw back to ACC/CACC initial state
	if (currentMode == ACC_ON_CC)
	{
		currentMode = ACC_ON;
		this->setMode(ACC_ON);
	}

	if (currentMode == CACC_ON_Follower_Speed_Regulation || currentMode == CACC_ON_ACC)
	{
		currentMode = CACC_ON;
		this->setMode(CACC_ON);
	}

	//*****do not do any lane changes on node
	if (this->isCurrentLaneInNode() == true)
	{//this vehicle is on a node.
		switch (currentMode)
		{//keep current driving mode or reset to CF mode if this vehicle intends to make a lane change.
		case BCF:   //before lane change
			return setMode(CF);
		default:
			return getMode();
		}
	}

	currentMode = getMode();

	switch (currentMode)
	{
	case CF:
		currentMode = DetermineLcOrMergeOrCoopOrACC0rCACC_0729();//decide if lane change /merge /cooperation/ACC_ON is desired
		break;
	case ACF:
		currentMode = Determine2ndLcAfterLc_0729();
		break;
	case CCF:
		currentMode = determineCoopOrLc_0729(); //see if it is willing to yield or itself needs a change
		break;
	case RCF:
		currentMode = DetermineReceiveOrLcOrCoop_0729(); //see if it is needs a change or Coop or doing RCF
		break;
	case BCF:
		currentMode = determineGapOrGiveup_0729();
		break;
	case ACC_ON:
		currentMode = DetermineLcOrMergeOrCoopOrACC0rCACC_0729(); //decide if lane change /merge /cooperation is desirable
		break;
	case CACC_ON:
		currentMode = CACCDetermineLcOrCACC();   //driver can decide to disable CACC or keep turning on CACC
		break;
	default:
		currentMode = getMode();
		break;
	}

	return currentMode;

}


int myVehicleDef::determineDrivingModePlusACC_CACC()
{
	int currentMode = getMode();
	//****throw back to ACC/CACC initial state
	if (currentMode == ACC_ON_CC)
	{
		currentMode = ACC_ON;
		this->setMode(ACC_ON);
	}

	if (currentMode == CACC_ON_Follower_Speed_Regulation || currentMode == CACC_ON_ACC)
	{
		currentMode = CACC_ON;
		this->setMode(CACC_ON);
	}

	//*****do not do any lane changes on node
	if (this->isCurrentLaneInNode() == true)
	{//this vehicle is on a node.
		switch (currentMode)
		{//keep current driving mode or reset to CF mode if this vehicle intends to make a lane change.
		case BCF:   //before lane change
			return setMode(CACC_ON);
		default:
			return getMode();
		}
	}


	currentMode = getMode();

	switch (currentMode)
	{
	case CF:
		currentMode = DetermineLcOrMergeOrCoopOrACC0rCACC_0729();//decide if lane change /merge /cooperation/ACC_ON is desired
		break;
	case ACF:
		currentMode = Determine2ndLcAfterLc(); //decide if another lane change follows this lane change
		break;
	case CCF:
		currentMode = determineCoopOrLc(); //see if it is willing to yield or itself needs a change
		break;
	case RCF:
		currentMode = DetermineReceiveOrLcOrCoop(); //see if it is needs a change or Coop or doing RCF
		break;
	case BCF:
		currentMode = determineGapOrGiveup();
		break;
	case ACC_ON:
		currentMode = DetermineLcOrMergeOrCoopOrACC0rCACC_0729(); //decide if lane change /merge /cooperation is desirable
		break;
	case CACC_ON:
		currentMode = DetermineLcOrMergeOrCoopOrACC0rCACC_0729();   //driver can decide to disable CACC or keep turning on CACC
		break;
	default:
		currentMode = currentMode;
		break;
	}

	return currentMode;
}

int myVehicleDef::DetermineLcOrMergeOrCoopOrACC0rCACC_0729()
{
	double v_now = this->getSpeed();
	//first decide if lane change is necessary
	if (NeedCoop())
	{
		CACC_recover_time = 0;
		return this->setMode(CCF);
	}
	//if cooperation is needed
	else if (NeedLC())
	{
		CACC_recover_time = 0;
		return this->setMode(BCF);
	}
	else if (this->leader != NULL
		&&
		this->ACC_Manual_TakeOver_Check_CAMP(v_now))
	{
		//fail the CAMP safety criterion
		//keep manual driving mode.
		CACC_recover_time = 0;
		return this->setMode(CF);
	}
	else if (!this->IsSectionSource(this->getIdCurrentSection()) && CACC_recover_time < CACC_Recover_Time)
	{
		//CACC_escape();
		CACC_recover_time++;
		return this->setMode(CF);
	}
	else if (this->IsSectionSource(this->getIdCurrentSection()) && CACC_recover_time < 10)
	{
		//CACC_escape();
		CACC_recover_time++;
		return this->setMode(CF);
	}
	//else if (this->getVehType() == NGSIM_CACC_ACC_V2V
	//	&& this->approachingScenario == 4
	//	&& this->coordinationNeeded()
	//	&& this->sec_inf.length - this->getPosition(0) < 100)
	//{
	//	// Do not change status when approaching an cooperative signal
	//	return this->setMode(ACC_ON);
	//}
	else if (DetermineCACC()) //try CACC if driver has intentions other than Regular Car Following.
	{
		//CACC system is turned on.
		return this->setMode(CACC_ON);
	}
	else
	{
		// CACC vehicle without CACC leader
		return this->setMode(ACC_ON);
	}

}

int myVehicleDef::Determine2ndLcAfterLc_0729()
{
	//first decide if lane change is necessary


	if (NeedCoop())
	{
		return this->setMode(CCF);
	}
	else if (NeedLC())
	{
		return this->setMode(BCF);
	}
	else
	{
		if (isAfterLaneChangeFinish())
			return this->setMode(CF);
		else
			return getMode();
	}
}

int myVehicleDef::determineCoopOrLc_0729()
{
	if (NeedCoop())
	{
		return setMode(CCF);
	}
	else if (NeedLC())
	{
		return setMode(BCF);
	}
	else
	{
		return setMode(CF);
	}
}


int myVehicleDef::DetermineReceiveOrLcOrCoop_0729()
{
	//first decide if lane change is necessary
	if (NeedCoop())
	{
		return this->setMode(CCF);
	}
	else if (NeedLC())
	{
		return this->setMode(BCF);
	}
	/*else if (NeedMCF()!=0)
	{
	return this->setMode(MCF) ;
	}*/
	else
		return this->getMode();
}

int myVehicleDef::determineGapOrGiveup_0729()
{
	if (NeedCoop())
	{
		// Only cooperate to mandatory LC to the off-ramp, even if the subject vehicle needs to make a LC 
		// Otherwise the left leader will miss the turn
		return this->setMode(CCF);
	}
	else if (NeedLC() == true) //NeedLC
	{
		return setMode(BCF);
	}
	else
	{
		return setMode(CF);
	}
}

int myVehicleDef::CACCDetermineLcOrCACC()
{
	double v_now = this->getSpeed();

	if (this->leader != NULL
		&&
		this->ACC_Manual_TakeOver_Check_CAMP(v_now)
		)
	{
		//fail the CAMP safety criterion
		//keep manual driving mode.
		//CACC_escape();
		CACC_recover_time = 0;
		//if (this->getIdCurrentSection() != 3279 && this->getIdCurrentSection() != 4422)
		//{
		//	//Do not record in the first two sections
		//	RecordACC2Manual(1);
		//}
		return this->setMode(CF);

	}
	else if (NeedCoop())
	{
		CACC_recover_time = 0;
		//if (this->getIdCurrentSection() != 3279 && this->getIdCurrentSection() != 4422)
		//{
		//	//Do not record in the first two sections
		//	RecordACC2Manual(2);
		//}
		return this->setMode(CCF);
	}
	else if (NeedLC())
	{
		//CACC_escape();
		//if (this->getIdCurrentSection() != 3279 && this->getIdCurrentSection() != 4422)
		//{
		//	//Do not record in the first two sections
		//	if (this->getLCType() == OPTIONAL_LC)
		//	{
		//		RecordACC2Manual(3);
		//	}
		//	else
		//	{
		//		RecordACC2Manual(4);
		//	}

		//}
		CACC_recover_time = 0;
		return this->setMode(BCF);
	}
	else
	{
		if (!this->IsSectionSource(this->getIdCurrentSection()) && CACC_recover_time < CACC_Recover_Time)
		{
			//CACC_escape();
			CACC_recover_time++;
			return this->setMode(CF);
		}
		else if (this->IsSectionSource(this->getIdCurrentSection()) && CACC_recover_time < 10)
		{
			//CACC_escape();
			CACC_recover_time++;
			return this->setMode(CF);
		}
		else if (DetermineCACC())
		{
			return this->setMode(CACC_ON);
		}
		else
		{
			return this->setMode(ACC_ON);
		}

	}

}




myVehicleDef::PositionSpeed myVehicleDef::NGSIMPlusACC(bool mode_predetermined)
{
	myVehicleDef::PositionSpeed pos_speed;

	if (this->getId() == debug_track_id)
	{
		//AKIPrintString("pause1");
	}

	//here we let the determination of driving mode run every 0.2 second
	//to increase simulation speed
	int mode = getMode();


	// if mode has already been determined then just get the mode
	if (mode_predetermined == true
		||
		(int)(current_time * 10) % 2 != 1)
		mode = getMode();
	else
		mode = determineDrivingModePlusACC();  //update the driving mode every two time steps.

	if (mode == ACC_ON_CC)
	{
		mode = ACC_ON;
	}

	switch (mode)
	{
	case CF:         //Car following                  
		pos_speed = updateRegularCf();
		return pos_speed;
	case BCF:
		pos_speed = UpdateBeforeLaneChangeCf();
		return pos_speed;
	case ACF:
		pos_speed = UpdateAfterLaneChangeCf();
		return pos_speed;
	case CCF:
		pos_speed = updateCoopCf();
		return pos_speed;
	case LC:
		pos_speed = UpdateLc();
		return pos_speed;
	case RCF:
		pos_speed = UpdateReceiveCf();
		return pos_speed;
	case ACC_ON:
		pos_speed = Run_ACC();
		return pos_speed;
	default:
		break;
	}
}

myVehicleDef::PositionSpeed myVehicleDef::NGSIMPlusACC_CACC_V2VAHM(bool mode_predetermined)
{
	myVehicleDef::PositionSpeed pos_speed;

	if (this->getId() == debug_track_id)
	{
		char temp_a[200];
		sprintf(temp_a, "ID : %d, mode = %d ", this->getId(), mode);
		AKIPrintString(temp_a);
	}
	int mode = 0;
	mode = getMode();

	//If vehicles are in a sub-mode of CACC control, then return to main control mode to update driving mode. 
	if (mode == CACC_ON_Follower_Speed_Regulation || mode == CACC_ON_ACC || mode == CACC_ON_Leader)  //This sub-mode are for debugging purpose and also an attribute for a legend in Aimsun.
	{
		mode = this->setMode(CACC_ON);
	}
	//If vehicles are in a sub-mode of ACC control, then return to main control mode to update driving mode. 
	if (mode == ACC_ON_CC)
	{
		mode = this->setMode(ACC_ON);
	}

	//here we let the determination of driving mode run every 0.2 second
	//for the sake of simulation efficiency

	// if mode has already been determined then just get the mode
	if (mode_predetermined == true
		||
		(int)(current_time * 10) % 2 != 1)
		mode = getMode();
	else
	{
		int pre_mode = mode;
		mode = determineDrivingModePlusACC_CACC_V2XAHM();

		if (pre_mode == CACC_ON
			&& mode == ACC_ON)
		{
			// The CACC string leader does not switch mode within 100 meters of an intersection
			mode = pre_mode;
		}

		//Record CACC vehicle mode switch and CACC string length
		//Disable it to increase the simulation speed
		int modeSwitch = 0;
		if (pre_mode == CACC_ON)
		{
			if (mode == ACC_ON)
			{
				//Switched to ACC
				modeSwitch = 1;
			}
			else if (mode == BCF || mode == LC)
			{
				//Switched to LC
				if (this->getLCType() == OPTIONAL_LC)
				{
					modeSwitch = 2;
				}
				else
				{
					modeSwitch = 3;
				}
			}
			else if (mode == CCF)
			{
				//Switched to yielding
				modeSwitch = 4;
			}
			else if (mode == CF)
			{
				//Switched to manual CF
				modeSwitch = 5;
			}
		}

		if (modeSwitch > 1)
		{
			this->stringPosID = 0;
		}

		//Record information
		this->CACCModeSwitch = modeSwitch;
		//RecordACC2Manual(modeSwitch);

	}


	switch (mode)
	{
	case CF:         //Car following                  
		pos_speed = updateRegularCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case BCF:
		pos_speed = UpdateBeforeLaneChangeCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case ACF:
		pos_speed = UpdateAfterLaneChangeCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case CCF:
		pos_speed = updateCoopCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case LC:
		pos_speed = UpdateLc();
		CACC_clean_CACC_index();
		return pos_speed;
	case RCF:
		pos_speed = UpdateReceiveCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case ACC_ON:
		pos_speed = Run_ACC();
		CACC_clean_CACC_index();
		return pos_speed;
	case CACC_ON:
		pos_speed = Run_CACC();
		Update_PrecedingCACC_Len(); //Get the length of the preceding CACC platoon if this vehicle is a CACC leader.
		return pos_speed;
	default:
		break;
	}

}



myVehicleDef::PositionSpeed myVehicleDef::NGSIMPlusACC_CACC(bool mode_predetermined)
{
	myVehicleDef::PositionSpeed pos_speed;

	if (this->getId() == debug_track_id)
	{
		//char temp_a[200];
		//sprintf(temp_a,"ID : %d, mode = %d " ,this->getId(),mode);
		//AKIPrintString(temp_a);
	}
	int mode = 0;
	mode = getMode();

	//If vehicles are in a sub-mode of CACC control, then return to main control mode to update driving mode. 
	if (mode == CACC_ON_Follower_Speed_Regulation || mode == CACC_ON_ACC || mode == CACC_ON_Leader)  //This sub-mode are for debugging purpose and also an attribute for a legend in Aimsun.
	{
		mode = this->setMode(CACC_ON);
	}
	//If vehicles are in a sub-mode of ACC control, then return to main control mode to update driving mode. 
	if (mode == ACC_ON_CC)
	{
		mode = this->setMode(ACC_ON);
	}

	//here we let the determination of driving mode run every 0.2 second
	//for the sake of simulation efficiency

	// if mode has already been determined then just get the mode
	if (mode_predetermined == true
		||
		(int)(current_time * 10) % 2 != 1)
		mode = getMode();
	else
		mode = determineDrivingModePlusACC_CACC();


	switch (mode)
	{
	case CF:         //Car following                  
		pos_speed = updateRegularCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case BCF:
		pos_speed = UpdateBeforeLaneChangeCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case ACF:
		pos_speed = UpdateAfterLaneChangeCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case CCF:
		pos_speed = updateCoopCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case LC:
		pos_speed = UpdateLc();
		CACC_clean_CACC_index();
		return pos_speed;
	case RCF:
		pos_speed = UpdateReceiveCf();
		CACC_clean_CACC_index();
		return pos_speed;
	case ACC_ON:
		pos_speed = Run_ACC();
		CACC_clean_CACC_index();
		return pos_speed;
	case CACC_ON:
		pos_speed = Run_CACC();
		Update_PrecedingCACC_Len(); //Get the length of the preceding CACC platoon if this vehicle is a CACC leader.
		return pos_speed;
	default:
		break;
	}
}


bool myVehicleDef::DetermineCACC()  //check if the CACC system is enabled.
{
	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	bool CACC_flag = false;

	if (Leader == NULL)
	{//there's no leading vehicle ahead
		CACC_flag = true;  //CACC is not enabled
		CACC_clean_CACC_index();  //Set CACC index = 0
	}
	else if (Leader->isFictitious() == true)
	{
		// Signal ahead, use ACC mode
		CACC_flag = false;  //CACC is not enabled
		CACC_clean_CACC_index();  //Set CACC index = 0
	}
	else
	{
		if (Leader->getVehType() == NGSIM_CACC_ACC_V2V || Leader->getVehType() == CACC_HOV ||
			(Leader->getVehType() == CAR_Type && ((myVehicleDef*)Leader)->ConnectFlag) || 
			(Leader->getVehType() == HOVveh && ((myVehicleDef*)Leader)->ConnectFlag)
			)  //Check if the leader vehicle is also a CACC vehicle.
		{
			//preceding vehicle is a V2V CACC vehicle or VAD vehicle
			//Turn on CACC 
			CACC_flag = true;
		}
		else
		{
			//preceding vehicle is not a CACC vehicle
			//Cooperatively determine the sub-CACC-mode and CACC-index
			//do not join the preceding vehicle
			CACC_flag = false;  //CACC is not enabled
			CACC_clean_CACC_index();  //Set CACC index = 0
		}
	}

	return CACC_flag;
}

myVehicleDef::PositionSpeed myVehicleDef::Run_CACC_Follower() //Run CACC Follower control algorithm.
{//There are two sub-modes in a CACC_Follower mode: speed regulation and gap regulation.
	//Switch between these two sub-modes is based on time gap. When the time gap is larger than 2 seconds, the vehicle is in speed regulation mode.
	//When the time gap is shorter than 1.5 seconds, the vehicle is in gap regulation mode.
	//When the time gap is between 1.5 seconds and 2 seconds, a hysteresis kicked in, the vehicle is in a mode it was in.

	myVehicleDef::PositionSpeed pos_speed;

	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	if (Leader == NULL) // || Leader->isFictitious() == true)
	{
		//there's no preceding vehicle, this may be an exception.
		//get the number of this vehicle in the platoon.

		pos_speed = CACC_CC();
	}
	else
	{
		pos_speed = CACC_veh_model();
	}



	return pos_speed;
}

myVehicleDef::PositionSpeed myVehicleDef::Run_CACC_Leader()
{
	myVehicleDef::PositionSpeed pos_speed;

	//do the ACC following the preceding vehicle.
	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	double v_ref = MIN(120 / 3.6, this->createFreeFlowSpeed(true));
	double des_a = 0;
	double a_new = 0;
	double v_new = 0;
	double pos_new = 0;

	if (Leader == NULL) // || Leader->isFictitious() == true)//if exists no leading vehicle, then limit the AHM acceleration upper limit.
	{//Speed Control Mode.
		des_a = MIN(2, v_ref - this->getSpeed());
		this->setMode(CACC_ON_Follower_Speed_Regulation);
	}
	else
	{	//exists leading vehicle, do the car following ACC.
		double leader_dis = ((myVehicleDef *)Leader)->getPositionReferenceVeh(this);
		double leader_spd = ((myVehicleDef *)Leader)->getSpeed();
		//leader_dis = -1 * leader_dis;

		if (leader_dis >= 120) //exists leading vehicle but it is far away, out of the sensor range (radar range 100m).
		{//keep doing CC
			des_a = 0.4*(v_ref - this->getSpeed());
			setMode(CACC_ON_Follower_Speed_Regulation);
			ACC_state = ACC_ON_CC;
		}
		else if (leader_dis <= 100)
		{//do ACC
			//des_a = (1/ACC_CTG)*(leader_spd-this->getSpeed())+0.23*(leader_dis - ACC_CTG*this->getSpeed()-1);
			des_a = 0.07*(leader_spd - this->getSpeed()) + 0.23*(leader_dis - ACC_CTG*this->getSpeed());
			setMode(CACC_ON_ACC);
			ACC_state = ACC_ON;
		}
		else
		{//preceding gap length is between 100m and 120m
			//doing gap regulation if previously doing gap regulation
			//doing speed control if previously doing speed control
			if (ACC_state == ACC_ON_CC)
			{//if previous mode is CC mode
				des_a = MIN(2, 0.4*(v_ref - this->getSpeed()));
				setMode(CACC_ON_Follower_Speed_Regulation);
			}
			else
			{//if previous mode is CACC regulation mode.
				des_a = 0.07*(leader_spd - this->getSpeed()) + 0.23*(leader_dis - ACC_CTG*this->getSpeed());
				setMode(CACC_ON_ACC);
			}

		}
	}

	a_new = MIN(des_a, 2);
	VehKinematics(a_new, v_new, pos_new);

	if (v_new >= v_ref)
	{	//Do not allow vehicle driving over the speed limit.
		a_new = v_ref - this->getSpeed();
		VehKinematics(a_new, v_new, pos_new);
	}

	pos_speed = setNewPosition(pos_new, v_new);

	return pos_speed;


}

myVehicleDef::PositionSpeed myVehicleDef::CTG_CACC_update()
{
	myVehicleDef::PositionSpeed pos_speed;

	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());
	const unsigned short *temp1 = AKIConvertFromAsciiString("GKSimVehicle::CACCPlatoonNum");
	int num_platoon = ANGConnGetAttributeValueInt(ANGConnGetAttribute(temp1), VehANGId);   //the number of the vehicle ahead in the platoon.  
	AKIDeleteUNICODEString(temp1);

	if (Leader == NULL) // || Leader->isFictitious() == true)
	{//this vehicle may be the leader of the CACC 
		//get the number of this vehicle in the platoon.
		if (num_platoon == 0)
		{//this vehicle is the leader of the CACC platoon.
			//do CACC_cruise control 
			pos_speed = CACC_CC();
			this->setMode(CACC_ON_Follower_Speed_Regulation);
		}
		else
		{//this vehicle is in the CACC platoon and somehow can not find the preceding vehicle.
			InfVeh sjveh_inf = AKIVehGetInf(this->getId());
			if (sjveh_inf.idSection == 3612 && sjveh_inf.distance2End <= 40)
			{//leading vehicle vanished at the destination centroid.
				pos_speed = CACC_CC();
				this->setMode(CACC_ON_Follower_Speed_Regulation);
			}
			else
			{//exception case
				//this vehicle is in the platoon but the leading vehicle somehow disappeared
				char temp_a[200];
				sprintf(temp_a, "Error ID : %d", this->getId());
				AKIPrintString(temp_a);
				pos_speed = CACC_CC();
				this->setMode(CACC_ON_Follower_Speed_Regulation);
			}
		}

	}
	else
	{//exists a preceding vehicle
		if (num_platoon == 0)
		{//this vehicle is the leader of the CACC platoon.
			//do ACC 
			//CTG_ACC_update();
			pos_speed = Run_ACC();
		}
		else
		{//this vehicle is the follower in the CACC platoon.
			double spd_pre = ((myVehicleDef *)Leader)->getSpeed();
			double dis_pre = ((myVehicleDef *)Leader)->getPositionReferenceVeh(this);
			//dis_pre = -1 * dis_pre;


			//gap closing if distance is larger than twice the desired Constant-Time-GAP
			if (dis_pre > spd_pre * 2 * 0.6)
			{//distance is larger than a threshold
				pos_speed = CACC_veh_model(); //temporarily use CACC model to close the gap.
				this->setMode(CACC_ON_Follower_Speed_Regulation);
			}
			else
			{//distance is shorter than a threshold
				pos_speed = CACC_veh_model();
				this->setMode(CACC_ON);
			}
		}
	}

	return pos_speed;
}


myVehicleDef::PositionSpeed myVehicleDef::CACC_veh_model()  //CACC car simulation following model based on real experiment
{//from V.Milanes & Steven 2014.
	myVehicleDef::PositionSpeed pos_speed;

	//model parameters 
	double kp = 0.45;
	double dt = AKIGetSimulationStepTime();
	//double kd = 0.0125 / dt;
	double kd = 0.0125;

	double des_a = 0;
	double a_new = 0;
	double v_new = 0;
	double pos_new = 0;

	double Vk = 0;

	

	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	//preceding vehicle position & speed
	double spd_pre = ((myVehicleDef *)Leader)->getSpeed();  //preceding vehicle speed
	double dis_pre = ((myVehicleDef *)Leader)->getPositionReferenceVeh(this);
	dis_pre = dis_pre - leader->getLength();   //distance to preceding vehicle.

	double v_ref = this->createFreeFlowSpeed(false);  //driver desired driving speed.

	double leader_length = leader->getLength();
	double vf = this->createFreeFlowSpeed(false);

	if (this->CACCStringLeaderFollowerSwitch == 1)
	{
		// The subject vehicle just switched from leader to follower, do not let it accelerate too aggresively
		v_ref = MAX(this->getSpeed(0), spd_pre + 2);
	}

	// Linear function for the reference speed
	if (spd_pre <= v_ref)
	{
		double lowerDist = MAX(3, CACC_CTG*this->getSpeed()); // 20;
		double higherDist = MAX(12, 4 * CACC_CTG*this->getSpeed()); //120;
		if (dis_pre - leader_length <= lowerDist)
		{
			v_ref = spd_pre;
		}
		else if (dis_pre - leader_length > higherDist)
		{
			v_ref = vf;
		}
		else
		{
			v_ref = spd_pre + (dis_pre - leader_length - lowerDist)*(vf - spd_pre) / (higherDist - lowerDist);
		}
	}




	double acceleration = (this->getSpeed(0) - this->getSpeed(1)) / dt;

	double TG = CACC_CTG + (CACC_platoon_CTG - CACC_CTG)*this->CACC_mode_switch_time / CACC_Relaxation_Time;

	double distance_gap = MAX(1.5, TG*this->getSpeed());  //Clip the distance gap at 5 meters.
	double ek = dis_pre - TG*this->getSpeed();  //time gap error 
	double ek_dot = spd_pre - this->getSpeed() - TG*acceleration; //derivative of time gap error


	 Vk = this->getSpeed(0) + kp*ek + kd*ek_dot;


	//speed limit while doing CACC
	if (Vk > v_ref)
	{//command speed is too fast.
		Vk = v_ref;
	}

	des_a = (Vk - this->getSpeed()) / dt;  //convert model speed into acceleration command.

	des_a = (des_a + acceleration) / 2;
	a_new = des_a;

	VehKinematics(a_new, v_new, pos_new);
	pos_speed = setNewPosition(pos_new, v_new);
	return pos_speed;
}


myVehicleDef::PositionSpeed myVehicleDef::CACC_CC()
{//doing speed regulation in CACC mode
	myVehicleDef::PositionSpeed pos_speed;

	double v_ref = MIN(140 / 3.6, this->createFreeFlowSpeed(false));
	double a_new = 0;
	double v_new = 0;
	double pos_new = 0;

	double dis_pre;
	double spd_pre;

	if (this->leader != NULL) // && !this->leader->isFictitious())
	{
		dis_pre = ((myVehicleDef *)this->leader)->getPositionReferenceVeh(this);
		spd_pre = ((myVehicleDef *)this->leader)->getSpeed();
		dis_pre = dis_pre - leader->getLength();
		double leader_length = leader->getLength();
		double vf = this->createFreeFlowSpeed(false);
		// Linear function for the reference speed
		if (spd_pre <= v_ref)
		{
			double lowerDist = MAX(3, CACC_CTG*this->getSpeed()); // 20;
			double higherDist = MAX(12, 4 * CACC_CTG*this->getSpeed()); //120;
			if (dis_pre - leader_length <= lowerDist)
			{
				v_ref = spd_pre;
			}
			else if (dis_pre - leader_length > higherDist)
			{
				v_ref = vf;
			}
			else
			{
				v_ref = spd_pre + (dis_pre - leader_length - lowerDist)*(vf - spd_pre) / (higherDist - lowerDist);
			}
		}
	}



	a_new = 0.4*(v_ref - this->getSpeed(0));
	VehKinematics(a_new, v_new, pos_new);

	pos_speed = setNewPosition(pos_new, v_new);

	return pos_speed;

}


myVehicleDef::PositionSpeed myVehicleDef::CACC_fixed_timegap()
{//from V.Milanes & Steven 2014.
	myVehicleDef::PositionSpeed pos_speed;

	double kp = 0.45;   //parameters setting 
	double dt = AKIGetSimulationStepTime();
	//double kd = 0.0125 / dt;  //It used to be 0.45, showed on paper, but it is wrong.
	double kd = 0.0125;

	double des_a = 0;
	double a_new = 0;
	double v_new = 0;
	double pos_new = 0;

	double Vk = 0;

	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	//preceding vehicle position & speed
	double spd_pre = ((myVehicleDef *)Leader)->getSpeed();  //preceding vehicle speed
	double dis_pre = ((myVehicleDef *)Leader)->getPositionReferenceVeh(this);
	dis_pre = dis_pre - leader->getLength();   //distance to preceding vehicle. 

	double v_ref = this->createFreeFlowSpeed(false);  //driver desired driving speed.

	double leader_length = leader->getLength();
	double vf = this->createFreeFlowSpeed(false);
	// Linear function for the reference speed
	if (spd_pre <= v_ref)
	{
		double lowerDist = MAX(3, CACC_CTG*this->getSpeed()); // 20;
		double higherDist = MAX(12, 4 * CACC_CTG*this->getSpeed()); //120;
		if (dis_pre - leader_length <= lowerDist)
		{
			v_ref = spd_pre;
		}
		else if (dis_pre - leader_length > higherDist)
		{
			v_ref = vf;
		}
		else
		{
			v_ref = spd_pre + (dis_pre - leader_length - lowerDist)*(vf - spd_pre) / (higherDist - lowerDist);
		}
	}

	double acceleration = (this->getSpeed(0) - this->getSpeed(1)) / dt;

	double TG = CACC_platoon_CTG + (CACC_CTG - CACC_platoon_CTG)*this->CACC_mode_switch_time / CACC_Relaxation_Time;

	double ek = dis_pre - TG*this->getSpeed();  //time gap error 
	double ek_dot = spd_pre - this->getSpeed() - TG*acceleration; //derivative of time gap error

	 Vk = this->getSpeed(0) + kp*ek + kd*ek_dot;


	//speed limit while doing CACC
	if (Vk > v_ref)
	{//command speed is too fast.
		Vk = v_ref;
	}

	des_a = (Vk - this->getSpeed()) / dt;

	des_a = (des_a + acceleration) / 2;
	a_new = des_a;

	VehKinematics(a_new, v_new, pos_new);
	pos_speed = setNewPosition(pos_new, v_new);   //Set the new speed and position
	return pos_speed;
}


myVehicleDef::PositionSpeed myVehicleDef::CTG_CACC_update(double spd_pre, double dis_pre, double MakeLaneChange)
{//from V.Milanes & Steven 2014.
	myVehicleDef::PositionSpeed pos_speed;

	double kp = 0.45;
	double kd = 0.25;
	double dt = 0.1;

	double des_a = 0;
	double a_new = 0;
	double v_new = 0;
	double pos_new = 0;

	double Vk = 0;

	double v_ref = MIN(120 / 3.6, this->createFreeFlowSpeed(true));  //driver desired driving speed.
	v_ref = 120 / 3.6;

	//const A2SimVehicle *Leader = NULL;
	//Leader = this->leader;

	//preceding vehicle position & speed
	//double spd_pre = ((myVehicleDef *)Leader)->getSpeed();  //preceding vehicle speed
	//double dis_pre = this->getPositionReferenceVeh((myVehicleDef *)Leader);
	//dis_pre = -1*dis_pre;   //distance to preceding vehicle.

	double acceleration = (this->getSpeed(0) - this->getSpeed(1)) / dt;
	//int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());
	//const unsigned short *temp1 = AKIConvertFromAsciiString("GKSimVehicle::accelerationAtt");
	//acceleration = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(temp1), VehANGId);  //current subject vehicle acceleration 
	//AKIDeleteUNICODEString(temp1);

	double ek = dis_pre - CACC_CTG*this->getSpeed() - leader->getLength();
	double ek_dot = spd_pre - this->getSpeed() - CACC_CTG*acceleration;

	Vk = this->getSpeed() + kp*ek + kd*ek_dot;

	//speed limit while doing CACC
	if (Vk > v_ref)
	{//speed is too fast.
		Vk = v_ref;
	}

	des_a = (Vk - this->getSpeed()) / dt;

	des_a = (des_a + acceleration) / 2;
	a_new = des_a;

	VehKinematics(a_new, v_new, pos_new);

	if (MakeLaneChange)
	{
		//this->applyLaneChanging(left_leader, 1, pos_new, v_new);
		pos_speed = UpdateLc(left_leader, LEFT, pos_new, v_new);
	}
	else
	{
		pos_speed = setNewPosition(pos_new, v_new);
	}
	return pos_speed;

}


void myVehicleDef::update_Following_Veh_ID()
{//Pass subject vehicle's information to the Leader. The vehicles will have access to following vehicle, then. 
	//Save Follower ID, position and speed in subject vehicle's attributes. Aimsun does not have built-in function to access such information.

	myVehicleDef *Leader = (myVehicleDef *)this->leader;

	if (Leader == NULL || Leader->isFictitious() == true)  //There's no leader ahead. Do not need to save this information.
	{
		double dis_pre = -1;
		double leader_speed = -1;

		//int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());
		//const unsigned short *temp_string13 = AKIConvertFromAsciiString("GKSimVehicle::Dist_to_Leader");
		//ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string13), VehANGId, dis_pre);
		//AKIDeleteUNICODEString(temp_string13);

		//const unsigned short *temp_string14 = AKIConvertFromAsciiString("GKSimVehicle::Speed_Leader");
		//ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string14), VehANGId, leader_speed);
		//AKIDeleteUNICODEString(temp_string14);

		this->distanceToLeader = dis_pre;
		this->speedLeader = leader_speed;
	}
	else
	{
		//int LeadVehANGId = ANGConnVehGetGKSimVehicleId(Leader->getId());
		//const unsigned short *temp_string12 = AKIConvertFromAsciiString("GKSimVehicle::Following_Vehicle_ID");
		//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string12), LeadVehANGId, this->getId());
		//AKIDeleteUNICODEString(temp_string12);

		Leader->followerID = this->getId();

		double dis_pre = Leader->getPositionReferenceVeh(this);
		double leader_speed = Leader->getSpeed();

		//int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());
		//const unsigned short *temp_string13 = AKIConvertFromAsciiString("GKSimVehicle::Dist_to_Leader");
		//ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string13), VehANGId, dis_pre);
		//AKIDeleteUNICODEString(temp_string13);

		//const unsigned short *temp_string14 = AKIConvertFromAsciiString("GKSimVehicle::Speed_Leader");
		//ANGConnSetAttributeValueDouble(ANGConnGetAttribute(temp_string14), VehANGId, leader_speed);
		//AKIDeleteUNICODEString(temp_string14);

		this->distanceToLeader = dis_pre;
		this->speedLeader = leader_speed;

	}

	return;
}

void myVehicleDef::update_Driving_Mode()  //Update an Aimsun attribute, which represents driving mode(including sub-mode) of a vehicle.
{
	if (this->getId() == debug_track_id)
	{
		//AKIPrintString("pause1");
	}

	int current_mode = getMode();

	int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());
	const unsigned short *temp_string12 = AKIConvertFromAsciiString("GKSimVehicle::DrivingMode");
	//Save current_mode in an attribute "DrivingMode".
	//This attribute is for coloring of different driving mode.
	ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp_string12), VehANGId, current_mode);
	AKIDeleteUNICODEString(temp_string12);

	return;
}

myVehicleDef::PositionSpeed myVehicleDef::Run_CC()   //cruise control following a desired constant speed
{
	myVehicleDef::PositionSpeed pos_speed;

	double v_ref = MIN(120 / 3.6, this->createFreeFlowSpeed(true));
	double des_a = 0;
	double a_new = 0;
	double v_new = 0;
	double pos_new = 0;

	double leader_length = 4;

	des_a = 0.4*(v_ref - this->getSpeed());


	a_new = MIN(des_a, 2);
	VehKinematics(a_new, v_new, pos_new);

	if (v_new >= v_ref)
	{	//Do not allow vehicle driving over the speed limit.
		a_new = v_ref - this->getSpeed();
		VehKinematics(a_new, v_new, pos_new);
	}

	pos_speed = setNewPosition(pos_new, v_new);

	return pos_speed;
}

int myVehicleDef::DetemineVCACCOrCF()
{//Cancel virtual CACC following strategy if there's a vehicle cut-in the virtual string.

	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	double v_now = this->getSpeed();

	if (this->leader != NULL
		&&
		this->ACC_Manual_TakeOver_Check_CAMP(v_now)
		)
	{//fail the CAMP safety criterion
		//keep manual driving mode.
		//setLaneChangeDesireThrd(0.5);
		return this->setMode(CF);
	}
	else
	{
		return this->setMode(CACC_ON);


	}

}




void myVehicleDef::Update_PrecedingCACC_Len()
{	//If this vehicle is a CACC leader, then get the length of the CACC platoon ahead.
	myVehicleDef *Leader = (myVehicleDef *) this->leader;

	//int VehANGId = ANGConnVehGetGKSimVehicleId(this->getId());


	if (Leader != NULL && Leader->isFictitious() == false) //exists a leading vehicle.
	{
		if (this->getMode() == CACC_ON_Leader) //this vehicle is the leader of a CACC platoon
		{
			//int LeaderANGId = ANGConnVehGetGKSimVehicleId(Leader->getId());
			//const unsigned short *temp1 = AKIConvertFromAsciiString("GKSimVehicle::CACCPlatoonNum");
			//int num_leader = ANGConnGetAttributeValueInt(ANGConnGetAttribute(temp1), LeaderANGId);
			//AKIDeleteUNICODEString(temp1);

			int num_leader = Leader->stringPosID;

			//const unsigned short *temp6 = AKIConvertFromAsciiString("GKSimVehicle::PrecdeCACCLength");
			//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp6), VehANGId, num_leader);
			//AKIDeleteUNICODEString(temp6);

			this->PrecedingStringLength = num_leader;
		}
		else
		{
			//this vehicle is not a leader of a CACC platoon.

			//const unsigned short *temp6 = AKIConvertFromAsciiString("GKSimVehicle::PrecdeCACCLength");
			//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp6), VehANGId, -1);
			//AKIDeleteUNICODEString(temp6);

			this->PrecedingStringLength = -1;
		}
	}
	else
	{
		//this vehicle has no leader.

		//const unsigned short *temp6 = AKIConvertFromAsciiString("GKSimVehicle::PrecdeCACCLength");
		//ANGConnSetAttributeValueInt(ANGConnGetAttribute(temp6), VehANGId, -1);
		//AKIDeleteUNICODEString(temp6);

		this->PrecedingStringLength = -1;
	}


	return;
}


myVehicleDef::PositionSpeed myVehicleDef::Run_CACC()
{
	myVehicleDef::PositionSpeed pos_speed;

	if (debug_track_id == this->getId())
	{
		debug_track_id = this->getId();
	}

	const A2SimVehicle *Leader = NULL;
	Leader = this->leader;

	int mode = this->CACC_Mode; //CACC mode in previous step

	int CACC_mode_new = this->CACC_mode_switch();  //Switch between leader/follower depending on its position in a CACC platoon.
	
	if ((this->CACC_Mode == CACC_ON || this->CACC_Mode == CACC_ON_ACC || this->CACC_Mode == CACC_ON_Follower_Speed_Regulation)
		&& (mode == CACC_ON_Leader || mode == CACC_ON_Leader_Gap_Regulation || mode == CACC_ON_Leader_Speed_Regulation)
		&& this->sec_inf.length - this->getPosition(0) < 200)
	{
		// The CACC string leader does not switch mode within 100 meters of an intersection
		this->CACC_Mode = CACC_ON_Leader_Gap_Regulation;
		this->setMode(CACC_ON_Leader);
		CACC_mode_new = CACC_ON_Leader_Gap_Regulation;
		this->stringPosID = 0;
	}

	if ((mode == CACC_ON || mode == CACC_ON_ACC || mode == CACC_ON_Follower_Speed_Regulation)
		&& (this->CACC_Mode == CACC_ON_Leader || this->CACC_Mode == CACC_ON_Leader_Gap_Regulation || this->CACC_Mode == CACC_ON_Leader_Speed_Regulation))
	{
		//Relaxation when CACC vehicles switch between leader and follower mode_Hao 05/11/17
		this->CACC_mode_switch_time = CACC_Relaxation_Time;
		this->CACCStringLeaderFollowerSwitch = 2;
	}
	else if (mode == CACC_ON_Leader_Speed_Regulation && this->CACC_Mode == CACC_ON_Leader_Gap_Regulation)
	{
		this->CACC_mode_switch_time = CACC_Relaxation_Time;
		this->CACCStringLeaderFollowerSwitch = 1;
	}
	else if ((this->CACC_Mode == CACC_ON || this->CACC_Mode == CACC_ON_ACC || this->CACC_Mode == CACC_ON_Follower_Speed_Regulation)
		&& (mode == CACC_ON_Leader || mode == CACC_ON_Leader_Gap_Regulation || mode == CACC_ON_Leader_Speed_Regulation))
	{
		this->CACC_mode_switch_time = CACC_Relaxation_Time;
		this->CACCStringLeaderFollowerSwitch = 1;
	}
	else
	{
		if (this->CACC_mode_switch_time > 0)
		{
			this->CACC_mode_switch_time = this->CACC_mode_switch_time - 1;
		}
		else
		{
			this->CACC_mode_switch_time = 0;
			this->CACCStringLeaderFollowerSwitch = 0;
		}
	}


	if (CACC_mode_new == CACC_ON_Leader_Gap_Regulation)   //CACC leader in gap regulation mode
	{
		pos_speed = CACC_fixed_timegap();
	}
	else if (CACC_mode_new == CACC_ON_Leader_Speed_Regulation)  //CACC leader in speed regulation mode
	{
		pos_speed = CACC_CC();
	}
	else
	{
		const A2SimVehicle *Leader = NULL;
		Leader = this->leader;

		if (Leader == NULL) // || Leader->isFictitious() == true)
		{

			pos_speed = CACC_CC();
		}
		else
		{
			pos_speed = CACC_veh_model();
		}  //CACC follower
	}

	

	return pos_speed;
}






int myVehicleDef::CACC_mode_switch() //Switch between leader/follower depending on its position in a CACC platoon.
{
	int CACC_mode_new;
	//check the length of the CACC
	int CACCLenLimit = getCACC_length_limit();
	//CACCLenLimit = getCACC_length_limit_byLane();
	int lengthCACC = -1;

	myVehicleDef *Leader = (myVehicleDef *)(this->leader);

	// A simpler function to determine CACC mode_Hao 03/07/18
	if (Leader == NULL)
	{
		if (this->nextsec == NULL)
		{

			lengthCACC = this->stringPosID;

			//CACC vehicle does not change mode when exiting the network_Hao 4/24/17
			if (lengthCACC == 0)
			{
				CACC_mode_new = CACC_ON_Leader_Speed_Regulation;
				this->setMode(CACC_ON_Leader);
				this->CACC_Mode = CACC_ON_Leader_Speed_Regulation;
			}
			else
			{
				CACC_mode_new = this->getMode();
				this->CACC_Mode = this->getMode();
			}
		}
		else
		{
			//No leader exists. This vehicle itself is a leader of a CACC platoon.
			//CACC_mode = this->getMode();
			//this->CACC_Mode = this->getMode();

			CACC_mode_new = CACC_ON_Leader_Speed_Regulation;
			this->setMode(CACC_ON_Leader);
			this->CACC_Mode = CACC_ON_Leader_Speed_Regulation;

			this->stringPosID = 0;
		}

	}
	else
	{
		lengthCACC = Leader->stringPosID;
		double dis_pre = Leader->getPositionReferenceVeh(this);  //Get the distance to the preceding vehicle
		dis_pre = dis_pre - Leader->getLength();

		if (dis_pre > ACC_Detection_Range_Upper)
		{
			// The preceding vehicle is far away, use speed regulation
			CACC_mode_new = CACC_ON_Leader_Speed_Regulation;
			this->setMode(CACC_ON_Leader);
			this->CACC_Mode = CACC_ON_Leader_Speed_Regulation;

			this->stringPosID = 0;
		}
		else if (dis_pre > ACC_Detection_Range_Lower)
		{
			// Introduce hysteresis
			if (this->CACC_Mode == CACC_ON_Leader_Speed_Regulation)
			{
				CACC_mode_new = CACC_ON_Leader_Speed_Regulation;
				this->setMode(CACC_ON_Leader);
				this->CACC_Mode = CACC_ON_Leader_Speed_Regulation;
				this->stringPosID = 0;
			}
			else if (this->CACC_Mode == CACC_ON_Leader_Gap_Regulation)
			{
				CACC_mode_new = CACC_ON_Leader_Gap_Regulation;
				this->setMode(CACC_ON_Leader);
				this->CACC_Mode = CACC_ON_Leader_Gap_Regulation;

				this->stringPosID = 0;
			}
			else
			{
				CACC_mode_new = CACC_ON;
				this->setMode(CACC_ON);
				this->CACC_Mode = CACC_ON;
				this->stringPosID = lengthCACC + 1;
			}
		}
		else
		{
			if (lengthCACC < 0)  //If the preceding vehicle is not in a CACC platoon, it's index is -1
			{
				if (Leader->getVehType() == CAR_Type)
				{// The proceeding vehicle is a VAD vehicle 03/07/17 Hao
					
					CACC_mode_new = CACC_ON;
					this->setMode(CACC_ON);
					this->CACC_Mode = CACC_ON;
					Leader->stringPosID = 0;
					this->stringPosID = 1;
				}
				else
				{
					CACC_mode_new = CACC_ON_Leader_Speed_Regulation;
					this->setMode(CACC_ON_Leader);
					this->CACC_Mode = CACC_ON_Leader_Speed_Regulation;

					this->stringPosID = 0;
				}

			}
			else
			{
				if (lengthCACC >= CACCLenLimit - 1)  //if the length of the preceding CACC platoon is longer than constraint, than starts a new CACC platoon.
				{
					//if length is too long, then do the CACC leader (CACC with fixed longer gap)
					CACC_mode_new = CACC_ON_Leader_Gap_Regulation;
					this->setMode(CACC_ON_Leader);
					this->CACC_Mode = CACC_ON_Leader_Gap_Regulation;

					this->stringPosID = 0;
				}
				else if (Leader->getVehType() == CAR_Type)
				{// The proceeding vehicle is a VAD vehicle 03/07/17 Hao
					CACC_mode_new = CACC_ON;
					this->setMode(CACC_ON);
					this->CACC_Mode = CACC_ON;
					Leader->stringPosID = 0;
					this->stringPosID = 1;
				}
				else
				{
					CACC_mode_new = CACC_ON;
					this->setMode(CACC_ON);
					this->CACC_Mode = CACC_ON;
					this->stringPosID = lengthCACC + 1;
				}
			}
		}
	}

	if (this->nextsec != NULL)
	{
		if (this->stringPosID == 0)
		{
			this->string_leader = this;
		}
		else if (this->stringPosID == 1)
		{
			this->string_leader = Leader;
		}
		else if (this->stringPosID > 0)
		{
			this->string_leader = Leader->string_leader;
		}
		else
		{
			this->string_leader = NULL;
		}
	}



	return CACC_mode_new;
}

void myVehicleDef::CACC_clean_CACC_index() //This function resets CACC index to zero.
{
	if (this->nextsec == NULL && this->leader == NULL)
	{
		return;
	}



	this->stringPosID = 0;

	return;
}

int myVehicleDef::getCACC_length_limit_byLane()
{
	int CACCLengthLimit = 20;
	int laneNum = 0;
	//Get the lane number of the subject vehicle.
	laneNum = this->getIdCurrentLane();
	//Switch the CACC length limitation by lane.
	switch (laneNum)
	{
	case 1:  //Right Most Lane
		CACCLengthLimit = 5;
		break;
	case 2:
		CACCLengthLimit = 10;
		break;
	case 3:
		CACCLengthLimit = 15;
		break;
	case 4:
		CACCLengthLimit = 20;
		break;
	default:
		CACCLengthLimit = 20;
		break;
	}

	return CACCLengthLimit;
}


