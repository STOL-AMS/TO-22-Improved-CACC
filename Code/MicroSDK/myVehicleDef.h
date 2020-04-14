//-*-Mode: C++;-*-
#ifndef _myVehicleDef_h_
#define _myVehicleDef_h_

#include "A2BehavioralModelUtil.h"
#include "A2BehavioralModel.h"
#include "A2SimVehicle.h"
#include "ANGConProxie.h"
#include "CIProxie.h"
#include "akiproxie.h"
#include "behaviorParameters.h"
//#include "mybehavioralModel.h"
#include <stdio.h>
#include <deque>
#include <hash_map>
#include <map>


#define A_lim		1.0
#define D_lim		2.0
#define DEBUG_TRACK_ID 0
#define CAR_Type  53 
#define HOVveh  12773     //17859 12773

#define ACC_HOV  58  // 812
#define CACC_HOV 62  // 813

#define NGSIM_CACC_ACC_V2V 346 // Simple network and El Camino Real: 4369; SR-99: 26543 346
#define NGSIM_ACC 344  // Simple network and El Camino Real: 4476; SR-99: 26542 344



class A2BEHAVIORALEXPORT myVehicleDef : public A2SimVehicle
{
private:
	bool applyACC_CACC; //determines whether follows an Automatic Cruise Control 
	bool apply_HOV;//determine whether it is a HOV car independent on cacc, acc, or manually driven vehicles

	//*********************************************************
	//the following private variables are defined for Yeo's model
	int state;
	int mode;
	double jam_gap;
	double E;
	double T;
	double min_E;
	double min_T;
	double distConflict;
	int target_lane;
	int RMode;  // Relaxation Mode 0: No relaxation, 1: relaxation 2: relaxation transition
	int CoopTime;
	int LCTime;
	int LCType;  //Lane changing type 1= Exit or turning, LC, 2= ELC, 3=DLC
	double xExit;
	int nLC;
	bool CoopRequest;
	int CoopLane;
	int idDEBUG;
	//dongyan add, to avoid MCF and BCF loop
	int isFromMCF;	//if 0, means that it is not from MCF mode, if 1, it from MCF mode
	//jhsuh add
	int VehID;
	int MCFtoBCF;
	int numupdate;
	double dLC_forbidzone_before_exit_;
	double desireLC;
	double reaction_time_;
	double desireLC_force_left;
	double desireLC_force_right;
	double desireLC_option_left;
	double desireLC_option_right;
	double mandatory_LC_weight;
	double discretionary_LC_weight;
	double min_time_btw_lcs;
	double last_lc_time;
	double politeness_;
	double random_politeness_;
	double dLC_scan_range_;
	int dLC_scan_no_cars_;
	int source_section;


	int exit_lane_from;
	int exit_lane_to;
	int enter_lane_from;
	int enter_lane_to;
	double distance_to_off_ramp;
	//****************************

public:
	myVehicleDef(void *handlerVehicle, unsigned short idhandler, bool isFictitiousVeh);
	~myVehicleDef();

	// Properties and functions added by Hao Liu
	struct PositionSpeed {
		double position;
		double speed;
	};

	int targetLaneID;

	PositionSpeed BeforeLeftLaneChangingMove4HOV();

	//================================================================================================

	void setHOV(bool isHOV){ this->apply_HOV = isHOV; };
	bool getHOV(){ return this->apply_HOV; };

	double lane_change_prob;
	bool getapplyACC();
	const A2SimVehicle* leader;
	const A2SimVehicle* left_follower;
	const A2SimVehicle* left_leader;
	const A2SimVehicle* right_follower;
	const A2SimVehicle* right_leader;
	InfVeh info;
	StaticInfVeh staticinfo;
	const A2SimVehicle* getLeader();
	PositionSpeed RunNGSIM(bool mode_predetermined);

	bool ApplyNGSIMModel();
	bool has_leader;
	double getMAXacc();
	double getMAXdec();

	double BaseCfModel(double a_L, double a_U,
		double tau, double headway, double jamGap, double d_leader,
		double l_leader, double vf, double v, double x, double x_leader,
		double x_leader_steps_early,
		double lead_v, double min_headway);

	double BaseCfModel(double a_L, double a_U,
		double tau, double headway, double jamGap, double d_leader,
		double l_leader, double vf, double v, double x,
		double x_leader, double x_leader_steps_early,
		double lead_v, double min_headway,
		double &target_pos);

	int GapAcceptDecision_Sync_First();


	//*********the following functions and variables are defined for Yeo's model

	int getMode(){ return mode; };
	int setMode(int avalue);
	myVehicleDef* CoopRequester;
	myVehicleDef* LastCoopRequester;


	double CoopDist;
	int CoopRequesterID;
	void setCodist(double codist){ CoopDist = codist; }
	double getCodist(){ return CoopDist; }
	void setVehID(int aVehID){ VehID = aVehID; }
	double getJamGap(){ return jam_gap; };
	void setJamGap(double avalue){ jam_gap = avalue; };

	double getE(){ return E; };
	void setE(double avalue){ E = avalue; };

	double getT(){ return T; };
	void setT(double avalue){ T = avalue; };

	int getnLC(){ return nLC; };
	void setnLC(int value){ nLC = value; };

	double getxExit(){ return xExit; };
	void setxExit(double avalue){ xExit = avalue; };

	int getCoopTime(){ return CoopTime; };
	void setCoopTime(int avalue){ CoopTime = avalue; }

	int getLCTime(){ return LCTime; };
	void setLCTime(int avalue){ LCTime = avalue; };

	double getDistConflict(){ return distConflict; };
	void setDistConflict(double avalue){ distConflict = avalue; };

	int getLCType(){ return LCType; };
	void setLCType(int avalue){ LCType = avalue; };

	int getRMode(){ return RMode; };
	void setRMode(int avalue){ RMode = avalue; };

	int getTargetLane(){ return target_lane; };
	void setTargetLane(int avalue){ target_lane = avalue; };

	//double PosCfSkipGap(const A2SimVehicle* leader);
	double PosCfSkipGap(const A2SimVehicle* potential_leader, bool apply_creep_speed);
	double PosCf(const A2SimVehicle* leader, int shortGap, double beta, double alpha, double relaxation);
	double PosCf(const A2SimVehicle* leader);

	bool NeedLC();
	bool NeedDlc();
	bool NeedRampLc();
	bool NeedLc4Turning();
	bool NeedCoop();

	double getLastCoopTime()
	{
		return _coop_start_time;
	}

	void setLastCoopTime(double param)
	{
		_coop_start_time = param;
	}

	double getSpeed();
	double getPosition();
	PositionSpeed setNewPosition(double pos, double velocity);
	double getReactionTime();
	void setReactionTime(double val);

	double getPosition(int state);
	double getSpeed(int state);

	double delta_t;
	double alpha;
	double beta;
	int ACF_Steps, ACF_Step;
	double Relaxation;

	//IDM parameters
	double delta;

	double getFreeFlowSpeed();
	int determineDrivingMode();
	int DetermineLcOrMergeOrCoop();
	int Determine2ndLcAfterLc();
	int determineCoopOrLc();

	PositionSpeed UpdateBeforeLaneChangeCf();
	PositionSpeed UpdateAfterLaneChangeCf();
	PositionSpeed updateCoopCf();
	PositionSpeed UpdateReceiveCf();
	bool isAfterLaneChangeFinish();
	int determineGapOrGiveup();

	//std::deque<double> position_queue;
	double right_dlc_coeff_;
	double estimated_leader_dec_coeff_;
	double acc_smooth_coeff_;
	int last_lc_target;
	double lc_gap_reduction_factor;
	int gap_model;
	int debug_track_id;
	double gipps_theta;
	bool allow_unseqential_merging;
	int mandatory_type;
	double off_ramp_e;
	double off_ramp_t;
	double penalty_dlc_no_exitlane;
	double b_gap_reduction_factor_offramp;
	double b_gap_reduction_factor_onramp;
	double f_gap_reduction_factor_onramp;
	double f_gap_reduction_factor_offramp;
	double increase_DLC_close_ramp;
	bool new_need_adjust;
	bool first_cycle_after_adjust;
	double desire_headway;
	double friction_coeff;
	int initial_leader_id;
	int _ramp_lc_decision;
	int _smooth_transit_time;
	double politeness_optional;
	double random_politeness_optional;
	double left_avg_speed_ahead;
	double avg_speed_ahead;
	double right_avg_speed_ahead;
	A2KSectionInf sec_inf;
	int last_lc_type;
	double comf_dec_ramplc;
	double comf_dec_dlc;
	double freeflowspeed;
	double ramp_lc_slowdown_desire;
	double acc_exp;
	double _coop_start_time;
	double _hov_start_time;
	double _hov_end_time;
	bool _hov_active;
	double dis_2_turn;
	double eqv_dis_2_turn;
	double off_ramp_overpass_acc;
	bool _hov_include;
	int current_sec_id; //current sec id
	int nextsec; //next sec id
	int nextnextsec; // next next sec id
	int thirdsec; // next next next sec id
	double lane_change_prob_offramp;
	double current_time;
	bool closeness2Offramp;
	int exit_section_before_offramp;
	double _early_lane_keep_dis;
	bool Connect2RightMostLane;
	double f_gap_reduction_factor_DLC;
	double b_gap_reduction_factor_DLC;
	bool to_be_delete;
	std::vector<int> non_boost_dlc_sections;
	int waiting_time;
	bool acc_2_manual;
	std::string parastr;
	double cacc_percent;
	double acc_percent;
	int adaptive_mode;
	double acc_desire_headway;
	double cacc_desire_headway;
	int last_adaptive_mode;
	int CARveh;
	int HovTypeID;
	int Truck_Type;
	int CACCveh;
	int ACCveh;

	PositionSpeed BeforeOnRampLcSlowDown();
	PositionSpeed BeforeOnRampLcSync();
	double PosCf2EndofRamp();
	double posEndAccLane();
	PositionSpeed updateRegularCf();
	int RampCfDecision();
	double getLaneChangeDesire();
	double distance2EndAccLane();
	PositionSpeed UpdateLc();
	PositionSpeed UpdateLc(const A2SimVehicle *vehDown,
		const int targetLane, const double newpos, const double newspeed);
	bool Willing2Coop(myVehicleDef *coop_veh);
	int ExitCfDecision();
	void setLaneChangeDesire(double incentive);
	void setLaneChangeDesireForce(double incentive_left, double incentive_right);
	void setLaneChangeDesireOption(double incentive_left, double incentive_right);
	bool CombineLCDesires();
	void ResetDesires();
	double CalculateDesireForce(int n_lc, double d_exit, double speed
		, bool is_for_on_ramp);
	void setMinTimeBtwLcs(double val){
		this->min_time_btw_lcs =
			(val > 1 ? val : 1);
	};
	double getMinTimeBtwLcs(){ return min_time_btw_lcs; };
	double getLastLCTime(){ return this->last_lc_time <= 0 ? 0 : this->last_lc_time; };
	void setLastLCTime(double val){ this->last_lc_time = val; };
	void SetInitialVal();
	//override the default to add the lane change time

	void setPoliteness(double val){ this->politeness_ = val; };
	double getPoliteness();
	void setRandomPoliteness(double val){ this->random_politeness_ = val; };
	double getRandomPoliteness(){ return random_politeness_; };
	double GetPastPos(double reaction_time);


	bool IsCoopEffectMuch(myVehicleDef * coop_veh);
	int DLCCFDecision();

	double getDLCForbidZoneBeforeExit(){ return dLC_forbidzone_before_exit_; };
	void setDLCForbidZoneBeforeExit(double val){ dLC_forbidzone_before_exit_ = val; };

	double getRemainLength()
	{
		return this->sec_inf.length - this->getPosition();
		//AKIInfNetGetSectionANGInf(this->getIdCurrentSection()).length-this->getPosition();
	};

	double getDLCScanRange()
	{
		/*double remain_length = getRemainLength();
		return dLC_scan_range_>remain_length-1?remain_length-1:dLC_scan_range_;*/
		return this->dLC_scan_range_;
	};

	void setDLCScanRange(double val){ dLC_scan_range_ = val; };
	int getDLCScanNoCars(){ return dLC_scan_no_cars_; };
	void setDLCScanNoCars(int val){ dLC_scan_no_cars_ = val; };
	void setDLCWeight(double val){
		this->discretionary_LC_weight = val;
		this->mandatory_LC_weight = 1.0 - val;
	};
	double getRightDLCCoeff(){ return this->right_dlc_coeff_; };
	void setRightDLCCoeff(double val);
	void setLaneChangeDesireThrd(double val);
	double getEstimateLeaderDecCoeff(){ return this->estimated_leader_dec_coeff_; };
	void setEstimateLeaderDecCoeff(double val)
	{
		this->estimated_leader_dec_coeff_ = val;
	};
	double getAccSmoothCoef(){ return this->acc_smooth_coeff_; };
	void setAccSmoothCoef(double val)
	{
		this->acc_smooth_coeff_ = val;
	};
	void setLastLCTarget(int targetLane)
	{
		this->last_lc_target = targetLane;
	};
	int getLastLCTarget()
	{
		return (this->last_lc_target == LEFT || this->last_lc_target == RIGHT) ?
			this->last_lc_target : NOCHANGE;
	};
	void setSourceSection(int val)
	{
		this->source_section = val;
	};
	int getSourceSection()
	{
		return this->source_section;
	};
	void setLCGapReductionFactor(double val){
		this->lc_gap_reduction_factor =
			((val > 0 && val <= 1) ? val : 1);
	};
	double getLCGapReductionFactor(){ return this->lc_gap_reduction_factor; };
	int getGapAcceptanceModel(){ return this->gap_model; };
	void setGapAcceptanceModel(int val){ this->gap_model = val; };
	bool DisGapAccepted(double a_L, double a_U, double tau,
		double headway, double jamGap,
		double d_leader,
		double l_leader, double vf, double v,
		double x,
		double x_leader,
		double x_leader_steps_early,
		double lead_v, double min_headway,
		double Gap_AC_Thrd,
		double desire,
		bool on_ramp, bool forward, double self_acc);
	bool AccGapAccepted(double a_L,
		double a_U, double tau, double headway,
		double jamGap, double d_leader, double l_leader, double vf,
		double v, double x, double x_leader, double x_leader_steps_early,
		double lead_v, double min_headway, double Gap_AC_Thrd,
		double desire);
	int getDebugTrackID(){ return this->debug_track_id; };
	void setDebugTrackID(int val){ this->debug_track_id = val; };
	double getGippsTheta(){ return this->gipps_theta; };
	void setGippsTheta(double val){ this->gipps_theta = val; };
	bool AllowUnsequentialMerging(){ return allow_unseqential_merging; };
	void SetUnsequentialMerging(bool val){ allow_unseqential_merging = val; };
	double GippsDecelerationTerm(
		double maxDec, double reaction_time, double theta,
		double x_leader, double x, double jamGap,
		double l_leader, double v, double lead_v, double b_estimate);
	bool GippsGap(double maxDec, double reaction_time,
		double theta, double x_leader, double x,
		double jamGap, double l_leader, double v,
		double lead_v, double b_estimate);
	bool GippsGap(double maxDec, double reaction_time, double theta,
		double x_leader, double x, double jamGap, double l_leader,
		double v, double lead_v, double b_estimate, bool on_ramp, bool forward, double acc_self);
	double getLaneChangeDesireThrd();
	void setMandatoryType(int type){ this->mandatory_type = type; };
	int getMandatoryType(){ return this->mandatory_type; }
	PositionSpeed BeforeExitorTurningLcSync();
	double PosCf2EndofExitTurning();
	void setOffRampE(double val){ this->off_ramp_e = val; };
	double getE4OffRamp(){ return this->off_ramp_e; };
	void setOffRampT(double val){ this->off_ramp_t = val; };
	double getT4OffRamp(){ return this->off_ramp_t; };
	double GetAdditionalDlcDesire(int target_lane);
	double Bound_Function(double param1);
	double getPenaltyDLCNoExitLane(){ return penalty_dlc_no_exitlane; };
	void setPenaltyDLCNoExitLane(double val){ this->penalty_dlc_no_exitlane = val; };
	void setForwardGapReductionDLC(double val){ this->f_gap_reduction_factor_DLC = val; };
	void setBackwardGapReductionDLC(double val){ this->b_gap_reduction_factor_DLC = val; };
	void setForwardGapReductionOnRamp(double val)
	{
		this->f_gap_reduction_factor_onramp = ((val <= 0 || val > 1) ? 1 : val);
	};
	void setBackwardGapReductionOnRamp(double val)
	{
		this->b_gap_reduction_factor_onramp = ((val <= 0 || val > 1) ? 1 : val);
	};
	void setForwardGapReductionOffRamp(double val)
	{
		this->f_gap_reduction_factor_offramp = ((val <= 0 || val > 1) ? 1 : val);
	};
	void setBackwardGapReductionOffRamp(double val)
	{
		this->b_gap_reduction_factor_offramp = ((val <= 0 || val > 1) ? 1 : val);
	};
	double getForwardGapReductionOnRamp()
	{
		return this->f_gap_reduction_factor_onramp;
	};
	double getBackwardGapReductionOnRamp()
	{
		return this->b_gap_reduction_factor_onramp;
	};
	double getForwardGapReductionOffRamp()
	{
		return this->f_gap_reduction_factor_offramp;
	};
	double getBackwardGapReductionOffRamp()
	{
		return this->b_gap_reduction_factor_offramp;
	};
	double OnRampAddCoef(int num_lane_2_rightmost);
	double getIncreaseDLCCloseRamp();
	void setIncreaseDLCCloseRamp(double val);
	int GetRampType(int sec_id);
	int getOnRampVehCount(int next_sec, double *ramp_length);
	int GetOnAccLaneFlow(int next_sec);
	InfVeh GetOnAccLaneFlowInf(int next_sec);
	double GetEquPosition(double leader_pos, double leader_l, double v);
	void setNewArrivalAdjust(bool needadjust){ this->new_need_adjust = needadjust; };
	bool getNewArrivalAdjust(){ return this->new_need_adjust; };

	void setFirstCycleAfterAdjust(bool val)
	{
		first_cycle_after_adjust = val;
	};
	bool getFirstCycleAfterAdjust(){ return first_cycle_after_adjust; };
	void setHeadwayTime(double headway_time)
	{
		this->desire_headway = headway_time;
	};
	void setACCHeadwayTime(double headway_time)
	{
		this->acc_desire_headway = headway_time;
	};
	void setCACCHeadwayTime(double headway_time)
	{
		this->cacc_desire_headway = headway_time;
	};
	double getDesireHeadway();
	double getFrictionCoef();
	void setFrictionCoef(double val);
	bool IsSectionSource(int sec_id);
	int DetermineReceiveOrLcOrCoop();
	bool isLaneChangingPossible(int target_lane);
	void setInitialLeaderId(int id);
	int getInitialLeaderId(){ return this->initial_leader_id; };
	int determineDrivingModeACC();
	int getNextSectionRampType(int& next_sec_center_lanes);

	double getE4OnRamp(){ return E > getMinE4OnRamp() ? E : getMinE4OnRamp(); };
	double getMinE4OnRamp(){ return min_E < 50 ? 50 : min_E; };
	double getT4OnRamp(){ return T > getMinT4OnRamp() ? T : getMinT4OnRamp(); };
	double getMinT4OnRamp(){ return min_T < 10 ? 10 : min_T; };

	double getMinT4OffRamp(){ return getMinT4OnRamp(); };
	double getMinE4OffRamp(){ return getMinE4OnRamp(); };

	void DesireEquation(double& para1, double& para2, double dis2End, double time2End, int n_lc, double minT, double minE, double T, double E);
	double getMaxDecInSync();
	bool getNoOfVehsOnNextOnRampAccLane();
	bool PreventSimultaneousLC();
	double AnticipatedAcc(double a_L, double a_U, double tau, double headway, double jamGap, double d_leader, double l_leader, double vf, double v, double x, double x_leader, double x_leader_steps_early, double lead_v, double min_headway, double Gap_AC_Thrd, double desire);
	void setRampDecision(int ramp_lc_decision);
	int getRampDecision();
	int getSmoothTransitTime();
	void addOneStepTransitTime();
	double getPolitenessOptional();
	double getRandomPolitenessOptional();
	void setRandomPolitenessOptional(double param1);
	void setPolitenessOptional(double param1);
	bool ExistNewLCer(int direction);
	void getAroundSpeed();
	void getAroundLeaderFollowers();
	double DLCDesire(int target_lane);
	void getSectionInfo();
	void setLastLCType(int type);
	int getLastLCType();
	double getComfDecRampLC();
	void setComfDecRampLC(double param);
	double getComfDecDLC();
	void setComfDecDLC(double param);
	double createFreeFlowSpeed(bool ACCCACC);
	double getRampLCSlowDownDesire();
	double getAccExp();
	void setAccExp(double param1);
	void CrashAvoidancePosition(double& velocity,
		double& pos);
	void getGapHeadwayLeader(double& gap,
		double& headway,
		double& l_leader,
		double& ref_pos_front);



	void setRelaxationTime(double param);
	PositionSpeed AdjustArrivalVehicle_New(bool simpleNetwork);
	int GetSectionHOVLane();


	bool isHOVActive();
	void setHOVActive();
	void setDis2NextTurning(double dis){ this->dis_2_turn = dis; };
	double getDis2NextTurning(){ return this->dis_2_turn; };
	void setEqvDis2NextTurning(double dis){ this->eqv_dis_2_turn = dis; };//equivalent distance is discounted by number of lanes
	double getEqvDis2NextTurning(){ return this->eqv_dis_2_turn; }; //equivalent distance is discounted by number of lanes
	PositionSpeed BeforeExitorTurningOverPass();
	double getOffRampOverpassAcc(){ return off_ramp_overpass_acc; };
	void setOffRampOverpassAcc(double param){ off_ramp_overpass_acc = param; };
	//double PosCf2EndofTurningPoint();

	double MinCollisionAvoidancePos(const A2SimVehicle* leader, int param2, double beta, double alpha, double Relaxation);
	void setHOVStart(double hov_start_time){ _hov_start_time = hov_start_time; };
	void setHOVEnd(double hov_end_time){ _hov_end_time = hov_end_time; };
	double getHOVStart(){ return _hov_start_time; };
	double getHOVEnd(){ return _hov_end_time; };
	void setHOVIncluded(bool hov_include);
	bool getHOVIncluded();



	bool NotWithinOfframpAwarenessDis();

	double getMinTimeBtwLcs4DLC();
	void setExtraDesire4FeasibleGapOfframp(int targetlane);
	void EliminateDlcDesireOutSideRouteLanes(int fromlane, int tolane);
	void CheckCloseness2Offramp(double distance);
	PositionSpeed BeforeOffRampLcCruise();
	void BoostOnrampIncentive();
	int GetSectionOfframpLanes(int sec_id);
	int GetNoOfLanes(int tempsec);

	double getEarlyLaneKeepDis();
	void setEarlyLaneKeepDis(double param);

	void SetReportError();


	bool OverPassPossbile(myVehicleDef * templeader);
	void SetRiskyRelax();
	void ResetRelax();
	double getSectionSyncCoef();
	
	bool ACC_Manual_TakeOver_Check_CAMP(double& v_des);

	int CollisionScenario(double spacing, double leader_speed, double param3, double leader_acc, double decREQ);

	double sampleNormalDist(double mean, double std);
	void setNameByType();
	void SetMarkAttribute(int param1);
	void SetNameAttribute(std::string param1);
	double GenerateHeadway4Type(int vehTypeId);
	void SetExtraHeadways();
	void InitialCACCACCMode();
	void SetWidth4Coloring();
	void SetVehicleWidth(double width);
	int getLastAdaptiveMode();
	void setLastAdaptiveMode(int mode);
	void RecordCrashInformation();
	double getLengthCACC();
	bool UseMergingDemandReading();
	void SetVehTypeIDs(int CarTypeID, int HovTypeID, int TruckTypeID,
		int CACCTypeID, int ACCTypeID);

	// Variables used by Jerry's algorithm
	int DetermineLcOrMergeOrCoopOrACC();
	


	void myVehicleDef::VehKinematics(double &Accl, double &Spd, double &Pos);
	double myVehicleDef::getIdDistanceToEnd(int VehID);
	void myVehicleDef::Read_leader_follower_data();
	void myVehicleDef::set_ACC_CTG();
	void myVehicleDef::set_CACC_CTG();
	PositionSpeed myVehicleDef::CTG_ACC_update();
	PositionSpeed myVehicleDef::CTG_ACC_update(double leader_dis, double leader_spd, bool MakeLaneChange, bool SoftBrake);
	void myVehicleDef::MergeSecPosition();
	void myVehicleDef::update_Following_Veh_ID();
	void myVehicleDef::update_Driving_Mode();
	PositionSpeed myVehicleDef::NGSIMPlusACC(bool mode_predetermined);
	PositionSpeed myVehicleDef::NGSIMPlusACC_CACC(bool mode_predetermined);
	PositionSpeed myVehicleDef::NGSIMPlusACC_CACC_V2VAHM(bool mode_predetermined);

	int determineDrivingModePlusACC();
	int determineDrivingModePlusACC_CACC_V2XAHM();
	PositionSpeed myVehicleDef::V_CACC_update(double dis_pre, double spd_pre, bool MakeLaneChange);
	int myVehicleDef::DetemineVCACCOrCF();

	int myVehicleDef::determineDrivingModePlusACC_CACC();
	int myVehicleDef::DetermineLcOrMergeOrCoopOrACC0rCACC_0729();
	int myVehicleDef::Determine2ndLcAfterLc_0729();
	int myVehicleDef::determineCoopOrLc_0729();
	int myVehicleDef::DetermineReceiveOrLcOrCoop_0729();
	int myVehicleDef::determineGapOrGiveup_0729();
	
	int CACC_recover_time;
	int myVehicleDef::CACCDetermineLcOrCACC();

	PositionSpeed myVehicleDef::Run_CC();
	PositionSpeed myVehicleDef::CACC_veh_model();
	PositionSpeed myVehicleDef::CTG_CACC_update(double spd_pre, double dis_pre, double MakeLaneChange);
	PositionSpeed myVehicleDef::CTG_CACC_update();
	bool myVehicleDef::DetermineCACC();
	int myVehicleDef::CACCOnOrOff();
	
	PositionSpeed myVehicleDef::Run_ACC();
	PositionSpeed myVehicleDef::CACC_CC();
	PositionSpeed myVehicleDef::CACC_fixed_timegap();
	PositionSpeed myVehicleDef::Run_CACC_Follower();
	PositionSpeed myVehicleDef::Run_CACC_Leader();
	void myVehicleDef::Update_PrecedingCACC_Len();

	PositionSpeed myVehicleDef::Run_CACC();
	void setCACC_length_limit(int val){ this->CACC_length_limit = val; return; };
	int getCACC_length_limit(){ return CACC_length_limit; };
	int myVehicleDef::CACC_mode_switch();
	void myVehicleDef::CACC_clean_CACC_index(); //This function resets CACC index to zero.
	int myVehicleDef::getCACC_length_limit_byLane();


	double ACC_CTG;  //ACC constant time gap
	double CACC_CTG; //CACC constant time gap
	double HIST_Dens; //historic density, currently predetermined at the beginning.
	double mean_gap; //instant density on the merging conflict lane.
	double mean_speed;
	int MergeTargetID;
	bool MergeTargetFound;
	int Auto_On_Acc_Lane_count;
	const A2SimVehicle* left_leader_old;
	double desAccL;
	double virtual_des_AccL;
	double RampFollowDist;
	bool readyToMergeBehind;
	bool spdSync;
	double desAccel;
	bool OnRampreadyToMergeBehind;
	bool OnRampspdSync;
	bool ConnectFlag;
	InfVeh virtual_leading_vehicle;
	InfVeh virtual_following_vehicle;
	bool v_follower_flag;
	bool v_leader_flag;
	double vir_leader_speed;
	double vir_follower_speed;
	bool on_ramp_spd_syn;
	bool virtual_leader_found;
	double v_vleader;
	double x_vleader;
	double dis_vleader;
	double v_vfollower;
	double x_vfollower;
	double dis_vfollower;
	double v_real_leader;
	double dis_real_leader;
	int v_leader_ID;
	int v_follower_ID;
	int v_follower_follower_ID;
	int virtual_ACC_ID;
	int virtual_ACC_follower_ID;
	double spd_virtual_acc_leader;
	double dis_virtual_acc_leader;
	double a_old;
	double virtual_gap;
	int ACC_recover_time;
	int ACC_state;
	int CACC_state;
	double CACC_platoon_CTG;

	//*****vehicle speed response dynamics model 
	double Vcmd_d1;
	double Vcmd_d2;
	double Vcmd_d3;
	double Vcmd_d4;
	double Vreal_d1;
	double Vreal_d2;
	double Areal;
	int CACC_length_limit;
	bool CombineLCDesires_2();
	double DLCDesire_2(double target_lane);

	

	//Reaction time for ACC and CACC vehicle
	double reaction_time_ACC;
	double reaction_time_CACC;

	double getDistanceToOffRamp();
	PositionSpeed BeforeOffRampLcSlowDown();


	int old_V_leader;
	double V_CACC_init_time;
	bool AHM_ON;
	double MergeSectionSpd;
	bool Active_AHM;

	int GetCACCManagedLaneType(int sec_id);
	int CACC_mode_switch_time;
	int CACC_Mode;
	double getDistanceHeadway(int stateSV, myVehicleDef* refVeh, int stateRefVeh);
	double getPositionReferenceVeh(int stateSV, A2SimVehicle* refVeh, int stateRefVeh);
	double getPositionReferenceVeh(int stateSV, myVehicleDef* refVeh, int stateRefVeh);
	double getPositionReferenceVeh(myVehicleDef* ref_veh);
	//get past reference position
	double getPastPositionReferenceVehs
		(double reaction_time_ref, myVehicleDef* ref_veh,
		double reaction_time_this);

	double GetPositionRelativeFake
		(myVehicleDef* fake_veh, double reaction_time_fake,
		bool downstream);
	bool isCrashed;
	// To eliminate right turn and left turn vehicles take the same gap
	int rightTurnWaitTime;
	double rightTurnStartTime;
	double NGSIM_Speed(double x, double v);
	// Record bumper to bumper distance between the subject vehicle and the preceding vehicle
	double followingDistance;
	double leftLeadHeadway;
	double leftLagHeadway;
	double rightLeadHeadway;
	double rightLagHeadway;
	int offRampID;

	// CACC string operation parameters
	int stringPosID;
	int PrecedingStringLength;
	int CACC_ML_Activated;
	int number_CACC_ML;
	int CACC_ML_RA;
	double CACC_DLC_Thre;
	int CACCModeSwitch;
	int CACCStringLeaderFollowerSwitch; // 0: no switch; 1: leader to follower; 2: follower to leader
	myVehicleDef* string_leader;




	double distanceToLeader;
	double speedLeader;
	int followerID;




};
#endif
