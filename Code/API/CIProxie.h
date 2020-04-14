#ifndef _CIProxie_h_
#define _CIProxie_h_
//SDK
#include "AAPI_Util.h"

extern "C" {
	// Valid States
	#define RED 	0
	#define GREEN 	1
	#define YELLOW 	2

	// Types of Meterings
	#define NONE		0
	#define METVERDE 	1
	#define METFLUJO 	2
	#define METDELAY 	3
	#define METFLUJOALINEA 4
	#define METVERDEPERLANE 5

	#define CtrlControlNotLoaded -1000
	#define CtrlUnknownSignal 				-2002
	#define CtrlUnknownState 				-2003
	#define CtrlUnknownPhase 				-2005
	#define CtrlUnknownObject	 			-2007
	#define CtrlUnknownMet 					-2008
	#define CtrlIncorrectTypeMet 			-2009
	#define CtrlJunctionNotControled 		-2010
	#define CtrlMetNotControled 			-2011
	#define CtrlNotExternalControl 			-2012
	#define CtrlSameIniTimeControl 			-2013
	#define CtrlRemovingCurrentControl 		-2014
	#define CtrlUnknownPhaseName 			-2015
	#define CtrlUnknownVehTypeName 			-2021
	#define CtrlInvalidIndex	 			-2022
	#define CtrlGreenTimeLesserThanZero		-2023
	#define CtrlPhaseNotActuated	 	 	-2024
	#define CtrlJunctionNotCoordinated	 	-2025
	#define CtrlPhaseWithCoordinationRecall	-2026
	#define CtrlUnknownBusPreemptionNode	-2027
	AAPIEXPORT int ECIGetNumberJunctions();
	AAPIEXPORT int ECIGetJunctionId(int numelem);
	// Returns a utf16 enconded string. Has to be deleted with delete[]
	AAPIEXPORT const unsigned short *ECIGetJunctionName(int idJunction);
	AAPIEXPORT int ECIGetNumberSignalGroups(int idJunction);
    AAPIEXPORT const unsigned short *ECIGetLogicalNameofSignalGroup(int ajunction, int asignalgroup);
    AAPIEXPORT const unsigned short *ECIGetExternalIdofSignalGroup(int ajunction, int asignalgroup);
	AAPIEXPORT int ECIGetAimsunIdofSignalGroup(int ajunction, int asignalgroup);
	AAPIEXPORT int ECIGetJunctionIdFromExternalId(const unsigned short *externalId);
	AAPIEXPORT int ECIGetNumberTurningsofSignalGroup(int ajunction, int asignalgroup);
	AAPIEXPORT int ECIGetFromToofTurningofSignalGroup(int ajunction, int asignalgroup, int indexTuring, int *from, int *to);

    AAPIEXPORT int ECIGetSignalGroupGreenDuration(int idJunction, int signalPos, double timeSta );
    AAPIEXPORT int ECIGetSignalGroupYellowDuration(int idJunction, int signalPos, double timeSta );
    AAPIEXPORT int ECIGetSignalGroupRedDuration(int idJunction, int signalPos, double timeSta );

	AAPIEXPORT int ECIGetCurrentPhase(int idJunction);
	AAPIEXPORT int ECIGetCurrentPhaseInRing(int idJunction, int ring);
	AAPIEXPORT double ECIGetCurrentTimeInCycle(int idJunction, int ring);
	AAPIEXPORT int ECIGetNumberPhases(int idJunction);
	AAPIEXPORT int ECIGetNumberPhasesInRing(int idJunction, int ring);
	AAPIEXPORT double ECIGetStartingTimePhase(int idJunction);
	AAPIEXPORT double ECIGetStartingTimePhaseInRing(int idJunction, int ring);
	AAPIEXPORT int ECIGetDurationsPhase(int idJunction,int idPhase,double timeSta,double *dur, double *max, double *min);
	AAPIEXPORT int ECIGetYellowTimePhase(int idJunction,int idPhase,double timeSta,double *yellow);
	AAPIEXPORT int ECISetYellowTimePhase(int idJunction,int idPhase,double timeSta,double yellow);
	AAPIEXPORT int ECIIsAnInterPhase(int idJunction,int idPhase,double timeSta);
	
	AAPIEXPORT int ECIGetNbActiveControls();
	AAPIEXPORT int ECIGetActiveControls( int* activeControls );

	//Preemption
	AAPIEXPORT int ECIDisableBusPreemptionNode(int ajunction);
	AAPIEXPORT int ECIEnableBusPreemptionNode(int ajunction);
	AAPIEXPORT bool ECIIsBusPreemptionNodeEnabled(int idJunction);

	AAPIEXPORT int ECIGetNbPreemptionSets( int idJunction, double timeSta );
	AAPIEXPORT int ECIGetPreemptionSetParameters( int idJunction, double timeSta, int index, double* delay, double *minDwell, double *reserve, double * inhibit, double *maxDwell, int *type );
    AAPIEXPORT int ECIGetPreemptionSetNbLines( int idJunction, double timeSta, int index );
    AAPIEXPORT int ECIGetPreemptionSetLines( int idJunction, double timeSta, int index, int nblines, int *lines );
    AAPIEXPORT int ECIGetPreemptionSetNbPhases( int idJunction, double timeSta );
    AAPIEXPORT int ECIGetPreemptionSetPhases( int idJunction, double timeSta, int index, int nbPhases , int *phases );
    AAPIEXPORT int ECIGetPreemptionSetNbRequestDetectors( int idJunction, double timeSta, int index );
    AAPIEXPORT int ECIGetPreemptionSetRequestDetectors( int idJunction, double timeSta, int index, int nbDetectors, int * detectors );
    AAPIEXPORT int ECIGetPreemptionSetNbEndDetectors( int idJunction, double timeSta, int index );
    AAPIEXPORT int ECIGetPreemptionSetEndDetectors( int idJunction, double timeSta, int index, int nbDetectors, int * detectors );

	AAPIEXPORT int ECIGetNbSignalGroupsPhaseofJunction(int idJunction, int aidphase, double timeSta);
	AAPIEXPORT int ECIGetSignalGroupPhaseofJunction(int idJunction, int aidphase, int indexSG, double timeSta);
	AAPIEXPORT int ECIChangeColorSignalGroupPhaseofJunction(int idJunction, int aidphase, int indexSG, int astate, double timeSta);


	AAPIEXPORT int ECIGetControlType(int idJunction);
	AAPIEXPORT double ECIGetYellowTime(int idJunction);
	AAPIEXPORT double ECIGetOffset(int idJunction);


	AAPIEXPORT int ECIDisableEvents(int idJunction);
	AAPIEXPORT int ECIEnableEvents(int idJunction);
	AAPIEXPORT int ECIEnableEventsActivatingPhase(int idJunction, int idPhaseToActivateNow, double expiredTime, double currentTime);
	AAPIEXPORT int ECIIsEventsEnabled(int idJunction);
	AAPIEXPORT int ECIChangeDirectPhase(int idJunction, int idPhase, double timeSta, double Time, double Ciclo, double expiredTime=0.0);
	AAPIEXPORT int ECIChangeDirectPhaseWithInterphaseTransition(int idJunction, int idPhase, double timeSta, double Time, double Ciclo);
	AAPIEXPORT int ECIChangeTimingPhase(int idJunction, int idPhase, double newTime, double timeSta);

	AAPIEXPORT int ECIChangeSignalGroupState(int idJunction, int idSignalGroup, int aState, double timeSta, double Time, double Ciclo);
	AAPIEXPORT int ECIChangeSignalGroupStateToYellow( int idJunction, int idSignalGroup, int aState, double timeSta, double Time, double Ciclo, double greenTime );
	AAPIEXPORT int ECIGetCurrentStateofSignalGroup(int idJunction, int idSignalGroup);

	AAPIEXPORT int ECIChangeSignalGroupStatebyName(int idJunction, const unsigned short *nameSignalGroup, int aState, double timeSta, double Time, double Ciclo);
	AAPIEXPORT int ECIGetCurrentStateofSignalGroupbyName(int idJunction, const unsigned short *nameSignalGroup);


	AAPIEXPORT int ECIGetTypeControlofJunction(int elemControl,int ajunction);
	AAPIEXPORT int ECIGetNbPhasesofJunction(int elemControl, int ajunction);
	AAPIEXPORT int ECIGetNbRingsJunction(int elemControl, int ajunction);
	AAPIEXPORT int ECIGetNbBarriersJunction(int elemControl, int ajunction);
	AAPIEXPORT int ECIGetNumberPhasesInRingofJunction(int elemControl, int idJunction, int ring);
	AAPIEXPORT int ECIGetDurationsPhaseofJunction(int elemControl, int ajunction, int aidphase, double *amax, double *dur, double *amin);
	AAPIEXPORT double ECIGetYellowTimeofJunction(int elemControl, int ajunction);
	AAPIEXPORT int ECIGetYellowTimePhaseofJunction(int elemControl, int idJunction,int idPhase,double timeSta,double *yellow);
	AAPIEXPORT int ECISetYellowTimePhaseofJunction(int elemControl, int idJunction,int idPhase, double yellow);
	AAPIEXPORT int ECIIsAnInterPhaseofJunction(int elemControl, int idJunction,int idPhase);





	//functions on adaptative control
	AAPIEXPORT int ECIGetActuatedParamsPassageTime(int elemControl, int ajunction, int aidphase, double *apassageTime, double *atimeBeforeReduce, double *atimeToReduce, double *aminimumGap);
	AAPIEXPORT int ECISetActuatedParamsPassageTime(int elemControl, int ajunction, int aidphase, double apassageTime, double atimeBeforeReduce, double atimeToReduce, double aminimumGap);
	AAPIEXPORT int ECIGetActuatedParamsMinimumGreen(int elemControl, int ajunction, int aidphase, double *aMinDurationMinGreen, double *aMaxMinGreen, double *aSecActuation);
	AAPIEXPORT int ECISetActuatedParamsMinimumGreen(int elemControl, int ajunction, int aidphase, double aMinDurationMinGreen, double aMaxMinGreen, double aSecActuation);
	AAPIEXPORT double ECIGetActuatedParamsMaxGreen(int elemControl, int ajunction, int aidphase);
	AAPIEXPORT int  ECISetActuatedParamsMaxGreen(int elemControl, int ajunction, int aidphase, double aMaxDuration);
	AAPIEXPORT int ECIGetActuatedParamsForceOFFPermissivePeriod(int elemControl, int ajunction, int aidphase, double *forceOFF, double *permissivePeriodFrom, double *permissivePeriodTo);
	AAPIEXPORT int  ECISetActuatedParamsForceOFFPermissivePeriod(int elemControl, int ajunction, int aidphase, double forceOFF, double permissivePeriodFrom, double permissivePeriodTo);

	AAPIEXPORT int ECIGetCurrentNbRingsJunction(int ajunction);
	AAPIEXPORT int ECIGetCurrentNbBarriersJunction(int ajunction);

    struct AAPIEXPORT structA2KMetering{
        //! 0, OK, else error code
        int report;
        //! metering identifier
        int Id;
        //! section identifier where the metering is located
        int IdSection;
        //! first lane that the metering covers
        int IdFirstLane;
        //! last lane that the metering covers
        int IdLastLane;
        //! metering type 1: GREEN 2:FLOW 3:DELAY 4:FLOW ALINEA 5:GREEN BY LANE 0:NONE
        int Type;
        //! distance from the beggining of the section
        double Distance;
        //! metering name
        const unsigned short* Name;
        //! on flow meterings, the number of vehicle by lane that will allow to cross
        int Platoon;
    };

    AAPIEXPORT structA2KMetering ECIGetMeteringProperties(int element);
    AAPIEXPORT structA2KMetering ECIGetMeteringPropertiesById(int idMetering);

	AAPIEXPORT int ECIGetNumberMeterings();
	AAPIEXPORT int ECIGetMeteringIdSection(int elem);
	AAPIEXPORT const unsigned short *ECIGetMeteringName(int idsection);
	AAPIEXPORT int ECIGetMeteringId(int idsection);
	AAPIEXPORT int ECIGetMeteringIdByPos(int idsection, double metPos);
	AAPIEXPORT int ECIGetTypeMetering(int idsection);
	AAPIEXPORT int ECIGetParametersGreenMetering(int idsection, double timeSta, double *amax, double *greenTime, double *amin,double *cicleTime, double *offset, double *yellowTime);
	AAPIEXPORT int ECIChangeParametersGreenMetering(int idsection,double timeSta,double amax, double ngreenTime, double amin, double ncicleTime, double offset, double YellowTime, double currentTime, double currentTimeTrans, double cycle);
	AAPIEXPORT int ECIGetParametersFlowMetering(int idsection, double timeSta, double *amax, double *flow, double *amin);
	AAPIEXPORT int ECIChangeParametersFlowMetering(int idsection, double timeSta, double amax, double flow, double amin, double currentTime, double cycle);
	AAPIEXPORT int ECIGetParametersDelayMetering(int idsection, double timeSta, double *avg, double *dev);
	AAPIEXPORT int ECIChangeParametersDelayMetering(int idsection, double timeSta, double newavg, double newdev);
	AAPIEXPORT int ECIDisableEventsMetering(int idsection);
	AAPIEXPORT int ECIEnableEventsMetering(int idsection);
	AAPIEXPORT int ECIChangeStateMetering(int idsection, int aState, double Time, double Ciclo, int identity=0);	
	AAPIEXPORT int ECIGetCurrentStateofMetering(int idarc, int identity);

	AAPIEXPORT int ECIGetMeteringIdByPosition(int elem);
	AAPIEXPORT const unsigned short *ECIGetMeteringNameById(int idmetering);
	AAPIEXPORT int ECIGetTypeMeteringById(int idmetering);
	AAPIEXPORT int ECIGetParametersGreenMeteringById(int idmetering, double timeSta, double *amax, double *greenTime, double *amin,double *cicleTime, double *offset, double *yellowTime);
	AAPIEXPORT int ECIChangeParametersGreenMeteringById(int idmetering,double timeSta,double amax, double ngreenTime, double amin, double ncicleTime, double offset, double YellowTime, double currentTime, double currentTimeTrans, double cycle);
	AAPIEXPORT int ECIGetParametersGreenMeteringByLaneById(int idmetering, double timeSta, double *amax, double *greenTime, double *amin,double *cicleTime, double *offset, double *yellowTime, double *laneOffset);
	AAPIEXPORT int ECIChangeParametersGreenMeteringByLaneById(int idmetering,double timeSta,double amax, double ngreenTime, double amin, double ncicleTime, double offset, double YellowTime, double laneOffset, double currentTime, double currentTimeTrans, double cycle);
	AAPIEXPORT int ECIGetParametersFlowMeteringById(int idmetering, double timeSta, double *amax, double *flow, double *amin);
	AAPIEXPORT int ECIChangeParametersFlowMeteringById(int idmetering, double timeSta, double amax, double flow, double amin, double currentTime, double cycle);
	AAPIEXPORT int ECIChangeParametersFlowMeteringResetById(int idmetering, double timeSta, double amax, double flow, double amin, double currentTime, double cycle, bool resetToGreen);
	AAPIEXPORT int ECIChangeCurrentParametersFlowMeteringById(int idmetering, double amax, double flow, double amin, double currentTime, double cycle);
	AAPIEXPORT int ECIChangeCurrentParametersFlowMeteringResetById(int idmetering, double amax, double flow, double amin, double currentTime, double cycle, bool resetToGreen);
	AAPIEXPORT int ECIGetParametersDelayMeteringById(int idmetering, double timeSta, double *avg, double *dev);
	AAPIEXPORT int ECIGetParametersFlowAlineaMeteringById(int idmetering, double timeSta, double *amax, double *flow, double *amin, double *kr, double *ostar, double *intervalUpdate);
	AAPIEXPORT int ECIChangeParametersFlowAlineaMeteringResetById(int idmetering, double timeSta, double amax, double flow, double amin, double kr, double ostar, double intervalUpdate, double currentTime, double cycle, bool resetToGreen);
	AAPIEXPORT int ECIChangeParametersFlowAlineaMeteringById(int idmetering, double timeSta, double amax, double flow, double amin, double kr, double ostar, double intervalUpdate, double currentTime, double cycle);
	AAPIEXPORT int ECIChangeCurrentParametersFlowAlineaMeteringById(int idmetering, double amax, double flow, double amin, double kr, double ostar, double intervalUpdate, double currentTime, double cycle);
	AAPIEXPORT int ECIChangeCurrentParametersFlowAlineaMeteringResetById(int idmetering, double amax, double flow, double amin, double kr, double ostar, double intervalUpdate, double currentTime, double cycle, bool resetToGreen);
	AAPIEXPORT int ECIChangeParametersDelayMeteringById(int idmetering, double timeSta, double newavg, double newdev);
	AAPIEXPORT int ECIDisableEventsMeteringById(int idmetering);
	AAPIEXPORT int ECIEnableEventsMeteringById(int idmetering);
    AAPIEXPORT int ECIChangeStateMeteringById(int idmetering, int aState, double Time, double Ciclo, int identity=0, double timePorportion = 0.0);
	AAPIEXPORT int ECIGetCurrentStateofMeteringById(int idmetering, int idlane);

	AAPIEXPORT int ECIGetNumberofControls(int aidObj);
	AAPIEXPORT const unsigned short *ECIGetNameofControl(int aidObj, int elemControl);
	AAPIEXPORT double ECIGetIniTimeofControl(int aidObj, int elemControl);
	AAPIEXPORT double ECIGetOffsetofControl(int aidObj, int elemControl);
	AAPIEXPORT const unsigned short * ECIGetNameCurrentControl(int aidObj);
	AAPIEXPORT int ECIGetNumberCurrentControl(int aidObj);
	AAPIEXPORT int ECIRemoveControl(int aidObject, int elemControl);

	AAPIEXPORT int ECIGetNumberSem(int aidObject);
	AAPIEXPORT double ECIGetPositionSem(int aidObject, int anumsem);
	AAPIEXPORT int ECIGetStateSem(int aidObject, int anumsem);

	AAPIEXPORT int ECIChangeParametersDelayMeteringVehType(int idsection, double timeSta, double newavg, double newdev, int vehTypePos );
	AAPIEXPORT int ECIChangeParametersDelayMeteringVehTypeById(int idmetering, double timeSta, double newavg, double newdev, int vehTypePos );
 
}
 
#endif
