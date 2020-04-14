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

#include "A2BehavioralModelUtil.h"
#include "mybehavioralModelCreator.h"
#include "mybehavioralModel.h"


A2BehavioralModelCreator * mybehavioralModelFactory()
{
	A2BehavioralModelCreator * res = new mybehavioralModelCreator();
	return res;
}

mybehavioralModelCreator::mybehavioralModelCreator() : A2BehavioralModelCreator(){}

mybehavioralModelCreator::~mybehavioralModelCreator(){

	//delete this->res;

}


A2BehavioralModel * mybehavioralModelCreator::newModel()
{
	A2BehavioralModel *res = new mybehavioralModel();
	return res;
}

