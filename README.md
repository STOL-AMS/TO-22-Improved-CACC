
# Summary:

This repository contains the source code of the PATH microscopic traffic simulation model. The traffic model depicts the car-following and lane-changing behaviors of human drivers, connected human drivers, Adaptive Cruise Control (ACC) vehicles, and Cooperative ACC (CACC) vehicles. The source code was developed based on Aimsun 8.2 microSDK and API libraries. The source code generates two DLL files after a successful build: an SDK DLL file for replacing Aimsun's default car-following and lane-changing models, and an API DLL file for operating traffic control strategies and processing traffic data. The source code was programmed in Visual Studio 2013 with C++.  

# Organizational Outline:
* Project Title
* Release Notes
* Getting Started
* Prerequisites
* Installing
* Testing
* Authors
* License
* Acknowledgments
* Code.gov Registration

# Project Title

*Developing Analysis, Modeling, and Simulation (AMS) Tools for Connected Automated Vehicle Applications: PATH Model Description and Implementation in the Microscopic Traffic Simulation Platform*

As road users start to adopt different technologies, the traffic stream might consist of human vehicles (HV), connected vehicles (CV), autonomous vehicles (AV), and connected automated vehicles (CAV) at the same time. The interaction of various types of vehicle fleet may induce complex traffic flow patterns that have never been observed in the existing transportation system. Such complex traffic is difficult to model with existing microscopic simulation and evaluation approaches. To address the challenge, the Federal Highway Administration (FHWA) supported a research project entitled “Developing Analysis, Modeling, and Simulation (AMS) Tools for Connected Automated Vehicle Applications”.

## Release Notes

#### Release 1.0.0 (April 5, 2020)
- Initial release

## Getting Started

*Download the source code files and open the solution file in Visual Studio 2013. The source code is ready to build if the environment variables are ready (see Prerequisites).*

### Prerequisites

Requires:
- Aimsun 8.2 (with microSDK and API licenses) 
- Visual Studio 2013

### Installing

Step 1: Install software tools. 

```
- Install Aimsun 8.2 in ''C:\Program Files\'
- Install Visual Studio 2013
- Create a directory 'C:\CACC_Simu_Data\' 
- Create a txt file 'ParameterSet.txt' under 'C:\CACC_Simu_Data\'
- Create a xml file '01_mybehavioralModel.xml' under 'C:\Program Files\Aimsun\Aimsun Next 8.2\plugins\aimsun\models' 
```

Step 2: Set up compile environment.

```
- Obtain full file access control of 'C:\CACC_Simu_Data\'
- Obtain full file access control of 'C:\Program Files\Aimsun\Aimsun Next 8.2\plugins\aimsun\models'

* Environment Configuration for SDK:*
- Open PATH_MODEL.vcxproj in Visual Studio 2013, In the Properties window of the PATH_MODEL project, go to Configuration Properties -> Debugging -> Command. In the input box, put 'C:\Program Files\Aimsun\Aimsun Next 8.2\Aimsun.exe'
- In the Properties window of the PATH_MODEL project, go to Configuration Properties -> C/C++ -> Additional Include Directories
-In the input box, put 'C:\Program Files\Aimsun\Aimsun Next 8.2\programming\Aimsun microSDK\ext\include;C:\Program Files\Aimsun\Aimsun Next 8.2\programming\Aimsun microSDK\include;'
- In the Properties window of the PATH_MODEL project, go to Configuration Properties -> Linker -> Output File
- In the input box, put 'C:\Program Files\Aimsun\Aimsun Next 8.2\plugins\aimsun\models\mybehavioralModel.dll'
- In the Properties window of the PATH_MODEL project, go to Configuration Properties -> Linker -> Additional Library Directories
- In the input box, put 'C:\Program Files\Aimsun\Aimsun Next 8.2\programming\Aimsun microSDK\lib;C:\Program Files\Aimsun\Aimsun Next 8.2\programming\Aimsun microSDK\ext\lib;'
- In Visual Studio, open myVehicleDef.app from the PATH_MODEL project, search keywords LEFT and RIGHT
- Make sure that the global constants LEFT = -1 and RIGHT = 1

* Environment Configuration for API:*
- Open AAPI.vcxproj in Visual Studio 2013
- In the Properties window of the PATH_MODEL project, go to Configuration Properties -> Debugging -> Command
- In the input box, put 'C:\Program Files\Aimsun\Aimsun Next 8.2\Aimsun.exe'
- In the Properties window of the AAPI project, go to Configuration Properties -> Linker -> Output File
- In the input box, put C:\CACC_Simu_Data\AAPI_D.dll
```

Step 3: Prepare 'ParameterSet.txt'.

```
This file is used to define the simulation variables, such as the desired acceleration, drivers reaction time, and traffic management strategies on or off. An example file is given as the following:

*
car_reaction_time_avg_:0.4,car_headway_mean:1.25,car_GKVehicle::maxDecelMean:2.5,car_GKVehicle::maxAccelMean:2.0,exp_lane_change_desire_thrd:0.15,exp_GKExperiment::CACC_LC_Desire_Threshold:0.15,exp_early_lane_keep_dis:1.5,exp_GKExperiment::CACCLengthLimit:10,exp_GKExperiment::CACCPlatoonGAP:1.5,exp_Activate_CACC_Managed_Lane_Restricted_Access:0,exp_GKExperiment::VAD_Percent:0,exp_Activate_CACC_Managed_Lane:0,exp_Number_of_CACC_Managed_Lanes:0,exp_CACC_Percent:0,exp_ACC_Percent:0, exp_through_flow:16,exp_on_ramp_flow:0.9, exp_off_ramp_flow:0.9,
Sample_Parameter_Configuration
*
```

Step 3: Prepare '01_mybehavioralModel.xml'.

```
This file tells Aimsun to load the SDK DLL file in a simulation run. An example file is given as the following:

*
<plugin>
    <name>mybehavioralModel</name>
    <lib>mybehavioralModel</lib>
</plugin>
*
```

## Testing

*Here are the steps for test the model:*

Step 1: Build SDK DLL file.

```
- Right click PATH_MODEL in the Solution Explorer and click Rebuild
- If the PATH_MODEL is built successfully, a DLL file named mybehavioralModel.dll will be automatically generated in the folder 'C:\Program Files\Aimsun\Aimsun Next 8.2\plugins\aimsun\models\'
```

Step 2: Build API DLL file.

```
- Right click AAPI in the Solution Explorer and click Rebuild
- If the AAPI is built successfully, a DLL file named AAPI_D.dll and an ILK file named AAPI_D.ilk will be automatically generated in the folder C:\CACC_Simu_Data
```

Step 3: Test the models in an Aimsun network file.

```
- Download the sample network file from this repository
- Open the network file, double click Dynamic Scenario in the Project panel (usually the panel is docked on the right side of Aimsun’s main window)
- Go to Aimsun API -> Add, then add 'AAPI_D.dll'
- Double click Dynamic Experiment in the Project panel 
- Go to Behavior tab and check 'Activate external behavior model'
- Run a simulation replication and observe the traffic patterns with different market penetrations of manually driven vehicles, connected vehicles, ACC, and CACC vehicles
```

## Authors

Hao Liu, Ph.D.
California PATH
University of California, Berkeley
Richmond Field Station, Bldg. 177 
S. 46th Street, Richmond, CA 94804
Tel: (510) 665-3451
Email: liuhao@berkeley.edu

## License

This project is licensed under the apache-2.0 License.

## Acknowledgments

This research is supported by Federal Highway Administration  (FHWA) Office of Operations Research and Development HRDO program under the project entitled Developing Analysis, Modeling, and Simulation (AMS) Tools for Connected Automated Vehicle Applications (Project Number: DTFH61-12-D-00030-0022).

## Code.gov Registration Info

Agency: DOT
Short Description: Source code of the PATH traffic simulation model
Status: Alpha
Tags: Traffic simulation, Aimsun, ACC, CACC, 
Labor hours: 0
Contact Name: ??
Contact Phone: ??
