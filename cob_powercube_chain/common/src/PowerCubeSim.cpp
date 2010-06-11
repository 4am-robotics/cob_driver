/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_powercube_chain
 * Description: This class simulates the PowerCubes in a very simply and rough way.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Aug 2007
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <powercube_chain/PowerCubeSim.h>
#include <string>
#include <sstream>
#ifdef PYTHON_THREAD
#include <Python.h>
#endif
//#define __LINUX__

#define DEG 57.295779524
#define MANUAL_AXES0_OFFSET  1.8
#define MANUAL_AXES6_OFFSET 1.5


#define SIM_CLOCK_FREQUENCY 10.0 //ms
using namespace std;

#ifndef PCTRL_CHECK_INITIALIZED()
#define PCTRL_CHECK_INITIALIZED() \
if ( isInitialized()==false )													\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}
#endif


PowerCubeSim::PowerCubeSim()
{
	m_Dev=0;
	m_NumOfModules = 0;
	m_SimThreadArgs = NULL;
	m_SimThreadID = NULL;

	//pthreads variables
	
	
}


/** The Init function opens the bus and initializes it. Furthermore the Id`s of the Cubes are taken and  mapped. These function has to be used before the cubes can be positioned after the power up.
 * @param m_DOF gives the number of Degrees of freedom of the manipulator which is connected (the gripper is not counted as DOF)
 */

bool PowerCubeSim::Init(PowerCubeCtrlParams * params)
{
	//std::cout << "---------- PowerCubes-Simulation Init -------------" << std::endl;
	m_DOF=params->GetNumberOfDOF();

	m_Dev=0;
	m_NumOfModules = m_DOF;
	
	m_maxVel = params->GetMaxVel();
	m_maxAcc = params->GetMaxAcc();

	// Make sure m_IdModules is clear of Elements:
	m_IdModules.clear();
			

	m_CurrentAngles.resize(m_DOF);
	m_CurrentAngularMaxAccel.resize(m_DOF);
	m_CurrentAngularMaxVel.resize(m_DOF);
	m_CurrentAngularVel.resize(m_DOF);

	//m_AngleOffsets = m_Obj_Manipulator->GetThetaOffsets();

	for(int i=0; i<m_DOF; i++)
	{
		std::ostringstream os;
		os << "ID_Module_Number" << i+1;
		
		// Get the Module Id from the config file
		
		//set initial angles to zero
		m_CurrentAngles[i] = 0.0;
		m_CurrentAngularVel[i] =0.0;
		m_CurrentAngularMaxVel[i] = m_maxVel[i];
		m_CurrentAngularMaxAccel[i] =m_maxAcc[i];
		
		//printf("Offset Angle %d: %f\n",i,m_AngleOffsets[i]);
	}
	
	
	// Jetzt Winkelgrenzen setzen:
	//m_AngleLimits = m_Obj_Manipulator->GetLimitsTheta();

	// pthreads initialisation
	
	pthread_mutex_init(&m_Movement_Mutex,NULL);
	pthread_mutex_init(&m_Angles_Mutex,NULL);
	pthread_mutex_init(&m_AngularVel_Mutex,NULL);

	m_SimThreadID = (pthread_t*)malloc(m_DOF * sizeof(pthread_t));
	m_MovementInProgress.resize(m_DOF);
	m_SimThreadArgs = (SimThreadArgs**)malloc(m_DOF * sizeof(SimThreadArgs*));
	for (int i = 0; i < m_DOF; i++)
	{
		m_MovementInProgress[i] = false;
		m_SimThreadArgs[i] = new SimThreadArgs();
		m_SimThreadArgs[i]->cubeSimPtr = this;
		m_SimThreadArgs[i]->cubeID = i;
	}
	setMaxVelocity(MAX_VEL);
	setMaxAcceleration(MAX_ACC);


	//std::cout << "---------- PowerCubes Init fertig ----------" << std::endl;
	m_Initialized = true;
	return true;
}


/** The Deconstructor 
 */
PowerCubeSim::~PowerCubeSim()
{ 
	free(m_SimThreadID);
	for (int i = 0; i < m_DOF; i++)
	{
		delete m_SimThreadArgs[i];
	}
	free(m_SimThreadArgs);

}

/// @brief Returns the current Joint Angles
bool PowerCubeSim::getConfig(std::vector<double>& result)
{
	PCTRL_CHECK_INITIALIZED();
	//lock mutex
	//
	//std::cout << "getConfig: Waiting for Current_Angles_Mutex ... ";
	pthread_mutex_lock(&m_Angles_Mutex);
	//m_Out << "locked"<<endl;
	result.resize(m_DOF);
	for(int i; i < m_DOF; i++)
		result[i] = m_CurrentAngles[i]*DEG;
	//unlock mutex
	//
	//m_Out <<"getConfig: Unlocking Angles_Mutex ... ";
	pthread_mutex_unlock(&m_Angles_Mutex);
	//m_Out <<"unlocked "<<endl;
	
	return true;
}



/// @brief Returns the current Angular velocities (Rad/s)
bool PowerCubeSim::getJointVelocities(std::vector<double>& result)
{
	PCTRL_CHECK_INITIALIZED();
	//lock mutex
	//
	//std::cout << "getConfig: Waiting for Current_Angles_Mutex ... ";
	pthread_mutex_lock(&m_AngularVel_Mutex);
	//m_Out << "locked"<<endl;
	result.resize(m_DOF);
	result = m_CurrentAngularVel;
	//unlock mutex
	//
	//m_Out <<"getConfig: Unlocking Angles_Mutex ... ";
	pthread_mutex_unlock(&m_AngularVel_Mutex);
	//m_Out <<"unlocked "<<endl;

	return true;
}
void PowerCubeSim::setCurrentAngles(std::vector<double> Angles)
{

	//std::cout << "setCurrentAngles: " << Angles[0] << " " << Angles[1] << " " << Angles[2] << " " << Angles[3] << " " << Angles[4] << " " << Angles[5] << " \n";
	pthread_mutex_lock(&m_Angles_Mutex);
	//m_Out << "locked"<<endl;
	
	m_CurrentAngles = Angles;
	//unlock mutex
	//
	//m_Out <<"setCurrentAngles: Unlocking Angles_Mutex ... ";
	pthread_mutex_unlock(&m_Angles_Mutex);
	//m_Out <<"unlocked "<<endl;
	

}

void PowerCubeSim::setCurrentJointVelocities( std::vector<double> AngularVel)
{

	//lock mutex
	//m_Out << "setCurrentJointVelocities: Waiting for AngularVel_Mutex ... ";
	pthread_mutex_lock(&m_AngularVel_Mutex);
	//m_Out << "locked"<<endl;
	
	m_CurrentAngularVel = AngularVel;
	//unlock mutex
	//
	//m_Out <<"setCurrentJointVelocities: Unlocking AngularVel_Mutex ... ";
	pthread_mutex_unlock(&m_AngularVel_Mutex);
	//m_Out <<"unlocked "<<endl;

}
	


/// @brief same as MoveJointSpace, but final angles should by reached simultaniously!
bool PowerCubeSim::MoveJointSpaceSync(const std::vector<double>& target)
{
	PCTRL_CHECK_INITIALIZED();
	std::cout << "Starting MoveJointSpaceSync(Jointd Angle) ... "<<endl;
	std::vector<double> targetRAD;
	targetRAD.resize(m_DOF);
	for(int i = 0; i<m_DOF; i++)
		targetRAD[i] = target[i]/DEG;

	// Evtl. Fragen zur Rechnung / zum Verfahren an: Felix.Geibel@gmx.de
	std::vector<double> acc(m_DOF);
	std::vector<double> vel(m_DOF);
	
	double TG = 0;
	
	try
	{
		
		// Ermittle Joint, der bei max Geschw. und Beschl. am längsten braucht:
		int DOF = m_DOF;

		std::vector<double> posnow;
		if ( getConfig(posnow) == false )
		    return false;


		std::vector<double> velnow;
		if ( getJointVelocities(velnow) == false )
		    return false;

		std::vector<double> times(DOF);

		for (int i=0; i < DOF; i++)
		{
			RampCommand rm(posnow[i], velnow[i], targetRAD[i], m_maxAcc[i], m_maxVel[i]);
			times[i] = rm.getTotalTime();
		}
		
		// determine the joint index that has the greates value for time
		int furthest = 0;

		double max = times[0];

	    for (int i=1; i<m_DOF; i++)
	    {
		    if (times[i] > max)
		    {
			    max = times[i];
			    furthest = i;
		    }
	    }

		RampCommand rm_furthest(posnow[furthest], velnow[furthest], targetRAD[furthest], m_maxAcc[furthest], m_maxVel[furthest]);

		double T1 = rm_furthest.T1();
		double T2 = rm_furthest.T2();
		double T3 = rm_furthest.T3();

		// Gesamtzeit:
		TG = T1 + T2 + T3;
		
		// Jetzt Geschwindigkeiten und Beschl. für alle:
		acc[furthest] = m_maxAcc[furthest];
		vel[furthest] = m_maxVel[furthest];
		
		for (int i = 0; i < DOF; i++)
		{
			if (i != furthest)
			{
				double a; double v;
				// a und v berechnen:
				RampCommand::calculateAV(
					posnow[i],
					velnow[i],
					targetRAD[i],
					TG, T3,
					m_maxAcc[i],
					m_maxVel[i],
					a,
					v);

				acc[i] = a;
				vel[i] = v;
			}
		}
	}
	catch(...)
	{
		return false;
	}

	startSimulatedMovement(targetRAD);
	
	// Errechnete Gesamtzeit zurückgeben (könnte ja nützlich sein)
	return true;
		
}


		

/// @brief Returns the time for a ramp-move about dtheta with v, a would take, assuming the module is currently moving at vnowClose 
double PowerCubeSim::timeRampMove(double dtheta, double vnow, double v, double a)
{
	// die Zeiten T1, T2 und T3 errechnen
	// ACHTUNG: Hier wird vorläufig angenommen, dass die Bewegung groß ist, d.h. es eine Phase der Bewegung
	// mit konst. Geschw. gibt. Der andere Fall wird erst später hinzugefügt.
	//m_Out << "DEBUG: timeRampMove" << endl;
	//m_Out << "-------------------" << endl;
	//m_Out << "dtheta: " << dtheta << ", vnow: " << vnow << ", v: " << v << ", a: " << a << endl;
	//m_Out << endl;
	
	// Wird Joint mit +vmax oder -vmax drehen?
	double vm = (dtheta<0)?-v:v;
	double am = (dtheta<0)?-a:a;
	//m_Out << "vm: " << vm << endl;
	//m_Out << "am: " << am << endl;
	
	// Zeit bis vm erreicht:
	double T1 = (vm - vnow) / am;
	// Winkel, der hierbei zurückgelegt wird:
	double dtheta1 = vnow * T1 + 0.5 * am * T1 * T1;
	//m_Out << "T1: " << T1 << endl;
	//m_Out << "dtheta1: " << dtheta1 << endl;
	
	// Zeit zum Bremsen:
	double T3 = vm / am;
	// Winkel hierbei:
	double dtheta3 = 0.5 * vm * T3;
	
	// Verbleibender Winkel:
	double dtheta2 = dtheta - dtheta1 - dtheta3;
	// Also Restzeit (Bew. mit vm):
	double T2 = dtheta2 / vm;
	//m_Out << "T2: " << T2 << endl;
	//m_Out << "dtheta2: " << dtheta2 << endl;
	//m_Out << "T3: " << T3 << endl;
	//m_Out << "dtheta3: " << dtheta3 << endl;

	
	// Gesamtzeit zurückgeben:
	return T1 + T2 + T3;
}

		
/// @brief Starts moving all cubes with the given velocities
bool PowerCubeSim::MoveVel(const std::vector<double>& vel)
{
	PCTRL_CHECK_INITIALIZED();
	/* TODO
        if (getStatus() != PC_CTRL_OK)
        {
                printf("PowerCubeSim::MoveVel: canceled!\n");
                return;
        }
	for(int i=0;i<m_NumOfModules;i++)
	{
		PCube_moveVel(m_Dev,m_IdModules[i],AngularVelocity[i]);
	}
	PCube_startMotionAll(m_Dev);
	*/
}
		
				
/// @brief Stops the Manipulator immediately
bool PowerCubeSim::Stop()
{
	for (int i=0; i < m_DOF; i++)
		setStatusMoving(i, false);
	return true;
}

/// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeSim::setMaxVelocity(double radpersec)
{ 
	for (int i=0; i<m_DOF; i++)
	{
		m_CurrentAngularMaxVel[i] = radpersec;
	}
	return true;
}

bool PowerCubeSim::setMaxVelocity(const std::vector<double>& radpersec)
{
	for (int i=0; i<m_DOF; i++)
	{
		m_CurrentAngularMaxVel[i] = radpersec[i];
	}
	return true;
}


/// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeSim::setMaxAcceleration(double radPerSecSquared)
{
    PCTRL_CHECK_INITIALIZED();

	for (int i=0; i<m_DOF; i++)
	{
		m_CurrentAngularMaxAccel[i] = radPerSecSquared;
	}

    return true;
}
bool PowerCubeSim::setMaxAcceleration(const std::vector<double>& radPerSecSquared)
{
    PCTRL_CHECK_INITIALIZED();

	for (int i=0; i<m_DOF; i++)
	{
		m_CurrentAngularMaxAccel[i] = radPerSecSquared[i];
	}

    return true;
}



/// @brief Returns true if some cubes are still moving
bool PowerCubeSim::statusMoving()
{
	PCTRL_CHECK_INITIALIZED();
	bool isMoving = false;

	//m_Out << "statusMoving: Waiting for Movement_Mutex ... ";
	pthread_mutex_lock(&m_Movement_Mutex);
	//m_Out << "locked"<<endl;
	for (int i = 0; i < m_DOF; i++)
	{
		if (m_MovementInProgress[i])
		{
			isMoving = true;
			break;
		}
	}

	//unlock mutex
	//m_Out <<"statusMoving: Unlocking Movement_Mutex ... ";
	pthread_mutex_unlock(&m_Movement_Mutex);
	//m_Out <<"unlocked "<<endl;

	return isMoving;
}

bool PowerCubeSim::statusMoving(int cubeNo)
{
	PCTRL_CHECK_INITIALIZED();
	bool isMoving = false;

	//m_Out << "statusMoving: Waiting for Movement_Mutex ... ";
	pthread_mutex_lock(&m_Movement_Mutex);
	//m_Out << "locked"<<endl;
	if (m_MovementInProgress[cubeNo])
	{
		isMoving = true;
	}

	//unlock mutex
	//m_Out <<"statusMoving: Unlocking Movement_Mutex ... ";
	pthread_mutex_unlock(&m_Movement_Mutex);
	//m_Out <<"unlocked "<<endl;

	return isMoving;
}
		
void PowerCubeSim::setStatusMoving (int cubeNo, bool moving)
{
	//m_Out << "setStatusMoving: Waiting for Movement_Mutex ... ";
	pthread_mutex_lock(&m_Movement_Mutex);
	//m_Out << "locked"<<endl;

	m_MovementInProgress[cubeNo] = moving;

	//m_Out <<"setStatusMoving: Unlocking Movement_Mutex ... ";
	pthread_mutex_unlock(&m_Movement_Mutex);
	//m_Out <<"unlocked "<<endl;

}

/// @brief Returns true if any of the Joints are decelerating
bool PowerCubeSim::statusDec()
{
	PCTRL_CHECK_INITIALIZED();
	/* TODO
	for (int i=0; i<m_NumOfModules; i++)
	{
		unsigned long status;
		PCube_getModuleState(m_Dev,m_IdModules[i], &status);
		if (status & STATEID_MOD_RAMP_DEC)
			return true;
	}
	*/
	return false;
}


/// @brief Returs true if any of the Joints are accelerating
bool PowerCubeSim::statusAcc()
{
	PCTRL_CHECK_INITIALIZED();
	/* TODO
	for (int i=0; i<m_NumOfModules; i++)
	{
		unsigned long status;
		PCube_getModuleState(m_Dev,m_IdModules[i], &status);
		if (status & STATEID_MOD_RAMP_ACC)
			return true;
	}

	*/
	return false;
}

void  * SimThreadRoutine(void* threadArgs)
{
	//get argument
	SimThreadArgs * args =  (SimThreadArgs*)threadArgs;
	PowerCubeSim* cubeSimPtr = args->cubeSimPtr;
	int cubeNo = args->cubeID;
	double targetAngle = args->targetAngle;

	//std::cout << "Thread started for cube no "<< cubeNo<<"["<<(cubeSimPtr->getModuleMap())[cubeNo]<<"]"<<endl;
	
	//calculate phases of movement
	float t1,t2,t,tges; //acceleration time t1/t , duration of maximum vel t2, total time tges
	double deltaT = SIM_CLOCK_FREQUENCY/1000; //clock period
	t1=t2=t=tges=0;
	float maxVel = (cubeSimPtr->getCurrentAngularMaxVel())[cubeNo];
	float maxAccel = (cubeSimPtr->getCurrentAngularMaxAccel())[cubeNo];
	std::vector<double> currAngles;
	cubeSimPtr->getConfig(currAngles);
	std::vector<double> currVels;
	cubeSimPtr->getJointVelocities(currVels);

	double deltaAngle = targetAngle - currAngles[cubeNo];
	
	//acceleration phase
	t1 = maxVel/maxAccel;

	//constant velocity phase
	t2 = abs(deltaAngle)/maxVel - t1;

	//cubeSimPtr->getOutputStream()<<" (abs(deltaAngle) >, maxVel*maxVel/maxAccel)?"<< abs(deltaAngle) <<"; "<< maxVel*maxVel/maxAccel<<endl;
	
	//is maxVel reached?
	if (abs(deltaAngle) > maxVel*maxVel/maxAccel)
	{
		tges=2*t1+t2;
	}
	else
	{

		//how long will we accelerate?
		t = sqrt(abs(deltaAngle)/(maxAccel));
		//what velocity will be reached?
		//maxVel = maxAccel*t;
		tges = 2*t;
	}

	//determine direction of movement
	if (deltaAngle<0)
	{
		maxAccel = -maxAccel;
		maxVel = -maxVel;
	}
	std::cout << "maxVel: "<<maxVel<<"; maxAccel: "<<maxAccel<< "; t1 [ms]: "<<t1<<"; t2 [ms]: "<<t2 <<"; deltaAngle[rad]" <<deltaAngle<<endl;

	//control loop
	
	double simulatedTime = 0.0; 
	int n = 0;  
	float currDeltaAngle = deltaAngle;

	if (abs(deltaAngle) > 0.001)
	{
		while ((simulatedTime <= tges) && cubeSimPtr->getStatusMoving(cubeNo) /*||(abs(currDeltaAngle) > 0.001)*/)
		//while (abs(deltaAngle)>0.005)
		{
			//advavnce in simulated time
			cubeSimPtr->millisleep((int)SIM_CLOCK_FREQUENCY);
			simulatedTime += SIM_CLOCK_FREQUENCY/1000; //=n*deltaT
			n++;
			cubeSimPtr->getJointVelocities(currVels);
			//calculate delta phi
			double deltaPhi = 0.0;

			//is max Vel reached?
			if (abs(deltaAngle) > maxVel*maxVel/abs(maxAccel))
			{
				if (simulatedTime < t1)
				{
					deltaPhi = 0.5*maxAccel*(n*deltaT*n*deltaT - (n-1)*deltaT*(n-1)*deltaT);
					currVels[cubeNo] = maxAccel * n*deltaT;
					std::cout << "Phase 1, maxVel ->";

				}
				else if ((t1 < simulatedTime) && (simulatedTime < t1+t2))
				{
					deltaPhi = maxVel * deltaT;
					currVels[cubeNo] = maxVel;
					std::cout << "Phase 2, maxVel ->";
				}
				else if ((simulatedTime > t1+t2) && (simulatedTime < 2*t1+t2))
				{
					//deltaPhi = maxVel*simulatedTime - 0.5*maxAccel*(deltaT - (t1+t2))*(simulatedTime - (t1+t2));
					deltaPhi = maxVel*deltaT - 0.5*maxAccel*((n*deltaT-(t1+t2))*(n*deltaT-(t1+t2))-((n-1)*deltaT-(t1+t2))*((n-1)*deltaT-(t1+t2)));
					currVels[cubeNo] = maxVel - maxAccel * (simulatedTime -(t1+t2));
					std::cout << "Phase 3, maxVel ->";
				}
				else
				{
					deltaPhi = 0.0;
					currVels[cubeNo] = 0.0;
					std::cout << "Phase 4, maxVel ->";
				}
			}
			//no
			else
			{
				if (simulatedTime < t)
				{
					//deltaPhi = 0.5*maxAccel*simulatedTime*simulatedTime;
					deltaPhi = 0.5*maxAccel*(n*deltaT*n*deltaT - (n-1)*deltaT*(n-1)*deltaT);
					currVels[cubeNo] = maxAccel * simulatedTime;
					std::cout  << "Phase 1 ->";
				}
				else if ((simulatedTime > t) && (simulatedTime <= 2*t))
				{
					//deltaPhi = maxVel *simulatedTime - 0.5*maxAccel*(simulatedTime -t)*(simulatedTime-t);	
					deltaPhi = maxAccel*t*deltaT - 0.5*maxAccel*((n*deltaT-t)*(n*deltaT-t)-((n-1)*deltaT-t)*((n-1)*deltaT-t));
					currVels[cubeNo] = maxAccel*t - maxAccel * (simulatedTime -t);
					std::cout  << "Phase 2 ->";
				}
				else
				{
					deltaPhi = 0.0;
					currVels[cubeNo] = 0.0;
					std::cout  << "Phase 3 ->" << t << "\n";
				}

			}
			cubeSimPtr->getConfig(currAngles);

			currAngles[cubeNo] = currAngles[cubeNo]+deltaPhi;
			currDeltaAngle = targetAngle - currAngles[cubeNo];
			//std::cout << "cube "<<cubeNo<<"["<<simulatedTime<<"]: deltaPhi: "<<deltaPhi<<"; deltaAngle:"<<currDeltaAngle<<endl;

			//write new angle and velocity
			cubeSimPtr->setCurrentAngles(currAngles);
			cubeSimPtr->setCurrentJointVelocities(currVels);
		}
	}

	//we have finished our move
	cubeSimPtr->setStatusMoving(cubeNo,false);
	currVels[cubeNo] = 0.0;
	cubeSimPtr->setCurrentJointVelocities(currVels);

	//cubeSimPtr->getOutputStream() << "Thread finished for cube ID "<< (cubeSimPtr->getModuleMap())[cubeNo]<<endl;
	pthread_exit(NULL);
}


int PowerCubeSim::startSimulatedMovement(std::vector<double> target)
{

	if (statusMoving())
	{
		std::cout << "startSimulatedMovement: Movement already in progress, preemption not implemented yet! Aborting .."<<endl;
	}
	else
	{
		//create thread

		for (int i = 0; i < m_DOF; i++)
		{
			setStatusMoving(i,true);
			m_SimThreadArgs[i]->targetAngle = target[i];
			pthread_create(&m_SimThreadID[i],NULL,SimThreadRoutine,(void*)m_SimThreadArgs[i]);
		}
	}
	return 0;

}
void PowerCubeSim::millisleep(unsigned int milliseconds) const
{
	timespec warten;
	// Millisekunden in Sekunden und Nanosekunden aufteilen
	warten.tv_sec = milliseconds / 1000;
	warten.tv_nsec = (milliseconds % 1000) * 1000000;
	timespec gewartet;
	nanosleep(&warten, &gewartet);
}


/** Wait functions which is waiting that all cubes have done their homing 
*/
/* Not necessary
   void PowerCubeSim::HomingDone()
   {
   for(int i=0; i<m_NumOfModules; i++)
   {
   unsigned long int help;
   std::cout << "Warte auf Modul " << m_IdModules[i] << endl;
   do
   {
   PCube_getModuleState(m_Dev,m_IdModules[i],&help);
//printf("Status: 0x%lx\n",help);
millisleep(100);
} while ( (help & STATEID_MOD_HOME) == 0 );
}

}

*/

/*
   vector<int> PowerCubeSim::getModuleMap(int dev)
   {
   vector<int> mod_map;
   for( int i = 1; i < MAX_MODULES; i++ )
   {
   unsigned long serNo;
   int ret = PCube_getModuleSerialNo( dev, i, &serNo );
   if( ret == 0 )
   {
   std::cout << "Found module " << i << " with SerialNo " << serNo << "\n";
   mod_map.push_back(i);
   }
//		else
//		printf( "Module %d not found(%d)\n", i, ret);

}
return mod_map;
}
*/

/* TODO: necessary?
   void PowerCubeSim::waitForSync()
   {
   for (int i=0; i < m_DOF; i++)
   {
   unsigned long confword;
   millisleep(4);
   PCube_getConfig(m_Dev, m_IdModules[i], &confword );
   millisleep(4);
   PCube_setConfig(m_Dev, m_IdModules[i], confword | CONFIGID_MOD_SYNC_MOTION);
   }
   }

*/

/* TODO: necessary?
   void PowerCubeSim::dontWaitForSync()
   {
   for (int i=0; i < m_DOF; i++)
   {
   unsigned long confword;
   millisleep(4);
   PCube_getConfig(m_Dev, m_IdModules[i], &confword );
   millisleep(4);
   PCube_setConfig(m_Dev, m_IdModules[i], confword & (~CONFIGID_MOD_SYNC_MOTION));
   }
   }
   */

/*
   int PowerCubeSim::getStatus()
   {

   PC_CTRL_STATE error = PC_CTRL_OK;

   for(int i=0; i<m_NumOfModules; i++)
   {
   unsigned long int state;
   PCube_getModuleState(m_Dev,m_IdModules[i],&state);

   if (state & STATEID_MOD_POW_VOLT_ERR)
   {
   printf("Error in Module %d: Motor voltage below minimum value!\n",m_IdModules[i]);
   error = PC_CTRL_POW_VOLT_ERR;
   }
   else if (!(state & STATEID_MOD_HOME))
   {
   printf("Warning: Module %d is not referenced!\n",m_IdModules[i]);
   error = PC_CTRL_NOT_REFERENCED;
   }
   else if (state & STATEID_MOD_ERROR)
   {
   printf("Error in  Module %d: 0x%lx\n",m_IdModules[i],state);
   error = PC_CTRL_ERR;
   }
   }
        return error;

}*/
