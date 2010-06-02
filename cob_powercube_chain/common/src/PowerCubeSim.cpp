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

PowerCubeSim::PowerCubeSim()
{
	PowerCubeSim(NULL, std::cout);
}

PowerCubeSim::PowerCubeSim(Manipulator * mani, ostream & o)
	: m_Out(o)
{
	m_Dev=0;
	m_NumOfModules = 0;
	m_Obj_Manipulator = mani;
	m_MovementInProgress = NULL;
	m_SimThreadArgs = NULL;
	m_SimThreadID = NULL;

	//pthreads variables
	
	setMaxVelocity(MAX_VEL);
	setMaxAcceleration(MAX_ACC);
	
}


/** The Init function opens the bus and initializes it. Furthermore the Id`s of the Cubes are taken and  mapped. These function has to be used before the cubes can be positioned after the power up.
 * @param m_DOF gives the number of Degrees of freedom of the manipulator which is connected (the gripper is not counted as DOF)
 */

bool PowerCubeSim::Init(char * iniFile)
{
	if (m_Obj_Manipulator == NULL)
	{
		m_Obj_Manipulator = new Manipulator(cout);
	}
	bool success = m_Obj_Manipulator->Init(iniFile);
	if (success)
	{
		success = Init();
	}
	if (success)
	{
		m_Initialized = true;
	}
	return success;
}

bool PowerCubeSim::Init(char* iniFile,bool home)
{

	if (m_Obj_Manipulator == NULL)
	{
		m_Obj_Manipulator = new Manipulator(cout);
	}
	bool success = m_Obj_Manipulator->Init(iniFile);
	if (success)
	{
		success = Init(home);
	}
	if (success)
	{
		m_Initialized = true;
	}
	return success;
}


bool PowerCubeSim::Init(bool home)
{
	m_Out << "---------- PowerCubes-Simulation Init -------------" << std::endl;
#ifdef COB3
	m_DOF = m_Obj_Manipulator->GetDOF();
#else
	m_DOF = 7;
#endif
	m_Dev=0;
	m_NumOfModules = m_DOF;
	
	// Make sure m_IdModules is clear of Elements:
	m_IdModules.clear();
			
	m_CurrentAngles.setNrJoints(m_DOF);
	m_CurrentAngularMaxAccel.setNrJoints(m_DOF);
	m_CurrentAngularMaxVel.setNrJoints(m_DOF);
	m_CurrentAngularVel.setNrJoints(m_DOF);

	//m_AngleOffsets = m_Obj_Manipulator->GetThetaOffsets();

	for(int i=0; i<m_DOF; i++)
	{
		std::ostringstream os;
		os << "ID_Module_Number" << i+1;
		
		// Get the Module Id from the config file
		
		//set initial angles to zero
		m_CurrentAngles.set(i,0.0);		
		m_CurrentAngularVel.set(i,0.0);
		m_CurrentAngularMaxVel.set(i, maxVel);
		m_CurrentAngularMaxAccel.set(i,maxAcc);
		
		//printf("Offset Angle %d: %f\n",i,m_AngleOffsets[i]);
	}
	
	
	// Jetzt Winkelgrenzen setzen:
	//m_AngleLimits = m_Obj_Manipulator->GetLimitsTheta();

	// pthreads initialisation
	
	pthread_mutex_init(&m_Movement_Mutex,NULL);
	pthread_mutex_init(&m_Angles_Mutex,NULL);
	pthread_mutex_init(&m_AngularVel_Mutex,NULL);

	m_SimThreadID = (pthread_t*)malloc(m_DOF * sizeof(pthread_t));
	m_MovementInProgress = (bool*)malloc(m_DOF * sizeof(bool));
	m_SimThreadArgs = (SimThreadArgs**)malloc(m_DOF * sizeof(SimThreadArgs*));
	for (int i = 0; i < m_DOF; i++)
	{
		m_MovementInProgress[i] = false;
		m_SimThreadArgs[i] = new SimThreadArgs();
		m_SimThreadArgs[i]->cubeSimPtr = this;
		m_SimThreadArgs[i]->cubeID = i;
	}

	m_Out << "---------- PowerCubes Init fertig ----------" << std::endl;
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
	free(m_MovementInProgress);
}

/// @brief Returns the current Joint Angles
Jointd PowerCubeSim::getConfig()
{

	//lock mutex
	//
	//m_Out << "getConfig: Waiting for Current_Angles_Mutex ... ";
	pthread_mutex_lock(&m_Angles_Mutex);
	//m_Out << "locked"<<endl;
	
	Jointd Angles = m_CurrentAngles;
	//unlock mutex
	//
	//m_Out <<"getConfig: Unlocking Angles_Mutex ... ";
	pthread_mutex_unlock(&m_Angles_Mutex);
	//m_Out <<"unlocked "<<endl;
	
	return Angles;
}

void PowerCubeSim::setCurrentAngles(Jointd & Angles)
{

	//m_Out << "setCurrentAngles: Waiting for Current_Angles_Mutex ... ";
	pthread_mutex_lock(&m_Angles_Mutex);
	//m_Out << "locked"<<endl;
	
	m_CurrentAngles = Angles;
	//unlock mutex
	//
	//m_Out <<"setCurrentAngles: Unlocking Angles_Mutex ... ";
	pthread_mutex_unlock(&m_Angles_Mutex);
	//m_Out <<"unlocked "<<endl;
	

}

void PowerCubeSim::setCurrentJointVelocities( Jointd & AngularVel)
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

/// @brief Returns the current Angular velocities (Rad/s)
Jointd PowerCubeSim::getJointVelocities()
{
	
	//lock mutex
	//
	//m_Out << "getConfig: Waiting for Current_Angles_Mutex ... ";
	pthread_mutex_lock(&m_AngularVel_Mutex);
	//m_Out << "locked"<<endl;
	
	Jointd AngularVel = m_CurrentAngularVel;
	//unlock mutex
	//
	//m_Out <<"getConfig: Unlocking Angles_Mutex ... ";
	pthread_mutex_unlock(&m_AngularVel_Mutex);
	//m_Out <<"unlocked "<<endl;
	
	
	return AngularVel;
}

/// @brief Moves all modules to a certain angle and waits until movement done
void PowerCubeSim::MoveJointSpaceWait(Jointd Angle)
{
	MoveJointSpace(Angle);
	while (statusMoving())
	{
		// Wait until Movement stopped
		millisleep(100);
	}
}
	
/// @brief Moves all modules by a certain angle and waits until movement done
void PowerCubeSim::MoveRelJointSpaceWait(Jointd Angle)
{
	

	MoveRelJointSpace(Angle);
	while (statusMoving())
	{
		// Wait until Movement stopped
		millisleep(100);
	}
}		

/// @brief Moves all modules to a certain angle, don't wait until movement done
/// Movement is done as fast as possible (all modules go at maxVel)
void PowerCubeSim::MoveJointSpace(Jointd Angle)
{


	//set max vel and acc for all modules
	for (int i=0; i < m_DOF; i++)
	{
		m_CurrentAngularMaxVel.set(i, maxVel);
		m_CurrentAngularMaxAccel.set(i, maxAcc);
	}

	startSimulatedMovement(Angle);
}

/// @brief same as MoveJointSpace, but final angles should by reached simultaniously!
double PowerCubeSim::MoveJointSpaceSync(Jointd Angle)
{
	m_Out << "Starting MoveJointSpaceSync(Jointd Angle) ... "<<endl;
	
	// Ermittle Winkel der sich am weitesten drehen muss:
	//Angle.set(0,Angle[0]+MANUAL_AXES0_OFFSET);
	//Angle.set(6,Angle[6]+MANUAL_AXES6_OFFSET);
	//Angle.set(1,Angle[1]+MANUAL_AXES1_OFFSET);
	Jointd current = getConfig();
	Jointd delta = Angle - current;
	// Delta soll hier nur die Beträge enthalten:
	for (int i=0; i<m_DOF; i++)
		delta.set(i, abs(delta[i]) );
		
	int furthest = delta.getMaxInd();
	
	// Für Rückgabewert, Geschw., Beschl.:
	double time = 0;
	Jointd v(m_DOF);
	Jointd a(m_DOF);
	
	// Fallunterscheidung nötig, wird Phase mit konstanter Geschw. erreicht?
	//m_Out << "maxVel: "<<maxVel<<"; maxAcc: "<<maxAcc<<endl;
	//m_Out << "delta[furthest]:" <<delta[furthest]<<"; maxVel^2/maxAcc: "<<maxVel*maxVel/maxAcc<<endl;
	if (delta[furthest] > maxVel*maxVel/maxAcc)
	{
		// Mit maxVel und maxAcc Zeit t1 fuer Beschleunigungsphase und Gesamtzeit t3 berechnen:
		double t1 = maxVel / maxAcc;
		double t3 = delta[furthest] / maxVel + t1;
		//m_Out << "t1: "<<t1<<"; tges: "<<t3<<"; t2"<<t3-2*t1<<endl;
		// Jetzt Geschwindikeiten und Beschl. fuer alle Winkel errechnen
		m_CurrentAngularMaxVel.set(furthest, maxVel);
		m_CurrentAngularMaxAccel.set(furthest, maxAcc);
		
		for (int i=0; i<m_DOF; i++)
		{
			if (i!=furthest)
			{
				// v.set(i, delta[i] / (1.0 + t3 - 2.0*t1) );
				// Hier war Fehler in Rechnung. Richtig ist:
				m_CurrentAngularMaxVel.set(i, delta[i] / (t3 - t1) );
				//m_Out << "maxVel["<<i<<"]: "<<delta[i]/(t3-t1)<<endl;
				m_CurrentAngularMaxAccel.set(i, m_CurrentAngularMaxVel[i] / t1 );
				//m_Out << "maxAcc["<<i<<"]: "<<m_CurrentAngularMaxVel[i]/t1<<endl;
			}
		}
		time = t3;
	} else {
		// Konst Geschw. wird nicht erreicht
		
		// t1 und t3 ermitteln:
		double t1 = sqrt( delta[furthest] / maxAcc );
		double t3 = 2.0 * t1;
		
		//m_Out << "t1: "<<t1<<"; tges: "<<t3<<endl;
		// Beschleunigungen errechnen (Geschw. irrelevant, v[i]=maxVel)
		m_CurrentAngularMaxAccel.set(furthest, maxAcc);
		m_CurrentAngularMaxVel.set(furthest, maxVel);
		
		for (int i=0; i<m_DOF; i++)
		{
			if (i != furthest)
			{
				m_CurrentAngularMaxAccel.set(i, delta[i] / (t1*t1) );
				//m_Out << "maxVel["<<i<<"]: "<<delta[i]/(t1*t1)<<endl;
				m_CurrentAngularMaxVel.set(i, maxVel);
				//m_Out << "maxAcc["<<i<<"]: "<<maxVel<<endl;
			}
		}
		
		time = t3;
	}
	
	// Jetzt Bewegung starten:

	startSimulatedMovement(Angle);
	
	// Errechnete Gesamtzeit zurückgeben (könnte ja nützlich sein)
	return time;
		
}

int PowerCubeSim::MoveJointSpaceSyncWait(Jointd Angles)
{

	int error = 0;

	//ToDo: Do some error checking here
	//        //
	//
	//
	double calculatedTime = MoveJointSpaceSync(Angles);


	millisleep(int(calculatedTime*1000));

	return error;
}

/// @brief This is a temporary work version, which when done will do the same as MoveJointSpaceSync but will also work correctly when called at a moment where the arm is in movement already. When finished and tested it will replace the current MoveJointSpaceSync.
/// Returns the time that the movement will take
/// Note: Still in Development, don't use unless you are sure you know what you are doing!
double PowerCubeSim::MoveJointSpaceSyncV2(Jointd Angle)
{
	
	
	m_Out << "DEGUG MoveJointSpaceSyncV2:" << endl;
	m_Out << "---------------------------" << endl << endl;
	// Evtl. Fragen zur Rechnung / zum Verfahren an: Felix.Geibel@gmx.de
	
	// Ermittle Joint, der bei max Geschw. und Beschl. am längsten braucht:
	
	Jointd current = getConfig();
	Jointd delta = Angle - current;
	m_Out << "Jetzt: " << current.toString(true) << endl;
	m_Out << "Ziel: " << Angle.toString(true) << endl;
	m_Out << "Delta: " << delta.toString(true) << endl;	
	
	// Momentane Geschw. ermitteln:
	Jointd vNow = getJointVelocities();
	m_Out << "Momentane Geschwindigkeiten (°/s): " << endl;
	m_Out << vNow.toString(true) << endl;
	
	Jointd times(m_DOF);
	m_Out << endl << "Benötigte Zeiten:" << endl;
	for (int i=0; i < m_DOF; i++)
	{
		times.set(i, timeRampMove(delta[i], vNow[i], maxVel, maxAcc));
		m_Out << i+1 << ": " << times[i] << endl;
	}
		
	int furthest = times.getMaxInd();
	
	m_Out << "Joint der am länsgten braucht: Nr. " << furthest+1 << endl;
	
	// Jetzt die Zeiten T1, T2 und T3 errechnen
	// ACHTUNG: Hier wird vorläufig angenommen, dass die Bewegung groß ist, d.h. es eine Phase der Bewegung
	// mit konst. Geschw. gibt. Der andere Fall wird erst später hinzugefügt.
	
	// Wird Joint mit +vmax oder -vmax drehen?
	double vm = (delta[furthest]<0)?-maxVel:maxVel;
	double am = (delta[furthest]<0)?-maxAcc:maxAcc;
	m_Out << "vm: " << vm << endl;
	m_Out << "am: " << am << endl;	
	
	// Zeit bis vm erreicht:
	double T1 = (vm - vNow[furthest]) / am;
	// Winkel, der hierbei zurückgelegt wird:
	double dtheta1 = vNow[furthest] * T1 + 0.5 * am * T1 * T1;
	m_Out << "T1: " << T1 << endl;
	m_Out << "dtheta1: " << dtheta1 << endl;	
	
	// Zeit zum Bremsen:
	double T3 = vm / am;
	// Winkel hierbei:
	// UHR: warum nichtdd: dtheta3 = 0.5*am*t3*t3?
	// FG: kommt hier aufs selbe raus.
	double dtheta3 = 0.5 * vm * T3;
	
	// Verbleibender Winkel:
	double dtheta2 = delta[furthest] - dtheta1 - dtheta3;
	// Also Restzeit (Bew. mit vm):
	double T2 = dtheta2 / vm;
	m_Out << "T2: " << T2 << endl;
	m_Out << "dtheta2: " << dtheta2 << endl;	
	m_Out << "T3: " << T3 << endl;
	m_Out << "dtheta3: " << dtheta3 << endl;	
	
	// Gesamtzeit:
	double TG = T1 + T2 + T3;
	m_Out << "TG: " << TG << endl;
	m_Out << endl;
	
	// Jetzt Geschwindigkeiten und Beschl. für alle: 
	Jointd acc(m_DOF);
	Jointd vel(m_DOF);
	acc.set(furthest, am);
	vel.set(furthest, vm);
	
	for (int i = 0; i < m_DOF; i++)
	{
		if (i != furthest)
		{
			// v nach Rechnung Formel (7). a,b,c für Mitternachtsformel:
			double a = (TG / T3 - 1.0);
			double b = vNow[i] - delta[i]/T3;
			double c = - 0.5 * vNow[i] * vNow[i];
			m_Out << i << ": a=" << a << " | b=" << b << " | c=" << c << endl;
			// Mitternachtsformel:
			if (delta[i] >= 0)
				vel.set(i, (-b + sqrt(b*b - 4.0*a*c)) / (2.0 * a) );
			else 
				vel.set(i, (-b - sqrt(b*b - 4.0*a*c)) / (2.0 * a) );
			
			// Jetzt a nach Formel (1):
			acc.set(i, vel[i] / T3 );
		}
	}
	
	// Jetzt Bewegung starten:

	for (int i=0; i < m_DOF; i++)
	{
		m_CurrentAngularMaxVel.set(i,abs(vel[i]));
		m_CurrentAngularMaxAccel.set(i,abs(acc[i]));
		
		//PCube_moveRamp(m_Dev,m_IdModules[i],Angle[i], abs(vel[i]), abs(acc[i]));
		
		m_Out << "v" << i << ": " << vel[i] << endl;
		m_Out << "a" << i << ": " << acc[i] << endl;
	}

	startSimulatedMovement(Angle);
	//PCube_startMotionAll(m_Dev);
	
	m_Out << endl << "Bewegung gestartet." << endl;
	m_Out << endl << "DEGUG MoveJointSpaceSyncV2 ENDE." << endl;

	
	// Errechnete Gesamtzeit zurückgeben (könnte ja nützlich sein)
	return TG;
}
		

/// @brief Returns the time for a ramp-move about dtheta with v, a would take, assuming the module is currently moving at vnowClose 
double PowerCubeSim::timeRampMove(double dtheta, double vnow, double v, double a)
{
	// die Zeiten T1, T2 und T3 errechnen
	// ACHTUNG: Hier wird vorläufig angenommen, dass die Bewegung groß ist, d.h. es eine Phase der Bewegung
	// mit konst. Geschw. gibt. Der andere Fall wird erst später hinzugefügt.
	m_Out << "DEBUG: timeRampMove" << endl;
	m_Out << "-------------------" << endl;
	m_Out << "dtheta: " << dtheta << ", vnow: " << vnow << ", v: " << v << ", a: " << a << endl;
	m_Out << endl;
	
	// Wird Joint mit +vmax oder -vmax drehen?
	double vm = (dtheta<0)?-v:v;
	double am = (dtheta<0)?-a:a;
	m_Out << "vm: " << vm << endl;
	m_Out << "am: " << am << endl;	
	
	// Zeit bis vm erreicht:
	double T1 = (vm - vnow) / am;
	// Winkel, der hierbei zurückgelegt wird:
	double dtheta1 = vnow * T1 + 0.5 * am * T1 * T1;
	m_Out << "T1: " << T1 << endl;
	m_Out << "dtheta1: " << dtheta1 << endl;	
	
	// Zeit zum Bremsen:
	double T3 = vm / am;
	// Winkel hierbei:
	double dtheta3 = 0.5 * vm * T3;
	
	// Verbleibender Winkel:
	double dtheta2 = dtheta - dtheta1 - dtheta3;
	// Also Restzeit (Bew. mit vm):
	double T2 = dtheta2 / vm;
	m_Out << "T2: " << T2 << endl;
	m_Out << "dtheta2: " << dtheta2 << endl;	
	m_Out << "T3: " << T3 << endl;
	m_Out << "dtheta3: " << dtheta3 << endl;	

	
	// Gesamtzeit zurückgeben:
	return T1 + T2 + T3;
}

		
/// @brief same as above, final conf. should be reached after time (in sec.)
/// Note: if maxVelocity and max Acceleration don't allow that, actual time will be longer 
/// Returns the time they will actually need.
double PowerCubeSim::MoveJointSpaceSync(Jointd Angles, float timeWish)
{
	// Zunächst den Joint ermitteln, der sich am weitesten dreht
	
	// TO DO !!!!
	return 0;
}


	
/// @brief Moves all modules by a certain angle
void PowerCubeSim::MoveRelJointSpace(Jointd RelAngle)
{
	/* TODO
	float position;
	float Angle[m_NumOfModules];
        if (getStatus() != PC_CTRL_OK)
        {
                printf("PowerCubeSim::MoveRelJointSpace: canceled!\n");
                return;
        }
	for(int i=0;i< m_NumOfModules;i++)
	{	
		PCube_getPos(m_Dev,m_IdModules[i],&position);
		Angle[i]=RelAngle[i]+position;
		PCube_moveRamp(m_Dev,m_IdModules[i],Angle[i], maxVel, maxAcc);
	}
	PCube_startMotionAll(m_Dev);
	*/
}
		
/// @brief Starts moving all cubes with the given velocities
void PowerCubeSim::MoveVel(Jointd AngularVelocity)
{
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
void PowerCubeSim::stop()
{
	for (int i=0; i < m_DOF; i++)
		setStatusMoving(i, false);
}

/// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
void PowerCubeSim::setMaxVelocity(float radpersec) 
{ 
	maxVel = radpersec;
}

/// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
void PowerCubeSim::setMaxAcceleration(float radPerSecSquared)
{ 
	maxAcc = radPerSecSquared; 
}
		


/// @brief Returns true if some cubes are still moving
bool PowerCubeSim::statusMoving()
{
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
	float targetAngle = args->targetAngle;

	cubeSimPtr->getOutputStream() << "Thread started for cube no "<< cubeNo<<"["<<(cubeSimPtr->getModuleMap())[cubeNo]<<"]"<<endl;
	
	//calculate phases of movement
	float t1,t2,t,tges; //acceleration time t1/t , duration of maximum vel t2, total time tges
	double deltaT = SIM_CLOCK_FREQUENCY/1000; //clock period
	t1=t2=t=tges=0;
	float maxVel = (cubeSimPtr->getCurrentAngularMaxVel())[cubeNo];
	float maxAccel = (cubeSimPtr->getCurrentAngularMaxAccel())[cubeNo];
	Jointd currAngles = cubeSimPtr->getConfig();
	Jointd currVels = cubeSimPtr->getJointVelocities();
	float deltaAngle = targetAngle - currAngles[cubeNo];
	
	//acceleration phase
	t1 = maxVel/maxAccel;

	//constant velocity phase
	t2 = abs(deltaAngle)/maxVel - t1;

	cubeSimPtr->getOutputStream()<<" (abs(deltaAngle) >, maxVel*maxVel/maxAccel)?"<< abs(deltaAngle) <<"; "<< maxVel*maxVel/maxAccel<<endl;
	
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
	cubeSimPtr->getOutputStream() << "maxVel: "<<maxVel<<"; maxAccel: "<<maxAccel<< "; t1 [ms]: "<<t1<<"; t2 [ms]: "<<t2 <<"; deltaAngle[rad]" <<deltaAngle<<endl;

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
			millisleep((int)SIM_CLOCK_FREQUENCY);
			simulatedTime += SIM_CLOCK_FREQUENCY/1000; //=n*deltaT
			n++;
			currVels = cubeSimPtr->getJointVelocities();
			//calculate delta phi
			double deltaPhi = 0.0;

			//is max Vel reached?
			if (abs(deltaAngle) > maxVel*maxVel/abs(maxAccel))
			{
				if (simulatedTime < t1)
				{
					deltaPhi = 0.5*maxAccel*(n*deltaT*n*deltaT - (n-1)*deltaT*(n-1)*deltaT);
					currVels.set(cubeNo,maxAccel * n*deltaT);
					//cubeSimPtr->getOutputStream() << "Phase 1, maxVel ->";

				}
				else if ((t1 < simulatedTime) && (simulatedTime < t1+t2))
				{
					deltaPhi = maxVel * deltaT;
					currVels.set(cubeNo,maxVel);
					//cubeSimPtr->getOutputStream() << "Phase 2, maxVel ->";
				}
				else if ((simulatedTime > t1+t2) && (simulatedTime < 2*t1+t2))
				{
					//deltaPhi = maxVel*simulatedTime - 0.5*maxAccel*(deltaT - (t1+t2))*(simulatedTime - (t1+t2));
					deltaPhi = maxVel*deltaT - 0.5*maxAccel*((n*deltaT-(t1+t2))*(n*deltaT-(t1+t2))-((n-1)*deltaT-(t1+t2))*((n-1)*deltaT-(t1+t2)));
					currVels.set(cubeNo,maxVel - maxAccel * (simulatedTime -(t1+t2)));
					//cubeSimPtr->getOutputStream() << "Phase 3, maxVel ->";
				}
				else
				{
					deltaPhi = 0.0;
					currVels.set(cubeNo, 0.0);
					//cubeSimPtr->getOutputStream() << "Phase 4, maxVel ->";
				}
			}
			//no
			else
			{
				if (simulatedTime < t)
				{
					//deltaPhi = 0.5*maxAccel*simulatedTime*simulatedTime;
					deltaPhi = 0.5*maxAccel*(n*deltaT*n*deltaT - (n-1)*deltaT*(n-1)*deltaT);
					currVels.set(cubeNo,maxAccel * simulatedTime);
					//cubeSimPtr->getOutputStream() << "Phase 1 ->";
				}
				else if ((simulatedTime > t) && (simulatedTime <= 2*t))
				{
					//deltaPhi = maxVel *simulatedTime - 0.5*maxAccel*(simulatedTime -t)*(simulatedTime-t);	
					deltaPhi = maxAccel*t*deltaT - 0.5*maxAccel*((n*deltaT-t)*(n*deltaT-t)-((n-1)*deltaT-t)*((n-1)*deltaT-t));
					currVels.set(cubeNo,maxAccel*t - maxAccel * (simulatedTime -t));
					//cubeSimPtr->getOutputStream() << "Phase 2 ->";
				}
				else
				{
					deltaPhi = 0.0;
					currVels.set(cubeNo, 0.0);
					//cubeSimPtr->getOutputStream() << "Phase 3 ->";
				}

			}
			currAngles=cubeSimPtr->getConfig();
			currAngles.set(cubeNo,currAngles[cubeNo]+deltaPhi);
			currDeltaAngle = targetAngle - currAngles[cubeNo];
			//cubeSimPtr->getOutputStream() << "cube "<<cubeNo<<"["<<simulatedTime<<"]: deltaPhi: "<<deltaPhi<<"; deltaAngle:"<<currDeltaAngle<<endl;

			//write new angle and velocity
			cubeSimPtr->setCurrentAngles(currAngles);
			cubeSimPtr->setCurrentJointVelocities(currVels);
		}
	}

	//we have finished our move
	cubeSimPtr->setStatusMoving(cubeNo,false);
	currVels.set(cubeNo, 0.0);
	cubeSimPtr->setCurrentJointVelocities(currVels);

	cubeSimPtr->getOutputStream() << "Thread finished for cube ID "<< (cubeSimPtr->getModuleMap())[cubeNo]<<endl;
	pthread_exit(NULL);
}


int PowerCubeSim::startSimulatedMovement(Jointd & targetAngles)
{
	if (statusMoving())
	{
		m_Out << "startSimulatedMovement: Movement already in progress, preemption not implemented yet! Aborting .."<<endl;
	}
	else
	{
		//create thread

		for (int i = 0; i < m_DOF; i++)
		{
			setStatusMoving(i,true);
			m_SimThreadArgs[i]->targetAngle = targetAngles[i];
			pthread_create(&m_SimThreadID[i],NULL,SimThreadRoutine,(void*)m_SimThreadArgs[i]);
		}
	}
	return 0;

}


/** Wait functions which is waiting that all cubes have done their homing 
*/
/* Not necessary
   void PowerCubeSim::HomingDone()
   {
   for(int i=0; i<m_NumOfModules; i++)
   {
   unsigned long int help;
   m_Out << "Warte auf Modul " << m_IdModules[i] << endl;
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
   m_Out << "Found module " << i << " with SerialNo " << serNo << "\n";
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
