/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2010 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_powercube_chain
 *
 * \author
 *   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * \author
 *   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 *
 * \date Date of creation: Dec 2010
 *
 * \brief
 *   Implementation of powercube_chain node.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cob_powercube_chain/PowerCubeCtrl.h>

#define PCTRL_CHECK_INITIALIZED() \
if ( isInitialized()==false )													\
{																		\
    m_ErrorMessage.assign("Manipulator not initialized.");              \
	return false;														\
}

PowerCubeCtrl::PowerCubeCtrl()
{
  m_CANDeviceOpened = false;
  m_Initialized = false;
}

/** The Deconstructor
 */
PowerCubeCtrl::~PowerCubeCtrl()
{
  if (m_CANDeviceOpened)
  {
    PCube_closeDevice(m_DeviceHandle);
  }
}

bool PowerCubeCtrl::Init(PowerCubeCtrlParams * params)
{
  std::string CanModule = "";
  int CanDevice = 0;
  std::vector<int> ModulIDs;
  int CanBaudrate = 0;
  std::vector<double> maxVel;
  std::vector<double> maxAcc;
  std::vector<double> offsets;
  std::vector<double> upperLimits;
  std::vector<double> lowerLimits;

  if (params != NULL)
  {
    m_DOF = params->GetDOF();
    m_ModulIDs = params->GetModuleIDs();

    m_status.resize(m_DOF);
    m_dios.resize(m_DOF);
    m_positions.resize(m_DOF);

    CanModule = params->GetCanModule();
    CanDevice = params->GetCanDevice();
    CanBaudrate = params->GetBaudrate();
    maxVel = params->GetMaxVel();
    maxAcc = params->GetMaxAcc();
    lowerLimits = params->GetLowerLimits();
    upperLimits = params->GetUpperLimits();
    offsets = params->GetOffsets();
  }
  else
  {
    std::cout << "PowerCubeCtrl::Init: Error, parameters == NULL" << std::endl;
    return false;
  }
  std::cout << "=========================================================================== " << std::endl;
  std::cout << "PowerCubeCtrl:Init: Trying to initialize with the following parameters: " << std::endl;
  std::cout << "DOF: " << m_DOF << std::endl;
  std::cout << "CanModule: " << CanModule << std::endl;
  std::cout << "CanDevice: " << CanDevice << std::endl;
  std::cout << "CanBaudrate: " << CanBaudrate << std::endl;
  std::cout << "ModulIDs: ";
  for (int i = 0; i < m_DOF; i++)
  {
    std::cout << m_ModulIDs[i] << " ";
  }
  std::cout << std::endl << "maxVel: ";
  for (int i = 0; i < m_DOF; i++)
  {
    std::cout << maxVel[i] << " ";
  }
  std::cout << std::endl << "maxAcc: ";
  for (int i = 0; i < m_DOF; i++)
  {
    std::cout << maxAcc[i] << " ";
  }
  std::cout << std::endl << "upperLimits: ";
  for (int i = 0; i < m_DOF; i++)
  {
    std::cout << upperLimits[i] << " ";
  }
  std::cout << std::endl << "lowerLimits: ";
  for (int i = 0; i < m_DOF; i++)
  {
    std::cout << lowerLimits[i] << " ";
  }
  std::cout << std::endl << "offsets: ";
  for (int i = 0; i < m_DOF; i++)
  {
    std::cout << offsets[i] << " ";
  }
  std::cout << std::endl << "=========================================================================== " << std::endl;
  std::ostringstream initStr;
  initStr << CanModule << ":" << CanDevice << "," << CanBaudrate;
  std::cout << "initstring = " << initStr.str().c_str() << std::endl;
  int ret = 0;

  // open device
  ret = PCube_openDevice(&m_DeviceHandle, initStr.str().c_str());

  if (ret != 0)
  {
    std::ostringstream errorMsg;
    errorMsg << "Could not open device, m5api error code: " << ret;
    m_ErrorMessage = errorMsg.str();
    return false;
  }
  m_CANDeviceOpened = true;

  // reset all modules
  PCube_resetAll(m_DeviceHandle);

  // Make sure m_IdModules is clear of Elements:
  m_ModulIDs.clear();

  for (int i = 0; i < m_DOF; i++)
  {

    // Check if the Module is connected:
    unsigned long serNo;

    int ret = PCube_getModuleSerialNo(m_DeviceHandle, m_ModulIDs[i], &serNo);

    // if not return false ( ret == 0 means success)
    if (ret != 0)
    {
      std::ostringstream errorMsg;
      errorMsg << "Could not find Module with ID " << m_ModulIDs[i];
      m_ErrorMessage = errorMsg.str();
      return false;
    }

    // otherwise success, save Id in m_IdModules
    std::cout << "found module " << m_ModulIDs[i] << std::endl;
  }

  std::vector<std::string> errorMessages;
  PC_CTRL_STATE status;
  getStatus(status, errorMessages);
  if ((status != PC_CTRL_OK) && (status != PC_CTRL_NOT_REFERENCED))
  {
    m_ErrorMessage.assign("");
    for (int i = 0; i < m_DOF; i++)
    {
      m_ErrorMessage.append(errorMessages[i]);
      m_ErrorMessage.append("\n");
    }
    return false;
  }
  else if (status == PC_CTRL_NOT_REFERENCED)
  {
    std::cout << "PowerCubeCtrl:Init: Homing is executed ...\n";
    bool successful = false;
    successful = doHoming();
    if (!successful)
    {
      std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
      return false;
    }
  }

  // Set angle offsets to hardware
  for (int i = 0; i < m_DOF; i++)
  {
    PCube_setHomeOffset(m_DeviceHandle, m_ModulIDs[i], offsets[i]);
  }

  // Set limits to hardware
  for (int i = 0; i < m_DOF; i++)
  {
    PCube_setMinPos(m_DeviceHandle, m_ModulIDs[i], lowerLimits[i]);
    PCube_setMaxPos(m_DeviceHandle, m_ModulIDs[i], upperLimits[i]);
  }

  // Set max velocity to hardware
  setMaxVelocity(maxVel);

  // Set max acceleration to hardware
  setMaxAcceleration(maxAcc);

  // Configure powercubes to start all movements synchronously
  // Tells the Modules not to start moving until PCube_startMotionAll is called
  if (m_CANDeviceOpened)
  {
    for (int i = 0; i < m_DOF; i++)
    {
      unsigned long confword;

      // get config
      PCube_getConfig(m_DeviceHandle, m_ModulIDs[i], &confword);

      // set config to synchronous
      PCube_setConfig(m_DeviceHandle, m_ModulIDs[i], confword | CONFIGID_MOD_SYNC_MOTION);

      // alternatively set config to asynchronous
      //PCube_setConfig(m_DeviceHandle, m_ModulIDs[i], confword & (~CONFIGID_MOD_SYNC_MOTION));
    }
    return true;
  }
  else
  {
    return false;
  }

  m_Initialized = true;

  return true;
}

bool PowerCubeCtrl::Close()
{
  if (m_CANDeviceOpened)
  {
    m_Initialized = false;
    m_CANDeviceOpened = false;

    PCube_closeDevice(m_DeviceHandle);

    return true;
  }
  else
  {
    return false;
  }
}

bool PowerCubeCtrl::MoveJointSpaceSync(const std::vector<double>& target)
{
  PCTRL_CHECK_INITIALIZED();

  std::vector<std::string> errorMessages;
  PC_CTRL_STATE status;
  getStatus(status, errorMessages);
  if ((status != PC_CTRL_OK))
  {
    m_ErrorMessage.assign("");
    for (int i = 0; i < m_DOF; i++)
    {
      m_ErrorMessage.append(errorMessages[i]);
      m_ErrorMessage.append("\n");
    }
    return false;
  }

  std::vector<double> vel(m_DOF);
  std::vector<double> acc(m_DOF);

  double TG = 0;

  try
  {

    // calculate which joint takes the longest time to reach goal
    int DOF = m_DOF;

    std::vector<double> times(DOF);

    for (int i = 0; i < DOF; i++)
    {
      RampCommand rm(m_positions[i], m_velocities[i], target[i], m_MaxAccelerations[i], m_MaxVelocities[i]);
      times[i] = rm.getTotalTime();
    }

    // determine the joint index that has the greatest value for time
    int furthest = 0;

    double max = times[0];

    for (int i = 1; i < m_DOF; i++)
    {
      if (times[i] > max)
      {
        max = times[i];
        furthest = i;
      }
    }

    RampCommand rm_furthest(m_positions[furthest], m_velocities[furthest], target[furthest],
                            m_MaxAccelerations[furthest], m_MaxVelocities[furthest]);

    double T1 = rm_furthest.T1();
    double T2 = rm_furthest.T2();
    double T3 = rm_furthest.T3();

    // total time:
    TG = T1 + T2 + T3;

    // calculate velocity and acceleration for all joints:
    acc[furthest] = m_MaxAccelerations[furthest];
    vel[furthest] = m_MaxVelocities[furthest];

    for (int i = 0; i < DOF; i++)
    {
      if (i != furthest)
      {
        double a;
        double v;
        RampCommand::calculateAV(m_positions[i], m_velocities[i], target[i], TG, T3, m_MaxAccelerations[i],
                                 m_MaxVelocities[i], a, v);

        acc[i] = a;
        vel[i] = v;
      }
    }
  }
  catch (...)
  {
    return false;
  }

  // Send motion commands to hardware
  for (int i = 0; i < m_DOF; i++)
  {
    PCube_moveRamp(m_DeviceHandle, m_ModulIDs[i], target[i], fabs(vel[i]), fabs(acc[i]));
  }

  PCube_startMotionAll(m_DeviceHandle);

  return true;
}

bool PowerCubeCtrl::MoveVel(const std::vector<double>& vel)
{
  PCTRL_CHECK_INITIALIZED();
  /*
   vector<string> errorMessages;
   PC_CTRL_STATE status;
   getStatus(status, errorMessages);
   if ((status != PC_CTRL_OK))
   {
   m_ErrorMessage.assign("");
   for (int i=0; i<m_DOF; i++)
   {
   m_ErrorMessage.append(errorMessages[i]);
   m_ErrorMessage.append("\n");
   }
   return false;
   }
   */
  unsigned char pucDio;
  float posi = 0;
  unsigned long status;
  for (int i = 0; i < m_DOF; i++)
  {
    //PCube_moveVel(m_Dev, m_IdModules[i], vel[i] );
    PCube_moveVelExtended(m_DeviceHandle, m_ModulIDs[i], vel[i], &status, &pucDio, &posi);
    m_positions[i] = posi;
    m_status[i] = status;
  }
  PCube_startMotionAll(m_DeviceHandle);

  return true;
}

/// @brief Stops the manipulator immediately
bool PowerCubeCtrl::Stop()
{
  // stop should be executes without checking any conditions
  PCube_haltAll(m_DeviceHandle);

  // after halt the modules don't accept move commands any more, they first have to be reseted
  usleep(500000);
  PCube_resetAll(m_DeviceHandle);

  return true;
}

/// @brief Recovers the manipulator after an emergency stop
bool PowerCubeCtrl::Recover()
{
  std::vector<std::string> errorMessages;
  PC_CTRL_STATE status;
  getStatus(status, errorMessages);
  if (status == PC_CTRL_NOT_REFERENCED)
  {
    std::cout << "PowerCubeCtrl:Init: Homing is executed ...\n";
    bool successful = false;
    successful = doHoming();
    if (!successful)
    {
      std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
    }
  }
  PCube_resetAll(m_DeviceHandle);

  getStatus(status, errorMessages);
  if ((status != PC_CTRL_OK))
  {
    m_ErrorMessage.assign("");
    for (int i = 0; i < m_DOF; i++)
    {
      m_ErrorMessage.append(errorMessages[i]);
      m_ErrorMessage.append("\n");
    }
    return false;
  }
  else
  {
    return true;
  }

}

/// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeCtrl::setMaxVelocity(double maxVelocity)
{
  PCTRL_CHECK_INITIALIZED();
  for (int i = 0; i < m_DOF; i++)
  {
    PCube_setMaxVel(m_DeviceHandle, m_ModulIDs[i], maxVelocity);
    m_MaxVelocities[i] = maxVelocity;
  }

  return true;
}

bool PowerCubeCtrl::setMaxVelocity(const std::vector<double>& maxVelocities)
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_DOF; i++)
  {
    PCube_setMaxVel(m_DeviceHandle, m_ModulIDs[i], maxVelocities[i]);
    m_MaxVelocities[i] = maxVelocities[i];
  }

  return true;
}

/// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
/// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
bool PowerCubeCtrl::setMaxAcceleration(double maxAcceleration)
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_DOF; i++)
  {
    //m_maxAcc[i] = radPerSecSquared;
    PCube_setMaxAcc(m_DeviceHandle, m_ModulIDs[i], maxAcceleration);
    m_MaxAccelerations[i] = maxAcceleration;
  }

  return true;
}

bool PowerCubeCtrl::setMaxAcceleration(const std::vector<double>& maxAccelerations)
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_DOF; i++)
  {
    //m_maxAcc[i] = radPerSecSquared[i];
    PCube_setMaxAcc(m_DeviceHandle, m_ModulIDs[i], maxAccelerations[i]);
    m_MaxAccelerations[i] = maxAccelerations[i];
  }

  return true;
}

/// @brief Returns the current states
bool PowerCubeCtrl::getStates(std::vector<unsigned long>& states, std::vector<unsigned char>& dios,
                              std::vector<double>& positions)
{
  PCTRL_CHECK_INITIALIZED();

  states.resize(m_DOF);
  dios.resize(m_DOF);
  positions.resize(m_DOF);

  unsigned long state;
  unsigned char dio;
  float position;
  for (int i = 0; i < m_DOF; i++)
  {
    PCube_getStateDioPos(m_DeviceHandle, m_ModulIDs[i], &state, &dio, &position);
    states[i] = state;
    dios[i] = dio;
    positions[i] = position;
  }

  return true;
}

/// @brief Gets the status of the modules
bool PowerCubeCtrl::getStatus(PC_CTRL_STATE& error, std::vector<std::string>& errorMessages)
{
  errorMessages.clear();
  errorMessages.resize(m_DOF);

  error = PC_CTRL_OK;

  for (int i = 0; i < m_DOF; i++)
  {
    unsigned long int state;
    std::ostringstream errorMsg;

    //PCube_getModuleState(m_DeviceHandle, m_ModulIDs[i], &state);

    if (state & STATEID_MOD_POW_VOLT_ERR)
    {
      errorMsg << "Error in Module " << m_ModulIDs[i] << ": ";
      errorMsg << "Motor voltage below minimum value!";
      errorMessages[i] = errorMsg.str();
      error = PC_CTRL_POW_VOLT_ERR;
    }
    else if (!(state & STATEID_MOD_HOME))
    {
      errorMsg << "Warning: Module " << m_ModulIDs[i];
      errorMsg << " is not referenced!";
      errorMessages[i] = errorMsg.str();
      error = PC_CTRL_NOT_REFERENCED;
    }
    else if (state & STATEID_MOD_ERROR)
    {
      errorMsg << "Error in  Module " << m_ModulIDs[i];
      errorMsg << " : Status code: " << std::hex << state;
      errorMessages[i] = errorMsg.str();
      error = PC_CTRL_ERR;
    }
    else
    {
      errorMsg << "Module with Id " << m_ModulIDs[i];
      errorMsg << ": Status OK.";
      errorMessages[i] = errorMsg.str();
    }
  }
  return true;
}

/// @brief Returns true if some cubes are still moving
bool PowerCubeCtrl::statusMoving()
{
  PCTRL_CHECK_INITIALIZED();

  for (int i = 0; i < m_DOF; i++)
  {
    unsigned long status;

    //PCube_getModuleState(m_Dev,m_IdModules[i], &status);

    if (status & STATEID_MOD_MOTION)
      return true;
  }
  return false;
}

/// @brief does homing for all Modules
bool PowerCubeCtrl::doHoming()
{
  // start homing
  for (int i = 0; i < m_DOF; i++)
  {
    std::cout << "Module " << m_ModulIDs[i] << " homed" << std::endl;
    PCube_homeModule(m_DeviceHandle, m_ModulIDs[i]);
  }

  // wait until all modules are homed
  for (int i = 0; i < m_DOF; i++)
  {
    unsigned long int state;
    do
    {
      PCube_getModuleState(m_DeviceHandle, m_ModulIDs[i], &state);

      usleep(100000);
    } while ((state & STATEID_MOD_HOME) == 0);
  }

  return true;
}
