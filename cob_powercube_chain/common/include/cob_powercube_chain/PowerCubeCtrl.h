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
 *   Implementation of powercube control.
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

#ifndef __POWER_CUBE_CTRL_H_
#define __POWER_CUBE_CTRL_H_

#include <iostream>
#include <sstream>
#include <string>

#include <pthread.h>

#include <libm5api/m5apiw32.h>
#include <cob_powercube_chain/moveCommand.h>
#include <cob_powercube_chain/PowerCubeCtrlParams.h>

//-------------------------------------------------------------------------
//                              Defines
// -------------------------------------------------------------------------

/* uncomment the following line to switch on debugging output: */
// #define _POWER_CUBE_CTRL_DEBUG

class PowerCubeCtrl
{
public:

  PowerCubeCtrl(PowerCubeCtrlParams * params);
  ~PowerCubeCtrl();

  typedef enum
  {
    PC_CTRL_OK = 0, PC_CTRL_NOT_REFERENCED = -1, PC_CTRL_ERR = -2, PC_CTRL_POW_VOLT_ERR = -3
  } PC_CTRL_STATE;

  /////////////////////////////////////////////
  // Functions for initialization and close: //
  /////////////////////////////////////////////
  bool Init(PowerCubeCtrlParams * params);

  bool isInitialized() const
  {
    return m_Initialized;
  }

  std::string getErrorMessage() const
  {
    return m_ErrorMessage;
  }

  bool Close();

  ////////////////////////////
  // Functions for control: //
  ////////////////////////////

  /// @brief Send position goals to powercubes, the final angles will be reached simultaneously
  bool MoveJointSpaceSync(const std::vector<double>& angles);

  /// @brief Moves all cubes by the given velocities
  bool MoveVel(const std::vector<double>& velocities);

  /// @brief Stops the Manipulator immediately
  bool Stop();

  /// @brief Recovery after emergency stop or power supply failure
  bool Recover();

  //////////////////////////////////
  // functions to set parameters: //
  //////////////////////////////////

  /// @brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
  /// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
  bool setMaxVelocity(double velocity);
  bool setMaxVelocity(const std::vector<double>& velocities);

  /// @brief Sets the maximum angular acceleration (rad/s^2) for the Joints, use with care!
  /// A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
  bool setMaxAcceleration(double acceleration);
  bool setMaxAcceleration(const std::vector<double>& accelerations);

  /// @brief Configure powercubes to start all movements synchronously
  /// Tells the Modules not to start moving until PCube_startMotionAll is called
  bool setSyncMotion();

  /// @brief Configure powercubes to start all movements asynchronously
  /// Tells the Modules to start immediately
  bool setASyncMotion();

  /////////////////////////////////////////////////
  // Functions for getting state and monitoring: //
  /////////////////////////////////////////////////

  /// @brief Returns the state of all modules
  bool getStates(std::vector<unsigned long>& states, std::vector<unsigned char>& dios, std::vector<double>& positions);

  /// @brief Gets the status of the modules
  bool getStatus(PC_CTRL_STATE& error, std::vector<std::string>& errorMessages);

  /// @brief Returns true if any of the Joints are still moving
  /// Should also return true if Joints are accelerating or decelerating
  bool statusMoving();

  /// @brief Waits until all Modules are homed.
  bool doHoming();

protected:
  pthread_mutex_t m_mutex;

  int m_DeviceHandle;
  bool m_Initialized;
  bool m_CANDeviceOpened;

  PowerCubeCtrlParams* m_params;

  std::vector<unsigned long> m_status;
  std::vector<unsigned char> m_dios;
  std::vector<float> m_positions;
  std::vector<float> m_velocities;

  std::string m_ErrorMessage;

};

#endif
