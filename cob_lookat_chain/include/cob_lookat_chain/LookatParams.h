/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: cob_lookat_chain
 * \note
 *   ROS stack name: cob_driver
 * \note
 *   ROS package name: cob_lookat_chain
 *
 * \author
 *   Author: Felix Messmer, email:felix.messmer@ipa.fhg.de
 *
 * \date Date of creation: Feb 2014
 *
 * \brief
 *   Implementation of lookat parameters.
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

#ifndef __LOOKAT_PARAMS_H_
#define __LOOKAT_PARAMS_H_

/*!
 * \brief Parameters for cob_lookat_chain
 *
 * Initializing and setting parameters for cob_lookat_chain
 */
class LookatParams
{

public:

	/// Constructor
	LookatParams(unsigned int DOF)
	{
		m_DOF = DOF;
		m_UseMoveVel = true;
	};

	/// Destructor
	~LookatParams();

	/// Sets the DOF value
	void SetDOF(unsigned int DOF)
	{
		m_DOF = DOF;
	}
	/// Gets the DOF value
	unsigned int GetDOF()
	{
		return m_DOF;
	}

	/// Sets UseMoveVel
	void SetUseMoveVel(bool UseMoveVel)
	{
		m_UseMoveVel = UseMoveVel;
	}
	/// Gets UseMoveVel
	int GetUseMoveVel()
	{
		return m_UseMoveVel;
	}

	/// Sets the joint names
	int SetJointNames(std::vector<std::string> JointNames)
	{
		if (JointNames.size() == GetDOF())
		{
			m_JointNames = JointNames;
			return 0;
		}
		else
			return -1;
	}
	/// Gets the joint names
	std::vector<std::string> GetJointNames()
	{
		return m_JointNames;
	}

	/// Sets the upper angular limits (rad) for the joints
	int SetUpperLimits(std::vector<double> UpperLimits)
	{
		if (UpperLimits.size() == GetDOF())
		{
			m_UpperLimits = UpperLimits;
			return 0;
		}
		return -1;
	}
	/// Gets the upper angular limits (rad) for the joints
	std::vector<double> GetUpperLimits()
	{
		return m_UpperLimits;
	}

	/// Sets the lower angular limits (rad) for the joints
	int SetLowerLimits(std::vector<double> LowerLimits)
	{
		if (LowerLimits.size() == GetDOF())
		{
			m_LowerLimits = LowerLimits;
			return 0;
		}
		return -1;
	}
	/// Gets the lower angular limits (rad) for the joints
	std::vector<double> GetLowerLimits()
	{
		return m_LowerLimits;
	}

	/// Sets the max. angular velocities (rad/s) for the joints
	int SetMaxVel(std::vector<double> MaxVel)
	{
		if (MaxVel.size() == GetDOF())
		{
			m_MaxVel = MaxVel;
			return 0;
		}
		return -1;
	}
	/// Gets the max. angular velocities (rad/s) for the joints
	std::vector<double> GetMaxVel()
	{
		return m_MaxVel;
	}

	/// Sets the max. angular accelerations (rad/s^2) for the joints
	int SetMaxAcc(std::vector<double> MaxAcc)
	{
		if (MaxAcc.size() == GetDOF())
		{
			m_MaxAcc = MaxAcc;
			return 0;
		}
		return -1;
	}
	/// Gets the max. angular accelerations (rad/s^2) for the joints
	std::vector<double> GetMaxAcc()
	{
		return m_MaxAcc;
	}


private:
	unsigned int m_DOF;
	std::vector<std::string> m_JointNames;
	bool m_UseMoveVel;
	std::vector<double> m_UpperLimits;
	std::vector<double> m_LowerLimits;
	std::vector<double> m_MaxVel;
	std::vector<double> m_MaxAcc;
};

#endif
