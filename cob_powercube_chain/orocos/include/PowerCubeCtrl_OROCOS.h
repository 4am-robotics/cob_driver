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
 * ROS stack name: cob3_driver
 * ROS package name: powercube_chain
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Dec 2009
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

#ifndef _POWERCUBECTRL_OROCOS_
#define _POWERCUBECTRL_OROCOS_

#include <rtt/TaskContext.hpp>
#include <rtt/Command.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Property.hpp>
#include <rtt/marsh/PropertyMarshaller.hpp>
#include "PowerCubeCtrl.h"

#include <iostream>
using namespace std;


class PowerCubeCtrl_OROCOS : public RTT::TaskContext
{
	public:

		PowerCubeCtrl_OROCOS(std::string name, std::string file, int dof);
		//PowerCubeCtrl_OROCOS(std::string name);
		~PowerCubeCtrl_OROCOS();

		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook(){}

		/* The dataports have the following names:
		"in_Angles"
		"in_Velocities"
		"in_Currents"
		"out_Angles"
		"out_Velocities"
		*/

	protected:
	
		bool stopArm();
		bool isArmStopped();
		bool moveJointSpace(std::vector<double> target);
		//bool m_stopped;

		std::string m_xmlFile;
		int m_dof;

		RTT::ReadDataPort< std::vector<double> > m_in_Angles;
		bool m_in_Angles_connected;

		RTT::ReadDataPort< std::vector<double> > m_in_Velocities; // MoveVel
		bool m_in_Velocities_connected;

		RTT::ReadDataPort< std::vector<double> > m_in_Currents;
		bool m_in_Currents_connected;

		RTT::WriteDataPort< std::vector<double> > m_out_Angles; // getConfig
		RTT::WriteDataPort< std::vector<double> > m_out_Velocities; // getJointVelocities
		//WriteDataPort< std::vector<double> > m_out_Currents;

		RTT::Method<bool(void)> m_stop_method;
		//RTT::Method<bool(std::vector<double>)> m_moveJointSpace_method;

		RTT::Property<int> m_CanDev_prop;
		RTT::Property<int> m_CanBaud_prop;
		RTT::Property<int> m_dof_prop;
		RTT::Property<double> m_MaxVel_prop;
		RTT::Property<double> m_MaxAcc_prop;

		std::vector< RTT::PropertyBag > m_mod_params_bags;
		std::vector< RTT::Property<RTT::PropertyBag> > m_mod_params;

		std::vector< RTT::Property<int> > m_modId_props;
		std::vector< RTT::Property<double> > m_ul_props;
		std::vector< RTT::Property<double> > m_ll_props;
		std::vector< RTT::Property<double> > m_offset_props;

		PowerCubeCtrl m_powercubectrl;

};

#endif

