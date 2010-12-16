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

#ifndef __POWER_CUBE_CTRL_PARAMS_H_
#define __POWER_CUBE_CTRL_PARAMS_H_

class PowerCubeCtrlParams
{

        public:
                PowerCubeCtrlParams(){m_DOF = 0;};
                ~PowerCubeCtrlParams();

                int Init(std::string CanModule, int CanDevice, int Baudrate, std::vector<int> ModuleIDs)
                {
                        SetCanModule(CanModule);
                        SetCanDevice(CanDevice);
                        SetBaudrate(Baudrate);
                        SetNumberOfDOF(ModuleIDs.size());
                        for (unsigned int i=0; i < GetDOF() ;i++)
                        {
                                m_IDModulesNumber.push_back(ModuleIDs[i]);
                        }
                        return 0;
                }

                //DOF
                void SetNumberOfDOF(unsigned int DOF){m_DOF=DOF;}
                unsigned int GetDOF(){return m_DOF;}

                //Can Module
                void SetCanModule(std::string CanModule){m_CanModule = CanModule;}
                std::string GetCanModule(){return m_CanModule;}

                //Can Device
                void SetCanDevice(int CanDevice){m_CanDevice = CanDevice;}
                int GetCanDevice(){return m_CanDevice;}

                //Baudrate
                void SetBaudrate(int Baudrate){m_Baudrate=Baudrate;}
                int GetBaudrate(){return m_Baudrate;}

                //ModuleIDs
                std::vector<int> GetModuleIDs(){return m_IDModulesNumber;}
                int GetModuleID(unsigned int no){if (no < GetDOF()) return m_IDModulesNumber[no]; else return -1;}
                int SetModuleID(unsigned int no, int id){
                        if (no < GetDOF())
                        {
                                m_IDModulesNumber[no] = id;
                                return 0;
                        }
                        else
                                return -1;

                }

                //Angular Constraints
                int SetUpperLimits(std::vector<double> UpperLimits)
                {
                        if (UpperLimits.size() == GetDOF())
                        {
                                m_UpperLimits = UpperLimits;
                                return 0;

                        }
                        return -1;
                }
                int SetLowerLimits(std::vector<double> LowerLimits)
                {
                        if (LowerLimits.size() == GetDOF())
                        {
                                m_LowerLimits = LowerLimits;
                                return 0;
                        }
                        return -1;
                }
                int SetOffsets(std::vector<double> AngleOffsets)
                {
                        if (AngleOffsets.size() == GetDOF())
                        {
                                m_Offsets = AngleOffsets;
                                return 0;
                        }
                        return -1;
                }
                int SetMaxVel(std::vector<double> MaxVel)
                {
                        if (MaxVel.size() == GetDOF())
                        {
                                m_MaxVel = MaxVel;
                                return 0;
                        }
                        return -1;
                }
                int SetMaxAcc(std::vector<double> MaxAcc)
                {
                        if (MaxAcc.size() == GetDOF())
                        {
                                m_MaxAcc = MaxAcc;
                                return 0;
                        }
                        return -1;
                }

                std::vector<double> GetUpperLimits(){return m_UpperLimits;}
                std::vector<double> GetLowerLimits(){return m_LowerLimits;}
                std::vector<double> GetOffsets(){return m_Offsets;}
                std::vector<double> GetMaxAcc(){return m_MaxAcc;}
                std::vector<double> GetMaxVel(){return m_MaxVel;}


        private:
                std::vector<int> m_IDModulesNumber;
                unsigned int m_DOF;
                std::string m_CanModule;
                int m_CanDevice;
                int m_Baudrate;
                std::vector<double> m_Offsets;
                std::vector<double> m_UpperLimits;
                std::vector<double> m_LowerLimits;
                std::vector<double> m_MaxVel;
                std::vector<double> m_MaxAcc;
};

#endif
