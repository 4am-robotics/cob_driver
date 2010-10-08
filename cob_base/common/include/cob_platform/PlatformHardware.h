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
 * ROS package name: cob_platform
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2010
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

class PltfHardwareCoB3;

class PlatformHardware
{
public:
    PlatformHardware();
    ~PlatformHardware();

    /**
	 * Initializes system.
	 */
    bool initPltf();

	/// Disable motors, enable brake, disconnect.
    bool shutdownPltf();

    /**
	 * Sets the platform velocity.
	 *
	 * @param dVelLongMMS = Longitudinale Geschwindigkeit der Plattform
	 * @param dVelLatMMS = Transversale Geschwindigkeit der Plattform
	 * @param dRotRobRadS = Rotationsrate der Plattform
	 * @param dRotVelRadS = Rotationsrate des Geschwindigkeitsvektors der Plattform (derzeit nicht benutzt)
	 */
    void setVelPltf(double dVelLongMMS, double dVelLatMMS, double dRotRobRadS, double dRotVelRadS);

	/**
	 * Updates of the can buffer or simulation (deactivated in omnidir. Pltf's - COb3).
	 */
    int update();

	/**
	 * Gets the measured position increment and the measured velocities.
	 */
    void getDeltaPosePltf(double& dDeltaLongMM, double& dDeltaLatMM, double& dDeltaRotRobRad, double& dDeltaRotVelRad,
    					  double& dVelLongMMS, double& dVelLatMMS, double& dRotRobRadS, double& dRotVelRadS);

private:
    PltfHardwareCoB3* d;
};
