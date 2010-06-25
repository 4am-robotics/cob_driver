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
 * Description: Class definition of the datastructsManipulator.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: Nov 2006
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

#ifndef __DATASTRUCTS_MANIPULATOR_H_
#define __DATASTRUCTS_MANIPULATOR_H_

#include <libwm4/Wm4Matrix4.h>
#include <libwm4/Wm4Matrix3.h>
#include <cob_powercube_chain/Joint.h>
#include <iostream>

// never in headerfiles!
// using namespace std;

#ifdef SWIG
%module PowerCubeCtrl
%{
	#include <cob_powercube_chain/datastructsManipulator.h>
%}
#endif 


/** 
 * @brief Definition of the used structs in the project Path planner
 * @author Katrin Thurow <katrinthurow@hotmail.com>
 */

struct Point3D {
    double x;
    double y;
    double z;
};

struct AbsPos {
	AbsPos() : Eulerx(0.0), Eulery(0.0), Eulerz(0.0), Transx(0.0), Transy(0.0), Transz(0.0) {;}
	// Bitte beachten: Es werden von nun an XYZ-fixed Winkel benutzt
	// (Siehe Craig, S.45ff)
	// welche gleichbedeutend mit den Euler-ZYX Winkeln sind.
	// Die Bezeichnungen Eulerx, Eulery, Eulerz werden daher beibehalten. 
    double Eulerx;
    double Eulery;
    double Eulerz;
    double Transx;
    double Transy;
    double Transz;
	void set(double* p);
	void setTransX(double transx) {Transx=transx;}
	void setTransY(double transy) {Transy=transy;}
	void setTransZ(double transz) {Transz=transz;}
	void setEulerX(double eulerx) {Eulerx=eulerx;}
	void setEulerY(double eulery) {Eulery=eulery;}
	void setEulerZ(double eulerz) {Eulerz=eulerz;}
	double getTransX() {return Transx;}
	double getTransY() {return Transy;}
	double getTransZ() {return Transz;}
	double getEulerX() {return Eulerx;}
	double getEulerY() {return Eulery;}
	double getEulerZ() {return Eulerz;}
	void angleScale(double s);
	void toDeg() { angleScale(57.295779524); }
	void toRad() { angleScale(0.017453292); }
	AbsPos operator*(double s) const;
	AbsPos operator+(const AbsPos& abs2) const;
	AbsPos operator-(const AbsPos& abs2) const;
	double getPosLength() const { return sqrt(Transx*Transx + Transy*Transy + Transz*Transz); }
	/// @brief in distMeasure(otherPos) wird die Winkelabweichung berücksichtigt
	/// Hierbei wird der RMS der Differenzen der Eulerwinkel benutzt
	/// Wenn Trans... in mm wird 1° Abweichung wie 1mm Abweichung gewichtet.
	/// TODO: Wünschenswert wäre ein besseres Maß für die Lagedifferenz (z.B. Angle-Axis-Wikel)
	double distMeasure(AbsPos otherPos) const;
};

std::ostream& operator<< (std::ostream& os, const AbsPos& a);

std::ostream& operator<< (std::ostream& os, const Wm4::Matrix4 <double> & m);
std::ostream& operator<< (std::ostream& os, const Wm4::Matrix3 <double> & m);

inline AbsPos operator* (double s, const AbsPos& abs) { return abs * s; }



struct DH {
    Jointd  a;
    Jointd  d;
    Jointd  alpha;
    Jointd  theta;
};

struct LimitsTheta {
    Jointd max;
    Jointd min;	
};


#endif //__DATASTRUCTS_MANIPULATOR_H_

