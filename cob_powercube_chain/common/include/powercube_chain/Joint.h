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
 * Description: Definition and inline implementation of class template Joint.
 *     Note: Jointd = Joint<double>
 *           Jointf = Joint<float>
 *     There is no Joint.cpp, all functions are defined inline in this file.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Felix Geibel, email:Felix.Geibel@gmx.de
 * Supervised by: Alexander Bubeck, email:alexander.bubeck@ipa.fhg.de
 *
 * Date of creation: June 2008
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

#ifndef _Joint_H
#define _Joint_H

/*******************************************************************************
 *  includes & SWIG support                                                    *
 *******************************************************************************/

#include <math.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cstdio>
#include <stdexcept> // stdexcept header file contains runtime_error
#include <limits>




/*******************************************************************************
 *  defines:                                                                   *
 *******************************************************************************/

/* if length(Joint1 - Joint2) < JOINT_EPSILON, they are considered equal: */
#ifndef JOINT_EPSILON
#define JOINT_EPSILON .01 / 360.0
#endif

//#define PI 3.141592654
#define DEGS_PER_RAD 57.29577951
#define RADS_PER_DEG 0.017453292

/* if for some reason (faster performance) you want this class to not perform 
   rangechecking, define JOINT_NO_RANGECHECK in your config or makefile!       */
   
// #define JOINT_NO_RANGECHECK

/*******************************************************************************
 *  Exception class (definition & implementation)                              *
 *******************************************************************************/

/* In addition to the standard runtime class, this one takes file and Line number arguments. */
class Joint_Exception : public std::runtime_error
{
	public:
	Joint_Exception(const char* _file, int _line, const char* _msg) :
		std::runtime_error( _msg ),
		file(_file), line(_line) { msg = _msg; }
	const char* getFile() const { return file; }
	int getLine() const { return line; }
	const char* getMsg() const { return msg; }
	private:
	const char* file;
	int line;
	const char* msg;
};

/* operator<< so you can use an exception e like: "cout << e" */
inline std::ostream& operator<<(std::ostream& _os, const Joint_Exception& e)
{
	_os << "Error in " << e.getFile();
	_os << " at line " << e.getLine() << ":" << std::endl;
	_os << " " << e.getMsg() << std::endl;
	return _os;
}

/*******************************************************************************
 *  class definition of template class Joint                                   *
 *******************************************************************************/

template <class Real>
class Joint
{
public:

	/* constructors: */
	
	/* empty Joint, no elements: */
    Joint();
    /* joint of size NrJoints, zero=false: uninitialized, zero=true -> all elements zero: */
	Joint(unsigned int NrJoints, bool zero=false);
	/* copy constructor: */
	Joint(const Joint& joints);
	/* size NrJoints, content copied from d: */
	Joint(unsigned int NrJoints, const Real d[]);
    
    /* destructor: */
    virtual ~Joint();
    
    /* adjust the size of the Joint: */
	void setNrJoints(unsigned int NrJoints);
	/* make all elements zero: */
	void zero();

    /* element access (write): */
    
	/* set element i to value d: */
	void set(unsigned int i,  Real d);
	Real& operator[](unsigned int i);
	/* set Joint to size NrJoints and copy contents from d: */
	void set(unsigned int NrJoints, Real *d);
	/* copy constructor: */
	//void set(unsigned int const Joint& _rhs);
	
	/* element access (read): */
    
    /* return element i: */
	Real get(unsigned int i) const;
	Real operator[](unsigned int i) const;
	/* copy Joint values to array d (d must have enough allocated memory!): */
	void get(unsigned int NrJoints, Real* d) const;
	/* return number of elements: */
	unsigned int size() const;
	
	/* Return value of the largest/smallest component: */
	Real getMax() const;
	Real getMin() const;
	
	/* Return Index of largest/smallest component: */
	unsigned int getMaxInd() const;
	unsigned int getMinInd() const;
    
    /* calculate this + (j2-this)*f: */
	Joint interpolate( const Joint& j2, Real f ) const;
	/* set this joint to j1 + (j2-j1)*f: */
	static Joint interpolate( const Joint& j1, const Joint& j2, Real f );
	
	/* convert the Joint from Deg to Rad or vice versa: */
	void toRad();
	void toDeg();
	
	/* define various operators: */

	/* operator= */
	Joint& operator=(const Joint& joint);
	
	/* if length(this - rhs) < JOINT_EPSILON, they are considered equal: */
	bool operator==(const Joint& rhs) const;
    
    /* operators +, -, *, / */
	Joint operator-(const Joint& rhs) const;
	Joint operator+(const Joint& rhs) const;
	Joint operator*(Real s) const;
	Joint operator/(Real s) const;
	
	void operator+=(const Joint& rhs);
	void operator-=(const Joint& rhs);
	void operator*=(Real s);
	void operator/=(Real s);
	
	/* get euklidian norm (length), squared or normal */
	/* (not taking the root saves computation time) */
	Real lengthSqr() const;
	Real length() const;
    
    /* return string which looks like: (0.000, 1.000, 2.000, ...) */
    /* if convert is true, the values will be converted to degrees */ 
	std::string toString(bool convert = false) const;
	/* read in the joint from a c-string, format as above: */
	void fromString(unsigned int nrjoints, const char* str);
	void fromString(const char* str);

	
	/* input / output */
	
	/* operator<< is not a member function and defined below the class definition */
	/* same for operator>> */
	
	/* if for some reason printf should be used rather than operator<< use this: */
	void print();
	
	/* for compatibility with older code: */
	unsigned int getNrJoints() const { return size(); }


private:
	unsigned int m_NrJoints;
	Real* m_Joints;
	};

typedef Joint<double> Jointd;
typedef Joint<float> Jointf;


/*******************************************************************************
 * prototypes for std input output                                             *
 *******************************************************************************/

template <class Real>
std::ostream& operator<<(std::ostream& _os, const Joint<Real>& joint);

template <class Real>
std::istream& operator>>(std::istream&, Joint<Real>& joint);


/*******************************************************************************
 * function for spline compatibility                                           *
 *******************************************************************************/

template <class Real>
inline double Distance(const Joint<Real>& j1, const Joint<Real>& j2)
{
	return (j1-j2).length();
}

/*******************************************************************************
 *  Inline function implementation:                                            *
 *******************************************************************************/

/* empty Joint, no elements: */
template <class Real>
inline Joint<Real>::Joint()
{
	m_NrJoints = 0;
	m_Joints = NULL;
}

/* joint of size NrJoints, zero=false: uninitialized, zero=true -> all elements zero: */
template <class Real>
inline Joint<Real>::Joint(unsigned int NrJoints, bool _zero)
{
	m_NrJoints = NrJoints;
	m_Joints = new Real[m_NrJoints];
	
	if (_zero)
		zero();
}

/* copy constructor: */
template <class Real>
inline Joint<Real>::Joint(const Joint<Real>& rhs)
{
	m_NrJoints = rhs.m_NrJoints;
	m_Joints = new Real[rhs.m_NrJoints];
	
	for (unsigned int i = 0; i < m_NrJoints; i++)
		m_Joints[i] = rhs.m_Joints[i];
}

/* size NrJoints, content copied from d: */
template <class Real>
inline Joint<Real>::Joint(unsigned int NrJoints, const Real d[])
{
	m_NrJoints = NrJoints;
	m_Joints = new Real[m_NrJoints];

	for (unsigned int i = 0; i < m_NrJoints; i++)
		m_Joints[i] = d[i];
}

/* destructor: */
template <class Real>
inline Joint<Real>::~Joint()
{
	if (m_Joints)
		delete[] m_Joints;
}
    
/* adjust the size of the Joint: */
template <class Real>
inline void Joint<Real>::setNrJoints(unsigned int NrJoints)
{
	/* keep old values, if smaller than before cutoff, if larger fill with zeros: */

	Real* old_m_Joints = m_Joints;	
	m_Joints = new Real[NrJoints];
	
	for (unsigned int i = 0; i < NrJoints; i++)
	{
		if (i < m_NrJoints)
			m_Joints[i] = old_m_Joints[i];
		else
			m_Joints[i] = 0.0;
	}

	if (old_m_Joints)
          delete[] old_m_Joints;

	m_NrJoints = NrJoints;
}

/* make all elements zero: */
template <class Real>
inline void Joint<Real>::zero()
{
	for (unsigned int i=0; i < m_NrJoints; i++)
		m_Joints[i] = 0.0;
}



/* element access (write): */

/* set element i to value d: */
template <class Real>
inline void Joint<Real>::set(unsigned int i, Real d)
{
	#ifndef JOINT_NO_RANGECHECK
	if ( (m_Joints == NULL) || (i >= m_NrJoints) )
		throw Joint_Exception(__FILE__, __LINE__, "tried to acces an element out of Joint range!");
	#endif
	m_Joints[i] = d;
}

template <class Real>
inline Real& Joint<Real>::operator[](unsigned int i)
{
	#ifndef JOINT_NO_RANGECHECK
	if ( (m_Joints == NULL) || (i >= m_NrJoints) )
		throw Joint_Exception(__FILE__, __LINE__, "tried to acces an element out of Joint range!");
	#endif
	return m_Joints[i];
}

/* set Joint to size NrJoints and copy contents from d: */
template <class Real>
inline void Joint<Real>::set(unsigned int NrJoints, Real* d)
{
	#ifndef JOINT_NO_RANGECHECK
	if ((!m_Joints) || (NrJoints != m_NrJoints) )
          setNrJoints(NrJoints);
	#endif
        
	for (unsigned int i = 0; i < m_NrJoints; i++)
		m_Joints[i] = d[i];
}


/* element access (read): */

/* return element i: */
template <class Real>
inline Real Joint<Real>::get(unsigned int i) const
{
	#ifndef JOINT_NO_RANGECHECK
	if ((m_Joints==NULL) || (i >= m_NrJoints) ) 
		throw Joint_Exception(__FILE__, __LINE__, "tried to acces an element out of Joint range!");
	#endif

	return  m_Joints[i];
}

template <class Real>
inline Real Joint<Real>::operator[](unsigned int i) const
{
	return get(i);
}


/* copy Joint values to array d (d must have enough allocated memory!): */
template <class Real>
inline void Joint<Real>::get(unsigned int NrJoints, Real* d) const
{
	#ifndef JOINT_NO_RANGECHECK
	if ((m_Joints==NULL) || (NrJoints > m_NrJoints) ) 
		throw Joint_Exception(__FILE__, __LINE__, "tried to acces an element out of Joint range!");
	#endif

	for (unsigned int i = 0; i < m_NrJoints; i++)
		d[i] = m_Joints[i];
}

/* return number of elements: */
template <class Real>
inline unsigned int Joint<Real>::size() const
{
	return m_NrJoints;
}
	
/* Return value of the largest/smallest component: */
template <class Real>
inline Real Joint<Real>::getMax() const
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints==0)
		throw Joint_Exception(__FILE__, __LINE__, "tried to call getMax on empty Joint!");
	#endif
	
	Real max = m_Joints[0];
	for (unsigned int i=1; i<m_NrJoints; i++)
		max = (m_Joints[i]>max)?m_Joints[i]:max;
	return max;
}

template <class Real>
inline Real Joint<Real>::getMin() const
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints==0)
		throw Joint_Exception(__FILE__, __LINE__, "tried to call getMin on empty Joint!");
	#endif
	
	Real min = m_Joints[0];
	for (unsigned int i=1; i<m_NrJoints; i++)
		min = (m_Joints[i]<min)?m_Joints[i]:min;
	return min;
}


	
/* Return Index of largest/smallest component: */
template <class Real>
inline unsigned int Joint<Real>::getMaxInd() const
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints==0)
		throw Joint_Exception(__FILE__, __LINE__, "tried to call getMaxInd on empty Joint!");
	#endif
	
	Real max = m_Joints[0];
	unsigned int maxI = 0;
	
	for (unsigned int i=1; i<m_NrJoints; i++)
	{
		if (m_Joints[i]>max)
		{
			max = m_Joints[i];
			maxI = i;
		}
	}
	return maxI;
}

template <class Real>
inline unsigned int Joint<Real>::getMinInd() const
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints==0)
		throw Joint_Exception(__FILE__, __LINE__, "tried to call getMinInd on empty Joint!");
	#endif
	
	Real min = m_Joints[0];
	unsigned int minI = 0;
	
	for (unsigned int i=1; i<m_NrJoints; i++)
	{
		if (m_Joints[i]<min)
		{
			min = m_Joints[i];
			minI = i;
		}
	}
	return minI;
}

/* calculate this + (j2-this)*f: */
template <class Real>
inline Joint<Real> Joint<Real>::interpolate( const Joint& j2, Real f ) const
{
	#ifndef JOINT_NO_RANGECHECK
	if ((m_Joints == NULL) || (j2.size() != m_NrJoints) )
		throw Joint_Exception(__FILE__, __LINE__, "Joint dimensions mismatch in interpolate!");
	#endif
	
	Joint<Real> result(m_NrJoints);
	
	for (unsigned int i = 0; i < m_NrJoints; i++)
		result[i] = m_Joints[i] + ( j2[i] - m_Joints[i] ) * f;
	
	return result;
}

/* calculate j1 + (j2-j1)*f: */
template <class Real>
inline Joint<Real> Joint<Real>::interpolate( const Joint& j1, const Joint& j2, Real f )
{
	#ifndef JOINT_NO_RANGECHECK
	if ( j1.size() != j2.size() )
		throw Joint_Exception(__FILE__, __LINE__, "Joint dimensions mismatch in interpolate!");
	#endif
	
	Joint<Real> result(j1.size());
	
	for (unsigned int i = 0; i < j1.size(); i++)
		result[i] = j1[i] + f * (j2[i] - j1[i]);
	
	return result;
}

/* convert the Joint from Deg to Rad or vice versa: */
template <class Real>
inline void Joint<Real>::toRad()
{
	for (unsigned int i=0; i < m_NrJoints; i++)
	  m_Joints[i] *= RADS_PER_DEG; //0.017453292;
}

template <class Real>
inline void Joint<Real>::toDeg()
{
	for (unsigned int i=0; i < m_NrJoints; i++)
      m_Joints[i] *= DEGS_PER_RAD; //57.29577951;
      // 57.29577951 = 180 / Pi
}


/* define various operators: */

/* operator= */
template <class Real>
inline Joint<Real>& Joint<Real>::operator=(const Joint<Real>& joint)
{
	if ( joint.size() != m_NrJoints )
		setNrJoints(joint.size());

	for (unsigned int i = 0; i < m_NrJoints; i++)
		m_Joints[i] = joint[i];
		
	return *this;
}

/* if length(this - rhs) < JOINT_EPSILON, they are considered equal: */
template <class Real>
inline bool Joint<Real>::operator==(const Joint& joint2) const
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints != joint2.size())
		return false;
	else {
	#endif
	
		// Wenn betragsquadrat(joint1-joint2) < epsilon sind joints "gleich"
		Joint<Real> diff = *this - joint2;
		if (diff.lengthSqr() < JOINT_EPSILON)
			return true;
		else
			return false;

	#ifndef JOINT_NO_RANGECHECK
	}
	#endif
}

/* operators +, -, *, / */
template <class Real>
inline Joint<Real> Joint<Real>::operator+(const Joint<Real>& rhs) const
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints != rhs.size())
		throw Joint_Exception(__FILE__, __LINE__, "Joint dimensions mismatch in operator+ !");
	#endif

    Joint<Real> result(*this);
    for (unsigned int i=0; i < m_NrJoints; i++) {
        result.m_Joints[i] +=  rhs[i];
    }
    return result;
}


template <class Real>
inline Joint<Real> Joint<Real>::operator-(const Joint<Real>& rhs) const
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints != rhs.size())
		throw Joint_Exception(__FILE__, __LINE__, "Joint dimensions mismatch in operator- !");
	#endif

    Joint<Real> result(*this);
    for (unsigned int i=0; i < m_NrJoints; i++) {
        result.m_Joints[i] -=  rhs[i];
    }
    return result;
}

template <class Real>
inline Joint<Real> Joint<Real>::operator*(Real s) const
{
	Joint<Real> result(*this);
	for (unsigned int i=0; i < m_NrJoints; i++)
		result.m_Joints[i] *= s;
	return result;
}

template <class Real>
inline Joint<Real> Joint<Real>::operator/(Real s) const
{
	Joint<Real> result(*this);
	if (fabs(s) > std::numeric_limits<Real>::epsilon())
		for (unsigned int i=0; i < m_NrJoints; i++)
			result.m_Joints[i] /= s;
	else 
		throw Joint_Exception(__FILE__, __LINE__, "Attempt to divide by zero in operator/ !");
				
	return result;
}



/* operators +=, -=, *=, /= */
template <class Real>
inline void Joint<Real>::operator+=(const Joint<Real>& rhs)
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints != rhs.size())
		throw Joint_Exception(__FILE__, __LINE__, "Joint dimensions mismatch in operator+= !");
	#endif

    for (unsigned int i=0; i < m_NrJoints; i++) {
        m_Joints[i] +=  rhs[i];
    }
}

template <class Real>
inline void Joint<Real>::operator-=(const Joint<Real>& rhs)
{
	#ifndef JOINT_NO_RANGECHECK
	if (m_NrJoints != rhs.size())
		throw Joint_Exception(__FILE__, __LINE__, "Joint dimensions mismatch in operator-= !");
	#endif

    for (unsigned int i=0; i < m_NrJoints; i++) {
        m_Joints[i] -=  rhs[i];
    }
}

template <class Real>
inline void Joint<Real>::operator*=(Real s)
{
	for (unsigned int i=0; i < m_NrJoints; i++)
		m_Joints[i] *= s;
}

template <class Real>
inline void Joint<Real>::operator/=(Real s)
{
	if (fabs(s) > std::numeric_limits<Real>::epsilon())
		for (unsigned int i=0; i < m_NrJoints; i++)
			m_Joints[i] /= s;
	else 
		throw Joint_Exception(__FILE__, __LINE__, "Attempt to divide by zero in operator/= !");
}



/* get euklidian norm (length), squared or normal */
/* (not taking the root saves computation time) */
template <class Real>
inline Real Joint<Real>::lengthSqr() const
{
    Real l = 0;
    for (unsigned int i = 0; i < getNrJoints(); i++) {
        l += m_Joints[i] * m_Joints[i];
    }
    return l;
}

template <class Real>
inline Real Joint<Real>::length() const
{
	return sqrt( lengthSqr() );
}
    
/* return string which looks like: (0.000, 1.000, 2.000, ...) */
/* if convert is true, the values will be converted to degrees */ 
template <class Real>
inline std::string Joint<Real>::toString(bool convert) const
{
	char out[10];
	std::string str("(");
	for (unsigned int i = 0; i < m_NrJoints; i++) 
	{
		if (i != 0)	str+=",";
		if (convert)
			sprintf(out,"%3.3lf", m_Joints[i] * DEGS_PER_RAD );
		else	
			sprintf(out,"%3.6lf", m_Joints[i] );
		str+=std::string(out);
	}		
	str+=")";
	return str;
}

/* read in the joint from a c-string, format as above: */
template <class Real>
inline void Joint<Real>::fromString(unsigned int nrjoints, const char* str)
{
	/* resize the joint if necessary */
    if ( nrjoints != m_NrJoints )
        setNrJoints(nrjoints);
    
    const char * start = strrchr(str, '(');
    const char * end = strrchr(str, ')');
    if (end > start) {
        int n = end - start;
        char * numbers = new char[n];
        strncpy(numbers, start+1, n-1);
        char * pch = strtok (numbers,",");
        unsigned int i = 0;
        while (pch != NULL && i < nrjoints)
        {
            //cout << pch << endl;
            set(i, atof(pch));
            i++;
            pch = strtok (NULL, ",");
        }
	delete numbers;
    }
}

/* read in the joint from a c-string, format as above: */
template <class Real>
inline void Joint<Real>::fromString(const char* str)
{
	std::vector<Real> vec;
	    
    const char * start = strrchr(str, '(');
    const char * end = strrchr(str, ')');
    if (end > start) {
        int n = end - start;
        char * numbers = new char[n];
        strncpy(numbers, start+1, n-1);
        char * pch = strtok (numbers,",");
        //int i = 0;
        while (pch != NULL)
        {
            //cout << pch << endl;
            vec.push_back( atof(pch) );
            pch = strtok (NULL, ",");
        }
	}
	
	setNrJoints( vec.size() );
	for (unsigned int i=0; i < m_NrJoints; i++)
		m_Joints[i] = vec[i];
}

/* if for some reason printf should be used rather than operator<< use this: */
template <class Real>
inline void Joint<Real>::print()
{
	for (unsigned int i = 0; i < m_NrJoints; i++)
			printf("%f ",get(i));
	printf("\n");
}




template <class Real>
inline std::ostream& operator<<(std::ostream& _os, const Joint<Real>& joint)

{
	std::string str = joint.toString();
	_os << str;
	return _os;
}

template <class Real>
inline std::istream& operator>>(std::istream& _is, Joint<Real>& joint)
{
	char c_str[255];
	_is.get(c_str, 255 );
	joint.fromString(c_str);
	return _is;
}
	

#endif


#ifdef SWIG
%module Util
%include Source/Manipulation/ManipUtil/datastructsManipulator.h
%include Source/Manipulation/ManipUtil/Trajectory.h
%{
	#include "Joint.h"
%}
%include "std_vector.i"
%template(Jointd) Joint<double>;
#endif

