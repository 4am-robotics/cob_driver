//-----------------------------------------------
// Neobotix 
/***********************************************************************
 *                                                                     *
 *  written by Felix Geibel May 2009                                   *
 *  at Fraunhofer IPA                                                  *
 *                                                                     *
 *  based on BSpline2d by Neobotix (see below)                         *
 *                                                                     *
 ***********************************************************************/

// www.neobotix.de
// Copyright (c) 2003. All rights reserved.

// author: Bertram Rohrmoser
//-----------------------------------------------

#ifndef BSPLINE_ND_H
#define BSPLINE_ND_H
//-----------------------------------------------

#include <vector>
#include <cmath>
#include <assert.h>

#define BSPLINE_TINY 1e-20


/**
 * Implements a BSpline curve as a template class.
 * a PointND type needs the following operators:
 * operator=, copy constructor, p1 += p2, p1 * scalar
 * there needs to be a function Distance(p1,p2)
 * there needs to be a member function p1.zero()
 */
template <class PointND>
class BSplineND
{
public:
	//----------------------- Interface
	BSplineND();

	~BSplineND();

	void setCtrlPoints(const std::vector<PointND>& ctrlPointVec );

	// main functions
	bool ipoWithConstSampleDist(double dIpoDist, std::vector<PointND>& ipoVec);

	bool ipoWithNumSamples(int iNumPts, std::vector<PointND>& ipoVec);
	
	void eval(double dPos, PointND& point);
	
	double getMaxdPos() const { return m_dLength; }

private:
	//----------------------- Parameters
	double m_iGrad;

	//----------------------- Variables
	std::vector<PointND> m_CtrlPointVec;

	std::vector<double> m_KnotVec;

	double m_dLength;

	//----------------------- Member functions
	double evalBasis(double t, unsigned int i, int n);

};


/*****************************************************************************
 *                                                                           *
 *  Implementierung bei TemplateKlassen im Headerfile                        *
 *                                                                           *
 *****************************************************************************/

template <class PointND>
inline BSplineND<PointND>::BSplineND()
{
	m_iGrad = 3;
	m_dLength = 0;
}

//-----------------------------------------------
template <class PointND>
BSplineND<PointND>::~BSplineND()
{
}

//-----------------------------------------------
template <class PointND>
void BSplineND<PointND>::setCtrlPoints(const std::vector<PointND>& ctrlPointVec )
{
	int iNumCtrlPoint;
	int i;
	double d;

	m_CtrlPointVec = ctrlPointVec;
	iNumCtrlPoint = m_CtrlPointVec.size();

	if (iNumCtrlPoint < m_iGrad)
    {
        return;
    }

	m_KnotVec.resize( iNumCtrlPoint + m_iGrad );

	// Calculate knots
	for(i = 0; i< m_iGrad; i++)
	{
		m_KnotVec[i] = 0;
	}

	int iNumInternalKnots = iNumCtrlPoint - m_iGrad;
	for(i=0; i<iNumInternalKnots; i++)
	{
		double Distance1 = 0.0;
		for(unsigned int k = 0; k < m_CtrlPointVec[i+1].size(); k++)
		{
			Distance1 += (m_CtrlPointVec[i+1].at(k) - m_CtrlPointVec[i+2].at(k)) * (m_CtrlPointVec[i+1].at(k) - m_CtrlPointVec[i+2].at(k));
		}
		d= m_KnotVec[i+m_iGrad-1] + sqrt(Distance1);
		// OLD WITH JOINTD d= m_KnotVec[i+m_iGrad-1] + Distance( m_CtrlPointVec[i+1], m_CtrlPointVec[i+2] );
		m_KnotVec[i+m_iGrad] = d;
	}
	double Distance2 = 0.0;
	for(unsigned int k = 0; k <  m_CtrlPointVec[iNumInternalKnots+1].size(); k++)
	{
		Distance2 += ( m_CtrlPointVec[iNumInternalKnots+1].at(k) - m_CtrlPointVec[iNumInternalKnots+2].at(k)) * ( m_CtrlPointVec[iNumInternalKnots+1].at(k) - m_CtrlPointVec[iNumInternalKnots+2].at(k));
	}
	d = m_KnotVec[iNumCtrlPoint-1] + sqrt(Distance2);
	// OLD WITH JOINTD d = m_KnotVec[iNumCtrlPoint-1] + Distance( m_CtrlPointVec[iNumInternalKnots+1], m_CtrlPointVec[iNumInternalKnots+2] );

	for(i = 0; i< m_iGrad; i++)
	{
		m_KnotVec[i+iNumCtrlPoint] = d;
	}
	
	// This is not the arc-length but the maximum of the spline parameter.
	// eval(m_dLength, Point) should return the last point of the spline
	m_dLength = d;

}


//-----------------------------------------------
template <class PointND>
void BSplineND<PointND>::eval(double dPos, PointND& point)
{
	double dFak;

	for(unsigned int i = 0; i<point.size(); i++)
		point.at(i) = 0.0;
	for(unsigned int i = 0; i < m_CtrlPointVec.size(); i++)
	{
		dFak = evalBasis(dPos, i, m_iGrad);
		for(unsigned int j = 0; j<point.size(); j++)
			point.at(j) += m_CtrlPointVec[i][j] * dFak;  // !!!! TODO this might be wrong due to change from JointD to std::vector
	}
}


//-----------------------------------------------
template <class PointND>
bool BSplineND<PointND>::ipoWithConstSampleDist(double dIpoDist, std::vector<PointND >& ipoVec)
{
	PointND buf = m_CtrlPointVec.front();
	int iNumOfPoints;
	double dPos;
	//int iStart, iNextStart;

	if (m_CtrlPointVec.size() < m_iGrad)
	{
		ipoVec = m_CtrlPointVec;
		return false;
	}
	
	// Felix: Dies ist falsch? m_KnotVec.back() enthält nicht die tatsächliche 
	// Bogenlänge entlang des Splines. Folge: Ergebnisvektor ist am Ende abge-
	// schnitten.
	iNumOfPoints = m_KnotVec.back() / dIpoDist + 2;
	ipoVec.resize(iNumOfPoints);

	// Calculate x- and y-coordinates
	dPos = 0;
	for(int i=0; i < iNumOfPoints -1 ; i++)
	{
		eval(dPos, buf);
		ipoVec[i] = buf;
		dPos += dIpoDist;
	}

	ipoVec.back() = m_CtrlPointVec.back();

	return true;
}

/*
//-----------------------------------------------
//Florian
bool BSplineND<PointND>::ipoWithConstSampleDist(double dIpoDist, OmniPath& ipoVec)
{
	Neo::Vector2D buf;
	int iNumOfPoints;
	int iNumOfCtrlPoints;
	double dPos, viewang;
	double dx, dy, da;
	int i;
	bool bRet = true;
	int iStart, iNextStart;

	iNumOfCtrlPoints = m_OmniCtrlPointVec.getSize();

	if ( iNumOfCtrlPoints < m_iGrad )
	{
		bRet = false;

		iNumOfPoints = m_OmniCtrlPointVec.getSize();
		ipoVec.clearVec();
		for(i=0; i<iNumOfPoints; i++)
		{
			ipoVec.addFrame(m_OmniCtrlPointVec.m_VecPathPnt[i].Frm,m_OmniCtrlPointVec.m_VecPathPnt[i].ViewAng);
		}

		if ( iNumOfCtrlPoints < 2 )
		{
			return bRet;
		}
	}
	else
	{
		iNumOfPoints = m_KnotVec.back() / dIpoDist + 2;
		ipoVec.m_VecPathPnt.resize(iNumOfPoints);

		// Calculate x- and y-coordinates
		dPos = 0;
		iStart = 0;
		for(i=0; i < iNumOfPoints -1 ; i++)
		{
//			eval(dPos, buf);
			eval(dPos, buf, iStart, iNextStart, viewang);
			iStart = iNextStart;

			ipoVec.m_VecPathPnt[i].Frm.x() = buf.x();
			ipoVec.m_VecPathPnt[i].Frm.y() = buf.y();
			ipoVec.m_VecPathPnt[i].ViewAng=viewang;
			dPos += dIpoDist;
		}

		ipoVec.m_VecPathPnt.back().Frm.x() = m_OmniCtrlPointVec.m_VecPathPnt.back().Frm.x();
		ipoVec.m_VecPathPnt.back().Frm.y() = m_OmniCtrlPointVec.m_VecPathPnt.back().Frm.y();
		ipoVec.m_VecPathPnt.back().ViewAng = m_OmniCtrlPointVec.m_VecPathPnt.back().ViewAng;
	}

	// Calculate angle
	for(i=0; i < iNumOfPoints-1; i++)
	{
		dx = ipoVec.m_VecPathPnt[i+1].Frm.x() - ipoVec.m_VecPathPnt[i].Frm.x();
		dy = ipoVec.m_VecPathPnt[i+1].Frm.y() - ipoVec.m_VecPathPnt[i].Frm.y();
		da = atan2(dy, dx);
		MathSup::normalizePi(da);
		ipoVec.m_VecPathPnt[i].Frm.a() = da;
	}
	ipoVec.m_VecPathPnt.back().Frm.a() = da;

	return bRet;
}
*/

//-----------------------------------------------
template <class PointND>
bool BSplineND<PointND>::ipoWithNumSamples(int iNumOfPoints, std::vector<PointND >& ipoVec)
{
	PointND buf;
	double dPos, dInc;

	if (m_CtrlPointVec.size() < m_iGrad)
	{
		ipoVec = m_CtrlPointVec;
		return false;
	}

	dInc = m_KnotVec.back() / (double)(iNumOfPoints-1);
	ipoVec.resize(iNumOfPoints);

	// Calculate x- and y-coordinates
	dPos = 0;
	for(int i=0; i < iNumOfPoints -1 ; i++)
	{
		eval(dPos, buf);

		ipoVec[i] = buf;
		dPos += dInc;
	}

	ipoVec.back() = m_CtrlPointVec.back();
	return true;
}


template <class PointND>
double BSplineND<PointND>::evalBasis(double u, unsigned int i, int n)
{
	assert(i >= 0);
	assert(i < m_KnotVec.size() - 1 );

	if (n==1)
	{
		if ( (m_KnotVec[i]<=u) && ( u<m_KnotVec[i+1]) )
		{
			return 1.0;
		}
		else
		{
			return 0.0;
		}
	}
	else
	{
		double N;
		double dNum1, dNum2;
		double dDen1, dDen2;
		
		dDen1 = u - m_KnotVec[i];
		dNum1 = m_KnotVec[i+n-1] - m_KnotVec[i];

		dDen2 = m_KnotVec[i+n] - u;
		dNum2 = m_KnotVec[i+n] - m_KnotVec[i+1];

		if ( (fabs(dNum1) > BSPLINE_TINY)&&(fabs(dNum2) > BSPLINE_TINY) )
		{
			N = dDen1 / dNum1 * evalBasis(u, i , n-1);
			N += dDen2 / dNum2 * evalBasis(u, i+1 , n-1);
		}
		else if ( (fabs(dNum1) < BSPLINE_TINY)&&(fabs(dNum2) > BSPLINE_TINY) )
		{
			N = dDen2 / dNum2 * evalBasis(u, i+1 , n-1);
		}
		else if ((fabs(dNum1) > BSPLINE_TINY)&&(fabs(dNum2) < BSPLINE_TINY))
		{
			N = dDen1 / dNum1 * evalBasis(u, i , n-1);
		}
		else
		{
			N = 0;
		}

		return N;
	}
}



#endif
