/********************************************************************
 *                                                                  *
 *   Reference Values in Joint Space                                *
 *                                                                  *
 *   abstract base class defining the functions that a              *
 *   class implementing reference values in joint space             *
 *   has to provide. Values for                                  *
 *                                                                  *
 ********************************************************************/

#ifndef _REFVAL_JS_H_
#define _REFVAL_JS_H_

#include <vector>

 
class RefVal_JS
{
	public:
		virtual std::vector<double> r(double s) const=0;
		virtual double s(double t) const=0;
		virtual std::vector<double> r_t(double t) const { return r( s(t) ); }
		
		virtual std::vector<double> dr_ds(double s) const=0;
		virtual double ds_dt(double t) const=0;
		virtual std::vector<double> dr_dt(double t) const
		{
			std::vector<double> dr;
			dr.resize(dr_ds(t).size());
			for(unsigned int i = 0; i < dr_ds(t).size(); i++)
				dr.at(i) = dr_ds( s(t) ).at(i) * ds_dt( t );
			return dr;
		}
				
		virtual std::vector<double> getLast() const { return r_t( getTotalTime() ); }
		
		virtual double getTotalTime() const=0;
};

#endif

