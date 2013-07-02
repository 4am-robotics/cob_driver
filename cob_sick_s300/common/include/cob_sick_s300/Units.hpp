/* 
 * File:   Units.h
 * Author: jan
 *
 * Created on September 1, 2010, 5:36 PM
 */

#ifndef BRICS_OODL_UNITS_HPP
#define	BRICS_OODL_UNITS_HPP
#include <boost/units/pow.hpp>
#include <boost/units/systems/si.hpp>
#include <boost/units/systems/temperature/celsius.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/make_scaled_unit.hpp>
#include <boost/units/systems/si/prefixes.hpp>


using namespace boost::units;
using namespace boost::units::si;
using namespace boost::units::angle;

//typedef boost::units::si::length meter;
using boost::units::si::meters;


typedef boost::units::make_scaled_unit<si::length, boost::units::scale<10, boost::units::static_rational<-3> > >::type millimeter;
typedef boost::units::make_scaled_unit<si::length, boost::units::scale<10, boost::units::static_rational<-2> > >::type centimeter;
BOOST_UNITS_STATIC_CONSTANT(centimeters, centimeter);


typedef boost::units::make_scaled_unit<si::time, boost::units::scale<10, boost::units::static_rational<-3> > >::type millisecond;
//BOOST_UNITS_STATIC_CONSTANT(millimeters, millimeter);

#endif	/* BRICS_OODL_UNITS_HPP */

