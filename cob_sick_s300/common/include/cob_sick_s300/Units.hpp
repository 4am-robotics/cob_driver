/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

