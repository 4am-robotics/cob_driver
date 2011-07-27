//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_unit_converter_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Implementation of class #SDH::cUnitConverter

  \section sdhlibrary_cpp_unit_converter_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_unit_converter_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2009-03-18 09:39:27 +0100 (Mi, 18 Mrz 2009) $
      \par SVN file revision:
        $Id: unit_converter.cpp 4191 2009-03-18 08:39:27Z Osswald2 $

  \subsection sdhlibrary_cpp_unit_converter_cpp_changelog Changelog of this file:
      \include unit_converter.cpp.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>
#include <iostream>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------


#include "unit_converter.h"

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function definitions
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class definitions
//----------------------------------------------------------------------


cUnitConverter::cUnitConverter( char const* _kind, char const* _name, char const* _symbol, double _factor, double _offset, int _decimal_places )
{
    assert(_factor != 0.0); // must not be 0 since we devide by it

    kind   = _kind;
    name   = _name;
    symbol = _symbol;
    factor = _factor;
    offset = _offset;
    decimal_places = _decimal_places;
}
//----------------------------------------------------------------------


double cUnitConverter::ToExternal( double internal ) const
{
    return internal * factor + offset;
}
//----------------------------------------------------------------------


cSimpleVector cUnitConverter::ToExternal( cSimpleVector& internal ) const
{
    cSimpleVector rv;

    for ( int i = 0; i < internal.eNUMBER_OF_ELEMENTS; i++ )
    {
        if ( internal.Valid( i ) )
        rv[i] =  internal[i] * factor + offset;
    }
    return rv;
}
//----------------------------------------------------------------------


std::vector<double> cUnitConverter::ToExternal( std::vector<double> const& internal ) const
{
    std::vector<double> rv;

    for ( std::vector<double>::const_iterator vi = internal.begin();
          vi != internal.end();
          vi++ )
    {
        rv.push_back(  *vi * factor + offset );
    }
    return rv;
}
//----------------------------------------------------------------------


double cUnitConverter::ToInternal( double external ) const
{
    return (external - offset) / factor;
}
//----------------------------------------------------------------------


cSimpleVector cUnitConverter::ToInternal( cSimpleVector& external ) const
{
    cSimpleVector rv;

    for ( int i = 0; i < external.eNUMBER_OF_ELEMENTS; i++ )
    {
        if ( external.Valid( i ) )
        rv[i] =  (external[i] - offset) / factor;
    }
    return rv;
}
//----------------------------------------------------------------------


std::vector<double> cUnitConverter::ToInternal( std::vector<double> const& external ) const
{
    std::vector<double> rv;

    for ( std::vector<double>::const_iterator vi = external.begin();
          vi != external.end();
          vi++ )
    {
        rv.push_back( ( *vi - offset) / factor );
    }
    return rv;
}
//----------------------------------------------------------------------


char const* cUnitConverter::GetKind( void ) const
{
    return kind;
}
//----------------------------------------------------------------------


char const* cUnitConverter::GetName( void ) const
{
    return name;
}
//----------------------------------------------------------------------


char const* cUnitConverter::GetSymbol( void ) const
{
    return symbol;
}
//----------------------------------------------------------------------


double cUnitConverter::GetFactor( void ) const
{
    return factor;
}
//----------------------------------------------------------------------



double cUnitConverter::GetOffset( void ) const
{
    return offset;
}
//----------------------------------------------------------------------



int cUnitConverter::GetDecimalPlaces( void ) const
{
    return decimal_places;
}
//======================================================================

NAMESPACE_SDH_START

cUnitConverter const uc_identity( "any", "any", "?", 1.0, 0.0, 4 );

NAMESPACE_SDH_END


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================
