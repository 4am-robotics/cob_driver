//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_simplevector_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Implementation of class #SDH::cSimpleVector.

  \section sdhlibrary_cpp_simplevector_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_simplevector_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-02-04 19:26:19 +0100 (Fr, 04 Feb 2011) $
      \par SVN file revision:
        $Id: simplevector.cpp 6420 2011-02-04 18:26:19Z Osswald2 $

  \subsection sdhlibrary_cpp_simplevector_cpp_changelog Changelog of this file:
      \include simplevector.cpp.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "dbg.h"
#include "simplevector.h"

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function implementation (function definitions)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class member function definitions
//----------------------------------------------------------------------


cSimpleVector::cSimpleVector()
    throw (cSimpleVectorException*)
{
    for ( int i=0; i < eNUMBER_OF_ELEMENTS; i++ )
        value[ i ] = 0.0;
    valid = 0;
}
//-----------------------------------------------------------------


cSimpleVector::cSimpleVector( int nb_values, char const* str )
    throw (cSimpleVectorException*)
{
    FromString( nb_values, 0, str );
}
//-----------------------------------------------------------------

cSimpleVector::cSimpleVector( int nb_values, int start_index, float* values )
    throw (cSimpleVectorException*)
{
    valid = 0;
    int mask = (1<<start_index);
    for ( int i=0; i < nb_values; i++ )
    {
        value[ i+start_index ] = values[i];
        valid |= mask;
        mask <<= 1;
    }
}
//-----------------------------------------------------------------


cSimpleVector::cSimpleVector( int nb_values, int start_index, char const* str )
    throw (cSimpleVectorException*)
{
    FromString( nb_values, start_index, str );
}
//-----------------------------------------------------------------


void cSimpleVector::FromString( int nb_values, int start_index, char const* str )
    throw (cSimpleVectorException*)
{
    assert( start_index + nb_values <= eNUMBER_OF_ELEMENTS );

    for ( int i = 0; i < nb_values; i++ )
    {
        int n;         // number of chars scanned
        int nb_fields; // number of fields successfully scanned
        int vi = start_index+i; // index in value to write
        nb_fields = sscanf( str, " %lf%n", &(value[ vi ]), &n );

        if ( nb_fields != 1 )
            throw new cSimpleVectorException( cMsg( "cannot init simple vector from string <%s>", str ) );

        valid |= (1<<vi);


        str += n;

        // skip "," separators
        while ( *str == ',' )
            str++;
    }
}
//-----------------------------------------------------------------


double& cSimpleVector::operator[]( unsigned int index )
{
    assert( index < eNUMBER_OF_ELEMENTS );
    //assert( valid & (1 << index) );
    valid |= (1<<index);
    return value[ index ];
}
//-----------------------------------------------------------------


double& cSimpleVector::x(void)
{
    //assert( valid & (1<<0) );
    valid |= (1<<0);
    return value[0];
}
//-----------------------------------------------------------------



double& cSimpleVector::y(void)
{
    //assert( valid & (1<<1) );
    valid |= (1<<1);
    return value[1];
}
//-----------------------------------------------------------------



double& cSimpleVector::z(void)
{
    //assert( valid & (1<<2) );
    valid |= (1<<2);
    return value[2];
}
//-----------------------------------------------------------------


bool cSimpleVector::Valid( unsigned int index ) const
{
    assert( index < eNUMBER_OF_ELEMENTS );

    ////!!!return valid | (1<<index); // this seems wrong
    return (valid & (1<<index)) != 0;
}
//-----------------------------------------------------------------


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

