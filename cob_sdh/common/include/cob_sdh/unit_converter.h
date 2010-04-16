//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_unit_converter_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cUnitConverter.

  \section sdhlibrary_cpp_unit_converter_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_unit_converter_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2009-03-18 09:39:27 +0100 (Mi, 18 Mrz 2009) $
      \par SVN file revision:
        $Id: unit_converter.h 4191 2009-03-18 08:39:27Z Osswald2 $

  \subsection sdhlibrary_cpp_unit_converter_h_changelog Changelog of this file:
      \include unit_converter.h.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <vector>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "simplevector.h"
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------


/*!
  \brief Unit conversion class to convert values between physical unit systems

   An object of this class can be configured to convert values of a
   physical unit between 2 physical unit systems. An angle value given
   in degrees can e.g. be converted from/to radians or vice versa by
   an object of this class.

   <hr>
*/
class cUnitConverter
{
public:
    /*!
      Constructor of cUnitConverter class.

       At construction time the conversion parameters - a \a factor
       and an \a offset - must be provided along with elements that
       describe the unit of a value

       \param _kind   - a string describing the kind of unit to be converted (something like "angle" or "time")
       \param _name   - the name of the external unit (something like "degrees" or "milliseconds")
       \param _symbol - the symbol of the external unit (something like "deg" or "ms")
       \param _factor - the conversion factor from internal to external units
       \param _offset - the conversion offset from internal to external units
       \param _decimal_places - A usefull number of decimal places for printing values in the external unit system

       \attention
         The strings given _kind, _name and _symbol are \b NOT copied,
         just their address is stored <hr>
    */
    cUnitConverter( char const* _kind, char const* _name, char const* _symbol, double _factor = 1.0, double _offset = 0.0, int _decimal_places=1 );

    //----------------------------------------------------------------------
    /*!
        Convert single value \a internal given in internal units into external units.
        Returns \a internal * #factor + #offset
    */
    double ToExternal( double internal ) const;

    //----------------------------------------------------------------------
    /*!
        Convert values in simple array \a internal, each given in
        internal units into external units.  Returns a simple array
        with each valid element converted with \a internal[i] *
        #factor + #offset

        Only valid entries of the \a internal vector are converted.
    */
    cSimpleVector ToExternal( cSimpleVector& internal ) const;


    //----------------------------------------------------------------------
    /*!
        Convert values in vector \a internal, each given in
        internal units into external units.  Returns a vector
        with each element converted with \a internal[i] *
        #factor + #offset
    */
    std::vector<double> ToExternal( std::vector<double> const& internal ) const;


    //----------------------------------------------------------------------
    /*!
        Convert value \a external given in external units into internal units.
        Returns (\a external - #offset) / #factor
    */
    double ToInternal( double external ) const;

    //----------------------------------------------------------------------
    /*!
        Convert values in simple array \a external, each given in
        external units into internal units.  Returns a simple array
        with each valid element converted with \a external[i] *
        #factor + #offset

        Only valid entries of the \a external vector are converted.
    */
    cSimpleVector ToInternal( cSimpleVector& external ) const;


    //----------------------------------------------------------------------
    /*!
        Convert values in vector \a external, each given in
        external units into internal units.  Returns a vector
        with each valid element converted with \a external[i] *
        #factor + #offset
    */
    std::vector<double> ToInternal( std::vector<double> const& external ) const;

    //----------------------------------------------------------------------
    //! Return the kind of unit converted (something like "angle" or "time")
    char const* GetKind( void ) const;

    //----------------------------------------------------------------------
    //! Return the name of the external unit (something like "degrees" or "milliseconds")
    char const* GetName( void ) const;

    //----------------------------------------------------------------------
    //! Return the symbol of the external unit (something like "deg" or "ms")
    char const* GetSymbol( void ) const;

    //----------------------------------------------------------------------

    //! Return the conversion factor from internal to external units
    double GetFactor( void ) const;

    //----------------------------------------------------------------------
    //! Return the conversion offset from internal to external units
    double GetOffset( void ) const;

    //----------------------------------------------------------------------
    //! Return the number of decimal places for printing values in the external unit system
    int GetDecimalPlaces( void ) const;


protected:
    //! the kind of unit to be converted (something like "angle" or "time")
    char const* kind;

    //! the name of the external unit (something like "degrees" or "milliseconds")
    char const* name;

    //! the symbol of the external unit (something like "deg" or "ms")
    char const* symbol;

    //! the conversion factor from internal to external units
    double factor;

    //! the conversion offset from internal to external units
    double offset;

    //! A usefull number of decimal places for printing values in the external unit system
    int decimal_places;

}; // cUnitConverter
//======================================================================


//! Identity converter (internal = external)
extern cUnitConverter const uc_identity;

//----------------------------------------------------------------------
//! Type of a pointer to a function like 'double cUnitConverter::ToExternal( double )' or 'cUnitConverter::ToInternal( double )'
typedef double (cUnitConverter::*pDoubleUnitConverterFunction) ( double ) const;


//! Type of a pointer to a function like 'double cUnitConverter::ToExternal( double )' or 'cUnitConverter::ToInternal( double )'
//typedef std::vector<double> (cUnitConverter::*pDoubleVectorUnitConverterFunction) ( std::vector<double> ) const;


//
//#####################################################################

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
