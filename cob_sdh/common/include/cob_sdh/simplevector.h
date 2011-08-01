//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_simplevector_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cSimpleVector.

  \section sdhlibrary_cpp_simplevector_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_simplevector_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-03-09 11:55:11 +0100 (Mi, 09 Mrz 2011) $
      \par SVN file revision:
        $Id: simplevector.h 6526 2011-03-09 10:55:11Z Osswald2 $

  \subsection sdhlibrary_cpp_simplevector_h_changelog Changelog of this file:
      \include simplevector.h.log
*/
//======================================================================

#ifndef SIMPLEVECTOR_H_
#define SIMPLEVECTOR_H_

#include "sdhlibrary_settings.h"

#if SDH_USE_VCC
# pragma warning(disable : 4290)
#endif

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function implementation (function definitions)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

/*!
   \brief Derived exception class for low-level simple vector related exceptions.
*/
class VCC_EXPORT cSimpleVectorException: public cSDHLibraryException
{
public:
  cSimpleVectorException( cMsg const & _msg )
    : cSDHLibraryException( "cSimpleVectorException", _msg )
  {}
};
//======================================================================


//! A simple vector implementation.
/*!
    Objects of this class are used to return vector like answers from
    the %SDH firmware to the cSDHBase class. End users need not use
    this class, as cSDH, the real end user interface class provides a
    more convenient way using STL vectors.
*/
class VCC_EXPORT cSimpleVector
{
public:
    //! anonymous enum (instead of define like macros)
    enum
    {
      eNUMBER_OF_ELEMENTS = 7 //!< number of elements in vector
    };

    //! Default constructor: init members to zero
    cSimpleVector()
        throw (cSimpleVectorException*);


    //! Constructor: init members from \a nb_values comma separated values in the give string \a str
    cSimpleVector( int nb_values, char const* str )
        throw (cSimpleVectorException*);


    //! Constructor: init members from \a nb_values comma separated values in the give string \a str
    cSimpleVector( int nb_values, int start_index, char const* str )
        throw (cSimpleVectorException*);

    //! Constructor: init members beginning with \a start_index from \a nb_values in arrray \a values
    cSimpleVector( int nb_values, int start_index, float* values )
        throw (cSimpleVectorException*);


    //! init \a nb_values starting from index \a start_index from comma separated values in \a str
    void FromString( int nb_values, int start_index, char const* str )
        throw (cSimpleVectorException*);


    //! index operator, return a reference to the \a index-th element of this
    double& operator[]( unsigned int index );


    //! Interpret object as x/y/z vector: return x = the first element, if that is valid.
    double& x(void);


    //! Interpret object as x/y/z vector: return x = the first element, if that is valid.
    double& y(void);


    //! Interpret object as x/y/z vector: return x = the first element, if that is valid.
    double& z(void);

    //! Return true if vector element \a index is valid (has been accessed at least once)
    bool Valid( unsigned int index ) const;

protected:

    double value[ eNUMBER_OF_ELEMENTS ];

    //! bit mask which values in #value are valid
    int valid;

}; // cSimpleVector
//-----------------------------------------------------------------

NAMESPACE_SDH_END

#endif


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
