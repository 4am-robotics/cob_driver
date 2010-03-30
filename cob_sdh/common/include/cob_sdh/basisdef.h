//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_basisdef_h_general General file information

    \author   Jan Grewe, Dirk Osswald
    \date     08.10.2004

  \brief
    This file contains some basic definitions (defines, macros, datatypes)

    - Datatypes: #SDH::Int8, #SDH::UInt8, #SDH::Int16, #SDH::UInt16, #SDH::Int32, #SDH::UInt32

  \section sdhlibrary_cpp_basisdef_h_copyright Copyright

  Copyright (c) 2006 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_basisdef_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-06-11 15:04:28 +0200 (Mi, 11 Jun 2008) $
      \par SVN file revision:
        $Id: basisdef.h 3180 2008-06-11 13:04:28Z Osswald2 $

  \subsection sdhlibrary_cpp_basisdef_h_changelog Changelog of this file:
      \include basisdef.h.log
*/
//======================================================================

#ifndef _SDHLIBRARY_BASISDEF_H
#define _SDHLIBRARY_BASISDEF_H

//----------------------------------------------------------------------
// Project includes
//----------------------------------------------------------------------

#include <assert.h>
#include "sdhlibrary_settings.h"


//----------------------------------------------------------------------
// Typedefs
//----------------------------------------------------------------------

NAMESPACE_SDH_START

typedef char            Int8;        //!< signed integer, size 1 Byte (8 Bit)
typedef unsigned char   UInt8;	     //!< unsigned integer, size 1 Byte (8 Bit)
typedef short           Int16;       //!< signed integer, size 2 Byte (16 Bit)
typedef unsigned short  UInt16;      //!< unsigned integer, size 2 Byte (16 Bit)
typedef long            Int32;	     //!< signed integer, size 4 Byte (32 Bit)
typedef unsigned long   UInt32;	     //!< unsigned integer, size 4 Byte (32 Bit)

//----------------------------------------------------------------------
// defines
//----------------------------------------------------------------------

//! macro to assert that the defined typedefs have the expected sizes
#define SDH_ASSERT_TYPESIZES()              \
    do {                                    \
        assert( sizeof( Int8 )   == 1 );    \
        assert( sizeof( UInt8 )  == 1 );    \
        assert( sizeof( Int16 )  == 2 );    \
        assert( sizeof( UInt16 ) == 2 );    \
        assert( sizeof( Int32 )  == 4 );    \
        assert( sizeof( UInt32 ) == 4 );    \
    } while (0)


NAMESPACE_SDH_END

//======================================================================
#endif
