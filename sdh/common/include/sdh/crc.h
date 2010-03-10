//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_crc_h_general General file information
    \author   Dirk Osswald
    \date     2008-06-09


  \brief
    This file contains interface to cCRC, a class to handle CRC calculation.


  \section sdhlibrary_cpp_crc_h_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_crc_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-14 11:32:33 +0200 (Di, 14 Okt 2008) $
      \par SVN file revision:
        $Id: crc.h 3692 2008-10-14 09:32:33Z Osswald2 $

  \subsection sdhlibrary_cpp_crc_h_changelog Changelog of this file:
      \include crc.h.log
*/
//======================================================================

#ifndef CRC_h_
#define CRC_h_

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "basisdef.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

NAMESPACE_SDH_START


typedef UInt16 tCRCValue;   //!< the data type used to calculate and exchange CRC values with DSACON32m (16 bit integer)

//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// External functions (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function prototypes (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

 /*!
     Generic class to calculate a CRC using a given, precalculated table.

     Use derived classes like cCRC_DSACON32m with a specifically set CRC table.
 */
class cCRC
{
 protected:
    //! current value of the CRC checksum
    tCRCValue current_crc;

    //! initial value of the CRC checksum
    tCRCValue initial_value;

    //! table with precalculated CRC values
    tCRCValue const* crc_table;

 public:
    //! constructor: create a new cCRC object and initialize the current value of the CRC checksum. \a crc_table is the CRC table to use.
    cCRC( tCRCValue const* _crc_table, tCRCValue _initial_value )
    {
        crc_table     = _crc_table;
        initial_value = _initial_value;
        current_crc   = initial_value;
    }

    //! insert byte into CRC calculation and return the new current CRC checksum
    tCRCValue AddByte( unsigned char byte )
    {
        current_crc = ( (current_crc & 0xFF00) >> 8 ) ^ crc_table[ ( current_crc & 0x00FF ) ^ (byte & 0x00FF)];
        return current_crc;
    }

    //! return the current CRC value
    inline tCRCValue GetCRC()
    {
        return current_crc;
    }

    //! return the low byte of the current CRC value
    inline UInt8 GetCRC_LB()
    {
        return current_crc & 0x00ff;
    }

    //! return the high byte of the current CRC value
    inline UInt8 GetCRC_HB()
    {
        return (current_crc >> 8) & 0x00ff;
    }

    //! reset the current CRC value to its initial value and return it;
    inline tCRCValue Reset()
    {
        current_crc = initial_value;
        return current_crc;
    }
};
//----------------------------------------------------------------------
//----------------------------------------------------------------------

//! A derived CRC class that uses a CRC table and initial value suitable for the Weiss Robotics DSACON32m controller
class cCRC_DSACON32m : public cCRC
{
 protected:
    //! the CRC table used by the DSACON32m controller
    static tCRCValue const crc_table_dsacon32m[256];

 public:
    //! constructor to create a cCRC object suitable for checksumming the communication with a DSACON32m tactile sensor controller
    inline cCRC_DSACON32m( void )
        : cCRC( crc_table_dsacon32m, 0xffff )
    {
        // nothing more to do
    }
};
//----------------------------------------------------------------------
//----------------------------------------------------------------------

NAMESPACE_SDH_END

#endif


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C
  mode:ELSE
  End:
*/
//======================================================================}
