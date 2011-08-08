//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_simpletime_h_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of auxilliary utility functions for SDHLibrary-CPP.

  \section sdhlibrary_cpp_simpletime_h_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_simpletime_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-08-08 19:05:54 +0200 (Fr, 08 Aug 2008) $
      \par SVN file revision:
        $Id: simpletime.h 3444 2008-08-08 17:05:54Z Osswald2 $

  \subsection sdhlibrary_cpp_simpletime_h_changelog Changelog of this file:
      \include simpletime.h.log
*/
//======================================================================

#ifndef SIMPLETIME_H_
#define SIMPLETIME_H_

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#if SDH_USE_VCC
#include <sys/timeb.h>
#include <time.h>
#else
# include <sys/time.h>
#endif

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
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------


/*!
  \brief Very simple class to measure elapsed time
*/
class cSimpleTime
{
protected:
#if SDH_USE_VCC
    struct _timeb timebuffer;
#else
    struct timeval a_time;
#endif

public:
    //! Constructor: store current time ("now") internally.
    cSimpleTime()
    {
        StoreNow();
    }
    //----------------------------------------------------------------------


    //! Store current time internally.
    void StoreNow( void )
    {
#if SDH_USE_VCC
         _ftime64_s( &timebuffer );

#else
         gettimeofday( &a_time, NULL );
#endif
    }
    //----------------------------------------------------------------------

    //! Return time in seconds elapsed between the time stored in the object and now.
    double Elapsed( void ) const
    {
        cSimpleTime now;

        return Elapsed( now );
    }
    //----------------------------------------------------------------------


    //! Return time in micro seconds elapsed between the time stored in the object and now.
    long Elapsed_us( void ) const
    {
        cSimpleTime now;

        return Elapsed_us( now );
    }
    //----------------------------------------------------------------------


#if SDH_USE_VCC
    //! Return time in seconds elapsed between the time stored in the object and \a other.
    double Elapsed( cSimpleTime const& other ) const
    {
        double seconds = double( other.timebuffer.time - timebuffer.time);
        double msec = double( other.timebuffer.millitm - timebuffer.millitm );

        return seconds + msec / 1000.0;
    }
    //----------------------------------------------------------------------

    //! Return time in micro seconds elapsed between the time stored in the object and \a other.
    long Elapsed_us( cSimpleTime const& other ) const
    {
        long seconds = long( other.timebuffer.time - timebuffer.time);
        long msec    = long( other.timebuffer.millitm - timebuffer.millitm );

        return seconds * 1000000 + msec * 1000;
    }
    //----------------------------------------------------------------------
#else
    //! Return time in seconds elapsed between the time stored in the object and \a other.
    double Elapsed( cSimpleTime const& other ) const
    {
        double seconds = double( other.a_time.tv_sec - a_time.tv_sec );
        double usec = double( other.a_time.tv_usec - a_time.tv_usec );

        return seconds + usec / 1000000.0;
    }
    //----------------------------------------------------------------------

    //! Return time in micro seconds elapsed between the time stored in the object and \a other.
    long Elapsed_us( cSimpleTime const& other ) const
    {
        long seconds = other.a_time.tv_sec - a_time.tv_sec;
        long usec    = other.a_time.tv_usec - a_time.tv_usec;

        return seconds * 1000000 + usec;
    }
    //----------------------------------------------------------------------


    //! Return the time stored as C struct timeval
    timeval Timeval( void )
    {
        return a_time;
    }
#endif
};


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
