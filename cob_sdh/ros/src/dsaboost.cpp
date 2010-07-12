//======================================================================
/*!

\file
\section sdhlibrary_cpp_sdh_dsaboost_cpp_general General file information

\author   Dirk Osswald
\date     2009-08-02

\brief    helper stuff for the "boosted" DSA stuff



\section sdhlibrary_cpp_sdh_dsaboost_cpp_copyright Copyright

Copyright (c) 2007 SCHUNK GmbH & Co. KG

<HR>
\internal

\subsection sdhlibrary_cpp_sdh_dsaboost_cpp_details SVN related, detailed file specific information:
$LastChangedBy: Osswald2 $
$LastChangedDate: 2009-11-02 09:20:24 +0100 (Mo, 02 Nov 2009) $
\par SVN file revision:
$Id: dsaboost.cpp 4932 2009-11-02 08:20:24Z Osswald2 $

\subsection sdhlibrary_cpp_sdh_dsaboost_cpp_changelog Changelog of this file:
\include dsaboost.cpp.log

*/
//======================================================================

#include <iostream>
#include <fstream>
#include <vector>


// Include the cSDH interface
#include "cob_sdh/sdh.h"
#include "cob_sdh/util.h"
#include "cob_sdh/sdhlibrary_settings.h"
#include "cob_sdh/basisdef.h"
#include "cob_sdh/dsa.h"
#include "cob_sdh/dsaboost.h"

#include <boost/bind.hpp>
#include <boost/mem_fn.hpp>

USING_NAMESPACE_SDH

//extern cDBG cdbg;
#define cdbg std::cout

//-----------------------------------------------------------------




/*!
run function of updater thread

This function reads frames from the remote DSACON32m tactile sensor
controllers continuously in an infinite loop.

Uses global variable g_ts as the tactile sensor object
which must point to a valid and connected cDSA object on call.
*/
void cDSAUpdater::Updater()
{
    assert( ts != NULL );

    int error_level = 0;

    cdbg << "cDSAUpdater::Updater(): thread started\n";
    while( true )
    {
        try
        {
            //cdbg << "_Updater: updating\n";

            //semaphore.acquire()
            ts->UpdateFrame();
            //semaphore.release()


            if ( error_level > 0 )
                error_level--;

            //
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000/40));

        }
        catch (cDSAException* e)
        {
            error_level++;

            // ignore errors like checksum errors and retry next time
            cdbg << "cDSAUpdater::Updater(): ignoring " << e->what() << "\n";

            delete e;

        }

        if ( error_level > error_threshold )
        {
            std::cerr << "cDSAUpdater::Updater(): Too many errors from Tactile sensors of SDH. Reconnecting to DSACON32m!\n";

            try
            {
                ts->Close();
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            }
            catch (cDSAException* e)
            {
                // ignore errors while trying to reconnect
                cdbg << "cDSAUpdater::Updater(): ignoring " << e->what() << " while reconnecting step 1\n";
                delete e;
            }

            try
            {
                ts->Open();
            }
            catch (cDSAException* e)
            {
                // ignore errors while trying to reconnect
                cdbg << "cDSAUpdater::Updater(): ignoring " << e->what() << " while reconnecting step 2\n";
                delete e;
            }

            try
            {
                ts->SetFramerate( 30, true );

                error_level = 0; // reset error_level after successfull reconnection
                std::cerr << "cDSAUpdater::Updater() Succesfully reconnected to DSACON32m of SDH\n";
            }
            catch (cDSAException* e)
            {
                // ignore errors while trying to reconnect
                cdbg << "cDSAUpdater::Updater(): ignoring " << e->what() << " while reconnecting step 3\n";
                delete e;
            }
        }
    }
}
//-----------------------------------------------------------------


cDSAUpdater::cDSAUpdater( cDSA* _ts, int _error_threshold ) :
    error_threshold(_error_threshold),
    ts(_ts)
{
    assert( ts != NULL );

    ts->SetFramerate( 30, true );

    // start thread that reads the responses
    cdbg << "cDSAUpdater::cDSAUpdater(): Starting new updater thread...";
    updater_thread = boost::thread( boost::bind(&cDSAUpdater::Updater, this ) );
    cdbg << "OK\n";
}
//-----------------------------------------------------------------


double cIsGraspedByArea::FullArea( int m )
{
    return ts->GetMatrixInfo(m).cells_x * ts->GetMatrixInfo(m).texel_width *
    ts->GetMatrixInfo(m).cells_y * ts->GetMatrixInfo(m).texel_height;
}


void cIsGraspedByArea::SetCondition( double eaf0p, double eaf0d, double eaf1p, double eaf1d, double eaf2p, double eaf2d )
{
    expected_area[0] = eaf0p * FullArea( 0 );
    expected_area[1] = eaf0d * FullArea( 1 );
    expected_area[2] = eaf1p * FullArea( 2 );
    expected_area[3] = eaf1d * FullArea( 3 );
    expected_area[4] = eaf2p * FullArea( 4 );
    expected_area[5] = eaf2d * FullArea( 5 );
}

//! overloaded variant which uses an array of doubles instead of 6 single double parameters
void cIsGraspedByArea::SetCondition( double* eafx )
{
    for ( int i=0; i<6; i++ )
        expected_area[i] = eafx[i] * FullArea( i );
}

//! default constructor which initializes the internal date
cIsGraspedByArea::cIsGraspedByArea( cDSA* _ts )
// call base class constructors:
: cIsGraspedBase( _ts )
{
    SetCondition( 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 );
}

/*! Implementation of is grasped condition.
*
* \return true if the object is grasped according to the actual tactile sensor
*         data in \a ts and the condition set with SetCondition()
*/
bool cIsGraspedByArea::IsGrasped(void)
{
    cDSA::sContactInfo contact_info;
    bool is_grasped = true;
    for ( int i=0; i<6; i++ )
    {
        contact_info = ts->GetContactInfo(i);

        if (contact_info.area < expected_area[ i ])
        {
            is_grasped = false;
            break;
        }
    }
    return is_grasped;
}

//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored)
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================]
