//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_dsa_h_general General file information
    \author   Dirk Osswald
    \date     2008-06-09

  \brief
    This file contains interface to #SDH::cDSA, a class to communicate with the tactile sensors of the SDH

  \section sdhlibrary_cpp_dsa_h_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_dsa_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-16 18:59:36 +0200 (Do, 16 Okt 2008) $
      \par SVN file revision:
        $Id: dsa.h 3722 2008-10-16 16:59:36Z Osswald2 $

  \subsection sdhlibrary_cpp_dsa_h_changelog Changelog of this file:
      \include dsa.h.log
*/
//======================================================================

#ifndef DSA_h_
#define DSA_h_

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "sdhexception.h"

#include "dbg.h"
#if SDH_USE_VCC
# include "rs232-vcc.h"
#else
# include "rs232-cygwin.h"
#endif
#include "simpletime.h"
#include "crc.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

NAMESPACE_SDH_START

#define DSA_MAX_PREAMBLE_SEARCH (2*3*(6*(14+13)) + 16)

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
   \brief Derived exception class for low-level DSA related exceptions.
*/
class cDSAException: public cSDHLibraryException
{
public:
  cDSAException( cMsg const & _msg )
    : cSDHLibraryException( "cDSAException", _msg )
  {}
};
//======================================================================


/*!
  Class to communicate with the tactile sensor controller DSACON32m of the SDH
*/
class cDSA
{
 public:
    //! data type for a single 'texel' (tactile sensor element)
    typedef UInt16 tTexel;

    //! A data structure describing the controller info about the remote DSACON32m controller
#if SDH_USE_VCC
#pragma pack(1)  // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
#endif
    struct sControllerInfo
    {
        UInt16 error_code;
        UInt32 serial_no;
        UInt8  hw_version;
        UInt16 sw_version;
        UInt8  status_flags;
        UInt8  feature_flags;
        UInt8  senscon_type;
        UInt8  active_interface;
        UInt32 can_baudrate;
        UInt16 can_id;
    }   SDH__attribute__((__packed__)); // for gcc we have to set the necessary 1 byte packing with this attribute

    //! A data structure describing the sensor info about the remote DSACON32m controller
#if SDH_USE_VCC
#pragma pack(1)  // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
#endif
    struct sSensorInfo
    {
        UInt16 error_code;
        UInt16 nb_matrices;
        UInt16 generated_by;
        UInt8  hw_revision;
        UInt32 serial_no;
        UInt8  feature_flags;
    } SDH__attribute__((__packed__)); // for gcc we have to set the necessary 1 byte packing with this attribute

    //! A data structure describing a single sensor matrix connected to the remote DSACON32m controller
#if SDH_USE_VCC
#pragma pack(1)   // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
#endif
    struct sMatrixInfo
    {
        UInt16 error_code;
        float  texel_width;
        float  texel_height;
        UInt16 cells_x;
        UInt16 cells_y;
        UInt8  uid[6];
        UInt8  reserved[2];
        UInt8  hw_revision;

        float  matrix_center_x;
        float  matrix_center_y;
        float  matrix_center_z;

        float  matrix_theta_x;
        float  matrix_theta_y;
        float  matrix_theta_z;
        float  fullscale;
        UInt8  feature_flags;
    } SDH__attribute__((__packed__)); // for gcc we have to set the necessary 1 byte packing with this attribute


    /*!
        A data structure describing a full tactile sensor frame read from the remote DSACON32m controller

        \remark
        - An object of this type is stored within the cDSA object
        - You can get a reference to the #sTactileSensorFrame object with the
          #GetFrame() function.
        - The object must be updated manually (for now)
          - by setting an appropriat framerate with #SetFramerate() once
          - by calling #UpdateFrame() periodically
        -
    */
    struct sTactileSensorFrame
    {
        UInt32  timestamp;  //!< the timestamp of the frame. Use #GetAgeOfFrame() to set this into relation with the time of the PC.
        UInt8   flags;      //!< internal data
        tTexel* texel;      //!< an 2D array of tTexel elements. Use #GetTexel() for easy access to specific individuall elements.

        //! constructor
        sTactileSensorFrame( void )
        {
            texel = NULL;
        }
    };

 protected:

    //! data structure for storing responses from the remote DSACON32m controller
#if SDH_USE_VCC
#pragma pack(1)    // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
#endif
     struct sResponse {
        UInt8   packet_id;
        UInt16  size;
        UInt8*  payload;
        int     max_payload_size;

        //! constructor to init pointer and max size
        sResponse( UInt8* _payload, int _max_payload_size )
        {
            payload = _payload;
            max_payload_size = _max_payload_size;
        }
    } SDH__attribute__((__packed__));  // for gcc we have to set the necessary 1 byte packing with this attribute

    friend std::ostream &operator<<(std::ostream &stream, cDSA::sResponse const &response );

    //! A stream object to print coloured debug messages
    cDBG dbg;

    //! the serial RS232 communication interface to use
    cRS232 comm_interface;

    //! flag, true if data should be sent Run Length Encoded by the remote DSACON32m controller
    bool do_RLE;

    //! the controller info read from the remote DSACON32m controller
    sControllerInfo controller_info;

    //! the sensor info read from the remote DSACON32m controller
    sSensorInfo     sensor_info;

    //! the matrix infos read from the remote DSACON32m controller
    sMatrixInfo*    matrix_info;

    //! the latest frame received from the remote DSACON32m controller
    sTactileSensorFrame frame;

    //! The total number of sensor cells
    int nb_cells;

    //! an array with the offsets of the first texel of all matrices into the whole frame
    int* texel_offset;

    //! default timeout used for reading in us
    long read_timeout_us;

    cSimpleTime start_pc;
    UInt32      start_dsa;

    void WriteCommandWithPayload( UInt8 command, UInt8* payload, UInt16 payload_len );

    inline void WriteCommand( UInt8 command )
    {
        WriteCommandWithPayload( command, NULL, 0 );
    }
    //----------------------------------------------------------------------

    /*!
        Read any response from the remote DSACON32m

        - tries to find the preamble within the next at most DSA_MAX_PREAMBLE_SEARCH bytes from the device
        - reads the packet id and size
        - reads all data indicated by the size read
        - if there is enough space in the payload of the response then the received data is stored there
          if there is not enough space in the payload of the response then the data is received but forgotten (to keep the communication line clear)
        - if sent, the CRC checksum is read and the data is checked. For invalid data an exception is thrown
    */
    void ReadResponse( sResponse* response );


    /*!
        Read and parse a controller info response from the remote DSA
    */
    void ReadControllerInfo( sControllerInfo* _controller_info );


    /*!
        Read and parse a sensor info response from the remote DSA
    */
    void ReadSensorInfo( sSensorInfo* _sensor_info );


    /*!
        Read and parse a matrix info response from the remote DSA
    */
    void ReadMatrixInfo( sMatrixInfo* _matrix_info  );


    /*!
        read and parse a full frame response from remote DSA
    */
    void ReadFrame( sTactileSensorFrame* frame_p  );


    /*!
        Send command to resquest controller info from remote DSACON32m.
        Read and parse the response from the remote DSACON32m.
    */
    void QueryControllerInfo( sControllerInfo* _controller_info  );


    /*!
        Send command to request sensor info from remote DSACON32m.
        Read and parse the response from the remote DSACON32m.
    */
    void QuerySensorInfo( sSensorInfo* _sensor_info );


    /*!
        Send command to request matrix info from remote DSACON32m.
        Read and parse the response from the remote DSACON32m.
    */
    void QueryMatrixInfo( sMatrixInfo* _matrix_info, int matrix_no );


    /*!
      Query all matrix infos
    */
    void QueryMatrixInfos( void );


    /*!
        Parse a full frame response from remote DSA
    */
    void ParseFrame( sResponse* response, sTactileSensorFrame* frame_p );


 public:
    /*!
        Constructor for cDSA
    */
    cDSA( int debug_level=0, int port=1 );


    //! Destructur: clean up and delete dynamically allocated memory
    ~cDSA();


    //! Return a reference to the sControllerInfo read from the remote DSACON32m controller
    inline sControllerInfo const & GetControllerInfo( void ) const
    {
        return controller_info;
    }
    //-----------------------------------------------------------------

    //! Return a reference to the sSensorInfo read from the remote DSACON32m controller
    inline sSensorInfo const & GetSensorInfo( void ) const
    {
        return sensor_info;
    }
    //-----------------------------------------------------------------

    //! Return a reference to the sMatrixInfo of matrix \a m read from the remote DSACON32m controller
    inline sMatrixInfo const & GetMatrixInfo( int m ) const
    {
        assert( 0 <= m  &&  m <= (int ) sensor_info.nb_matrices );
        return matrix_info[m];
    }
    //-----------------------------------------------------------------

    //! return a reference to the latest tactile sensor frame without reading from remote DSACON32m
    inline sTactileSensorFrame const & GetFrame() const
    {
        return frame;
    }
    //-----------------------------------------------------------------

    //! read the tactile sensor frame from remote DSACON32m and return a reference to it. A command to query the frame (periodically) must have been sent before.
    inline sTactileSensorFrame const & UpdateFrame()
    {
        ReadFrame( &frame );
        return frame;
    }
    //-----------------------------------------------------------------

    //! Set the framerate of the remote DSACON32m controller to 0 and close connection to it.
    void Close(void);


    void SetFramerate( UInt16 framerate, bool do_RLE = false, bool do_data_acquisition = true );


    /*!
        Return texel value at column \a x row \a y of matrix \a m of the last frame
    */
    tTexel GetTexel( int m, int x, int y ) const;



    /*!
        return the matrix index of the sensor matrix attached to finger with index \a fi [1..3] and \a part [0,1] = [proximal,distal]
    */
    inline int GetMatrixIndex( int fi, int part )
    {
        return fi * 2 + part;
    }


    /*!
        return age of frame in ms (time in ms from frame sampling until now)
    */
    inline unsigned long GetAgeOfFrame( sTactileSensorFrame* frame_p )
    {
        return ((unsigned long) (start_pc.Elapsed()*1000.0)) - (frame_p->timestamp - start_dsa);
    }
}; // end of class cDSA
//----------------------------------------------------------------------
//----------------------------------------------------------------------


std::ostream &operator<<( std::ostream &stream,  cDSA::sControllerInfo const &controller_info );


std::ostream &operator<<( std::ostream &stream,  cDSA::sSensorInfo const &sensor_info );


std::ostream &operator<<( std::ostream &stream,  cDSA::sMatrixInfo const &matrix_info );


std::ostream &operator<<( std::ostream &stream,  cDSA::sResponse const &response );


std::ostream &operator<<( std::ostream &stream,  cDSA const &dsa );


NAMESPACE_SDH_END

#endif


//======================================================================
/*
{
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C
  mode:ELSE
  End:
}
*/
//======================================================================}
