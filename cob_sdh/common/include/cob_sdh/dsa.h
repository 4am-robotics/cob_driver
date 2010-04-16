//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_dsa_h_general General file information
    \author   Dirk Osswald
    \date     2008-06-09

  \brief
    This file contains interface to #SDH::cDSA, a class to communicate with the tactile sensors of the %SDH

  \section sdhlibrary_cpp_dsa_h_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_dsa_h_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2010-03-05 10:00:43 +0100 (Fr, 05 Mrz 2010) $
      \par SVN file revision:
        $Id: dsa.h 5355 2010-03-05 09:00:43Z Osswald2 $

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

//! SDH::cDSA is the end user interface class to access the DSACON32m, the tactile sensor controller of the %SDH
/*!
  Class to communicate with the tactile sensor controller DSACON32m of the %SDH

  \anchor checksum_error_on_windows_console
  \bug cDSAException: Checksum Error on Windows console
    While communicating with the tactile sensor controller a "cDSAException: Checksum Error"
    might be thrown once in a while. This seems to happen only when the program
    is started from a native windows command console (cmd.exe), but not e.g.
    when the program is started from a cygwin console. Strange.
    \b Workaround is to catch the exception and ignore the frame.
*/
class cDSA
{
 public:
    //! data type for a single 'texel' (tactile sensor element)
    typedef UInt16 tTexel;

    //! error codes returned by the remote DSACON32m tactile sensor controller
    enum eDSAErrorCode
    {
        E_SUCCESS,
        E_NOT_AVAILABLE,
        E_NO_SENSOR,
        E_NOT_INITIALIZED,
        E_ALREADY_RUNNING,
        E_FEATURE_NOT_SUPPORTED,
        E_INCONSISTENT_DATA,
        E_TIMEOUT,
        E_READ_ERROR,
        E_WRITE_ERROR,
        E_INSUFFICIENT_RESOURCES,
        E_CHECKSUM_ERROR,
        E_CMD_NOT_ENOUGH_PARAMS,
        E_CMD_UNKNOWN,
        E_CMD_FORMAT_ERROR,
        E_ACCESS_DENIED,
        E_ALREADY_OPEN,
        E_CMD_FAILED,
        E_CMD_ABORTED,
        E_INVALID_HANDLE,
        E_DEVICE_NOT_FOUND,
        E_DEVICE_NOT_OPENED,
        E_IO_ERROR,
        E_INVALID_PARAMETER,
        E_INDEX_OUT_OF_BOUNDS,
        E_CMD_PENDING,
        E_OVERRUN,
        E_RANGE_ERROR
    };


    //! A data structure describing the controller info about the remote DSACON32m controller
#if SDH_USE_VCC
#pragma pack(push,1)  // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
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

    //! Structure to hold info about the sensitivity settings of one sensor patch
    struct sSensitivityInfo
    {
        UInt16 error_code;  //!< 0000h, if successful, otherwise error code
        UInt8  adj_flags;   //!< Bit vector indicating the sensitivity adjustment options:
                            //!< - D7...2 reserved
                            //!< - D1     1: user can change the sensitivity
                            //!<          0: sensitivity cannot be changed by the user
        float  cur_sens;    //!< Currently set sensitivity value. Floating point value.
                            //!< 0 is minimum sensitivity, 1.0 is maximum sensitivity.
        float  fact_sens;   //!< Sensitivity value that is used, if a factory restore command is
                            //!< issued. Floating point value. 0 is minimum sensitivity, 1.0 is
                            //!< maximum sensitivity.
    } SDH__attribute__((__packed__)); // for gcc we have to set the necessary 1 byte packing with this attribute

#if SDH_USE_VCC
#pragma pack(pop)   // for VCC (MS Visual Studio) restore normal packing
#endif


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

    //! Structure to hold info about the contact of one sensor patch
    struct sContactInfo
    {
        double force;  //!< accumulated force in N
        double area;   //!< area of contact in mm*mm.
        double cog_x;  //!< center of gravity of contact in x direction of sensor patch in mm
        double cog_y;  //!< center of gravity of contact in y direction of sensor patch in mm
    };


 protected:

    //! data structure for storing responses from the remote DSACON32m controller
#if SDH_USE_VCC
#pragma pack(push,1)    // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
#endif
     struct sResponse {
        UInt8   packet_id;
        UInt16  size;
        UInt8*  payload;
        Int32   max_payload_size;

        //! constructor to init pointer and max size
        sResponse( UInt8* _payload, int _max_payload_size )
        {
            payload = _payload;
            max_payload_size = _max_payload_size;
        }
    } SDH__attribute__((__packed__));  // for gcc we have to set the necessary 1 byte packing with this attribute
#if SDH_USE_VCC
#pragma pack(pop)   // for VCC (MS Visual Studio) restore normal packing
#endif

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

    //! threshold of texel cell value for detecting contacts with GetContactArea
    tTexel contact_area_cell_threshold;

    //! threshold of texel cell value for detecting forces with GetContactForce
    tTexel contact_force_cell_threshold;

    //! additional calibration factor for forces in GetContactForce
    double force_factor;


    /*! For the voltage to pressure conversion in _VoltageToPressure()
        enter one pressure/voltage measurement here (from demo-dsa.py --calibration):
    */
    double calib_pressure;  // N/(mm*mm)
    //! see calib_pressure
    double calib_voltage;   // "what the DSA reports:" ~mV


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
        Constructor for cDSA. This constructs a cDSA object to communicate with
        the remote DSACON32m controller within the %SDH.

        The connection is opened and established, and the sensor, controller and
        matrix info is queried from the remote DSACON32m controller.
        This initialization may take up to 9 seconds, since the
        DSACON32m controller needs > 8 seconds for "booting". If the %SDH is already powered
        for some time then this will be much quicker.

        \param debug_level - the level of debug messages to be printed:
                             - if > 0 (1,2,3...) then debug messages of cDSA itself are printed
                             - if > 1 (2,3,...) then debug messages of the low level communication interface object are printed too
        \param port - the RS232 port to use for communication. (port 0 = ttyS0 = COM1, port 1 = ttyS1 = COM2, ...)
        \param device_format_string - a format string (C string) for generating the device name, like "/dev/ttyS%d" (default) or "/dev/ttyUSB%d".
                                      Must contain a %d where the port number should be inserted.
                                      This char array is duplicated on construction.
                                      When compiled with VCC (MS-Visual C++) then this is not used.
    */
    cDSA( int debug_level=0, int port=1, char const* device_format_string="/dev/ttyS%d" );


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

    //! (Re-)open connection to DSACON32m controller, this is called by the constructor automatically, but is still usefull to call after a call to Close()
    void Open(void);

    //! Set the framerate of the remote DSACON32m controller to 0 and close connection to it.
    void Close(void);


    /*!
     * Set the \a framerate for querying full frames.
     *
     * @param framerate - rate of frames. 0 will make the remote DSACON32m in
     *                    %SDH stop sending frames, any value > 0 will make
     *                    the remote DSACON32m in %SDH send frames at the
     *                    highest possible rate (for now: 30 FPS (frames per second)).
     * @param do_RLE              - flag, if true then use RLE (run length encoding) for sending frames
     * @param do_data_acquisition - flag, enable or disable data acquisition. Must be true if you want to get new frames
     */
    void SetFramerate( UInt16 framerate, bool do_RLE=true, bool do_data_acquisition=true );

    /*!
     * Set the \a framerate for querying full frames.
     *
     * @param framerate - rate of frames. 0 will make the remote DSACON32m in
     *                    %SDH stop sending frames, any value > 0 will make
     *                    the remote DSACON32m in %SDH send frames at the
     *                    highest possible rate (for now: 30 FPS (frames per second)).
     * @param do_RLE              - flag, if true then use RLE (run length encoding) for sending frames
     * @param do_data_acquisition - flag, enable or disable data acquisition. Must be true if you want to get new frames
     * @param retries             - number of times the sending will be retried in case of an error (like timeout while waiting for response)
     * @param ignore_exceptions   - flag, if true then exceptions are ignored in case of error. After \a retries tries the function just returns even in case of an error
     */
    void SetFramerateRetries( UInt16 framerate, bool do_RLE=true, bool do_data_acquisition=true, unsigned int retries=0, bool ignore_exceptions=false )
    throw (cDSAException*);


    /*!
        Send command to get matrix sensitivity. Returns sensitivities of matrix no \a matrix_no.

        A struct is returned containing the members
        \a error_code - see DSACON32 Command Set Reference Manual
        \a adj_flags  - see DSACON32 Command Set Reference Manual
        \a cur_sens   - the currently set sensitivity as float [0.0 .. 1.0] (0.0 is minimum, 1.0 is maximum sensitivity)
        \a fact_sens  - the factory sensitivity as float [0.0 .. 1.0] (0.0 is minimum, 1.0 is maximum sensitivity)

        Raises a cDSAException in case of invalid responses from the remote DSACON32m controller.

        \bug With DSACON32m firmware R218 and before this did not work, instead the factory default (0.5) was always reported
        <br><b>=> Resolved in DSACON32m firmware R268</b>
    */
    sSensitivityInfo GetMatrixSensitivity( int matrix_no )
    throw (cDSAException*);


    /*!
        Send command to set matrix sensitivity to \a sensitivity as float [0.0 .. 1.0] (0.0 is minimum, 1.0 is maximum sensitivity).
        If \a do_all_matrices is true then the \a sensitivity is set for all matrices.
        If \a do_reset is true then the sensitivity is reset to the factory default.
        If \a do_persistent is true then the sensitivity is saved persistently to
        configuration memory and will thus remain after the next power off/power on
        cycle and will become the new factory default value. If \a do_persistent is
        false (default) then the value will be reset to default on the next power
        off/power on cycle
        \warning PLEASE NOTE: the maximum write endurance of the configuration
        memory is about 100.000 times!

        Raises a cDSAException in case of invalid responses from the remote DSACON32m controller.

        \remark Setting the matrix sensitivity persistently is only possible if the DSACON32m firmware is R268 or above.
    */
    void SetMatrixSensitivity( int matrix_no,
                               double sensitivity,
                               bool do_all_matrices=false,
                               bool do_reset=false,
                               bool do_persistent=false )
    throw (cDSAException*);


    /*!
        Send command to set matrix threshold to \a threshold as integer [0 .. 4095] (0 is minimum, 4095 is maximum threshold).
        If \a do_all_matrices is true then the \a threshold is set for all matrices.
        If \a do_reset is true then the threshold is reset to the factory default.
        If \a do_persistent is true then the threshold is saved persistently to
        configuration memory and will thus remain after the next power off/power on
        cycle and will become the new factory default value. If \a do_persistent is
        false (default) then the value will be reset to default on the next power
        off/power on cycle
        \warning PLEASE NOTE: the maximum write endurance of the configuration
        memory is about 100.000 times!

        Raises a cDSAException in case of invalid responses from the remote DSACON32m controller.

        \remark Getting the matrix threshold is only possible if the DSACON32m firmware is R268 or above.
    */
    void SetMatrixThreshold( int matrix_no,
                             UInt16 threshold,
                             bool do_all_matrices=false,
                             bool do_reset=false,
                             bool do_persistent=false )
    throw (cDSAException*);

    /*!
        Send command to get matrix threshold. Returns threshold of matrix no \a matrix_no.

        \return threshold  - the currently set threshold as integer [0 .. 4095] (0 is minimum, 4095 is maximum threshold)

        Raises a cDSAException in case of invalid responses from the remote DSACON32m controller.

        \remark Getting the matrix threshold is only possible if the DSACON32m firmware is R268 or above.
    */
    UInt16 GetMatrixThreshold( int matrix_no )
    throw (cDSAException*);

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

    double GetContactArea( int m );

 private:
    double VoltageToPressure( double voltage );

    void ReadAndCheckErrorResponse( char const* msg );


 public:
    sContactInfo GetContactInfo( int m );


    /*!
     * Return a short string description for \a error_code
     *
     * @param  error_code - error code as returned from the remote DSACON32m tactile sensor controller
     * @return short string description for \a error_code
     */
    static char const* ErrorCodeToString( eDSAErrorCode error_code );
    static char const* ErrorCodeToString( UInt16 error_code )
    {
        return ErrorCodeToString( (eDSAErrorCode) error_code );
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
