//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhserial_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cSDHSerial.

  \section sdhlibrary_cpp_sdhserial_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdhserial_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2011-03-09 11:55:11 +0100 (Mi, 09 Mrz 2011) $
      \par SVN file revision:
        $Id: sdhserial.cpp 6526 2011-03-09 10:55:11Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhserial_cpp_changelog Changelog of this file:
      \include sdhserial.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "util.h"
#include "sdhserial.h"
#include "crc.h"
#include "sdh_codes.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


NAMESPACE_SDH_START

enum {
    eNUMBER_OF_ELEMENTS = cSimpleVector::eNUMBER_OF_ELEMENTS,
};

#if SDH_USE_VCC
#pragma pack(push,1)  // for VCC (MS Visual Studio) we have to set the necessary 1 byte packing with this pragma
#endif
//! data structure with binary data for request from PC to SDH
struct sSDHBinaryRequest
{
    /*
     * Unfortunately g++ 3.4.4 can NOT handle packed enums.
     * Therefore we cannot use the eCommandCode type here
     */
    unsigned char  cmd_code;
    unsigned char  nb_data_bytes;
    unsigned char  nb_valid_parameters;
    union {
        float          parameter[ eNUMBER_OF_ELEMENTS ];
        unsigned char  parameter_bytes[ sizeof( float ) * eNUMBER_OF_ELEMENTS + sizeof(tCRCValue) ];
    };

    /*!
     *  ctor, create a request with cmd_code \a command and eNUMBER_OF_ELEMENTS parameter from \a value or no parameters if \a value is NULL.
     *  Add crc if \a use_crc16 is true and set nb_data_bytes appropriately
     */
    sSDHBinaryRequest( eCommandCode command, double* value, bool use_crc16 );

    //! return a ptr to the CRC value in parameter_bytes, assuming that nb_data_bytes is correct (including the CRC bytes)
    tCRCValue* CRC16() const
    {
        return (tCRCValue*) &(parameter_bytes[nb_data_bytes - sizeof( nb_valid_parameters ) - sizeof(tCRCValue)]);
    }

    //! return the total number of bytes to send
    int GetNbBytesToSend() const
    {
        return sizeof( cmd_code ) + sizeof( nb_data_bytes ) + nb_data_bytes;
    }

} SDH__attribute__((packed));

//! data structure with binary data for response from SDH to PC
struct sSDHBinaryResponse
{
    /*
     * Unfortunately g++ 3.4.4 can NOT handle packed enums.
     * Therefore we cannot use the eCommandCode and eReturnCode types here
     */
    unsigned char cmd_code;
    unsigned char nb_data_bytes;
    unsigned char nb_valid_parameters;
    unsigned char status_code;
    union {
        float         parameter[ eNUMBER_OF_ELEMENTS ];
        unsigned char  parameter_bytes[ sizeof( float ) * eNUMBER_OF_ELEMENTS + sizeof(tCRCValue) ];
    };

    //! return a ptr to the CRC value in parameter_bytes, assuming that nb_data_bytes is correct (including the CRC bytes)
    tCRCValue* CRC16() const
    {
        return (tCRCValue*) &(parameter_bytes[nb_data_bytes - sizeof( nb_valid_parameters ) - sizeof( status_code ) - sizeof(tCRCValue)]);
    }

    //! check the CRC value in parameter_bytes. Throw an exception if check fails
    void CheckCRC16() const
       throw (cSDHErrorCommunication*);

} SDH__attribute__((packed));
#if SDH_USE_VCC
#pragma pack(pop)   // for VCC (MS Visual Studio) restore normal packing
#endif

//! helper functions to insert a human readable form of the \a request into \a stream
std::ostream& operator<<( std::ostream &stream,  sSDHBinaryRequest const &request );

//! helper functions to insert a human readable form of the \a restore into \a stream
std::ostream& operator<<( std::ostream &stream,  sSDHBinaryResponse const &response );


NAMESPACE_SDH_END

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------


sSDHBinaryRequest::sSDHBinaryRequest( eCommandCode command, double* value, bool use_crc16 )
    :
    cmd_code( (unsigned char) command ),
    nb_data_bytes( sizeof( nb_valid_parameters ) ),
    nb_valid_parameters( value ? eNUMBER_OF_ELEMENTS : 0 )
{
    if ( value )
    {
        nb_data_bytes += sizeof( parameter );

        for ( int ai=0; ai<eNUMBER_OF_ELEMENTS; ai++ )
            parameter[ ai ] = float( value[ai] );
    }
    if ( use_crc16 )
    {
        nb_data_bytes  += 2;
        cCRC_SDH checksum;
        *CRC16() = checksum.AddBytes( (unsigned char*) &cmd_code,
                                      GetNbBytesToSend() - sizeof( tCRCValue ) );
    }
}
//-----------------------------------------------------------------
//-----------------------------------------------------------------

void sSDHBinaryResponse::CheckCRC16() const
throw (cSDHErrorCommunication*)
{
    cCRC_SDH checksum;
    tCRCValue crc_calculated = checksum.AddBytes( (unsigned char*) &cmd_code,
                                                  sizeof( cmd_code ) + sizeof( nb_data_bytes ) + nb_data_bytes - sizeof( tCRCValue ) );
    if ( crc_calculated != *CRC16() )
        throw new cSDHErrorCommunication( cMsg( "CRC error in response expected 0x%04x but got 0x%04x", crc_calculated, *CRC16() ) );
}
//-----------------------------------------------------------------
//-----------------------------------------------------------------

std::ostream & NS_SDH operator<<( std::ostream &stream,  sSDHBinaryRequest const &request )
{
    stream << "sSDHBinaryRequest:\n"
    << "  cmd_code=0x" << std::hex << std::setfill('0') << std::setw(2)<< int(request.cmd_code) << " (" << SDHCommandCodeToString( eCommandCode( request.cmd_code ) ) << ")\n"
    << "  nb_data_bytes=" << std::dec << int(request.nb_data_bytes) << "\n"
    << "  nb_valid_parameters=" << int(request.nb_valid_parameters) << "\n"
    << "  parameter=";
    char const* sep = "";
    for ( int i=0; i<request.nb_valid_parameters && i<eNUMBER_OF_ELEMENTS; i++ )
    {
        stream << sep << request.parameter[i];
        sep = ",";
    }
    if ( request.nb_valid_parameters > eNUMBER_OF_ELEMENTS )
        stream << sep << "... something is fishy here!";

    if ( (request.nb_data_bytes - sizeof( request.nb_valid_parameters )) % sizeof( float ) == sizeof( tCRCValue ) )
        stream << "\n  crc=0x" << std::hex << std::setfill('0') << std::setw(4) <<  *(request.CRC16()) << std::dec;
    stream << "\n";
    return stream;
}
//-----------------------------------------------------------------

std::ostream & NS_SDH operator<<( std::ostream &stream,  sSDHBinaryResponse const &response )
{
    stream << "sSDHBinaryResponse:\n"
    << "  cmd_code=0x" << std::hex << std::setfill('0') << std::setw(2)<< int(response.cmd_code) << " (" << SDHCommandCodeToString( eCommandCode( response.cmd_code ) ) << ")\n"
    << "  nb_data_bytes=" << std::dec << int(response.nb_data_bytes) << "\n"
    << "  nb_valid_parameters=" << int(response.nb_valid_parameters) << "\n"
    << "  status_code=" << int(response.status_code) << " (" << SDHReturnCodeToString( eReturnCode( response.status_code ) ) << ")\n"
    << "  parameter=";
    char const* sep = "";
    for ( int i=0; i<response.nb_valid_parameters && i<eNUMBER_OF_ELEMENTS; i++ )
    {
        stream << sep << response.parameter[i];
        sep = ",";
    }
    if ( response.nb_valid_parameters > eNUMBER_OF_ELEMENTS )
        stream << sep << "... something is fishy here!";

    if ( (response.nb_data_bytes - sizeof( response.nb_valid_parameters ) - sizeof( response.status_code )) % sizeof( float ) == sizeof( tCRCValue ) )
        stream << "\n  crc=0x" << std::hex << std::setfill('0') << std::setw(4) <<  *(response.CRC16()) << std::dec;
    stream << "\n";
    return stream;
}
//-----------------------------------------------------------------
//-----------------------------------------------------------------

cSDHSerial::cSDHSerial( int _debug_level )
    :
    // call base class constructors:
    cSDHBase( _debug_level ),

    // init member objects:
    com( NULL )
{
    //---------------------
    // type size checking
    sSDHBinaryRequest request( eCommandCode(RC_OK), NULL, false );    // unused dummy variables, just used for checking offsets of members
    sSDHBinaryResponse response;
    /*
    assert( sizeof( enum eCommandCodeEnum ) == 1 ); // fails when compiled with g++ v3.4.4!
    assert( sizeof( eCommandCode ) == 1 );          // fails when compiled with g++ v3.4.4!
    assert( sizeof( enum eReturnCodeEnum ) == 1 );  // fails when compiled with g++ v3.4.4!
    assert( sizeof( eReturnCode ) == 1 );           // fails when compiled with g++ v3.4.4!
    */
    assert( sizeof( request.cmd_code ) == 1 );
    assert( sizeof( request.nb_data_bytes ) == 1 );
    assert( sizeof( request.nb_valid_parameters ) == 1 );
    assert( ((unsigned char*) &(request.cmd_code) ) +1             == ((unsigned char*) &(request.nb_data_bytes) ) );
    assert( ((unsigned char*) &(request.nb_data_bytes) ) +1        == ((unsigned char*) &(request.nb_valid_parameters) ) );
    assert( ((unsigned char*) &(request.nb_valid_parameters) ) +1  == ((unsigned char*) &(request.parameter) )  );
    assert( sizeof( response.cmd_code ) == 1 );
    assert( sizeof( response.nb_data_bytes ) == 1 );
    assert( sizeof( response.nb_valid_parameters ) == 1 );
    assert( sizeof( response.status_code ) == 1 );
    assert( ((unsigned char*) &(response.cmd_code) ) +1            == ((unsigned char*) &(response.nb_data_bytes) ) );
    assert( ((unsigned char*) &(response.nb_data_bytes) ) +1       == ((unsigned char*) &(response.nb_valid_parameters) ) );
    assert( ((unsigned char*) &(response.nb_valid_parameters) ) +1 == ((unsigned char*) &(response.status_code) )  );
    assert( ((unsigned char*) &(response.status_code) ) +1         == ((unsigned char*) &(response.parameter) )  );
    //---------------------


    //---------------------
    // Option handling:

    // use green as color for messages from cSDHSerial
    cdbg.SetColor( "green" );
    cdbg << "Debug messages of cSDHSerial are printed like this.\n";

    //---------------------
    // initialize additional member variables:

    m_sequtime = 0.0; // !!!

    //! String to use as "End Of Line" marker when sending to %SDH

    EOL="\r\n";
    //---------------------
}
//-----------------------------------------------------------------


void cSDHSerial::Open( cSerialBase* _com )
    throw (cSDHLibraryException*)
{
    com = _com;
    assert( com != NULL );

    //---------------------
    // open connection to SDH:

    nb_lines_to_ignore = 0;

    com->Open();
    // the above call will succeed even if the hand is connected but off
    //---------------------


    //---------------------
    // Clean up communication line. (To make sure that no previous
    // communication that was received only partly by the SDH will
    // lead to an error when sending the "ver" command below. This
    // can happen if a PC program gets interrupted while in the
    // middle of sending a command to the SDH. Then any new
    // command for the SDH sent by the next PC program will
    // confuse the SDH

    cSerialBase::cSetTimeoutTemporarily set_timeout_temporarily( com, 1.0 );
    // previous timeout will be automatically restored when function is left (by return or exception)
    try
    {
        Send( " " ); // empty command to terminate any potential partly received previous command
    }
    catch (cSDHErrorCommunication* e)
    {
        cdbg << "caught <" << *e << "> (ignored while cleaning up communication)\n";
        delete e;
    }
    //---------------------


    //---------------------
    // to make sure that the SDH is connected:
    // try to get the SDH firmware version with timeout
    try
    {
        Send( "ver" );
    }
    catch (cSDHErrorCommunication* e)
    {
        cdbg << "caught <" << *e << ">\n";
        //std::cerr << "SDHLibrary-CPP: cSDHSerial.Open(): Timeout while trying to get SDH firmware version\n  Is the SDH really connected and powered?\n";

        Close(); // make sure this cSDHSerial object does not appear open in case of error

        cSDHErrorCommunication*  e2 = new cSDHErrorCommunication( cMsg( "Could not connect to SDH firmware! Is the SDH really connected and powered? (Sending \"ver\" caused: %s)", e->what() ) );
        delete e;

        throw e2;
    }
    //---------------------
}
//-----------------------------------------------------------------


void cSDHSerial::Close()
    throw (cSDHLibraryException*)
{
    com->Close();
}
//-----------------------------------------------------------------


bool cSDHSerial::IsOpen( void )
{
    return com != NULL  &&  com->IsOpen();
}
//----------------------------------------------------------------------



void cSDHSerial::Send( char const* s, int nb_lines, int nb_lines_total, int max_retries )
//void cSDHSerial::Send( char const* s, int nb_lines, int nb_lines_total )
    throw( cSDHLibraryException* )
{
    int retries = max_retries; // retry sending at most this many times
    while (retries > 0)
    {
        try
        {
            //---------------------
            // first read all lines to ignore (replies of previous commands)
            while ( nb_lines_to_ignore > 0 )
            {
                com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n" );
                nb_lines_to_ignore -= 1;
                cdbg << "ignoring line <" << reply.CurrentLine() << ">\n";

                reply.Reset();
            }
            //---------------------


            firmware_state = eEC_SUCCESS;
            reply.Reset();

            //---------------------
            // send new command to SDH
            cdbg << "cSDHSerial::Send: sending command '" << s << "' to SDH\n";
            cdbg << "  nb_lines=" << nb_lines << "  nb_lines_total=" << nb_lines_total << "  nb_lines_to_ignore=" << nb_lines_to_ignore << "\n";

            com->write( s );
            com->write( EOL );

            cdbg << "sent command\n";
            //---------------------

            //---------------------
            // read reply if requested
            while (nb_lines == All || nb_lines > 0)
            {

                //---------------------
                // now read requested reply lines of current command
                com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n" );
                cdbg << "read line '" << reply.CurrentLine() << "'\n";
                if (nb_lines != All)
                {
                    nb_lines -= 1;
                }
                if (nb_lines_total != All)
                {
                    nb_lines_total -= 1;
                }

                // remove beginning and trailing "\r\n" of the line just read

                //   remove beginning "\r\n"-s:
                char* startp = reply.CurrentLine();

                startp[ reply.eMAX_CHARS ] = '\0';
                // strchr will also find the matching \0 char!
                while ( *startp != '\0' && strchr( "\r\n", *startp ) != NULL )
                {
                    startp += 1;
                }
                if ( startp != reply.CurrentLine() )
                {
                    memmove( reply.CurrentLine(), startp, strlen( startp ) );
                }

                //   remove trailing "\r\n"-s:
                char* endp = reply.CurrentLine() + strlen( reply.CurrentLine() )-1;
                while ( endp >= reply.CurrentLine() && strchr( "\r\n", *endp ) != NULL )
                {
                    *endp = '\0';
                    endp -= 1;
                }

                cdbg << "appending cleaned up line '" << reply.CurrentLine() << "'\n";
                if (reply.CurrentLine()[0] != '@')
                {
                    break;
                }
                //---------------------
            }

            //---------------------
            // remember if there are more lines to be ignored next time
            if (nb_lines_total != All)
            {
                nb_lines_to_ignore = nb_lines_total;
            }
            cdbg << nb_lines_to_ignore <<" lines remain to be ignored\n";
            //---------------------

            //---------------------
            // set state if possible
            if (nb_lines_to_ignore == 0)
            {
                ExtractFirmwareState();
            }
            //---------------------

            // finished, so no more retries needed
            retries = 0;

        } // end of try
        catch (cSDHErrorCommunication* e)
        {
            // some communication error occured, so retry:
            retries -= 1;
            if (retries <= 0)
            {
                cdbg << "Retried sending, but still got errors from SDH!\n";
                // rethrow e:
                throw;
            }
            cdbg << "ignoring cSDHErrorCommunication: " << *e << "\n";

            // resync first:
            Sync();
            //now start over again

        } // end of catch
    } // end of while (retries > 0)
    cdbg << "got reply: " << reply;
}
//-----------------------------------------------------------------


void cSDHSerial::ExtractFirmwareState()
    throw (cSDHErrorCommunication*)
{
    //---------------------
    // check first char of last line of lines

    if   (reply[-1][0] == 'E')
    {
        // it is an error message:
        sscanf( reply[-1] +1, "%d", (int*) (&firmware_state) );
        cdbg << "got error reply '" << reply[-1] << "' = " << firmware_state << " = " << firmware_error_codes[firmware_state] << "\n";
        throw new cSDHErrorCommunication( cMsg( "SDH firmware reports error %d = %s", firmware_state, firmware_error_codes[firmware_state]) );
    }

    else if (reply[-1][0] == '@')
    {
        // it is an debug message (should not happen):
        throw new cSDHErrorCommunication( cMsg( "Cannot get SDH firmware state from lines" ) );
    }

    else
    {
        // it is a normal "command completed" line:
        firmware_state = eEC_SUCCESS;
    }
}
//-----------------------------------------------------------------


double cSDHSerial::GetDuration( char* line )
    throw (cSDHErrorCommunication*)
{
    char* pattern_at = strstr( line, "=" );

    if (pattern_at == NULL)
        throw new cSDHErrorCommunication( cMsg( "Could not extract duration from lines '%s'", line ) );

    double duration;
    sscanf( pattern_at, "=%lf", &duration );

    cdbg << "extracted duration " << duration << "\n";
    return duration;
}
//-----------------------------------------------------------------


double cSDHSerial::get_duration( void )
{
    // Actual input/output for the command looks like:
    //--
    // get_duration
    // GET_DURATION=4.51
    //
    // before firmware 0.0.3.1 output input/output for the command looks like:
    // get_duration
    // @max distance=45.06, T=4.51s, num_points: 451
    // GET_DURATION=4.51

    //---------------------
    // settings for sequ/non-sequ:
    int nb_lines_total = 1;
    int nb_lines = nb_lines_total;
    //---------------------

    //---------------------
    // send command and parse reply
    Send( "get_duration", nb_lines, nb_lines_total );
    double T = GetDuration( reply[0] );
    //---------------------

    return T;
}
//----------------------------------------------------------------------


void cSDHSerial::Sync( )
    throw( cSDHErrorCommunication* )
{
    // first read all lines to ignore (replies of previous commands)
    while ( nb_lines_to_ignore > 0 )
    {
        com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n" );
        nb_lines_to_ignore -= 1;
        cdbg << "syncing: ignoring line <" << reply.CurrentLine() << ">\n";

        reply.Reset();
    }
    if (reply.Length() > 0)
        ExtractFirmwareState();
}
//-----------------------------------------------------------------

void cSDHSerial::BinarySync( double timeout_s )
    throw( cSDHErrorCommunication* )
{
    // read and ignore all bytes available within timeout_s
    char* buffer[ 256 ];
    int nb_bytes_received = com->Read( buffer, 256, long(timeout_s*1E6), false );
    cdbg << "cSDHSerial::BinarySync: ignoring " << nb_bytes_received << " bytes\n";
}
//-----------------------------------------------------------------

void cSDHSerial::SyncUnknown( )
    throw( cSDHErrorCommunication* )
{
    // read all lines until timeout
    while (1)
    {
        try
        {
            com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n", true );
            cdbg << "syncing unknown: ignoring line <" << reply.CurrentLine() << ">\n";

            reply.Reset();
        }
        catch (cSerialBaseException* e)
        {
            cdbg << __FILE__ << ":" << __LINE__ << " ignoring cSerialBaseException: " << *e << "\n";
            delete e;
            break;
        }
    }
    if (reply.Length() > 0)
        ExtractFirmwareState();
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::AxisCommand( char const* command, int axis, double* value )
    throw (cSDHLibraryException*)
{
#if SDH_USE_VCC
    int cutoff  = static_cast<int>(strlen(command)) + 1;
#else
    int cutoff  = strlen( command ) + 1;
#endif
    int cutoff1 = cutoff +3;
    char cmd[ cSimpleStringList::eMAX_CHARS ];
    int retries = 3; // retry sending at most this many times
    while (retries > 0)
    {
        try
        {
            if (axis == All && value == NULL)
            {
                Send( command );
                return cSimpleVector( NUMBER_OF_AXES, reply[0]+cutoff );
            }

            if (axis != All)
            {
                CheckIndex( axis, NUMBER_OF_AXES, "axis" );
                if (value == NULL)
                {
#if SDH_USE_VCC
      _snprintf_s( cmd, cSimpleStringList::eMAX_CHARS, cSimpleStringList::eMAX_CHARS-1, "%s(%d)", command, axis );
#else
      snprintf( cmd, cSimpleStringList::eMAX_CHARS-1, "%s(%d)", command, axis );
#endif
                    Send( cmd );
                    return cSimpleVector( 1, axis, reply[0]+cutoff1 );
                }

#if SDH_USE_VCC
      _snprintf_s( cmd, cSimpleStringList::eMAX_CHARS, cSimpleStringList::eMAX_CHARS-1, "%s(%d)=%12.3f", command, axis, *value );
#else
                snprintf( cmd, cSimpleStringList::eMAX_CHARS-1, "%s(%d)=%12.3f", command, axis, *value );
#endif
                Send( cmd );
                return cSimpleVector( 1, axis, reply[0]+cutoff1 );
            }

            if (axis == All)
            {
                Send( cMsg( "%s=%f,%f,%f,%f,%f,%f,%f", command, value[0], value[1], value[2], value[3], value[4], value[5], value[6] ).c_str() );
                return cSimpleVector( NUMBER_OF_AXES, reply[0] + cutoff );
            }

            throw new cSDHErrorInvalidParameter( cMsg( "Invalid parameter in AxisCommand( command=%s, axis=%d, value=%p )'", command, axis, value ) );

        } // end of try
        //catch (cSimpleVectorException* e)
        catch (cSDHLibraryException* e)
        {
            // these errors seem to happen on linux only (not cygwin) where a reply can be partly received:

            // assume some communication error occured, so retry:
            retries -= 1;
            if (retries > 0)
                //cdbg << __FILE__ << ":" << __LINE__ << " ignoring cSimpleVectorException: " << *e << "\n";
                cdbg << __FILE__ << ":" << __LINE__ << " ignoring cSDHLibraryException: " << *e << " (retrying)\n";
            else
            {
                cdbg << __FILE__ << ":" << __LINE__ << " retried but giving up now\n";
                // rethrow e:
                throw;
            }

            // resync first:
            Sync();
            //now start over again

            delete e;

        } // end of catch

    } // end of while (retries > 0)

    cdbg << "Retried sending, but still didnt work!\n";
    throw new cSDHLibraryException( "cSDHLibraryException", cMsg( "Unknown error while retrying" ) );
}
//----------------------------------------------------------------------

cSimpleVector cSDHSerial::BinaryAxisCommand( eCommandCode command, int axis, double* value )
    throw (cSDHLibraryException*)
{
    bool use_crc16 = com->UseCRC16();
    sSDHBinaryRequest  request( command, value, use_crc16 );
    sSDHBinaryResponse response;

    int nb_bytes_to_receive =
        sizeof( response.cmd_code )
        + sizeof( response.nb_data_bytes )
        + sizeof( response.nb_valid_parameters )
        + sizeof( response.status_code )
        + sizeof( request.parameter )
        + ((use_crc16)? sizeof( tCRCValue ) : 0); // ANOTE: this will fail if the response unexpectedly contains a CRC (But this should not happen)

    int retries = 3; // retry sending at most this many times
    while (retries > 0)
    {
        try
        {
            if ( cdbg.GetFlag() )
                cdbg << "cSDHSerial::BinaryAxisCommand: sending '" << request << "' to SDH\n";

            com->write( (char*)(&request), request.GetNbBytesToSend() );

            int nb_bytes_received = com->Read( &response, nb_bytes_to_receive, long( com->GetTimeout() * 1E6 ), false );

            if ( nb_bytes_received != nb_bytes_to_receive )
                throw new cSDHErrorCommunication( cMsg( "Received only %d/%d binary bytes", nb_bytes_received, nb_bytes_to_receive ) );

            if ( cdbg.GetFlag() )
                cdbg << "cSDHSerial::BinaryAxisCommand: received '" << response << "' from SDH\n";

            if ( use_crc16 )
                response.CheckCRC16();

            firmware_state = eErrorCode(response.status_code);
            if ( response.status_code != RC_OK )
                throw new cSDHErrorCommunication( cMsg( "Received error code 0x%02x (%s) from SDH", response.status_code, SDHReturnCodeToString( eReturnCode( response.status_code ) ) ) );

            if ( axis == All )
                return cSimpleVector( NUMBER_OF_AXES, 0, response.parameter );
            else
                return cSimpleVector( 1, axis, &(response.parameter[ axis ]) );

        } // end of try
        catch (cSDHLibraryException* e)
        {
            // these errors seem to happen on linux only (not cygwin) where a reply can be partly received:

            // assume some communication error occured, so retry:
            retries -= 1;
            if (retries > 0)
                cdbg << __FILE__ << ":" << __LINE__ << " ignoring cSDHLibraryException: " << *e << " (retrying)\n";
            else
            {
                cdbg << __FILE__ << ":" << __LINE__ << " retried but giving up now\n";
                // rethrow e:
                throw;
            }

            // resync first:
            BinarySync();
            //now start over again

            delete e;

        } // end of catch

    } // end of while (retries > 0)

    cdbg << "Retried sending, but still didnt work!\n";
    throw new cSDHLibraryException( "cSDHLibraryException", cMsg( "Unknown error while retrying" ) );
}
//----------------------------------------------------------------------

cSimpleVector cSDHSerial::pid( int axis, double* p, double* i, double* d )
    throw (cSDHLibraryException*)
{
    CheckIndex( axis, NUMBER_OF_AXES, "axis" );

    if (p == NULL  &&  i == NULL  &&  d == NULL)
    {
        Send( cMsg( "pid(%d)", axis ).c_str() );
        return cSimpleVector( 3, reply[0] + 7 );
    }
    if (p != NULL  &&  i != NULL  &&  d != NULL)
    {
        Send( cMsg( "pid(%d)=%f,%f,%f", axis, *p, *i, *d ).c_str() );
        return cSimpleVector( 3, reply[0]+7 );
    }

    throw new cSDHErrorInvalidParameter( cMsg( "Invalid parameter in call' pid(axis=%d, p=%f, i=%f, d=%f )'", axis, *p, *i, *d ) );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::kv( int axis, double* kv )
    throw (cSDHLibraryException*)
{
    if (axis == All)
    {
        // SDH firmware cannot handle setting / getting all values at once
        // so emulate that

        // get/set all kv:

        cSimpleVector rv;

        for ( int i=0; i < NUMBER_OF_AXES; i++ )
        {
        cSimpleVector rvi;

        if (kv == NULL)
            rvi = AxisCommand( "kv", i, NULL );
        else
            rvi = AxisCommand( "kv", i, &(kv[i]) );
        rv[i] = rvi[i];
        }
        return rv;
    }
    else
    {
        return AxisCommand( "kv", axis, kv );
    }
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::ilim( int axis, double* limit )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_ILIM, axis, limit );
#else
    return AxisCommand( "ilim", axis, limit );
#endif
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::power( int axis, double* flag )
    throw (cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // power=0,0,0,0,0,0,0
    // POWER=0,0,0,0,0,0,0

#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_POWER, axis, flag );
#else
    return AxisCommand( "power", axis, flag );
#endif
}
//----------------------------------------------------------------------


void cSDHSerial::demo( bool onoff )
{
    Send( cMsg( "demo=%d", int( onoff ) ).c_str() );
}
//-----------------------------------------------------------------


int cSDHSerial::property( char const* propname, int value )
{
    Send( cMsg( "%s=%d", propname, value ).c_str() );
    int v;
    sscanf( reply[0] + strlen(propname), "%d", &v );
    return v;
}
//-----------------------------------------------------------------


int cSDHSerial::user_errors( int value )
{
    return property( "user_errors", value );
}
//-----------------------------------------------------------------


int cSDHSerial::terminal( int value )
{
    return property( "terminal", value );
}
//-----------------------------------------------------------------


int cSDHSerial::debug( int value )
{
    return property( "debug", value );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::v( int axis, double* velocity )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_V, axis, velocity );
#else
    return AxisCommand( "v", axis, velocity );
#endif
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::tvav( int axis, double* velocity )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_TVAV, axis, velocity );
#else
    return AxisCommand( "tvav", axis, velocity );
#endif
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::vlim( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "vlim", axis, NULL );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::alim( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "alim", axis, NULL );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::a( int axis, double* acceleration )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_A, axis, acceleration );
#else
    return AxisCommand( "a", axis, acceleration );
#endif
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::p( int axis, double* angle )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_P, axis, angle );
#else
    return AxisCommand( "p", axis, angle );
#endif
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::tpap( int axis, double* angle )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_TPAP, axis, angle );
#else
    return AxisCommand( "tpap", axis, angle );
#endif
}
//-----------------------------------------------------------------


double cSDHSerial::m( bool sequ )
    throw(cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // m
    // M=4.51
    //
    // Before Firmware 0.0.3.1 input/output for the command looked like:
    // --
    // m
    // @Enabling all axis
    // @max distance=45.06, T=4.51s, num_points: 451
    // m

    //---------------------
    // settings for sequential/non-sequential:
    int nb_lines_total = 1;
    int nb_lines = nb_lines_total;
    //---------------------

    //---------------------
    // send command and parse reply
    Send( "m", nb_lines, nb_lines_total );


    double T =  GetDuration( reply[0] );
    //---------------------

    // the SDH firmware does NOT produce an output after the command has finished
    if (sequ)
        // so just wait as long as the movement will take
        SleepSec( T + m_sequtime );

    return T;
}
//-----------------------------------------------------------------


void cSDHSerial::stop( void )
    throw (cSDHLibraryException*)
{
    Send( "stop" );
}
//----------------------------------------------------------------------


cSDHBase::eVelocityProfile cSDHSerial::vp( eVelocityProfile velocity_profile )
    throw (cSDHLibraryException*)
{
    char cmd[5];
    if ( velocity_profile < 0 )
        sprintf( cmd, "vp" );
    else if ( velocity_profile < eVP_DIMENSION )
        sprintf( cmd, "vp=%d", velocity_profile );
    else
        throw new cSDHErrorInvalidParameter( cMsg( "Invalid parameter in vp( velocity_profile=%d )'", velocity_profile ) );

    Send( cmd );

    int new_vp;
    sscanf( reply[0]+3, "%d", &new_vp );
    return (eVelocityProfile) new_vp;
}
//----------------------------------------------------------------------


cSDHBase::eControllerType cSDHSerial::con( eControllerType controller )
    throw (cSDHLibraryException*)
{
    char cmd[6];
    if ( controller == eCT_INVALID )
        sprintf( cmd, "con" );
    else if ( controller < eCT_DIMENSION )
        sprintf( cmd, "con=%d", controller );
    else
        throw new cSDHErrorInvalidParameter( cMsg( "Invalid parameter in con( controller=%d )'", controller ) );

    Send( cmd );

    int new_con;
    sscanf( reply[0]+4, "%d", &new_con );
    return (eControllerType) new_con;
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::pos( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_POS, axis );
#else
    return AxisCommand( "pos", axis );
#endif
}
//----------------------------------------------------------------------

cSimpleVector cSDHSerial::pos_save( int axis, double* value )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "pos_save", axis, value );
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::ref( int axis, double* value )
    throw (cSDHLibraryException*)
{
    // referncing is done with 25 deg/s thus it may take up to 180/25 = 7.2s plus safety time = 10s
    cSerialBase::cSetTimeoutTemporarily set_timeout_temporarily( com, 10.0 );
    // previous timeout will be automatically restored when function is left (by return or exception)

    return AxisCommand( "ref", axis, value );
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::vel( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_VEL, axis );
#else
    return AxisCommand( "vel", axis );
#endif
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::rvel( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_RVEL, axis );
#else
    return AxisCommand( "rvel", axis );
#endif
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::state( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
#if SDH_USE_BINARY_COMMUNICATION
    return BinaryAxisCommand( CMDC_STATE, axis );
#else
    return AxisCommand( "state", axis );
#endif
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::temp( void )
    throw(cSDHLibraryException*)
{
    cSimpleVector rv;

    Send( "temp" );

    sscanf( reply[0] + 5, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &(rv[0]), &(rv[1]), &(rv[2]), &(rv[3]), &(rv[4]), &(rv[5]), &(rv[6]) );
    return rv;
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::temp_electronics( void )
    throw(cSDHLibraryException*)
{
    cSimpleVector dummy;
    cSimpleVector rv;

    Send( "temp" );

    sscanf( reply[0] + 5, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &(dummy[0]), &(dummy[1]), &(dummy[2]), &(dummy[3]), &(dummy[4]), &(dummy[5]), &(dummy[6]),
            &(rv[0]), &(rv[1]) );
    return rv;
}
//-----------------------------------------------------------------


char* cSDHSerial::ver( void  )
    throw(cSDHLibraryException*)
{
    Send( "ver" );
    return reply[0] + 4;
}
//-----------------------------------------------------------------


char* cSDHSerial::ver_date( void  )
    throw(cSDHLibraryException*)
{
    Send( "ver_date" );
    return reply[0] + 9;
}
//-----------------------------------------------------------------


char* cSDHSerial::id( void )
    throw(cSDHLibraryException*)
{
    Send( "id" );
    return reply[0] + 3;
}
//-----------------------------------------------------------------


char* cSDHSerial::sn( void )
    throw(cSDHLibraryException*)
{
    Send( "sn" );
    return reply[0] + 3;
}
//-----------------------------------------------------------------


char* cSDHSerial::soc( void )
    throw(cSDHLibraryException*)
{
    Send( "soc" );
    return reply[0] + 4;
}
//-----------------------------------------------------------------


char* cSDHSerial::soc_date( void )
    throw(cSDHLibraryException*)
{
    Send( "soc_date" );
    return reply[0] + 9;
}
//-----------------------------------------------------------------


int cSDHSerial::numaxis( void )
    throw(cSDHLibraryException*)
{
    Send( "numaxis" );
    ////!!!return int( reply[0]+8 ); // did this ever work???
    int numaxis;
    sscanf( reply[0]+8, "%d", &numaxis );
    return numaxis;
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::igrip( int axis, double* limit )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "igrip", axis, limit );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::ihold( int axis, double* limit )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "ihold", axis, limit );
}
//-----------------------------------------------------------------


double cSDHSerial::selgrip( eGraspId grip, bool sequ )
    throw (cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // selgrip=1
    // SELGRIP=0.0,1
    //
    // Before firmware 0.0.3.1 actual input/output for the command looked like:
    //--
    // selgrip=1
    //   NO LONGER SENT! @Krit prox angle -0.00
    //   NO LONGER SENT! @Joint finger 1 proximal (-34.72 deg) is not critical.
    //   NO LONGER SENT! @Joint finger 2 proximal (-34.54 deg) is not critical.
    //   NO LONGER SENT! @Joint finger 3 proximal (-35.15 deg) is not critical.
    // @Enabling all axis
    // @Setting current limit to @1.0 @0.5 @0.5 @0.5 @0.5 @0.5 @0.5 @
    // @max distance=0.00, T=0.00s, num_points: 1
    // @max distance=0.00, T=0.00s, num_points: 1
    // @Setting current limit to @0.1 @0.2 @0.2 @0.2 @0.2 @0.2 @0.2 @
    // @Disabling axis 0
    // SELGRIP=1

    CheckIndex( grip, eGID_DIMENSION, "grip" );

    //---------------------
    // ensure sin square velocity profile is used:
    vp( eVP_SIN_SQUARE );
    //---------------------

    //---------------------
    // settings for sequential/non-sequential:
    int nb_lines_total = 1;
    int nb_lines = nb_lines_total;
    //---------------------

    //---------------------
    // send command and parse reply
    Send( cMsg( "selgrip=%d", grip).c_str(), nb_lines, nb_lines_total );

    double T;
    T = GetDuration( reply[0] );
    //---------------------

    return T;
}
//-----------------------------------------------------------------


double cSDHSerial::grip( double close, double velocity, bool sequ )
    throw (cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // grip=0.1,40
    // GRIP=0.42,0.1
    //
    // Before firmware 0.0.3.1 actual input/output for the command looked like:
    //--
    // grip=0.1,40
    //   no longer sent @Grip vector is: @0.0 @-66.8 @66.8 @-66.8 @66.8 @-66.8 @66.8 @
    // @Enabling finger axis
    // @Setting current limit to @1.0 @0.5 @0.5 @0.5 @0.5 @0.5 @0.5 @
    // @max distance=8.31, T=0.42s, num_points: 42
    // @Setting current limit to @0.1 @0.2 @0.2 @0.2 @0.2 @0.2 @0.2 @
    // GRIP=0.1

    CheckRange( close, 0.0, 1.0, "close ratio" );
    CheckRange( velocity, 0.0+eps, 100.0, "velocity" );

    //---------------------
    // ensure sin square velocity profile is used:
    vp( eVP_SIN_SQUARE );
    //---------------------

    //---------------------
    // settings for sequential/non-sequential:
    int nb_lines_total = 1;
    int nb_lines = nb_lines_total;
    //---------------------

    //---------------------
    // send command and parse reply
    char cmd[] = "grip=CCCCCCCCCCCCCCC,VVVVVVVVVVVVVVV";
    sprintf( cmd, "grip=%f,%f", close, velocity );
    Send( cmd, nb_lines, nb_lines_total );

    double T = GetDuration( reply[0] );
    //---------------------

    return T;
}
//----------------------------------------------------------------------


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
