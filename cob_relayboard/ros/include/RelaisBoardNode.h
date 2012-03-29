// ROS includes
#include <ros/ros.h>
#include <iostream>
#include <neo_SerRelayBoard/SerRelayBoard.h>

// ROS message includes
#include <pr2_msgs/PowerState.h>
#include <pr2_msgs/PowerBoardState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <cob_relayboard/EmergencyStopState.h>
#include <cob_relayboard/Temperatur.h>
#include <cob_relayboard/DriveStates.h>
#include <cob_relayboard/DriveCommands.h>
#include <cob_relayboard/Keypad.h>
#include <cob_relayboard/LCDOutput.h>
#include <cob_relayboard/IRSensors.h>
#include <cob_relayboard/GyroBoard.h>
#include <cob_relayboard/RadarBoard.h>
#include <cob_relayboard/USBoard.h>
#include <cob_relayboard/IOOut.h>
#include <cob_relayboard/IOAnalogIn.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### node class ####
class RelaisBoardNode
{
	//
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle n;
                
		// topics:
		//basic topics:
		ros::Publisher topicPub_isEmergencyStop;
		ros::Publisher topicPub_batVoltage;
		ros::Publisher topicPub_temperatur;
		ros::Publisher topicPub_keypad;
		ros::Publisher topicPub_IRSensor;
		ros::Publisher topicPub_boardState;
		//optional topics:
		enum Modules {
			DRIVE1=0,
			DRIVE2=1,
			US_BOARD=2,
			RADAR_BOARD=3,
			IO_BOARD=4,
			GYRO_BOARD=5
		};
		//drives:
		ros::Publisher topicPub_drives;			//both motors are connected: motor callback
		ros::Subscriber topicSub_drives;		//set velocity of both drives
		//ultrasonic board:
		ros::Publisher topicPub_usBoard;
		ros::Subscriber topicSub_startUSBoard;
		ros::Subscriber topicSub_stopUSBoard;		
		//radar board:
		ros::Publisher topicPub_radarBoard;
		//io board:
		ros::Publisher topicPub_ioDigIn;
		ros::Publisher topicPub_ioDigOut;
		ros::Publisher topicPub_analogIn;
		ros::Subscriber topicSub_lcdDisplay;
		ros::Subscriber topicSub_setDigOut;
		//gyro board:
		ros::Publisher topicPub_gyroBoard;
		ros::Subscriber topicSub_zeroGyro;

		// Constructor
		RelaisBoardNode()
		{
			//topics which allways get published
			topicPub_isEmergencyStop = n.advertise<cob_relayboard::EmergencyStopState>("/emergency_stop_state", 1);
			topicPub_batVoltage = n.advertise<pr2_msgs::PowerState>("/power_state", 1);
			topicPub_temperatur = n.advertise<cob_relayboard::Temperatur>("/temperature", 1);
			topicPub_boardState = n.advertise<pr2_msgs::PowerBoardState>("/power_board/state",1);
			// Make sure member variables have a defined state at the beginning
			EM_stop_status_ = ST_EM_FREE;
			relayboard_available = false;
			relayboard_online = false;
			relayboard_timeout_ = 2.0;
			protocol_version_ = 1;
			duration_for_EM_free_ = ros::Duration(1);
		}
        
		// Destructor
		~RelaisBoardNode() 
		{
			delete m_SerRelayBoard;
		}
    		//basic:
		void sendEmergencyStopStates();
		void sendAnalogIn();
		//IOBoard
		void getNewLCDOutput(const cob_relayboard::LCDOutput&); //output on a 20 char lcd display
		void sendIOBoardDigIn();
		void sendIOBoardDigOut();
		void getIOBoardDigOut(const cob_relayboard::IOOut&);
		void sendIOBoardAnalogIn();
		//motor:
		void sendDriveStates();
		void getNewDriveStates(const cob_relayboard::DriveCommands& driveCommands);
		//gyroBoard:
		void sendGyroBoard();
		void zeroGyro(const std_msgs::Bool& b);
		//radarBoard:
		void sendRadarBoard();
		//usBoard:
		void sendUSBoard();
		void startUSBoard(const std_msgs::Int16& configuration);
		void stopUSBoard(const std_msgs::Empty& empty);
		
		int init();
		int requestBoardStatus();
		double getRequestRate();
		void readConfig(int protocol_version_);

	private:
		int activeModule[6]; //are the modules available (else ther won't be any datastreaming);
		std::string sComPort;
		SerRelayBoard * m_SerRelayBoard;

		int EM_stop_status_;
		ros::Duration duration_for_EM_free_;
		ros::Time time_of_EM_confirmed_;
		double relayboard_timeout_;
		int protocol_version_;
		double requestRate;

		ros::Time time_last_message_received_;
		bool relayboard_online; //the relayboard is sending messages at regular time
		bool relayboard_available; //the relayboard has sent at least one message -> publish topic

		// possible states of emergency stop
		enum
		{
			ST_EM_FREE = 0,
			ST_EM_ACTIVE = 1,
			ST_EM_CONFIRMED = 2
		};
		int motorCanIdent[2];
		std::string joint_names[2];
		int hasKeyPad, hasIRSensors, hasLCDOut;
		double voltage_min_, voltage_max_, charge_nominal_, voltage_nominal_, current_voltage;
};

