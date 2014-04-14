// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <pr2_msgs/PowerBoardState.h>
#include <pr2_msgs/PowerState.h>
#include <cob_relayboard/EmergencyStopState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <cob_voltage_control_common.cpp>

//#include <libphidgets/phidget21.h>



class cob_voltage_control_ros
{
	public:
		ros::NodeHandle n_;

		ros::Publisher pub_em_stop_state__;
		ros::Publisher pub_powerstate__;
		ros::Publisher pub_relayboard_state__;
		ros::Publisher topicPub_Voltage;
		
		CPhidgetInterfaceKitHandle IFK;

        
 
        cob_voltage_control_data component_data_;
	cob_voltage_control_config component_config_;
        cob_voltage_control_impl component_implementation_;

        cob_voltage_control_ros()
        {

        	//pub_em_stop_state__ = n_.advertise<pr2_msgs::PowerBoardState>("pub_em_stop_state_", 1);
        	//pub_powerstate__ = n_.advertise<pr2_msgs::PowerState>("pub_powerstate_", 1);
	        //pub_relayboard_state__ = n_.advertise<cob_relayboard::EmergencyStopState>("pub_relayboard_state_", 1);
		topicPub_Voltage = n_.advertise<std_msgs::Float64>("/power_board/voltage", 10);
			
		n_.param("battery_max_voltage", component_config_.max_voltage, 50.0);
		n_.param("battery_min_voltage", component_config_.min_voltage, 44.0);
		n_.param("robot_max_voltage", component_config_.max_voltage_res, 61.0);
		n_.param("voltage_analog_port", component_config_.num_voltage_port, 1);
		n_.param("em_stop_dio_port", component_config_.num_em_stop_port, 0); 
		n_.param("scanner_stop_dio_port", component_config_.num_scanner_em_port, 1);            
		

        }

        void configure()
        {
            component_implementation_.configure();
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
            topicPub_Voltage.publish(component_data_.out_pub_voltage);   
        }
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "cob_voltage_control");

	cob_voltage_control_ros node;
    ROS_INFO("blub");
    node.configure();

 	ros::Rate loop_rate(100); // Hz // if cycle time == 0 do a spin() here without calling node.update() 

	while(node.n_.ok())
	{
        node.update();
		loop_rate.sleep();
		ros::spinOnce();
	}
    return 0;
}
