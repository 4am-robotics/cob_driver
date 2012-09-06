// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <pr2_msgs/PowerBoardState.h>
#include <pr2_msgs/PowerState.h>

#include <cob_voltage_control_common.cpp>


class cob_voltage_control_ros
{
	public:
		ros::NodeHandle n_;

		ros::Publisher pub_em_stop_state__;
		ros::Publisher pub_powerstate__;
		

        
 
        cob_voltage_control_data component_data_;
        cob_voltage_control_impl component_implementation_;

        cob_voltage_control_ros()
        {

        	pub_em_stop_state__ = n_.advertise<pr2_msgs::PowerBoardState>("pub_em_stop_state_", 1);
        	pub_powerstate__ = n_.advertise<pr2_msgs::PowerState>("pub_powerstate_", 1);
			
            
        }

        void configure()
        {

        }

        void update()
        {
            component_implementation_.update(component_data_);
            pub_em_stop_state__.publish(component_data_.out_pub_em_stop_state_);
            pub_powerstate__.publish(component_data_.out_pub_powerstate_);
    
        }
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "cob_voltage_control");

	cob_voltage_control_ros node;
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
