// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <cob_msgs/PowerBoardState.h>
#include <cob_msgs/PowerState.h>
#include <cob_msgs/EmergencyStopState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <cob_voltage_control_common.cpp>
#include <cob_phidgets/AnalogSensor.h>
#include <cob_phidgets/DigitalSensor.h>


class cob_voltage_control_ros
{

    private:
        int EM_stop_status_;

    public:
        ros::NodeHandle n_;

        ros::Publisher topicPub_em_stop_state_;
        ros::Publisher topicPub_power_board_state;
        ros::Publisher topicPub_Voltage;
	ros::Publisher topicPub_PowerState;
	
        ros::Subscriber topicSub_AnalogInputs;
        ros::Subscriber topicSub_DigitalInputs;

        cob_voltage_control_data component_data_;
        cob_voltage_control_config component_config_;
        cob_voltage_control_impl component_implementation_;

        //
        bool last_rear_em_state;
        bool last_front_em_state;

        // possible states of emergency stop
        enum
        {
          ST_EM_FREE = 0,
          ST_EM_ACTIVE = 1,
          ST_EM_CONFIRMED = 2
        };

        cob_voltage_control_ros()
        {
            topicPub_power_board_state = n_.advertise<cob_msgs::PowerBoardState>("pub_relayboard_state_", 1);    
            topicPub_em_stop_state_ = n_.advertise<cob_msgs::EmergencyStopState>("pub_em_stop_state_", 1);
            topicPub_Voltage = n_.advertise<std_msgs::Float64>("/power_board/voltage", 10);
// 	    topicPub_PowerState = n_.advertise<cob_msgs::PowerState>("/power_state", 1);
	    
            topicSub_AnalogInputs = n_.subscribe("/analog_sensors", 10, &cob_voltage_control_ros::analogPhidgetSignalsCallback, this);
            topicSub_DigitalInputs = n_.subscribe("/digital_sensors", 10, &cob_voltage_control_ros::digitalPhidgetSignalsCallback, this);
                
            n_.param("battery_max_voltage", component_config_.max_voltage, 50.0);
            n_.param("battery_min_voltage", component_config_.min_voltage, 44.0);
            n_.param("robot_max_voltage", component_config_.max_voltage_res, 60.0);
	    n_.param("phidget_max_counts", component_config_.max_counts, 1024); 
        
            last_rear_em_state = false;
            last_front_em_state = false;

            EM_stop_status_ = ST_EM_ACTIVE;
            component_data_.out_pub_em_stop_state_.scanner_stop = false;
        }

        void configure()
        {
            component_implementation_.configure();
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
            topicPub_Voltage.publish(component_data_.out_pub_voltage);
// 	    topicPub_PowerState.publish(component_data_.out_pub_powerstate_);
            topicPub_power_board_state.publish(component_data_.out_pub_power_board_state);
            topicPub_em_stop_state_.publish(component_data_.out_pub_em_stop_state_);
        }
        
        void analogPhidgetSignalsCallback(const cob_phidgets::AnalogSensorConstPtr &msg)
        {
	    int voltageState = -1;
	    
            for(int i = 0; i < msg->uri.size(); i++)
            {
                if( msg->uri[i] == "bat1")
                {
                    voltageState = msg->value[i];
                    break;
                }
            }
            // Calculatio of real voltage
            double voltage = voltageState * component_config_.max_voltage_res/component_config_.max_counts;
            component_data_.out_pub_voltage.data = voltage;
	    
	    // Linear calculatio for percentag and PowerState,
	    double percentage =  (voltage - component_config_.min_voltage) * 100/(component_config_.max_voltage - component_config_.min_voltage);
	    
	    component_data_.out_pub_powerstate_.header.stamp = ros::Time::now();
	    component_data_.out_pub_powerstate_.power_consumption = 0.0;
	    component_data_.out_pub_powerstate_.time_remaining = ros::Duration(1000);
	    component_data_.out_pub_powerstate_.relative_capacity = percentage;
        }

        void digitalPhidgetSignalsCallback(const cob_phidgets::DigitalSensorConstPtr &msg)
        {
            bool em_scanner_active = false;
            bool em_butt_active = false;
            cob_msgs::EmergencyStopState EM_msg;
            bool EM_signal = false;
            bool got_message = false;

            for(int i = 0; i < msg->uri.size(); i++)
            {
                if( msg->uri[i] == "em_butt_not_stop")
                {
                    em_butt_active = !((bool)msg->state[i]);
                    got_message = true;
                }
                else if( msg->uri[i] == "em_scanner_not_stop")
                {
                    em_scanner_active = !((bool)msg->state[i]);
                    got_message = true;
                }
            }
            if(got_message)
            {
		component_data_.out_pub_em_stop_state_.emergency_button_stop = em_butt_active;
		component_data_.out_pub_em_stop_state_.scanner_stop = em_scanner_active;
		
		EM_signal = (bool)(em_butt_active | em_scanner_active);
		
		switch (EM_stop_status_)
		{
			case ST_EM_FREE:
			{
				if (EM_signal == true)
				{
					ROS_INFO("Emergency stop was issued");
					EM_stop_status_ = EM_msg.EMSTOP;
				}
				break;
			}
			case ST_EM_ACTIVE:
			{
				if (EM_signal == false)
				{
					ROS_INFO("Emergency stop was confirmed");
					EM_stop_status_ = EM_msg.EMCONFIRMED;
				}
				break;
			}
			case ST_EM_CONFIRMED:
			{
				if (EM_signal == true)
				{
					ROS_INFO("Emergency stop was issued");
					EM_stop_status_ = EM_msg.EMSTOP;
				}
				else
				{
					ROS_INFO("Emergency stop released");
					EM_stop_status_ = EM_msg.EMFREE;
				}
				break;
			}
		};

		component_data_.out_pub_em_stop_state_.emergency_state = EM_stop_status_;
	    }
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
