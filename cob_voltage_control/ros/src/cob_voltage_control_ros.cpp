// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <pr2_msgs/PowerBoardState.h>
#include <pr2_msgs/PowerState.h>
#include <cob_relayboard/EmergencyStopState.h>
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
        ros::Publisher topicPub_powerstate;

        ros::Publisher topicPub_Voltage;
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
            topicPub_powerstate = n_.advertise<pr2_msgs::PowerBoardState>("pub_em_stop_state_", 1);
            topicPub_em_stop_state_ = n_.advertise<cob_relayboard::EmergencyStopState>("pub_relayboard_state_", 1);

            topicPub_Voltage = n_.advertise<std_msgs::Float64>("/power_board/voltage", 10);
            topicSub_AnalogInputs = n_.subscribe("/analog_sensors", 10, &cob_voltage_control_ros::analogPhidgetSignalsCallback, this);
            topicSub_DigitalInputs = n_.subscribe("/digital_sensors", 10, &cob_voltage_control_ros::digitalPhidgetSignalsCallback, this);
                
            n_.param("battery_max_voltage", component_config_.max_voltage, 50.0);
            n_.param("battery_min_voltage", component_config_.min_voltage, 44.0);
            n_.param("robot_max_voltage", component_config_.max_voltage_res, 70.0);
            n_.param("voltage_analog_port", component_config_.num_voltage_port, 1);
            n_.param("em_stop_dio_port", component_config_.num_em_stop_port, 0); 
            n_.param("scanner_stop_dio_port", component_config_.num_scanner_em_port, 1);
        
            last_rear_em_state = false;
            last_front_em_state = false;

            EM_stop_status_ = ST_EM_ACTIVE;
            component_data_.out_pub_relayboard_state.scanner_stop = false;
        }

        void configure()
        {
            component_implementation_.configure();
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
            topicPub_Voltage.publish(component_data_.out_pub_voltage);
            topicPub_powerstate.publish(component_data_.out_pub_em_stop_state_);
            topicPub_em_stop_state_.publish(component_data_.out_pub_relayboard_state);
        }
        
        void analogPhidgetSignalsCallback(const cob_phidgets::AnalogSensorConstPtr &msg)
        {
            for(int i = 0; i < msg->uri.size(); i++)
            {
                if( msg->uri[i] == "bat1")
                {
                    component_data_.in_phidget_voltage = msg->value[i];
                    break;
                }
            }
        }

        void digitalPhidgetSignalsCallback(const cob_phidgets::DigitalSensorConstPtr &msg)
        {
            bool front_em_active = false;
            bool rear_em_active = false;
            static bool em_caused_by_button = false;
            cob_relayboard::EmergencyStopState EM_msg;
            bool EM_signal = false;
            bool got_message = false;

            for(int i = 0; i < msg->uri.size(); i++)
            {
                if( msg->uri[i] == "em_stop_laser_rear")
                {
                    rear_em_active = !((bool)msg->state[i]);
                    got_message = true;
                }
                else if( msg->uri[i] == "em_stop_laser_front")
                {
                    front_em_active = !((bool)msg->state[i]);
                    got_message = true;
                }
            }
            if(got_message)
            {
				if( (front_em_active && rear_em_active) && (!last_front_em_state && !last_rear_em_state))
				{
					component_data_.out_pub_relayboard_state.emergency_button_stop = true;
					em_caused_by_button = true;
				}
				else if((!front_em_active && !rear_em_active) && (last_front_em_state && last_rear_em_state))
				{
					component_data_.out_pub_relayboard_state.emergency_button_stop = false;
					em_caused_by_button = false;
				}
				else if((front_em_active != rear_em_active) && em_caused_by_button)
				{
					component_data_.out_pub_relayboard_state.emergency_button_stop = false;
					em_caused_by_button = false;
					component_data_.out_pub_relayboard_state.scanner_stop = (bool)(front_em_active | rear_em_active);
				}
				else
				{
					component_data_.out_pub_relayboard_state.scanner_stop = (bool)(front_em_active | rear_em_active);
					ROS_INFO_STREAM("scanner_stop: "<<component_data_.out_pub_relayboard_state.scanner_stop);
				}

				EM_signal = component_data_.out_pub_relayboard_state.scanner_stop | component_data_.out_pub_relayboard_state.emergency_button_stop;

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


				component_data_.out_pub_relayboard_state.emergency_state = EM_stop_status_;

				last_front_em_state = front_em_active;
				last_rear_em_state = rear_em_active;
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
