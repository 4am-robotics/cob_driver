// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cob_safety_controller/safety_controllerConfig.h>

// ROS message includes




#include <safety_controller_common.cpp>


class safety_controller_ros
{
	public:
		ros::NodeHandle n_;
		
		dynamic_reconfigure::Server<cob_safety_controller::safety_controllerConfig> server;
  		dynamic_reconfigure::Server<cob_safety_controller::safety_controllerConfig>::CallbackType f;
		

		

        
 
        safety_controller_data component_data_;
        safety_controller_config component_config_;
        safety_controller_impl component_implementation_;

        safety_controller_ros()
        {
       	
  			f = boost::bind(&safety_controller_ros::configure_callback, this, _1, _2);
  			server.setCallback(f);
        	
        
  	

            
        }
        
		
		void configure_callback(cob_safety_controller::safety_controllerConfig &config, uint32_t level) 
		{
		}

        void configure()
        {
			component_implementation_.configure(component_config_);
        }


	    void activate_all_output()
        {
        }

        void update()
        {
            activate_all_output();
            component_implementation_.update(component_data_, component_config_);
    
        }
 
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "safety_controller");

	safety_controller_ros node;
    node.configure();

	
 	ros::Rate loop_rate(10.0); // Hz

	while(node.n_.ok())
	{
        node.update();
		loop_rate.sleep();
		ros::spinOnce();
	}
	
    return 0;
}
