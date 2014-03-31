// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cob_safety_controller/safety_controllerConfig.h>

// ROS message includes
#include <visualization_msgs/MarkerArray.h>
#include <cob_relayboard/EmergencyStopState.h>
#include <nav_msgs/Odometry.h>
#include <cob_srvs/SetInt.h>

// other includes
#include <safety_controller_common.cpp>


class safety_controller_ros
{
    public:
    ros::NodeHandle n_;

    dynamic_reconfigure::Server<cob_safety_controller::safety_controllerConfig> server;
    dynamic_reconfigure::Server<cob_safety_controller::safety_controllerConfig>::CallbackType f;

    ros::Publisher marker_;
    ros::Publisher emergency_stop_state_;
    ros::Subscriber odometry_;
    ros::ServiceServer set_mode_;

    safety_controller_data component_data_;
    safety_controller_config component_config_;
    safety_controller_impl component_implementation_;

    safety_controller_ros()
    {
        f = boost::bind(&safety_controller_ros::configure_callback, this, _1, _2);
        server.setCallback(f);

        std::string set_mode_remap;
        n_.param("set_mode_remap", set_mode_remap, (std::string)"set_mode");
        set_mode_ = n_.advertiseService<cob_srvs::SetInt::Request , cob_srvs::SetInt::Response>(set_mode_remap, boost::bind(&safety_controller_impl::callback_set_mode, &component_implementation_,_1,_2,component_config_));

        marker_ = n_.advertise<visualization_msgs::MarkerArray>("marker", 1);
        emergency_stop_state_ = n_.advertise<cob_relayboard::EmergencyStopState>("emergency_stop_state", 1);
        odometry_ = n_.subscribe("odometry", 1, &safety_controller_ros::topicCallback_odometry, this);

        n_.param("port", component_config_.port, (std::string)"empty");
        n_.param("host", component_config_.host, (std::string)"empty");
        n_.param("threshold_linear_slow", component_config_.threshold_linear_slow, (double)0.0);
        n_.param("threshold_linear_fast", component_config_.threshold_linear_fast, (double)0.0);
        n_.param("threshold_angular_fast", component_config_.threshold_angular_fast, (double)0.0);
        }


    void topicCallback_odometry(const nav_msgs::Odometry::ConstPtr& msg)
    {
        component_data_.in_odometry = *msg;
    }

    void configure_callback(cob_safety_controller::safety_controllerConfig &config, uint32_t level)
    {
        component_config_.port = config.port;
        component_config_.host = config.host;
        component_config_.threshold_linear_slow = config.threshold_linear_slow;
        component_config_.threshold_linear_fast = config.threshold_linear_fast;
        component_config_.threshold_angular_fast = config.threshold_angular_fast;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_marker_active = true;
        component_data_.out_emergency_stop_state_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_marker_active)
            marker_.publish(component_data_.out_marker);
        if (component_data_.out_emergency_stop_state_active)
            emergency_stop_state_.publish(component_data_.out_emergency_stop_state);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "safety_controller");

    safety_controller_ros node;
    node.configure();

    ros::Rate loop_rate(1/10.0);

    while(node.n_.ok())
    {
        node.update();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
