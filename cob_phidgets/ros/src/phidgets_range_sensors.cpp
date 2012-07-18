// see http://www.phidgets.com/products.php?category=2&product_id=3520_0
// distance calculation: Distance (cm) = 2076/(SensorValue - 11)             (the formula is only valid for a SensorValue between 80 to 530)
// TODO: separate into two packages: cob_phidgets_driver (publishing only raw values as sensor_msgs/Range message) and cob_tray_status (evaluating the Range messages to decide if tray is occupied or not, eventually also at which position it is occupied) 


#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"

#include <libphidgets/phidget21.h>
#include <sstream>

class Sensor
{
	ros::NodeHandle n_;
	ros::Publisher pub_range_;
	int id_, filter_size_;
	std::list<int> vals_;
	std::string frame_id_;
public:

Sensor(const std::string &fr_id, const int id, const int filter_size=10):id_(id),filter_size_(filter_size),frame_id_(fr_id) {
	char buffer[256];
	sprintf(buffer,"range_%d",id);
	pub_range_ = n_.advertise<sensor_msgs::Range>(buffer, 0);
}

void publish()
{
	pub_range_.publish((sensor_msgs::Range)*this);
}

operator sensor_msgs::Range() const
{
	sensor_msgs::Range msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = frame_id_;
	msg.radiation_type = sensor_msgs::Range::INFRARED;
	msg.min_range = 0.04;
	msg.max_range = 0.3;
	msg.field_of_view = 0; //not given!

	int sum=0, num=0;
	for(std::list<int>::const_iterator it=vals_.begin(); it!=vals_.end(); it++)
	{
		sum+=*it;
		++num;
	}
	msg.range = 20.76/(sum/(double)num - 11.);

	return msg;
}

int getId() const {return id_;}

void update(const int v)
{
	if(vals_.size()<filter_size_) vals_.push_back(v);
	else {vals_.push_back(v); vals_.pop_front();}
}

};

int i;

bool bOccupied_;
std::vector<Sensor> g_sensors;

void display_generic_properties(CPhidgetHandle phid)
{
	int sernum, version;
	const char *deviceptr, *label;
	CPhidget_getDeviceType(phid, &deviceptr);
	CPhidget_getSerialNumber(phid, &sernum);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidget_getDeviceLabel(phid, &label);

	ROS_INFO("%s", deviceptr);
	ROS_INFO("Version: %8d SerialNumber: %10d", version, sernum);
	ROS_INFO("Label: %s", label);
	return;
}

int IFK_AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	//CPhidgetInterfaceKit_setSensorChangeTrigger((CPhidgetInterfaceKitHandle)IFK, 0, 0);
	//printf("Attach handler ran!\n");
	return 0;
}


int IFK_SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *userptr, int Index, int Value)
{

	for(size_t i=0; i<g_sensors.size(); i++)
		if(g_sensors[i].getId()==Index) g_sensors[i].update(Value);
	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_phidgets");

	ros::NodeHandle nh_("~");

			if (nh_.hasParam("sensors"))
			{
	XmlRpc::XmlRpcValue v;
    nh_.param("sensors", v, v);
    for(int i =0; i < v.size(); i++)
    {
	ROS_ASSERT(v[i].size()>=2);

	int id = v[i][0];
	std::string fr_id = v[i][1];
	int filter = v.size()>2?(int)v[i][2]:10;

	g_sensors.push_back(Sensor(fr_id,id,filter));
    }
			}
			else
			{
				ROS_ERROR("Parameter sensors not set, shutting down node...");
				nh_.shutdown();
				return false;
			}


	ros::Rate loop_rate(10);

	//init and open phidget
	int numInputs, numOutputs, numSensors;
	int err;

	CPhidgetInterfaceKitHandle IFK = 0;
	CPhidget_enableLogging(PHIDGET_LOG_VERBOSE, NULL);
	CPhidgetInterfaceKit_create(&IFK);
	CPhidgetInterfaceKit_set_OnSensorChange_Handler(IFK, IFK_SensorChangeHandler, NULL);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)IFK, IFK_AttachHandler, NULL);
	//opening phidget
	CPhidget_open((CPhidgetHandle)IFK, -1);

	//wait 5 seconds for attachment
	ROS_INFO("waiting for phidgets attachement...");
	if((err = CPhidget_waitForAttachment((CPhidgetHandle)IFK, 10000)) != EPHIDGET_OK )
	{
		const char *errStr;
		CPhidget_getErrorDescription(err, &errStr);
		ROS_ERROR("Error waiting for attachment: (%d): %s",err,errStr);
		goto exit;
	}
	ROS_INFO("... attached");

	display_generic_properties((CPhidgetHandle)IFK);
	CPhidgetInterfaceKit_getOutputCount((CPhidgetInterfaceKitHandle)IFK, &numOutputs);
	CPhidgetInterfaceKit_getInputCount((CPhidgetInterfaceKitHandle)IFK, &numInputs);
	CPhidgetInterfaceKit_getSensorCount((CPhidgetInterfaceKitHandle)IFK, &numSensors);
	//CPhidgetInterfaceKit_setOutputState((CPhidgetInterfaceKitHandle)IFK, 0, 1);

	ROS_INFO("Sensors:%d Inputs:%d Outputs:%d", numSensors, numInputs, numOutputs);

	while (ros::ok())
	{
		for(size_t i=0; i<g_sensors.size(); i++) g_sensors[i].publish();
		ros::spinOnce();
		loop_rate.sleep();
	}

exit:
	CPhidget_close((CPhidgetHandle)IFK);
	CPhidget_delete((CPhidgetHandle)IFK);


  return 0;
}
