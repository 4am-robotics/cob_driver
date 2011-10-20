#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "cob_tray_sensors/CheckOccupied.h"

#include <libphidgets/phidget21.h>
#include <sstream>


int wert[100];
int i;

bool bOccupied_;

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

	//printf("Sensor %d is %d\n", Index, Value);
	if (i <10)
	{
		CPhidgetInterfaceKit_getSensorValue(IFK,0, &wert[0+(10*i)]);
		//printf("Sensor0 manually is %d\n", wert[0]);
		CPhidgetInterfaceKit_getSensorValue(IFK,1, &wert[1+(10*i)]);
		//printf("Sensor0 manually is %d\n", wert[0]);
		CPhidgetInterfaceKit_getSensorValue(IFK,2, &wert[2+(10*i)]);
		//printf("Sensor0 manually is %d\n", wert[0]);
		CPhidgetInterfaceKit_getSensorValue(IFK,3, &wert[3+(10*i)]);
		//printf("Sensor0 manually is %d\n", wert[0]);
	}

	else 
	{

		//CPhidgetInterfaceKit_getSensorValue(IFK,1, &wert[1]);
		//printf("Sensor1 manually is %d\n", wert[1]);
		//CPhidgetInterfaceKit_getSensorValue(IFK,2, &wert[2]);
		//printf("Sensor2 manually is %d\n", wert[2]);
		//CPhidgetInterfaceKit_getSensorValue(IFK,3, &wert[3]);
		//printf("Sensor0123 manually are %d %d %d %d ; \n", wert[0],wert[1],wert[2],wert[3]);
		wert[94] = ((wert[0] + wert[10]+wert[20]+wert[30]+wert[40]+wert[50]+wert[60]+wert[70]+wert[80]+wert[90]) / 10);
		wert[95] = ((wert[1] + wert[11]+wert[21]+wert[31]+wert[41]+wert[51]+wert[61]+wert[71]+wert[81]+wert[91]) / 10);
		wert[96] = ((wert[2] + wert[12]+wert[22]+wert[32]+wert[42]+wert[52]+wert[62]+wert[72]+wert[82]+wert[92]) / 10);
		wert[97] = ((wert[3] + wert[13]+wert[23]+wert[33]+wert[43]+wert[53]+wert[63]+wert[73]+wert[83]+wert[93]) / 10);

		if ((wert[94] < 200) && (wert[95] <200) && (wert[96] < 200) && (wert[97] < 200))  
		{
			ROS_DEBUG("Sensor0123 manually are %d %d %d %d ; ", wert[94],wert[95],wert[96],wert[97]);
			ROS_DEBUG("tablet empty");
			bOccupied_ = false;
		}
		else 	
		{
			ROS_DEBUG("Sensor0123 manually are %d %d %d %d ;", wert[94],wert[95],wert[96],wert[97]);
			ROS_DEBUG("tablet NOT empty!");
			bOccupied_ = true;
		}

		for( i = 0; i < 100; i++)
			wert[i]=0;
		i =0;
	}
	i++;
	//printf("i is %d \n",i);
	return 0;
}


bool check_occupied(cob_tray_sensors::CheckOccupied::Request  &req,
         cob_tray_sensors::CheckOccupied::Response &res )
{
  res.occupied.data = bOccupied_;
  return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cob_tray_sensors");
	ros::NodeHandle n;
	ros::Publisher occupied_pub = n.advertise<std_msgs::Bool>("/tray/occupied", 0);
	ros::ServiceServer service = n.advertiseService("/tray/check_occupied", check_occupied);

	ros::Rate loop_rate(10);

	bOccupied_ = false;

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
	if((err = CPhidget_waitForAttachment((CPhidgetHandle)IFK, 0)) != EPHIDGET_OK )
	{
		const char *errStr;
		CPhidget_getErrorDescription(err, &errStr);
		ROS_ERROR("Error waiting for attachment: (%d): %s",err,errStr);
		goto exit;
	}

	display_generic_properties((CPhidgetHandle)IFK);
	CPhidgetInterfaceKit_getOutputCount((CPhidgetInterfaceKitHandle)IFK, &numOutputs);
	CPhidgetInterfaceKit_getInputCount((CPhidgetInterfaceKitHandle)IFK, &numInputs);
	CPhidgetInterfaceKit_getSensorCount((CPhidgetInterfaceKitHandle)IFK, &numSensors);
	CPhidgetInterfaceKit_setOutputState((CPhidgetInterfaceKitHandle)IFK, 0, 1);

	ROS_INFO("Sensors:%d Inputs:%d Outputs:%d", numSensors, numInputs, numOutputs);

	while (ros::ok())
	{
		std_msgs::Bool occupied;
		occupied.data = bOccupied_;
		occupied_pub.publish(occupied);
		ros::spinOnce();
		loop_rate.sleep();
	}

exit:
	CPhidget_close((CPhidgetHandle)IFK);
	CPhidget_delete((CPhidgetHandle)IFK);


  return 0;
}
