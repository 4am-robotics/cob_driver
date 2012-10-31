// InterfacekitTest.cpp : Defines the entry point for the console application.
//


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <phidget21.h>
int wert[100];
int i;

void display_generic_properties(CPhidgetHandle phid)
{
	int sernum, version;
	const char *deviceptr, *label;
	CPhidget_getDeviceType(phid, &deviceptr);
	CPhidget_getSerialNumber(phid, &sernum);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidget_getDeviceLabel(phid, &label);

	printf("%s\n", deviceptr);
	printf("Version: %8d SerialNumber: %10d\n", version, sernum);
	printf("Label: %s\n", label);
	return;
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

		if ((wert[94] < 120) && (wert[95] <300) && (wert[96] < 110) && (wert[97] < 120))  
		{
			printf("Sensor0123 manually are %d %d %d %d ; \n", wert[94],wert[95],wert[96],wert[97]);
			printf("tablet empty\n");
		}
		else 	
		{
			printf("Sensor0123 manually are %d %d %d %d ; \n", wert[94],wert[95],wert[96],wert[97]);
			printf("tablet NOT empty!\n");
		}

		for( i = 0; i < 100; i++)
			wert[i]=0;
		i =0;
	}
	i++;
	printf("i is %d \n",i);
	return 0;
}


int IFK_AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	//CPhidgetInterfaceKit_setSensorChangeTrigger((CPhidgetInterfaceKitHandle)IFK, 0, 0);
	printf("Attach handler ran!\n");
	return 0;
}

int test_interfacekit()
{
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
		printf("Error waiting for attachment: (%d): %s\n",err,errStr);
		goto exit;
	}

	display_generic_properties((CPhidgetHandle)IFK);
	CPhidgetInterfaceKit_getOutputCount((CPhidgetInterfaceKitHandle)IFK, &numOutputs);
	CPhidgetInterfaceKit_getInputCount((CPhidgetInterfaceKitHandle)IFK, &numInputs);
	CPhidgetInterfaceKit_getSensorCount((CPhidgetInterfaceKitHandle)IFK, &numSensors);
	CPhidgetInterfaceKit_setOutputState((CPhidgetInterfaceKitHandle)IFK, 0, 1);

	printf("Sensors:%d Inputs:%d Outputs:%d\n", numSensors, numInputs, numOutputs);

	
	//err = CPhidget_setDeviceLabel((CPhidgetHandle)IFK, "test");

	
	while(1)
	{
		sleep(1);
	}
	
	while(1)
	{
		CPhidgetInterfaceKit_setOutputState(IFK, 7, 1);
		CPhidgetInterfaceKit_setOutputState(IFK, 7, 0);
	}



	CPhidgetInterfaceKit_setOutputState(IFK, 0, 1);
	sleep(1);
	CPhidgetInterfaceKit_setOutputState(IFK, 0, 0);
	sleep(1);
	CPhidgetInterfaceKit_setOutputState(IFK, 0, 1);
	sleep(1);
	CPhidgetInterfaceKit_setOutputState(IFK, 0, 0);

	sleep(5);

exit:
	CPhidget_close((CPhidgetHandle)IFK);
	CPhidget_delete((CPhidgetHandle)IFK);

	return 0;
}

int main(int argc, char* argv[])
{
	test_interfacekit();
	return 0;
}

