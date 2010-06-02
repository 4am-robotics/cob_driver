#include <stdint.h>
typedef unsigned char uint8_t;
#include <inttypes.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cob_forcetorque/ForceTorqueCtrl.h>
#include <cob_forcetorque/UDPSocketASIO.h>

#include <math.h>
#include <iostream>
#define PI 3.14159265


int main(int argc, char ** argv)
{
	bool simu = true;
	ros::init(argc, argv, "talker");
  	ros::NodeHandle n;
  	ros::Publisher force_pub = n.advertise<std_msgs::Float32MultiArray>("force_values", 100);

	ForceTorqueCtrl ftc;
	SNDServer * m_sender;
	boost::asio::io_service SND_service;

	std::string SNDHost = "192.168.0.101";
	int SNDPort = 2408;
	m_sender = new SNDServer(SND_service, boost::asio::ip::address::from_string(SNDHost), SNDPort);

	ftc.SetFXGain(-1674.08485641479, 25.3936432491561, 3936.02718786968, -26695.2539299392, -3463.73728677908, 32320.8777656041);
	ftc.SetFYGain(-4941.11252317989, 32269.5827812235, 1073.82949467087, -15541.8400780814, 3061.89541712948, -18995.9891819409);
	ftc.SetFZGain(39553.9250733854, -501.940034213822, 40905.2545309848, 85.1095865539103, 38879.4015426067, 541.344775537753);
	ftc.SetTXGain(-57.4775857386444, 225.941430274037, -638.238694389357, -116.780649376712, 645.133934885308, -116.310081348745 );
	ftc.SetTYGain(786.70602313107, -4.36504382717595, -422.360387149734, 180.7428885668, -352.389412256677, -232.293941041101);
	ftc.SetTZGain(60.1009854270179, -400.19573754971, 29.142908672741, -392.119024237625, 70.9306507180567, -478.104759057292);

	ftc.SetCalibMatrix();
	if(simu == false)
		ftc.Init();

	ros::Rate loop_rate(5);
	//for(int i = 0; i<100; i++)
	while(ros::ok())
	{
		double Fx, Fy, Fz, Tx, Ty, Tz = 0;
		
		if(simu == false)
			ftc.ReadSGData(Fx, Fy, Fz, Tx, Ty, Tz);
		else
		{
			Fx = 10.0;
			Fy = -10.0;
			Fz = 4.0;
			Tx = 1.5;
			Ty = -1.0;
			Tz = 1.2;
		}
		std_msgs::Float32MultiArray msg;
		msg.data.push_back(Fx);
		msg.data.push_back(Fy);
		msg.data.push_back(Fz);
		msg.data.push_back(Tx);
		msg.data.push_back(Ty);
		msg.data.push_back(Tz);
		force_pub.publish(msg);
/*		Fx -= -13.4964;
		Fy -= -1.3;
		Fz -= 18;

		Tx -= 1.72288; 
		Ty -= 1.34723;
		Tz -= -0.460466;*/
		m_sender->sendForce(Fx, Fy, Fz, Tx, Ty, Tz);
		SND_service.poll();
		ros::spinOnce();
		//std::cout<<"Fx: "<<Fx<<" Fy: "<<Fy<<" Fz: "<<Fz<<" Tx: "<<Tx<<" Ty: "<<Ty<<" Tz: "<<Tz<<std::endl;
		loop_rate.sleep();
	}
	return 0;
}

