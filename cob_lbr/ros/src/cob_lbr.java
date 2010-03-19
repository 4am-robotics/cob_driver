//package robotApplication;

import ros.*;
import ros.communication.*;
import ros.pkg.std_msgs.msg.String;
import ros.pkg.cob_msgs.msg.JointCommand;
import ros.pkg.trajectory_msgs.msg.JointTrajectory;

import kukaRoboter.roboticsAPI.RobotApplication;
import kukaRoboter.roboticsAPI.controllerModel.Controller;
import kukaRoboter.roboticsAPI.deviceModel.JointPosition;
import kukaRoboter.roboticsAPI.deviceModel.LBR;
import kukaRoboter.roboticsAPI.deviceModel.MountPointObject;
import kukaRoboter.roboticsAPI.geometricModel.World;

public class cob_lbr extends RobotApplication
{

	public void sleep (int n)
	{
		long t0, t1;
		t0 =  System.currentTimeMillis();
		do{
			t1 = System.currentTimeMillis();
		}
		while ((t1 - t0) < (n * 1000));
	}
	private void run()
	{
		try{
		
			Controller.getController("Beckhoff").setGlobalMotionOverride(1.0); //permanent
			
			LBR lbr = (LBR)Controller.getController("Beckhoff").getDeviceService().getDevice("LBRLeft");
			
			lbr.setDefaultVelocity(0.2);
			//lbr.ptp(new JointPosition(0.0,0.0,0,0.0,0,0,0));
			System.out.println(lbr.getCurrentJointPosition());
			World.Current.getAcceleration().set(-1.7954, -6.9367, 6.7004);
			//MountPointObject o = new MountPointObject();
			//o.setGrams(1950);
			//o.setCenterOfGravityMillimeter(0, 0, 70);
			//lbr.mount(o);

			int MAX_TRAJ_CNT = 7;
			int traj_cnt=0;
			JointPosition trajectory[] = new JointPosition[MAX_TRAJ_CNT];
			boolean trajectory_valid = false;
			for (int i = 0; i < MAX_TRAJ_CNT; i++)
				trajectory[i] = new JointPosition(0,0,0,0,0,0,0);
			
			int mode = 0;
			while (mode != 'q')
			{
				System.out.println();
				System.out.println("--------------------------");
				System.out.println("Choose application:");
				System.out.println("GravComp (g)");
				System.out.println("Move Saved Trajectory (t)");
				System.out.println("Quit (q)");
				System.out.println("--------------------------");


				mode = System.in.read();
				System.in.skip(System.in.available());
				
				if (mode == 'g')
				{
					System.out.println("Warning: starting gravComp in 5 seconds ...");
					System.out.println("Press 0 to quit GravComp mode!");
					//sleep(10);
					Thread.sleep(5000);

					lbr.startGravComp();
					traj_cnt = 0;
					while (mode != '0')
					{	
						System.out.println("[GRAV_COMP MODE] 0 to quit, 1 to print current position, 2 to store trajectory ...");
						mode = System.in.read();
						System.in.skip(System.in.available());
						if (mode == '1')
						{
							System.out.println(lbr.getCurrentJointPosition());
						}
						else if (mode =='2')
						{
							if (traj_cnt < MAX_TRAJ_CNT)
							{

								trajectory[traj_cnt] = lbr.getCurrentJointPosition();
								System.out.println("Configuration "+traj_cnt+": "+trajectory[traj_cnt++]+" added");
								if (traj_cnt == MAX_TRAJ_CNT-1) trajectory_valid = true;
							}
							else
							{
								System.out.println("Sorry, max number of configurations per trajectory reached!");
							}

						}
					}
					System.out.println("Quitting GravComp Mode ...");
					lbr.stopGravComp();
					System.out.println("done.");
				}
				else if (mode=='t')
				{	
					if (traj_cnt > 0)
					{	
						System.out.println("Replaying trajectory:");
						for (int i = 0; i < traj_cnt; i++)
							System.out.println(trajectory[i]);
					
						switch (traj_cnt)
						{	
							case 0:
								break;
							case 1:
								lbr.ptp(trajectory[0]);break;
							case 2:
								lbr.spline(trajectory[0],trajectory[1]);break;
							case 3:
								lbr.spline(trajectory[0],trajectory[1],trajectory[2]);break;
							case 4:
								lbr.spline(trajectory[0],trajectory[1],trajectory[2],trajectory[3]);break;
							case 5:
								lbr.spline(trajectory[0],trajectory[1],trajectory[2],trajectory[3],trajectory[4]);break;
							case 6:
								lbr.spline(trajectory[0],trajectory[1],trajectory[2],trajectory[3],trajectory[4],trajectory[5]);break;
							default:
						}

					}
					else
					{
						System.out.println("No trajectory availbale, please choose 'g' first");
					}
				}

			}
			System.out.println("Quitting ...");
			//			
			//			lbr.stopGravComp();

		}catch (Exception ex){
			Controller.disposeControllers();
		}



		// your code comes here...
	}
	
	public static void main(java.lang.String[] args) 
	{
        System.out.println("tut");	
        Ros ros = Ros.getInstance();
        ros.init("kukaNode");

        NodeHandle n = ros.createNodeHandle();

		Controller.loadConfiguration("common/files/RoboticsAPI.config.xml");
		cob_lbr sample = new cob_lbr();
		sample.setup("Beckhoff", "LBRLeft");
		sample.run();

	}//main
}//class
