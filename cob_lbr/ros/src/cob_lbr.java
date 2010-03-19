import ros.*;
import ros.communication.*;
import ros.pkg.std_msgs.msg.String;
import ros.pkg.cob_msgs.msg.JointCommand;
import ros.pkg.trajectory_msgs.msg.JointTrajectory;


class cob_lbr {


	public static void main(java.lang.String[] args) {

        System.out.println("tut");	
        Ros ros = Ros.getInstance();
        ros.init("kukaNode");

        NodeHandle n = ros.createNodeHandle();

	    try
	    {
	        Publisher<ros.pkg.std_msgs.msg.String> pub = n.advertise("/sub", new ros.pkg.std_msgs.msg.String(), 100);
	        ros.pkg.std_msgs.msg.String m = new ros.pkg.std_msgs.msg.String();
	        m.data = "Hello, ROS";
	        pub.publish(m);
/*	        try {
	            Thread.sleep(4000);
	        } catch (InterruptedException e)
	        {
	        }
	        pub.shutdown();
	    }
	    catch(ros.RosException e)
	    {
	        System.out.println("Exception");
	    }

		try
		{
*/
			ros.logInfo("inside try");
			Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback = new Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String>(); 
			Subscriber<ros.pkg.std_msgs.msg.String> sub = n.subscribe("/sub", new ros.pkg.std_msgs.msg.String(), callback, 10);

			int i = 0;
			while (i<50)
			{
			    m.data = "Hello, ROS";
			    pub.publish(m);
	        
				ros.logInfo("spin");
				n.spinOnce();

				try 
				{
					ros.logInfo("sleep1");
			    	Thread.sleep(100);
			    }
			    catch (InterruptedException e)
			    {
		        }

				ros.logInfo("while");
				while (!callback.isEmpty())
				{
					ros.logInfo("callback");
					System.out.println(callback.pop().data);
				}
				i = i+1;
			}
			
			try 
			{
				ros.logInfo("sleep2");
	        	Thread.sleep(4000);
	        }
	        catch (InterruptedException e)
	        {
	        }

			ros.logInfo("shutdown");
			sub.shutdown();
			pub.shutdown();

		}//try
		catch(Exception e)
		{
	        System.out.println("Exception");			
		}
	}//main
}//class
