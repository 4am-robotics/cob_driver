import ros.*;
import ros.communication.*;
import ros.pkg.std_msgs.msg.String;
import ros.pkg.cob_msgs.msg.JointCommand;


class cob_lbr {


	public static void main(java.lang.String[] args) {

        System.out.println("tut");	
        Ros ros = Ros.getInstance();
        ros.init("kukaNode");

        NodeHandle n = ros.createNodeHandle();

        try
        {
            Publisher<ros.pkg.std_msgs.msg.String> pub = n.advertise("/pub", new ros.pkg.std_msgs.msg.String(), 100);
            ros.pkg.std_msgs.msg.String m = new ros.pkg.std_msgs.msg.String();
            m.data = "Hello, ROS";
            pub.publish(m);
            try {
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


	// Definiere eine Zeichenkette.
	
	// Gebe die Zeichenkette an der Konsole aus.
	}
}
