#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_relayboard')
import rospy
import time
import csv
from cob_relayboard.msg import EmergencyStopState
from std_msgs.msg import Float64
import savizky
#from pr2_msgs.msg import PowerState

class volts_filter():
    
    def __init__(self):
        self.volts = 0.
        self.wsize = 61
        self.filter_order = 3
        self.theta = rospy.get_param("theta")
        self.off_y = rospy.get_param("off_y")
        self.abcd = rospy.get_param("abcd")
        self.hvalue = 0
        self.sg = savitzky.savitzky_golay(window_size=wsize, order=filter_order)
        self.volt_filt = None
        
        rospy.Subscriber("/power_board/voltage", Float64, callback)
    
    def callback(self, data):

        if(self.hvalue == 0):
            self.hvalue = data
            self.volt_filt = hvalue*np.ones(2*wsize+1)
            
	    if(data <= 44000):
	        self.volts = 44000
            time_r = 0.
            return
        elif(data >= 48000):
	        self.volts = 48000
	        
        else:
            self.volts = data
            
        self.volt_filt = np.insert(self.volt_filt, 0, self.volts)
        self.volt_filt = np.delete(self.volt_filt, -1)

        vfilt = sg.filter(volt_filt)
        old_settings = np.seterr(all='raise')
        
        self.t_est = np.polyval(self.abcd, self.vfilt[self.wsize])
        
        self.t_est = vfilt[self.wsize]*sin(self.theta) + self.t_est*cos(self.theta)

        self.t_est = self.t_est + off_y

	
if __name__ == '__main__':
    rospy.init_node('volt_filt')
	vf = volts_filter()
	
	while not rospy.is_shutdown():
		rospy.sleep(1.0)


