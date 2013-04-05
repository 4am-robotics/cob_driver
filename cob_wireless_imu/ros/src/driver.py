#!/usr/bin/python

import roslib; roslib.load_manifest('cob_wireless_imu')
import rospy
import tf
import tf.transformations as tf_trans
import numpy
from sensor_msgs.msg import Imu
import socket, traceback, math


def getRotationMatrix(gravity, geomagnetic):
	(Ax,Ay,Az) = gravity
	(Ex,Ey,Ez) = geomagnetic

        Hx = Ey*Az - Ez*Ay
        Hy = Ez*Ax - Ex*Az
        Hz = Ex*Ay - Ey*Ax

        normH = math.sqrt(Hx*Hx + Hy*Hy + Hz*Hz)

        if normH < 0.1:
		print "device is close to free fall (or in space?), or close to magnetic north pole. Typical values are  > 100."
		return []

        invH = 1. / normH

        Hx *= invH
        Hy *= invH
        Hz *= invH

        invA = 1. / math.sqrt(Ax*Ax + Ay*Ay + Az*Az)

        Ax *= invA;
        Ay *= invA;
        Az *= invA;

        Mx = Ay*Hz - Az*Hy
        My = Az*Hx - Ax*Hz
        Mz = Ax*Hy - Ay*Hx

	return numpy.array((
		(Hx,     Hy,     Hz,	0),
		(Mx,     My,     Mz,	0),
		(Ax,     Ay,     Az,	0),
		(0,	 0,	 0,	1)
	))

gravity = [0,0,0]
geomag = [0,0,0]
orientation = numpy.empty((4, ))

def on_data(msg_id, d):
    global gravity, geomag, orientation
    if msg_id==5:
      for i in [0,1,2]: geomag[i]=(geomag[i]*1+d[i])*0.5
    elif msg_id==3:
      for i in [0,1,2]: gravity[i]=(gravity[i]*2+d[i])/3.

    if msg_id==5 or msg_id==3:
      r = getRotationMatrix(gravity, geomag)
      #print r
      #print tf_trans.rotation_matrix(0.123, (1, 2, 3))
      if len(r)>0: orientation = tf_trans.quaternion_from_matrix(r)

def set_cov(cov, val):
    for i in range(0,9):
	cov[i] = val

#3 - Accelerometer (m/s^2)
#4 - Gyroscope (rad/s)
#5 - Magnetometer (micro-Tesla uT)

def driver():
    global orientation

    pub = rospy.Publisher('/imu/data', Imu)
    rospy.init_node('imu')

    data = Imu()
    data.header.frame_id = rospy.get_param('frame_id','imu')
    set_cov(data.orientation_covariance, -1)
    set_cov(data.angular_velocity_covariance, -1)
    set_cov(data.linear_acceleration_covariance, -1)

    id_map = {3: [False, data.linear_acceleration_covariance], 4: [False, data.angular_velocity_covariance], 5: [False, data.orientation_covariance]}

    host = ''
    port = 5555

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((host, port))

    while not rospy.is_shutdown():
        message, address = s.recvfrom(8192)
        smsgs = message.split(",")
	msgs=[]
	for m in smsgs:
		msgs.append(m.strip())
        i=1
        while i<len(msgs):
		msgid = int(msgs[i])
		x = float(msgs[i+1])
		y = float(msgs[i+2])
		z = float(msgs[i+3])
		if id_map[msgid][0]==False:
			id_map[msgid][0]=True
			set_cov(id_map[msgid][1], 0)
		on_data(msgid, [x,y,z])
		if msgid==3:
			data.linear_acceleration.x = x
			data.linear_acceleration.y = y
			data.linear_acceleration.z = z
		elif msgid==4:
			data.angular_velocity.x = x
			data.angular_velocity.y = y
			data.angular_velocity.z = z
		elif msgid==5:
			#print orientation
			data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w = orientation
        	i+=4
	data.header.stamp = rospy.Time.now()
        pub.publish(data)


if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass

