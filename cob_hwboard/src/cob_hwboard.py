#!/usr/bin/env python
#################################################################
# 
# Copyright (c) 2012
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA)
#
#################################################################
#
# Project name: care-o-bot
# ROS stack name: cob_driver
# ROS package name: cob_hwboard
#
# Author: Eduard Herkel, email: eduard.herkel@ipa.fraunhofer.de
# Supervised by: Eduard Herkel, email: eduard.herkel@ipa.fraunhofer.de
#
# Date of creation: October 2012
#
# ToDo
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib; roslib.load_manifest('cob_hwboard')
import rospy
from serial import *
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

class HwBoard:
	def __init__(self):

		rospy.init_node('hwboard')

		# get parameters from parameter server
		if not rospy.has_param("~devicestring"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","devicestring")
			sys.exit()
		devicestring_param = rospy.get_param("~devicestring")

		if not rospy.has_param("~head_sensor"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","head_sensor")
			sys.exit()
		self.head_sensor_param = rospy.get_param("~head_sensor")

		if not rospy.has_param("~eye_sensor"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","eye_sensor")
			sys.exit()
		self.eye_sensor_param = rospy.get_param("~eye_sensor")

		if not rospy.has_param("~torso_module_sensor"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","torso_module_sensor")
			sys.exit()
		self.torso_module_sensor_param = rospy.get_param("~torso_module_sensor")

		if not rospy.has_param("~torso_sensor"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","torso_sensor")
			sys.exit()
		self.torso_sensor_param = rospy.get_param("~torso_sensor")

		if not rospy.has_param("~pc_sensor"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","pc_sensor")
			sys.exit()
		self.pc_sensor_param = rospy.get_param("~pc_sensor")

		if not rospy.has_param("~engine_sensor"):
			rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...","engine_sensor")
			sys.exit()
		self.engine_sensor_param = rospy.get_param("~engine_sensor")
	
		# open serial connection
		rospy.loginfo("trying to initializing serial connection")
		try:
			self.s = Serial(port=devicestring_param,baudrate=230400,bytesize=EIGHTBITS,parity=PARITY_NONE,stopbits=STOPBITS_ONE,timeout=3)
		except serial.serialutil.SerialException:
			rospy.logerr("Could not initialize serial connection on %s, aborting...",devicestring_param)
			sys.exit()
		rospy.loginfo("serial connection initialized successfully")
		self.s.open()



	def reset(self):

		# initialize message and local variables
		send_buff_array=[0xFF,0x0E,0x00,0x00,0x00]
		message= ""
		preamble_bytes = 4
		preamble_error = 1
		crc_error = 1
		retry = 0

		# calculate crc
		crc = 0x00
		for i in range(4):
			data = send_buff_array[i]
			for k in range(8):
				feedback_bit = (crc^data) & 0x80
				feedback_bit = (feedback_bit>>7) & 0xFF
		
				if feedback_bit == 1:
					crc = (crc<<1) & 0xFF
					crc = crc^0x31
				else:
					crc = (crc<<1) & 0xFF
		
				data = (data<<1) & 0xFF
		send_buff_array[4] = crc

		# send message
		while (preamble_error == 1 or crc_error == 1) and retry < 8:
			message= ""
			for i in range(preamble_bytes):
				message += chr(0x55)
			for i in send_buff_array:
				message += chr(i)
			self.s.write(message)

		#receive message
			# check for first preamble byte of reveiced message
			read_buff_array = []
			buff = self.s.read(1)
			preamble_count = 0
			for i in buff:
				read_buff_array.append(ord(i))

			if read_buff_array[0] == 0x55:

				# check for following preamble bytes
				while read_buff_array[0] == 0x55 and preamble_count < 10:
					read_buff_array = []
					buff = self.s.read(1)
					for i in buff:
						read_buff_array.append(ord(i))
					preamble_count = preamble_count + 1
	
				buff = self.s.read(13)

				# check preamble length

				if preamble_count > 6:
					preamble_error = 1
					preamble_bytes = preamble_bytes + 1
					retry = retry + 1
					if preamble_bytes == 7:
						preamble_bytes = 2
				elif preamble_count < 2:
					preamble_error = 1
					preamble_bytes = preamble_bytes + 1
					retry = retry + 1
					if preamble_bytes == 7:
						preamble_bytes = 2
				else:
					# preamble ok. evaluate message
					preamble_error = 0

					# get remaining message
					for i in buff:
						read_buff_array.append(ord(i))

					#check crc
					crc = 0x00
					for i in range(14):
						data = read_buff_array[i]
						for k in range(8):
							feedback_bit = (crc^data) & 0x80
							feedback_bit = (feedback_bit>>7) & 0xFF
					
							if feedback_bit == 1:
								crc = (crc<<1) & 0xFF
								crc = crc^0x31
							else:
								crc = (crc<<1) & 0xFF
					
							data = (data<<1) & 0xFF
					if crc != 0:
						crc_error = 1
						preamble_bytes = preamble_bytes + 1
						retry = retry + 1
						if preamble_bytes == 7:
							preamble_bytes = 2
					else:
						crc_error = 0

			# no preamble detected
			else:
				buff = s.read(14)
				preamble_error = 1
				preamble_bytes = preamble_bytes + 1
				retry = retry + 1
				if preamble_bytes == 7:
					preamble_bytes = 2



	def hwboard(self):

		# initialize local variables
		send_channel = 0
		read_channel = 0
		send_specifier = 0
		read_specifier = 0
		read_status = 0
		read_data = 0
		read_id = 0
		read_crc = 0

		# init ros-node
		pub = rospy.Publisher('diagnostics',DiagnosticArray)
		
		while not rospy.is_shutdown():

			# init publisher message
			pub_message = DiagnosticArray()	

			# init array for storing data
			status_array = []

			# init local variable for error detection
			error_while_reading = 0

			for send_specifier in range(0,7,3):

				if send_specifier == 0:
					count_range = range(6)
				elif send_specifier == 3:
					count_range = [0,1,2,3,6,7]
				else:
					count_range = [1,2,3,6,7]

				for send_channel in count_range:							
			
					# init message and local variables
					send_buff_array = [send_channel,send_specifier,0x00,0x00,0x00]
					message = ""
					preamble_bytes = 4
					preamble_error = 1
					crc_error = 1
					retry = 0
				
					# calculate crc
					crc = 0x00
					for i in range(4):
						data = send_buff_array[i]
						for k in range(8):
							feedback_bit = (crc^data) & 0x80
							feedback_bit = (feedback_bit>>7) & 0xFF
		
							if feedback_bit == 1:
								crc = (crc<<1) & 0xFF
								crc = crc^0x31
							else:
								crc = (crc<<1) & 0xFF
		
							data = (data<<1) & 0xFF
					send_buff_array[4] = crc

					# send message
					while (preamble_error == 1 or crc_error == 1) and retry < 8:
						message= ""
						for i in range(preamble_bytes):
							message += chr(0x55)
						for i in send_buff_array:
							message += chr(i)
						self.s.write(message)



					# receive message
						# check for first preamble byte of reveiced message
						read_buff_array = []
						buff = self.s.read(1)
						preamble_count = 0
						for i in buff:
							read_buff_array.append(ord(i))

						if read_buff_array[0] == 0x55:

							# check for following preamble bytes
							while read_buff_array[0] == 0x55 and preamble_count < 10:
								read_buff_array = []
								buff = self.s.read(1)
								for i in buff:
									read_buff_array.append(ord(i))
								preamble_count = preamble_count + 1
	
							buff = self.s.read(13)

							# check preamble length

							if preamble_count > 6:
								preamble_error = 1
								preamble_bytes = preamble_bytes + 1
								retry = retry + 1
								if preamble_bytes == 7:
									preamble_bytes = 2
							elif preamble_count < 2:
								preamble_error = 1
								preamble_bytes = preamble_bytes + 1
								retry = retry + 1
								if preamble_bytes == 7:
									preamble_bytes = 2
							else:
								# preamble ok. evaluate message
								preamble_error = 0

								# get remaining message
								for i in buff:
									read_buff_array.append(ord(i))

								#check crc
								crc = 0x00
								for i in range(14):
									data = read_buff_array[i]
									for k in range(8):
										feedback_bit = (crc^data) & 0x80
										feedback_bit = (feedback_bit>>7) & 0xFF
					
										if feedback_bit == 1:
											crc = (crc<<1) & 0xFF
											crc = crc^0x31
										else:
											crc = (crc<<1) & 0xFF
					
										data = (data<<1) & 0xFF
								if crc != 0:
									crc_error = 1
									preamble_bytes = preamble_bytes + 1
									retry = retry + 1
									if preamble_bytes == 7:
										preamble_bytes = 2
								else:
									crc_error = 0

						# no preamble detected
						else:
							buff = s.read(14)
							preamble_error = 1
							preamble_bytes = preamble_bytes + 1
							retry = retry + 1
							if preamble_bytes == 7:
								preamble_bytes = 2



					# get channel byte
					read_channel = int(read_buff_array[0])
		
					# get specifier byte
					read_specifier = int(read_buff_array[1])

					# get status byte
					read_status = int(read_buff_array[2])

					# get data bytes
					read_data = 256 * int(read_buff_array[3])
					read_data = read_data + int(read_buff_array[4])

					# get id bytes
					read_id = read_buff_array[5]<<8
					read_id = (read_id | read_buff_array[6])<<8
					read_id = (read_id | read_buff_array[7])<<8
					read_id = read_id | read_buff_array[8]
				
					# evaluate recieved message
					if read_channel == send_channel:
						if read_specifier == send_specifier:
							if read_status == 0 or read_status == 8:
								if send_specifier == 0:
									read_data = read_data / 10.0
								else:
									read_data = read_data / 1000.0
								erro_while_reading = 0
							else:
								read_data = 0
								error_while_reading = 1
						else:
							read_data = 0
							error_while_reading = 1
					else:
						read_data = 0
						error_while_reading = 1


					#prepare status object for publishing

					# init sensor object
					status_object = DiagnosticStatus()

					# init local variable for data
					key_value = KeyValue()

					# set values for temperature parameters
					if send_specifier == 0:
						if read_data == 85:
							level = 1
							status_object.message = "sensor damaged"
						elif read_data > 50:
							level = 2
							status_object.message = "temperature critical"
						elif read_data >40:
							level = 1
							status_object.message = "temperature high"
						elif read_data > 10:
							level = 0
							status_object.message = "temperature ok"
						elif read_data > -1:
							level = 1
							status_object.message = "temperature low"
						else:
							level = 2
							status_object.message = "temperature critical"

						# mapping for temperature sensors
						if read_id == self.head_sensor_param:
							status_object.name = "Head Temperature"
							status_object.hardware_id = "hwboard_channel " + str(send_channel)
						elif read_id == self.eye_sensor_param:
							status_object.name = "Eye Camera Temperature"
							status_object.hardware_id = "hwboard_channel = " + str(send_channel)
						elif read_id == self.torso_module_sensor_param:
							status_object.name = "Torso Module Temperature"
							status_object.hardware_id = "hwboard_channel =" + str(send_channel)
						elif read_id == self.torso_sensor_param:
							status_object.name = "Torso Temperature"
							status_object.hardware_id = "hwboard_channel =" + str(send_channel)
						elif read_id == self.pc_sensor_param:	
							status_object.name = "PC Temperature"
							status_object.hardware_id = "hwboard_channel =" + str(send_channel)
						elif read_id == self.engine_sensor_param:
							status_object.name = "Engine Temperature"
							status_object.hardware_id = "hwboard_channel = " + str(send_channel)
						else:
							level = 1
							status_object.message = "cannot map if from yaml file to temperature sensor"

					# set values for voltage parameters
					elif send_specifier == 3:

						if send_channel == 0:
							if read_data > 58:
								level = 2
								status_object.message = "voltage critical"		
							elif read_data > 56:
								level = 1
								status_object.message = "voltage high"
							elif read_data > 44:
								level = 0
								status_object.message = "voltage ok"
							elif read_data > 42:
								level = 1
								status_object.message = "voltage low"
							else:
								level = 2
								status_object.message = "voltage critical"
						else:
							if read_data > 27:
								level = 2
								status_object.message = "voltage critical"				
							elif read_data > 25:
								level = 1
								status_object.message = "voltage_high"
							elif read_data > 23:
								level = 0
								status_object.message = "voltage ok"
							elif read_data > 19:
								level = 1
								status_object.message = "voltage low"				
							else:
								level = 2
								status_object.message = "voltage critical"

						if send_channel == 0:
							status_object.name = "Akku Voltage"						
							status_object.hardware_id = "hwboard_channel = 0"
						elif send_channel == 1:
							status_object.name = "Torso Engine Voltage"						
							status_object.hardware_id = "hwboard_channel = 1"
						elif send_channel == 2:
							status_object.name = "Torso Logic Voltage"						
							status_object.hardware_id = "hwboard_channel = 2"
						elif send_channel == 3:
							status_object.name = "Tray Logic Voltage"						
							status_object.hardware_id = "hwboard_channel = 3"
						elif send_channel == 6:
							status_object.name = "Arm Engine Voltage"						
							status_object.hardware_id = "hwboard_channel = 6"
						elif send_channel == 7:
							status_object.name = "Tray Engine Voltage"						
							status_object.hardware_id = "hwboard_channel = 7"

					# set values for current parameters
					else:
						if read_data > 15:
							level = 2
							status_object.message = "current critical"
						elif read_data > 10:
							level = 1
							status_object.message = "current high"
						elif read_data < 0:
							level = 2
							status_object.message = "current critical"
						else:
							level = 0
							status_object.message = "current ok"

						if send_channel == 1:
							status_object.name = "Torso Engine Current"						
							status_object.hardware_id = "hwboard_channel = 1"
						elif send_channel == 2:
							status_object.name = "Torso Logic Current"						
							status_object.hardware_id = "hwboard_channel = 2"
						elif send_channel == 3:
							status_object.name = "Tray Logic Current"						
							status_object.hardware_id = "hwboard_channel = 3"
						elif send_channel == 6:
							status_object.name = "Arm Engine Current"						
							status_object.hardware_id = "hwboard_channel = 6"
						elif send_channel == 7:
							status_object.name = "Tray Engine Current"						
							status_object.hardware_id = "hwboard_channel = 7"

					# evaluate error detection
					if error_while_reading == 1:
						level = 1
						status_object.message = "detected error while receiving answer from hardware"

					# append status object to publishing message

					status_object.level = level
					key_value.value = str(read_data)		
					status_object.values.append(key_value)
					pub_message.status.append(status_object)

			# publish message
			pub.publish(pub_message)
			rospy.sleep(1.0)
			

				
#######################################
#
#######################################

if __name__ == '__main__':
	hwb = HwBoard()
	hwb.reset()
	hwb.hwboard()
