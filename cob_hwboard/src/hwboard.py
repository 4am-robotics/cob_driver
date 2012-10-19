#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_hwboard')
import rospy
from serial import *
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

############################################
#	check yaml parameters
############################################

if rospy.has_param("/hwboard/hwboarddevicestring") and rospy.has_param("/hwboard/head_sensor") and rospy.has_param("/hwboard/eye_sensor") and rospy.has_param("/hwboard/torso_module_sensor") and rospy.has_param("/hwboard/torso_sensor") and rospy.has_param("/hwboard/pc_sensor") and rospy.has_param("/hwboard/engine_sensor"):
	devicestring = rospy.get_param("/hwboard/hwboarddevicestring")
	#print devicestring
	head_sensor_id = rospy.get_param("/hwboard/head_sensor")
	#print "%0#x " % head_sensor_id
	eye_sensor_id = rospy.get_param("/hwboard/eye_sensor")
	#print "%0#x " % eye_sensor_id
	torso_module_sensor_id = rospy.get_param("/hwboard/torso_module_sensor")
	#print "%0#x " % torso_module_sensor_id
	torso_sensor_id = rospy.get_param("/hwboard/torso_sensor")
	#print "%0#x " % torso_sensor_id
	pc_sensor_id = rospy.get_param("/hwboard/pc_sensor")
	#print "%0#x " % pc_sensor_id
	engine_sensor_id = rospy.get_param("/hwboard/engine_sensor")
	#print "%0#x " % engine_sensor_id

############################################
#	initialize serial port
############################################

	s = Serial(port=devicestring,baudrate=230400,bytesize=EIGHTBITS,parity=PARITY_NONE,stopbits=STOPBITS_ONE,timeout=3)
	s.open()

############################################
#	send reset data message
############################################

	#print ""
	#print "sending reset messages"
	#print ""

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


	#print ""
	#print "send message"
	#for i in range(len(send_buff_array)):
	#	print "\t\t%0#x " % send_buff_array[i]


	# send message
	while (preamble_error == 1 or crc_error == 1) and retry < 8:
		message= ""
		for i in range(preamble_bytes):
			message += chr(0x55)
		for i in send_buff_array:
			message += chr(i)
		s.write(message)

	#receive message
		#print "receive message"
		# check for first preamble byte of reveiced message
		read_buff_array = []
		buff = s.read(1)
		preamble_count = 0
		for i in buff:
			read_buff_array.append(ord(i))

		if read_buff_array[0] == 0x55:

			# check for following preamble bytes
			while read_buff_array[0] == 0x55 and preamble_count < 10:
				read_buff_array = []
				buff = s.read(1)
				for i in buff:
					read_buff_array.append(ord(i))
				preamble_count = preamble_count + 1
	
			buff = s.read(13)

			# check preamble length

			if preamble_count > 6:
				#print "\t\tpreamble error"
				preamble_error = 1
				preamble_bytes = preamble_bytes + 1
				retry = retry + 1
				if preamble_bytes == 7:
					preamble_bytes = 2
			elif preamble_count < 2:
				#print "\t\tpreamble error"
				preamble_error = 1
				preamble_bytes = preamble_bytes + 1
				retry = retry + 1
				if preamble_bytes == 7:
					preamble_bytes = 2
			else:
				#print "\t\tpreamble ok"
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
					#print "\t\tprint crc error"
					crc_error = 1
					preamble_bytes = preamble_bytes + 1
					retry = retry + 1
					if preamble_bytes == 7:
						preamble_bytes = 2
				else:
					#print "\t\tcrc ok"
					crc_error = 0

		# no preamble detected
		else:
			#print "\t\tpreamble error"
			buff = s.read(14)
			preamble_error = 1
			preamble_bytes = preamble_bytes + 1
			retry = retry + 1
			if preamble_bytes == 7:
				preamble_bytes = 2


############################################
#	send reset timer message
############################################

# initialize message and local variables
#send_buff_array=[0x88,0x0E,0x00,0x00,0x00]
#message= ""
#preamble_bytes = 4
#preamble_error = 1
#crc_error = 1
#retry = 0
#
# calculate crc
#crc = 0x00
#for i in range(4):
#	data = send_buff_array[i]
#	for k in range(8):
#		feedback_bit = (crc^data) & 0x80
#		feedback_bit = (feedback_bit>>7) & 0xFF
#		
#		if feedback_bit == 1:
#			crc = (crc<<1) & 0xFF
#			crc = crc^0x31
#		else:
#			crc = (crc<<1) & 0xFF
#		
#		data = (data<<1) & 0xFF
#send_buff_array[4] = crc
#
#
#print ""
#print "send message"
#for i in range(len(send_buff_array)):
#	print "\t\t%0#x " % send_buff_array[i]
#
#
# send message
#while (preamble_error == 1 or crc_error == 1) and retry < 8:
#	message= ""
#	for i in range(preamble_bytes):
#		message += chr(0x55)
#	for i in send_buff_array:
#		message += chr(i)
#	s.write(message)
#
# receive message
#	print "receive message"
#	# check for first preamble byte of reveiced message
#	read_buff_array = []
#	buff = s.read(1)
#	preamble_count = 0
#	for i in buff:
#		read_buff_array.append(ord(i))
#
#	if read_buff_array[0] == 0x55:
#
#		# check for following preamble bytes
#		while read_buff_array[0] == 0x55 and preamble_count < 10:
#			read_buff_array = []
#			buff = s.read(1)
#			for i in buff:
#				read_buff_array.append(ord(i))
#			preamble_count = preamble_count + 1
#	
#		buff = s.read(13)
#
#		# check preamble length
#
#		if preamble_count > 6:
#			print "\t\tpreamble error"
#			preamble_error = 1
#			preamble_bytes = preamble_bytes + 1
#			retry = retry + 1
#			if preamble_bytes == 7:
#				preamble_bytes = 2
#		elif preamble_count < 2:
#			print "\t\tpreamble error"
#			preamble_error = 1
#			preamble_bytes = preamble_bytes + 1
#			retry = retry + 1
#			if preamble_bytes == 7:
#				preamble_bytes = 2
#		else:
#			print "\t\tpreamble ok"
#			# preamble ok. evaluate message
#			preamble_error = 0
#			# get remaining message
#			for i in buff:
#				read_buff_array.append(ord(i))
#
#			#check crc
#			crc = 0x00
#			for i in range(14):
#				data = read_buff_array[i]
#				for k in range(8):
#					feedback_bit = (crc^data) & 0x80
#					feedback_bit = (feedback_bit>>7) & 0xFF
#					
#					if feedback_bit == 1:
#						crc = (crc<<1) & 0xFF
#						crc = crc^0x31
#					else:
#						crc = (crc<<1) & 0xFF
#					
#					data = (data<<1) & 0xFF
#			if crc != 0:
#				print "\t\tcrc error"
#				crc_error = 1
#				preamble_bytes = preamble_bytes + 1
#				retry = retry + 1
#				if preamble_bytes == 7:
#					preamble_bytes = 2
#			else:
#				print "\t\tcrc ok"
#				crc_error = 0
#
#	# no preamble detected
#	else:
#		print "\t\tpreamble error"
#		buff = s.read(14)
#		preamble_error = 1
#		preamble_bytes = preamble_bytes + 1
#		retry = retry + 1
#		if preamble_bytes == 7:
#			preamble_bytes = 2

############################################
#	initialize variables
############################################


	send_channel = 0
	read_channel = 0
	send_specifier = 0
	read_specifier = 0
	read_status = 0
	read_data = 0
	read_id = 0
	read_crc = 0

############################################
#	invalid yaml parameter
############################################
else:
	rospy.logerror("could not load parameters from yaml-file for hardware monitor")

############################################
#	send data messages
############################################

def hwboard():
	#print ""
	#print "start evaluation"
	# init rosnode
	pub = rospy.Publisher('diagnostics',DiagnosticArray)
	rospy.init_node('hwboard')
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


				#print ""
				#print "\tsend message"
				#for i in range(len(send_buff_array)):
				#	print "\t\t%0#x " % send_buff_array[i]


				# send message
				while (preamble_error == 1 or crc_error == 1) and retry < 8:
					message= ""
					for i in range(preamble_bytes):
						message += chr(0x55)
					for i in send_buff_array:
						message += chr(i)
					s.write(message)



				# receive message
					#print "\treceive message"
					# check for first preamble byte of reveiced message
					read_buff_array = []
					buff = s.read(1)
					preamble_count = 0
					for i in buff:
						read_buff_array.append(ord(i))

					if read_buff_array[0] == 0x55:

						# check for following preamble bytes
						while read_buff_array[0] == 0x55 and preamble_count < 10:
							read_buff_array = []
							buff = s.read(1)
							for i in buff:
								read_buff_array.append(ord(i))
							preamble_count = preamble_count + 1
	
						buff = s.read(13)

						# check preamble length

						if preamble_count > 6:
							#print "\t\t\tpreamble error"
							preamble_error = 1
							preamble_bytes = preamble_bytes + 1
							retry = retry + 1
							if preamble_bytes == 7:
								preamble_bytes = 2
						elif preamble_count < 2:
							#print "\t\t\tpreamble error"
							preamble_error = 1
							preamble_bytes = preamble_bytes + 1
							retry = retry + 1
							if preamble_bytes == 7:
								preamble_bytes = 2
						else:
							#print "\t\t\tpreamble ok"
							# preamble ok. evaluate message
							preamble_error = 0
							# get remaining message
							for i in buff:
								read_buff_array.append(ord(i))
							#for i in range (len(read_buff_array)):
							#	print "\t\t\t%0#x " % read_buff_array[i]

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
								#print "\t\t\tcrc error"
								crc_error = 1
								preamble_bytes = preamble_bytes + 1
								retry = retry + 1
								if preamble_bytes == 7:
									preamble_bytes = 2
							else:
								#print "\t\t\tcrc ok"
								crc_error = 0

					# no preamble detected
					else:
						#print "\t\t\tpreamble error"
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
				
				# evaluate status byte
				if read_channel == send_channel:
					if read_specifier == send_specifier:
						if read_status == 0 or read_status == 8:
							if send_specifier == 0:
								#print "\t\t\t\tno error detected"
								read_data = read_data / 10.0
							else:
								#print "\t\t\t\tno error detected"
								read_data = read_data / 1000.0
							erro_while_reading = 0
						else:
							#print "\t\t\t\terror detected"
							read_data = 0
							error_while_reading = 1
					else:
						#print "\t\t\t\terror detected"
						read_data = 0
						error_while_reading = 1
				else:
					#print "\t\t\t\terror detected"
					read_data = 0
					error_while_reading = 1


				#prepare status message for publishing

				# init sensor object
				status_object = DiagnosticStatus()

				# init local variable for data
				key_value = KeyValue()

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

					# mapping for temperature channels
					#if read_id == 0x106EB90A020800:
						#print "\t\t\t\t\t\tsensor correctly evaluated"
					if read_id == head_sensor_id:
						status_object.name = "Head Temperature"
						status_object.hardware_id = "hcboard_channel " + str(send_channel)
						#print "\t\t\t\t\t\tchannel " + str(send_channel)
						#key_value.key = "Head Temperature"
					elif read_id == eye_sensor_id:
						status_object.name = "Eye Camera Temperature"
						status_object.hardware_id = "hcboard_channel = " + str(send_channel)
						#key_value.key = "Eye Camera Temperature"
					elif read_id == torso_module_sensor_id:
						status_object.name = "Torso Module Temperature"
						status_object.hardware_id = "hcboard_channel =" + str(send_channel)
						#key_value.key = "Torso Module Temperature"
					elif read_id == torso_sensor_id:
						status_object.name = "Torso Temperature"
						status_object.hardware_id = "hcboard_channel =" + str(send_channel)
						#key_value.key = "Torso Temperature"
					elif read_id == pc_sensor_id:	
						status_object.name = "PC Temperature"
						status_object.hardware_id = "hcboard_channel =" + str(send_channel)
						#key_value.key = "PC Temperature"
					elif read_id == engine_sensor_id:
						status_object.name = "Engine Temperature"
						status_object.hardware_id = "hcboard_channel = " + str(send_channel)
						#key_value.key = "Engine Temperature"

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
						status_object.hardware_id = "hcboard_channel = 0"
						#key_value.key = "Akku Voltage"
					elif send_channel == 1:
						status_object.name = "Torso Engine Voltage"						
						status_object.hardware_id = "hcboard_channel = 1"
						#key_value.key = "Torso Engine Voltage"
					elif send_channel == 2:
						status_object.name = "Torso Logic Voltage"						
						status_object.hardware_id = "hcboard_channel = 2"
						#key_value.key = "Torso Logic Voltage"
					elif send_channel == 3:
						status_object.name = "Tray Logic Voltage"						
						status_object.hardware_id = "hcboard_channel = 3"
						#key_value.key = "Tray Logic Voltage"
					elif send_channel == 6:
						status_object.name = "Arm Engine Voltage"						
						status_object.hardware_id = "hcboard_channel = 6"
						#key_value.key = "Arm Logic Voltage"
					elif send_channel == 7:
						status_object.name = "Tray Engine Voltage"						
						status_object.hardware_id = "hcboard_channel = 7"
						#key_value.key = "Tray Engine Voltage"

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
						status_object.hardware_id = "hcboard_channel = 1"
						#key_value.key = "Torso Engine Current"
					elif send_channel == 2:
						status_object.name = "Torso Logic Current"						
						status_object.hardware_id = "hcboard_channel = 2"
						#key_value.key = "Torso Logic Current"
					elif send_channel == 3:
						status_object.name = "Tray Logic Current"						
						status_object.hardware_id = "hcboard_channel = 3"
						#key_value.key = "Tray Logic Current"
					elif send_channel == 6:
						status_object.name = "Arm Engine Current"						
						status_object.hardware_id = "hcboard_channel = 6"
						#key_value.key = "Arm Logic Current"
					elif send_channel == 7:
						status_object.name = "Tray Engine Current"						
						status_object.hardware_id = "hcboard_channel = 7"
						#key_value.key = "Tray Engine Current"
				if error_while_reading == 1:
					level = 1
					status_object.message = "detected error while receiving answer from hardware control board"

				status_object.level = level

				key_value.value = str(read_data)
				#key_value.value = send_channel + send_specifier 
		
				status_object.values.append(key_value)
				pub_message.status.append(status_object)
	
				#print ""
				#print key_value.key
				#print key_value.value
				#print status_object.values[0].key
				#print status_object.values[0].value
				#print ""

		#for i in range (len(pub_message.status)):
		#	print pub_message.status[i].values[0].key
		#	print pub_message.status[i].values[0].value
		#	print ""

		pub.publish(pub_message)
		rospy.sleep(1.0)
			

				
#######################################
#
#######################################

if __name__ == '__main__':
	try:
		hwboard()
	except rospy.ROSInterruptException: pass
