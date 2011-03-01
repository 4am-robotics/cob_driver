# This message holds the emergency stop (EMStop) status of the robot. It detects wether an EMStop is caused by the safety laserscanner or the emergency stop buttons. Moreover, it gives signalizes wether the EMStop was confirmed (after Button press stop) and the system is free again.

# Possible EMStop States
int16 EMFREE = 0 		# system operatign normal
int16 EMSTOP = 1 		# emergency stop is active (Button pressed; obstacle in safety field of scanner)
int16 EMCONFIRMED = 2 	# emergency stop was confirmed system is reinitializing and going back to normal

bool emergency_button_stop	# true = emergency stop signal is issued by button pressed
bool scanner_stop			# true = emergency stop signal is issued by scanner
int16 emergency_state		# state (including confimation by key-switch), values see above

