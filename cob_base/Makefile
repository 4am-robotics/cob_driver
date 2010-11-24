all: cob_base

cob_base:
	# create link to lib depending on version (32-bit/64-bit)
	cd common/lib && ln -sf `uname -m`/* .
	
	# create link to ini-files
	#cd ros/bin/Platform/IniFiles && ln -sf $(ROBOT)/* .

include $(shell rospack find mk)/cmake.mk
