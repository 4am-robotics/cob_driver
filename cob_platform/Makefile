all: cob_platform

cob_platform:
	# create link to lib depending on version (32-bit/64-bit)
	cd common/lib && ln -sf `uname -m`/* .
clean:
	rm -rf common $(SOURCE_DIR)
wipe: clean
	-rm -rf $(SOURCE_DIR)

include $(shell rospack find mk)/cmake.mk
