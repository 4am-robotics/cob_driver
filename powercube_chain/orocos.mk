TARGETLIB = powercubechainRTT
DEBUG = 1

LIBMODULES =  common/src/PowerCubeCtrl\
			common/src/PowerCubeSim\
			common/src/moveCommand\
			common/src/TimeStamp\
			orocos/src/PowerCubeSim_OROCOS
MODULES = $(TARGET) $(LIBMODULES)

LIBS = orocos-rtt-gnulinux
CFLAGS =-DOROCOS_TARGET=gnulinux -D__LINUX__ #needed for m5apiw32
LFLAGS =
INCPATH += common/include/ orocos/include/ 
LIBPATH += common/lib/ orocos/lib/

CXXFLAGS =  -D__LINUX__ -Wall -Wno-deprecated $(INCPATH:%=-I%) $(CFLAGS)
LDFLAGS = $(LFLAGS) $(LIBPATH:%=-L%) $(LIBS:%=-l%)
LIBOBJS = $(LIBMODULES:%=%.o)

all: $(TARGETLIB)

# rule for compiling
%.o: %.cpp
	@echo "C $*"
	$(CXX) -c -o $@ $(CXXFLAGS) $<


# rule for building static library
$(TARGETLIB): $(LIBOBJS)
	@echo "LL shared $@"
	#ar -rcs lib$(TARGETLIB).a $(LIBADD) $^ 
	g++ -fPIC -shared -o  lib$(TARGETLIB).so  $(LDFLAGS) $^
	#ln -fs $(CURDIR)/lib$(TARGETLIB).a $(LIBROOT)/lib$(TARGETLIB).a
	#ln -fs $(CURDIR)/lib$(TARGETLIB).so $(LIBROOT)/lib$(TARGETLIB).so