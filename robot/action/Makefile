###############################################################
#
# Purpose: Makefile for "action_script"
# Author.: robotis
# Version: 0.1
# License: GPL
#
###############################################################

TARGET = action_script

INCLUDE_DIRS = -I../Linux/include -I../Framework/include

CXX = g++
CXXFLAGS += -O2 -DLINUX -Wall $(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DLINUX -DDEBUG -Wall $(INCLUDE_DIRS)
LFLAGS += -lpthread -ljpeg -lrt 

OBJECTS =   main.o VisionMode.o StatusCheck.o cmd_process.o

all: $(TARGET)

clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

libclean:
	make -C ../Linux/build clean

distclean: clean libclean

darwin.a:
	make -C ../Linux/build

$(TARGET): darwin.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../Linux/lib/darwin.a -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
tgz: clean
	mkdir -p backups
	tar czvf ./backups/camera_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
