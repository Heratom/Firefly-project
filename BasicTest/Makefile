CC=gcc
CFLAGS= -Wall
LIB=../packages
INCLUDES=-I$(LIB)/libARSAL/Includes -I$(LIB)/libARController/Includes -I$(LIB)/libARNetwork/Includes -I$(LIB)/libARNetworkAL/Includes -I$(LIB)/libARDiscovery/Includes -I$(LIB)/libARController/gen/Includes/ -I$(LIB)/ARSDKTools/ -I$(LIB)/libARCommands/Includes/ -I$(LIB)/libARCommands/gen/Includes/
LIBDIR=/home/heratom/Firefly-project/out/Unix-base/staging/usr/lib/
LDFLAGS=-L$(LIBDIR)
EXEC=Pilot

all: $(EXEC)

Pilot : BebopPiloting.o ihm.o
	$(CC) $^ -o $@ $(LDFLAGS) $(INCLUDES) -I$(LIB)/Samples/Unix/BebopPilotingNewAPI/
	
%.o: %.c
	$(CC) $(INCLUDES) -I$(LIB)/Samples/Unix/BebopPilotingNewAPI/ -o $@ -c $< $(CFLAGS)
	
clean :
	rm *.o *~ $(EXEC)
	
