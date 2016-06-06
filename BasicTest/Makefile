CC=gcc
CFLAGS= -Wall
LIB=../packages
INCLUDES=-I$(LIB)/libARSAL/Includes -I$(LIB)/libARController/Includes -I$(LIB)/libARNetwork/Includes -I$(LIB)/libARNetworkAL/Includes -I$(LIB)/libARDiscovery/Includes -I$(LIB)/libARController/gen/Includes/ -I$(LIB)/ARSDKTools/ -I$(LIB)/libARCommands/Includes/ -I$(LIB)/libARCommands/gen/Includes/
LDFLAGS=-L../out/arsdk-native/staging/usr/lib
BIBLI=-larcontroller -lardiscovery -larcommands -lardatatransfer -larmavlink -larmedia -larnetwork -larnetworkal -larsal -larstream2 -larstream -larupdater -larutils -lcrypto -lcurl -ljson -lssl -ltls -lcurses
EXEC=Move

all: $(EXEC)

Pilot : BebopPiloting.o ihm.o
	$(CC) -o $@ $^ $(LDFLAGS) $(INCLUDES) -I$(LIB)/Samples/Unix/BebopPilotingNewAPI/ $(BIBLI)
	
Move : Move.o ihm.o
	$(CC) -o $@ $^ $(LDFLAGS) $(INCLUDES) -I$(LIB)/Samples/Unix/BebopPilotingNewAPI/ $(BIBLI)
	
%.o: %.c
	$(CC) $(INCLUDES) -I$(LIB)/Samples/Unix/BebopPilotingNewAPI/ -o $@ -c $< $(CFLAGS)
	
BebopPiloting.o:
	$(CC) $(INCLUDES) -I$(LIB)/Samples/Unix/BebopPilotingNewAPI/ -o $@ -c $(LIB)/Samples/Unix/BebopPilotingNewAPI/BebopPiloting.c $(CFLAGS)
	
clean :
	rm *.o *~ $(EXEC)
	
