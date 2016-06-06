
/**
 * @file Move.c
 * @brief This file contains sources about basic drone movements, such as go forward or up
 * @date 06/06/2016
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/

#include <stdlib.h>
#include <curses.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>

#include <libARSAL/ARSAL.h>
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>

#include "Move.h"
#include "ihm.h"

/*****************************************
 *
 *             define :
 *
 *****************************************/
#define TAG "Mouvement"

#define ERROR_STR_LENGTH 2048

#define BEBOP_IP_ADDRESS "192.168.42.1"
#define BEBOP_DISCOVERY_PORT 44444

#define DISPLAY_WITH_MPLAYER 1

#define FIFO_DIR_PATTERN "/tmp/arsdk_XXXXXX"
#define FIFO_NAME "arsdk_fifo"

#define VMAX 3	//en m/s
#define VTURNMAX 15 // en °/s

#define IHM
/*****************************************
 *
 *             private header:
 *
 ****************************************/


/*****************************************
 *
 *             implementation :
 *
 *****************************************/

static char fifo_dir[] = FIFO_DIR_PATTERN;
static char fifo_name[128] = "";

int gIHMRun = 1;
char gErrorStr[ERROR_STR_LENGTH];
IHM_t *ihm = NULL;

FILE *videoOut = NULL;
int frameNb = 0;
ARSAL_Sem_t stateSem;
pid_t child = 0;

static void signal_handler(int signal)
{
    gIHMRun = 0;
}

ARCONTROLLER_Device_t* init(void)
{
    // local declarations
    int failed = 0;
    ARDISCOVERY_Device_t *device = NULL;
    ARCONTROLLER_Device_t *deviceController = NULL;
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
    eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_DEVICE_STATE_MAX;

    /* Set signal handlers */
    struct sigaction sig_action = {
        .sa_handler = signal_handler,
    };

    int ret = sigaction(SIGINT, &sig_action, NULL);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Unable to set SIGINT handler : %d(%s)",
                    errno, strerror(errno));
        return NULL;
    }
    ret = sigaction(SIGPIPE, &sig_action, NULL);
    if (ret < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Unable to set SIGPIPE handler : %d(%s)",
                    errno, strerror(errno));
        return NULL;
    }


    if (mkdtemp(fifo_dir) == NULL)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkdtemp failed.");
        return NULL;
    }
    snprintf(fifo_name, sizeof(fifo_name), "%s/%s", fifo_dir, FIFO_NAME);

    if(mkfifo(fifo_name, 0666) < 0)
    {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, "ERROR", "Mkfifo failed: %d, %s", errno, strerror(errno));
        return NULL;
    }

    ARSAL_Sem_Init (&(stateSem), 0, 0);

    if (!failed)
    {
        if (DISPLAY_WITH_MPLAYER)
        {
            // fork the process to launch mplayer
            if ((child = fork()) == 0)
            {
                execlp("xterm", "xterm", "-e", "mplayer", "-demuxer",  "h264es", fifo_name, "-benchmark", "-really-quiet", NULL);
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Missing mplayer, you will not see the video. Please install mplayer and xterm.");
                return NULL;
            }
        }

        if (DISPLAY_WITH_MPLAYER)
        {
            videoOut = fopen(fifo_name, "w");
        }
    }

#ifdef IHM
    ihm = IHM_New (&onInputEvent);
    if (ihm != NULL)
    {
        gErrorStr[0] = '\0';
        ARSAL_Print_SetCallback (customPrintCallback); //use a custom callback to print, for not disturb ncurses IHM
    }
    else
    {
        ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of IHM failed.");
        failed = 1;
    }
#endif

    // create a discovery device
    if (!failed)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- init discovery device ... ");
        eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;

        device = ARDISCOVERY_Device_New (&errorDiscovery);

        if (errorDiscovery == ARDISCOVERY_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - ARDISCOVERY_Device_InitWifi ...");
            // create an ARdrone discovery device (ARDISCOVERY_PRODUCT_ARDRONE)
            errorDiscovery = ARDISCOVERY_Device_InitWifi (device, ARDISCOVERY_PRODUCT_ARDRONE, "bebop", BEBOP_IP_ADDRESS, BEBOP_DISCOVERY_PORT);

            if (errorDiscovery != ARDISCOVERY_OK)
            {
                failed = 1;
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Discovery error :%s", ARDISCOVERY_Error_ToString(errorDiscovery));
            failed = 1;
        }
    }

    // create a device controller
    if (!failed)
    {
        deviceController = ARCONTROLLER_Device_New (device, &error);

        if (error != ARCONTROLLER_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "Creation of deviceController failed.");
            failed = 1;
        }
        else
        {
            IHM_setCustomData(ihm, deviceController);
        }
    }

    if (!failed)
    {
        ARDISCOVERY_Device_Delete (&device);
    }

    // add the state change callback to be informed when the device controller starts, stops...
    if (!failed)
    {
        error = ARCONTROLLER_Device_AddStateChangedCallback (deviceController, stateChanged, deviceController);

        if (error != ARCONTROLLER_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add State callback failed.");
            failed = 1;
        }
    }

    // add the command received callback to be informed when a command has been received from the device
    if (!failed)
    {
        error = ARCONTROLLER_Device_AddCommandReceivedCallback (deviceController, commandReceived, deviceController);

        if (error != ARCONTROLLER_OK)
        {
            ARSAL_PRINT (ARSAL_PRINT_ERROR, TAG, "add callback failed.");
            failed = 1;
        }
    }

    // add the frame received callback to be informed when a streaming frame has been received from the device
    if (!failed)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- set Video callback ... ");
        error = ARCONTROLLER_Device_SetVideoStreamCallbacks (deviceController, decoderConfigCallback, didReceiveFrameCallback, NULL , NULL);

        if (error != ARCONTROLLER_OK)
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%", ARCONTROLLER_Error_ToString(error));
        }
    }

    if (!failed)
    {
        IHM_PrintInfo(ihm, "Connecting ...");
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Connecting ...");
        error = ARCONTROLLER_Device_Start (deviceController);

        if (error != ARCONTROLLER_OK)
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
        }
    }

    if (!failed)
    {
        // wait state update update
        ARSAL_Sem_Wait (&(stateSem));

        deviceState = ARCONTROLLER_Device_GetState (deviceController, &error);

        if ((error != ARCONTROLLER_OK) || (deviceState != ARCONTROLLER_DEVICE_STATE_RUNNING))
        {
            failed = 1;
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- deviceState :%d", deviceState);
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
        }
    }

    // send the command that tells to the Bebop to begin its streaming
    if (!failed)
    {
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "- send StreamingVideoEnable ... ");
        error = deviceController->aRDrone3->sendMediaStreamingVideoEnable (deviceController->aRDrone3, 1);
        if (error != ARCONTROLLER_OK)
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "- error :%s", ARCONTROLLER_Error_ToString(error));
            failed = 1;
        }
    }

    if (!failed)
    {
        IHM_PrintInfo(ihm, "Running ... ('q' to quit)");
    }
	return(deviceController);
}

int supp_ihm(ARCONTROLLER_Device_t *deviceController) 
{
    eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
    eARCONTROLLER_DEVICE_STATE deviceState = ARCONTROLLER_DEVICE_STATE_MAX;
#ifdef IHM
    IHM_Delete (&ihm);
#endif

    // we are here because of a disconnection or user has quit IHM, so safely delete everything
    if (deviceController != NULL)
    {


        deviceState = ARCONTROLLER_Device_GetState (deviceController, &error);
        if ((error == ARCONTROLLER_OK) && (deviceState != ARCONTROLLER_DEVICE_STATE_STOPPED))
        {
            IHM_PrintInfo(ihm, "Disconnecting ...");
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Disconnecting ...");

            error = ARCONTROLLER_Device_Stop (deviceController);

            if (error == ARCONTROLLER_OK)
            {
                // wait state update update
                ARSAL_Sem_Wait (&(stateSem));
            }
        }

        IHM_PrintInfo(ihm, "");
        ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "ARCONTROLLER_Device_Delete ...");
        ARCONTROLLER_Device_Delete (&deviceController);

        if (DISPLAY_WITH_MPLAYER)
        {
            fflush (videoOut);
            fclose (videoOut);

            if (child > 0)
            {
                kill(child, SIGKILL);
            }
        }
    }

    ARSAL_Sem_Destroy (&(stateSem));

    unlink(fifo_name);
    rmdir(fifo_dir);

    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "-- END --");

    return EXIT_SUCCESS;
}

/*****************************************
 *
 *             private implementation:
 *
 ****************************************/

// called when the state of the device controller has changed
void stateChanged (eARCONTROLLER_DEVICE_STATE newState, eARCONTROLLER_ERROR error, void *customData)
{
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "    - stateChanged newState: %d .....", newState);

    switch (newState)
    {
    case ARCONTROLLER_DEVICE_STATE_STOPPED:
        ARSAL_Sem_Post (&(stateSem));
        //stop
        gIHMRun = 0;

        break;

    case ARCONTROLLER_DEVICE_STATE_RUNNING:
        ARSAL_Sem_Post (&(stateSem));
        break;

    default:
        break;
    }
}

// called when a command has been received from the drone
void commandReceived (eARCONTROLLER_DICTIONARY_KEY commandKey, ARCONTROLLER_DICTIONARY_ELEMENT_t *elementDictionary, void *customData)
{
    ARCONTROLLER_Device_t *deviceController = customData;

    if (deviceController != NULL)
    {
        // if the command received is a battery state changed
        if (commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED)
        {
            ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;
            ARCONTROLLER_DICTIONARY_ELEMENT_t *singleElement = NULL;

            if (elementDictionary != NULL)
            {
                // get the command received in the device controller
                HASH_FIND_STR (elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY, singleElement);

                if (singleElement != NULL)
                {
                    // get the value
                    HASH_FIND_STR (singleElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT, arg);

                    if (arg != NULL)
                    {
                        // update UI
                        batteryStateChanged (arg->value.U8);
                    }
                    else
                    {
                        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg is NULL");
                    }
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "singleElement is NULL");
                }
            }
            else
            {
                ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
            }
        }
    }

    if (commandKey == ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED)
    {
        ARCONTROLLER_DICTIONARY_ARG_t *arg = NULL;

        if (elementDictionary != NULL)
        {
            ARCONTROLLER_DICTIONARY_ELEMENT_t *dictElement = NULL;
            ARCONTROLLER_DICTIONARY_ELEMENT_t *dictTmp = NULL;

            eARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME sensorName = ARCOMMANDS_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME_MAX;
            int sensorState = 0;

            HASH_ITER(hh, elementDictionary, dictElement, dictTmp)
            {
                // get the Name
                HASH_FIND_STR (dictElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORNAME, arg);
                if (arg != NULL)
                {
                    sensorName = arg->value.I32;
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorName is NULL");
                }

                // get the state
                HASH_FIND_STR (dictElement->arguments, ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_SENSORSSTATESLISTCHANGED_SENSORSTATE, arg);
                if (arg != NULL)
                {
                    sensorState = arg->value.U8;

                    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "sensorName %d ; sensorState: %d", sensorName, sensorState);
                }
                else
                {
                    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "arg sensorState is NULL");
                }
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "elements is NULL");
        }
    }
}

void batteryStateChanged (uint8_t percent)
{
    // callback of changing of battery level

    if (ihm != NULL)
    {
        IHM_PrintBattery (ihm, percent);
    }

}

eARCONTROLLER_ERROR decoderConfigCallback (ARCONTROLLER_Stream_Codec_t codec, void *customData)
{
    if (videoOut != NULL)
    {
        if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264)
        {
            if (DISPLAY_WITH_MPLAYER)
            {
                fwrite(codec.parameters.h264parameters.spsBuffer, codec.parameters.h264parameters.spsSize, 1, videoOut);
                fwrite(codec.parameters.h264parameters.ppsBuffer, codec.parameters.h264parameters.ppsSize, 1, videoOut);

                fflush (videoOut);
            }
        }

    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "videoOut is NULL.");
    }

    return ARCONTROLLER_OK;
}


eARCONTROLLER_ERROR didReceiveFrameCallback (ARCONTROLLER_Frame_t *frame, void *customData)
{
    if (videoOut != NULL)
    {
        if (frame != NULL)
        {
            if (DISPLAY_WITH_MPLAYER)
            {
                fwrite(frame->data, frame->used, 1, videoOut);

                fflush (videoOut);
            }
        }
        else
        {
            ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "frame is NULL.");
        }
    }
    else
    {
        ARSAL_PRINT(ARSAL_PRINT_WARNING, TAG, "videoOut is NULL.");
    }

    return ARCONTROLLER_OK;
}

/* NEW FUNCTIONS */
void land(ARCONTROLLER_Device_t* deviceController)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->sendPilotingLanding(deviceController->aRDrone3);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "Landing failed");
	}
}

void takeoff(ARCONTROLLER_Device_t* deviceController)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}	
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->sendPilotingTakeOff(deviceController->aRDrone3);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "Take Off failed");
	}
}

void goforward(ARCONTROLLER_Device_t* deviceController, int distance)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(distance < 0){ 
		distance *= -1;
	}
	float time = (float) distance/(VMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDPitch(deviceController->aRDrone3, 50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going forward failed");
		return;
	}
        error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going forward failed");
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void gobackward(ARCONTROLLER_Device_t* deviceController, int distance)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(distance < 0){ 
		distance *= -1;
	}
	float time = (float) distance/(VMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDPitch(deviceController->aRDrone3, -50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going backward failed");
		return;
	}
        error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going backward failed");
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void goup(ARCONTROLLER_Device_t* deviceController, int distance)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(distance < 0){ 
		distance *= -1;
	}
	float time = (float) distance/(VMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDGaz(deviceController->aRDrone3, 50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going up failed");
		return;
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void godown(ARCONTROLLER_Device_t* deviceController, int distance)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(distance < 0){ 
		distance *= -1;
	}
	float time = (float) distance/(VMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDGaz(deviceController->aRDrone3, -50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going down failed");
		return;
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void goleft(ARCONTROLLER_Device_t* deviceController, int distance)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(distance < 0){ 
		distance *= -1;
	}
	float time = (float) distance/(VMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDRoll(deviceController->aRDrone3, -50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going left failed");
		return;
	}
        error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going left failed");
		return;
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void goright(ARCONTROLLER_Device_t* deviceController, int distance)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(distance < 0){ 
		distance *= -1;
	}
	float time = (float) distance/(VMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDRoll(deviceController->aRDrone3, 50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going right failed");
		return;
	}
        error = deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "going right failed");
		return;
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void turnright(ARCONTROLLER_Device_t* deviceController, int angle)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(angle < 0){ 
		angle *= -1;
	}
	float time = (float) angle/(VTURNMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDYaw(deviceController->aRDrone3, 50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "turning right failed");
		return;
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void turnleft(ARCONTROLLER_Device_t* deviceController, int angle)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	if(angle < 0){ 
		angle *= -1;
	}
	float time = (float) angle/(VTURNMAX*0.5);
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
	error = deviceController->aRDrone3->setPilotingPCMDYaw(deviceController->aRDrone3, -50);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "turning left failed");
		return;
	}
	sleep(time);
	// distance traveled, now stop movement, send order until it is correctly done
	error = !error;
	while(error != ARCONTROLLER_OK) {
        error = deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, 0, 0, 0);
        }
}

void emergency(ARCONTROLLER_Device_t* deviceController)
{
	if(deviceController == NULL) {
		IHM_PrintInfo(ihm, "The given controller is NULL");
		return;
	}
	eARCONTROLLER_ERROR error = ARCONTROLLER_OK;
            error = deviceController->aRDrone3->sendPilotingEmergency(deviceController->aRDrone3);
	if(error != ARCONTROLLER_OK) {
		IHM_PrintInfo(ihm, "turning left failed");
		return;
	}
}

// IHM callbacks:

void onInputEvent (eIHM_INPUT_EVENT event, void *customData)
{
    // Manage IHM input events
    ARCONTROLLER_Device_t *deviceController = (ARCONTROLLER_Device_t *)customData;

    IHM_PrintInfo(ihm, "IHM_INPUT_EVENT_EXIT ...");
    gIHMRun = 0;

    if (supp_ihm(deviceController) != EXIT_SUCCESS)
    {
        printf("Error sending an event\n");
    }
}

int customPrintCallback (eARSAL_PRINT_LEVEL level, const char *tag, const char *format, va_list va)
{
    // Custom callback used when ncurses is runing for not disturb the IHM

    if ((level == ARSAL_PRINT_ERROR) && (strcmp(TAG, tag) == 0))
    {
        // Save the last Error
        vsnprintf(gErrorStr, (ERROR_STR_LENGTH - 1), format, va);
        gErrorStr[ERROR_STR_LENGTH - 1] = '\0';
    }

    return 1;
}

// permet juste de passer la compilation sans soucis, pourra être utilisé pour tester les fonctions
int main() {
	printf("Je compile bien !\n");
	return EXIT_SUCCESS;
}

