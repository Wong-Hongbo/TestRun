#ifndef LOCALPLANMONITORN_HH
#define LOCALPLANMONITORN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "NMLmsgExn.hh"

#define LOCAL_PATH_MONITOR_CMD_MSG_TYPE 64000
#define LOCAL_PATH_MONITOR_STATUS_MSG_TYPE 64001


// Define the NML Message Classes

// Status Class
class LOCAL_PATH_MONITOR_STATUS_MSG : public NMLmsgEx
{
public:

    // Normal Constructor
    LOCAL_PATH_MONITOR_STATUS_MSG();

    // Constructor used by derived classes

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    int startFrameNum;
    int totalFrame;
    int currentFrame;
};

// Command Classes

class LOCAL_PATH_MONITOR_CMD_MSG : public NMLmsgEx
{
public:

    //Constructor
    LOCAL_PATH_MONITOR_CMD_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    char cmdMsgSave;  // 0-no command, 1-save, 2-save stop,

    char cmdOffline;  //0-no command, 1-work offline, 2-sleep offline, 3 pause;
                      //4-continue;5-a setp forwardsetp;6-a step backwardstep
                      //7-jump to frameNum2go

    char cmdOnlineProcess;// 0-no command, 1-work online, 2-sleep online

    char cmdSetLoadPath;  // 0-no command, 1-set path
    char cmdSetSavePath;  // 0-no command, 1-set path

    char savePath[100];
    char savedFileName[100];

    int  frameNum2go;    // 0-no command, >0 ,number to go
};

// Declare NML format function
extern int LocalPathMonitorFormat(NMLTYPE, void *, CMS *);


#endif // LOCALPLANMONITORN_HH
