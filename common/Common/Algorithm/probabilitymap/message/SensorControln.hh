/*
SensorControln.hh

This C++ header file defines the NML Messages used for command and status by SENSORCONTROL_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:01 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SENSORCONTROLN_HH
#define SENSORCONTROLN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include <stdio.h>
// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define SENSORCONTROL_STATUS_TYPE 34000
#define SENSORCONTROL_CMD_TYPE 34001
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
struct Save_Cmd
{
    UINT8 Save_ColorImage;  //0-not save,1-save
    UINT8 Save_Ladar64;    //0-not save,1-save
    UINT8 Save_Ladar32;    //0-not save,1-save
    UINT8 Save_Ladar8;    //0-not save,1-save
    UINT8 Save_Radar;    //0-not save,1-save
    UINT8 Save_IRImage;    //0-not save,1-save

    Save_Cmd()
    {
        reset();
    }

    void reset()
    {
        Save_ColorImage = 1;
        Save_Ladar64 = 1;
        Save_Ladar32 = 1;
        Save_Ladar8 = 1;
        Save_Radar = 1;
        Save_IRImage = 1;
    }

	void print()
	{
	    printf("Save_ColorImage :%d.\n", Save_ColorImage);
		printf("Save_Ladar64 :%d.\n", Save_Ladar64);
		printf("Save_Ladar32 :%d.\n", Save_Ladar32);
		printf("Save_Ladar8 :%d.\n", Save_Ladar8);
		printf("Save_Radar :%d.\n", Save_Radar);
		printf("Save_IRImage :%d.\n", Save_IRImage);
		
	}
};

// Define the NML Message Classes

// Status Class
class SENSORCONTROL_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	SENSORCONTROL_STATUS();

	// Constructor used by derived classes
    //SENSORCONTROL_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class SENSORCONTROL_CMD : public NMLmsgEx
{
public:

	//Constructor
	SENSORCONTROL_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT32 StartFrame;
    UINT8 Cmd_NewSave;      // 0-no new save, if this process is saving info, and Cmd_NewSave=1, start a new save
    UINT8 Cmd_SaveType;     // 0-zhengchang 1-xiulu 2-chalu 3-shoufeizhan 4-duche 5-weizhi
    UINT8 Cmd_StartOnline;  // 0-no command, 1-work, 2-sleep
    UINT8 Cmd_SaveData;     // 0-no command, 1-save, 2-save stop
    UINT8 Cmd_StartOffline; // 0-no command, 1-work offline, 2-sleep offline
    UINT8 Cmd_SetLoadPath;  // 0-no command, 1-set path
    UINT8 Cmd_SetSavePath;  // 0-no command, 1-set path
    Save_Cmd Cmd_Save_Sensor;
    UINT8 Path[100];

};

// Declare NML format function
extern int SensorControlFormat(NMLTYPE, void *, CMS *);

#endif 	// SENSORCONTROLN_HH
