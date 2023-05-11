/*
SensorControln.hh

This C++ header file defines the NML Messages used for command and status by SENSORCONTROL_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:01 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SENSORCONTROLFRAMEN_HH
#define SENSORCONTROLFRAMEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define SENSORCONTROL_FRAME_TYPE 34002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.



class SENSORCONTROL_FRAME : public NMLmsgEx
{
public:

	//Constructor
	SENSORCONTROL_FRAME();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    INT64 FrameNumber;

};

// Declare NML format function
extern int SensorControlFrameFormat(NMLTYPE, void *, CMS *);

#endif 	// SENSORCONTROLN_HH
