/*
SceneEnhancen.hh

This C++ header file defines the NML Messages used for command and status by SCENEENHANCE_MODULE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:01 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SCENEENHANCEN_HH
#define SCENEENHANCEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define SCENEENHANCE_STATUS_TYPE 3102
#define SCENEENHANCE_CMD_TYPE 3101
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

// Status Class
class SCENEENHANCE_STATUS : public NMLmsgEx
{
public:

	// Normal Constructor
	SCENEENHANCE_STATUS();

	// Constructor used by derived classes
    //SCENEENHANCE_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Command Classes

class SCENEENHANCE_CMD : public NMLmsgEx
{
public:

	//Constructor
	SCENEENHANCE_CMD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
};

// Declare NML format function
extern int SceneEnhanceFormat(NMLTYPE, void *, CMS *);

#endif 	// SCENEENHANCEN_HH
