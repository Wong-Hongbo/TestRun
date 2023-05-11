/*
LocalPlanStaten.hh

This C++ header file defines the NML Messages for LocalPlanStaten
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOCALPLANSTATEN_HH
#define LOCALPLANSTATEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.
#include "NMLmsgExn.hh"
#include "TaskListn.hh"


// Define the integer type ids.
#define LOCALPLANSTATE_MSG_TYPE 1104
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes

class LOCALPLANSTATE_MSG : public NMLmsgEx
{
public:
	//Constructor
	LOCALPLANSTATE_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    bool NeedReplan;
    bool IsArrive;

    double timeNow,timeInput;
    int  timeUsed,sequcence;
    int  currentTask,taskStage,taskQuality;
    int     sysState,decisionState;
    int     planState;
    int  noData,ifManual;
};

// Declare NML format function
extern int LocalPlanStateFormat(NMLTYPE, void *, CMS *);

#endif 	// LOCALPLANSTATEN_HH
