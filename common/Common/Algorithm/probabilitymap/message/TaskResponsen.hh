/*
TaskResponsen.hh

This C++ header file defines the NML Messages for TASKRESPONSE
Template Version 1.1

MODIFICATIONS:
Wed Jul 03 00:22:04 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef TASKRESPONSEN_HH
#define TASKRESPONSEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.
#include "NMLmsgExn.hh"


// Define the integer type ids.
#define TASKRESPONSE_MSG_TYPE 3304
#define TASKRESPONSE_MSG_SIZE 4096
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


class TASKRESPONSE_MSG : public NMLmsgEx
{
public:

    //Constructor
    TASKRESPONSE_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    //PositionData Position;
    int msg_id;
    int len;
    char data[TASKRESPONSE_MSG_SIZE];
    int seq_id;
    long timestamp;
//    char route_path[1024];
//    char path_valid; // -1, not valid this time; 0, there is no valid path; 1 valid path;
//    int station_id;
//    char control_type;
//    char success_for_control_type; //1,success; 0, not success; -1, not valid this time
//    char is_arrive; //-1, not valid this thime; 0, not arrive; 1, arrive;
};

// Declare NML format function
extern int TaskResponseFormat(NMLTYPE, void *, CMS *);

#endif 	// TASKRESPONSEN_HH
