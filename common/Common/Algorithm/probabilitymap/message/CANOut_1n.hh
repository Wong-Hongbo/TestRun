/*
CANOut_1n.hh

This C++ header file defines the NML Messages for CANOUT_1
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:50 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef CANOUT_1N_HH
#define CANOUT_1N_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"// head for CAN USB device
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define CANOUT_1_MSG_TYPE 142000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
typedef struct
{
  UINT32 ID;              // 11/29 bit code
  UINT8  MSGTYPE;         // bits of MSGTYPE_*
  UINT8  LEN;             // count of data bytes (0..8)
  UINT8  DATA[8];         // data bytes, up to 8
} CANMsg;              // for CAN_WRITE_MSG


// Define the NML Message Classes

class CANOUT_1_MSG : public NMLmsg
{
public:

	//Constructor
	CANOUT_1_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
//	char 	CAN_data[16];//buf for CAN data
    CANMsg   CAN_data;
};

// Declare NML format function
extern int CANOut_1Format(NMLTYPE, void *, CMS *);

#endif 	// CANOUT_1N_HH
