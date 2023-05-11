/*
CANIn_1n.hh

This C++ header file defines the NML Messages for CANIN_1
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:50 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef CANIN_1N_HH
#define CANIN_1N_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"// head for CAN device

// Define the integer type ids.
#define CANIN_1_MSG_TYPE 143000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
typedef struct
{
  UINT32 ID;              // 11/29 bit code
  UINT8  MSGTYPE;         // bits of MSGTYPE_*
  UINT8  LEN;             // count of data bytes (0..8)
  UINT8  DATA[8];         // data bytes, up to 8
} CAN_Msg;              // for CAN_WRITE_MSG

typedef struct
{
  CAN_Msg Msg;          // the above message
  UINT32    dwTime;       // a timestamp in msec, read only
  UINT16     wUsec;        // remainder in micro-seconds
} CANRdMsg;            // for CAN_READ_MSG

// Define the NML Message Classes

class CANIN_1_MSG : public NMLmsg
{
public:

	//Constructor
	CANIN_1_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
//	char 	CAN_data[24];//buf for CAN data
    CANRdMsg CAN_data;

};

// Declare NML format function
extern int CANIn_1Format(NMLTYPE, void *, CMS *);

#endif 	// CANIN_1N_HH
