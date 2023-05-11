/*
CANProcess_1n.hh

This C++ header file defines the NML Messages used for command and status by CANPROCESS_1_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:52 CST 2013	Modified by rcsdesign.
Thu Apr 25 09:53:43 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef CANPROCESS_1N_HH
#define CANPROCESS_1N_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define CANPROCESS_1_STATUS_TYPE 39000
#define CANPROCESS_1_HALT_TYPE 39001
#define CANPROCESS_1_INIT_TYPE 39002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
typedef struct
{
  UINT32    open_time;
  bool      is_open;
  UINT32    error;
  UINT32    send_count;
  UINT32    receive_count;
} CAN_STATUS;

// Status Class
class CANPROCESS_1_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	CANPROCESS_1_STATUS();

	// Constructor used by derived classes
	CANPROCESS_1_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CAN_STATUS status_data;
};

// Command Classes

class CANPROCESS_1_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	CANPROCESS_1_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class CANPROCESS_1_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	CANPROCESS_1_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int CANProcess_1Format(NMLTYPE, void *, CMS *);

#endif 	// CANPROCESS_1N_HH
