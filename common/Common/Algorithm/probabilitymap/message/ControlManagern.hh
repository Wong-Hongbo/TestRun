/*
ControlManagern.hh

This C++ header file defines the NML Messages used for command and status by CONTROLMANAGER_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:52 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef CONTROLMANAGERN_HH
#define CONTROLMANAGERN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define CONTROLMANAGER_STATUS_TYPE 1000
#define CONTROLMANAGER_HALT_TYPE 1001
#define CONTROLMANAGER_INIT_TYPE 1002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
typedef struct
{
  UINT32    cycle_count;
  bool      is_can_bus_ok[2];
  bool      is_ethernet_ok[4];
  bool      is__path_tracking_ok;
  bool      is_local_sensor_ok;
  bool      is_global_sensor_ok;
  bool      is_steering_servo_ok;
  bool      is_speed_servo_ok;
  bool      is_steer_ok;
  bool      is_brake_ok;
  bool      is_fuel_ok;
  bool      is_trans_ok;
  bool      is_logic_ok;
} CONTROL_STATUS;
// Status Class
class CONTROLMANAGER_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	CONTROLMANAGER_STATUS();

	// Constructor used by derived classes
	CONTROLMANAGER_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    CONTROL_STATUS status_data;
};

// Command Classes

class CONTROLMANAGER_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	CONTROLMANAGER_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class CONTROLMANAGER_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	CONTROLMANAGER_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int ControlManagerFormat(NMLTYPE, void *, CMS *);

#endif 	// CONTROLMANAGERN_HH
