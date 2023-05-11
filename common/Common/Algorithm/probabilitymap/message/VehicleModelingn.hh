/*
VehicleModelingn.hh

This C++ header file defines the NML Messages used for command and status by VEHICLEMODELING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:52 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef VEHECLEMODELINGN_HH
#define VEHECLEMODELINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define VEHICLEMODELING_STATUS_TYPE 11000
#define VEHICLEMODELING_HALT_TYPE 11001
#define VEHICLEMODELING_INIT_TYPE 11002
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
typedef struct
{
  UINT32    cycle_count;
  bool      is_steer_sensor_ok;
  bool      is_motion_sensor_ok;
  bool      is_brake_sensor_ok;
  bool      is_fuel_sensor_ok;
  bool      is_trans_sensor_ok;
} MODELING_STATUS;
// Status Class
class VEHICLEMODELING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	VEHICLEMODELING_STATUS();

	// Constructor used by derived classes
	VEHICLEMODELING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    	MODELING_STATUS status_data;
};

// Command Classes

class VEHICLEMODELING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	VEHICLEMODELING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class VEHICLEMODELING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	VEHICLEMODELING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int VehicleModelingFormat(NMLTYPE, void *, CMS *);

#endif 	// VEHICLEMODELINGN_HH
