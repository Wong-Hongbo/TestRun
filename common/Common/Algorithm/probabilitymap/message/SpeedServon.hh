/*
SpeedServon.hh

This C++ header file defines the NML Messages used for command and status by SPEEDSERVO_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef SPEEDSERVON_HH
#define SPEEDSERVON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define SPEEDSERVO_STATUS_TYPE 21000
#define SPEEDSERVO_BACKWARD_TYPE 21001
#define SPEEDSERVO_FORWARD_TYPE 21002
#define SPEEDSERVO_HALT_TYPE 21003
#define SPEEDSERVO_INIT_TYPE 21004
#define SPEEDSERVO_STOP_VEHICLE_TYPE 21005
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.



// Define the NML Message Classes
class SPEEDSERVO_DATA {
public:
    unsigned int	timeflag;
    LOGIC_STATE     is_speed_control;
    CMD_SOURCE	    source;
    unsigned int	confidence;
    double			expected_speed;
    double			actual_speed;
    double			expected_acc;
    double			actual_acc;
    TRANS_POSITION  trans_pos;
    double          fuel;
    double          brake;
    LOGIC_STATE     brake_initialized_flag;
    LOGIC_STATE     brake_motor_enable_switch;
    LOGIC_STATE     trans_initialized_flag;
    LOGIC_STATE     trans_motor_enable_switch;
};


const int SPEED_KEEPING=0, SPEED_PROFILE=1, OBJECT_FOLLOWING=2, DIRECT_BRAKE_FUEL=3 ;
#define SPEED_PROFILE_TYPE int


typedef struct {
    double  timeflag; //unit: second, get by etime();
    SPEED_PROFILE_TYPE   profile_type;
    float   cruise_speed;
    float   desired_acc;
    float   target_x;
    float   target_y;
    float   target_vx;
    float   target_vy;
    float   desired_brake;
    float   desired_fuel;
    float   gps_speed;
}SPEED_CMD;


// Status Class
class SPEEDSERVO_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	SPEEDSERVO_STATUS();

	// Constructor used by derived classes
	SPEEDSERVO_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    SPEEDSERVO_DATA status_data;
};

// Command Classes

class SPEEDSERVO_BACKWARD : public RCS_CMD_MSG
{
public:

	//Constructor
	SPEEDSERVO_BACKWARD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    SPEED_CMD cmd;
};

class SPEEDSERVO_FORWARD : public RCS_CMD_MSG
{
public:

	//Constructor
	SPEEDSERVO_FORWARD();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    SPEED_CMD cmd;
};

class SPEEDSERVO_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	SPEEDSERVO_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class SPEEDSERVO_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	SPEEDSERVO_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class SPEEDSERVO_STOP_VEHICLE : public RCS_CMD_MSG
{
public:

	//Constructor
	SPEEDSERVO_STOP_VEHICLE();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    SPEED_CMD cmd;
};

// Declare NML format function
extern int SpeedServoFormat(NMLTYPE, void *, CMS *);

#endif 	// SPEEDSERVON_HH
