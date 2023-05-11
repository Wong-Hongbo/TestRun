/*
Steeringn.hh

This C++ header file defines the NML Messages used for command and status by STEERING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef STEERINGN_HH
#define STEERINGN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files

#include "usertype.hh"

// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define STEERING_STATUS_TYPE 16000
#define STEERING_HALT_TYPE 16001
#define STEERING_INIT_TYPE 16002
#define STEERING_STEERTO_TYPE 16003
#define STEERING_MOTORINIT_TYPE 16004
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
class STEER_STATUS {
public:
    unsigned int      timeflag;
    CMD_SOURCE        source;
    double            value;
    unsigned int      confidence;
    double            desired;
    double            actual;
    LOGIC_STATE       is_initialized;
    LOGIC_STATE       motor_enable_switch;
    LOGIC_STATE       motor_status;
    LOGIC_STATE       CAN_status;
};

typedef struct {
    double steer_position; //  degree
    double steer_velocity;// degree/s
    double steer_acc;     // degree/s^2
    double steer_dec;     // degree/s^2
    CMD_SOURCE        source;
}STEER_CMD;
// Status Class
class STEERING_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	STEERING_STATUS();

	// Constructor used by derived classes
	STEERING_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    STEER_STATUS status_data;
};

// Command Classes

class STEERING_MOTORINIT : public RCS_CMD_MSG
{
public:

    //Constructor
    STEERING_MOTORINIT();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    CMD_SOURCE        source;
};

class STEERING_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	STEERING_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
};

class STEERING_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	STEERING_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
};

class STEERING_STEERTO : public RCS_CMD_MSG
{
public:

	//Constructor
	STEERING_STEERTO();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    STEER_CMD cmd;
};

// Declare NML format function
extern int SteeringFormat(NMLTYPE, void *, CMS *);

#endif 	// STEERINGN_HH
