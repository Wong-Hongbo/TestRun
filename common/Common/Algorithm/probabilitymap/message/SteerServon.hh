/*
SteerServon.hh

This C++ header file defines the NML Messages used for command and status by STEERSERVO_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef STEERSERVON_HH
#define STEERSERVON_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Predefined type files
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define STEERSERVO_STATUS_TYPE 15000
#define STEERSERVO_GOATCIRCLE_TYPE 15001
#define STEERSERVO_GOSTRAIGHT_TYPE 15002
#define STEERSERVO_HALT_TYPE 15003
#define STEERSERVO_INIT_TYPE 15004
#define STEERSERVO_GOBACKWARD_TYPE  15005
//  RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.


// Define the NML Message Classes
class STEERSERVO_DATA {
public:
    unsigned int	timeflag;
    LOGIC_STATE     is_steer_control;
    CMD_SOURCE	    source;
    unsigned int	confidence;
    double			expected_curvature;
    double			actual_curvature;
    double          actual_steer;
    double          expected_steer;
    LOGIC_STATE     steer_initialized_flag;
    LOGIC_STATE     steer_motor_enable_switch;
};

const int CURVATURE_ERROR=0, CURVATURE_CONTROL=1, JERK_CONTROL=2,DIRECT_STEER=3;
#define STEER_PROFILE_TYPE int

typedef struct {
    double  timeflag; //unit: second, get by etime();
    STEER_PROFILE_TYPE profile_type;
    float   desired_curvature;
    float   desired_jerk;
    float   speed_limit;
    float   desired_steer;
    float   y_error;
}STEERSERVO_CMD;

// Status Class
class STEERSERVO_STATUS : public RCS_STAT_MSG
{
public:

	// Normal Constructor
	STEERSERVO_STATUS();

	// Constructor used by derived classes
	STEERSERVO_STATUS(NMLTYPE t, size_t s) :  RCS_STAT_MSG(t,s) {};

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    STEERSERVO_DATA status_data;
};

// Command Classes

class STEERSERVO_GOATCIRCLE : public RCS_CMD_MSG
{
public:

	//Constructor
	STEERSERVO_GOATCIRCLE();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
   STEERSERVO_CMD cmd;
};


class STEERSERVO_GOBACKWARD: public RCS_CMD_MSG
{
public:

    //Constructor
    STEERSERVO_GOBACKWARD();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
   STEERSERVO_CMD cmd;
};

class STEERSERVO_GOSTRAIGHT : public RCS_CMD_MSG
{
public:

	//Constructor
	STEERSERVO_GOSTRAIGHT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
};

class STEERSERVO_HALT : public RCS_CMD_MSG
{
public:

	//Constructor
	STEERSERVO_HALT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class STEERSERVO_INIT : public RCS_CMD_MSG
{
public:

	//Constructor
	STEERSERVO_INIT();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

// Declare NML format function
extern int SteerServoFormat(NMLTYPE, void *, CMS *);

#endif 	// STEERSERVON_HH
