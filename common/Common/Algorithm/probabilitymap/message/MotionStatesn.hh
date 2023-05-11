/*
MotionStatesn.hh

This C++ header file defines the NML Messages for MOTIONSTATES
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef MOTIONSTATESN_HH
#define MOTIONSTATESN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define MOTIONSTATES_MSG_TYPE 147000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes
typedef struct {
    double      GetMsgTime;
    float	    rl_dis_real;   // rear left wheel odometer   unit: m  no considering the direction
    float       rr_dis_real;   // rear right wheel odometer  unit: m
    float	    fl_dis_real;   // front left wheel odometer  unit: m
    float       fr_dis_real;   // front right wheel odometer unit: m
    float		fl_speed_real; //velocity    unit:m/s
    float		fr_speed_real;
    float		rl_speed_real;
    float		rr_speed_real;
    float       wheel_speed;
    float       X_acc;              //Acceleration of X axis  unit: m/s^2
    float       Y_acc;              //Acceleration of Y axis  unit: m/s^2
    float       Z_acc;              //Acceleration of Z axis  unit: m/s^2
    float       Yaw_rate;           // gyroscope  unit: deg/s
    int         Driving_direction;  // Driving direction: 1:Forward -1:Backward
    unsigned int    frame_id;
}MOTION_STATES;

class MOTIONSTATES_MSG : public NMLmsg
{
public:

	//Constructor
	MOTIONSTATES_MSG();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.
    MOTION_STATES data;
};

// Declare NML format function
extern int MotionStatesFormat(NMLTYPE, void *, CMS *);

#endif 	// MOTIONSTATESN_HH
