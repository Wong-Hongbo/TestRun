/*
LOGICCMDn.hh

This C++ header file defines the NML Messages used for command for  LOGICSETING_MODULE
Template Version 1.1

MODIFICATIONS:
Thu Apr 25 09:55:53 CST 2013	Created by rcsdesign.

*/

// Prevent Multiple Inclusion
#ifndef LOGICCMDN_HH
#define LOGICCMDN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions

// Predefined type files
#include "usertype.hh"


// Define the integer type ids.
//  RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.

#define LOGICCMD_LEFT_LIGHT_OFF_TYPE 20101
#define LOGICCMD_LEFT_LIGHT_ON_TYPE 20102
#define LOGICCMD_RIGHT_LIGHT_OFF_TYPE 20103
#define LOGICCMD_RIGHT_LIGHT_ON_TYPE 20104
#define LOGICCMD_VEHICLE_CLOSING_TYPE 20105
#define LOGICCMD_HEADLIGHT_OFF_TYPE 20108
#define LOGICCMD_HEADLIGHT_ON_TYPE 20109
#define LOGICCMD_HORN_OFF_TYPE 20110
#define LOGICCMD_HORN_ON_TYPE 20111
#define LOGICCMD_VEHICLE_ON_TYPE 20112
#define LOGICCMD_STARTER_ON_TYPE 20113
#define LOGICCMD_STARTER_OFF_TYPE 20114
#define LOGICCMD_HAND_BRAKE_RELEASE_TYPE 20116
#define LOGICCMD_HAND_BRAKE_ON_TYPE 20118
#define LOGICCMD_ALTERNATOR_OFF_TYPE 20119
#define LOGICCMD_ALTERNATOR_ON_TYPE 20120


class LOGICCMD_HORN_ON : public NMLmsg
{
public:

	//Constructor
    LOGICCMD_HORN_ON();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOGICCMD_HORN_OFF: public NMLmsg
{
public:

    //Constructor
    LOGICCMD_HORN_OFF();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};

class LOGICCMD_LEFT_LIGHT_OFF : public NMLmsg
{
public:

	//Constructor
    LOGICCMD_LEFT_LIGHT_OFF();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOGICCMD_LEFT_LIGHT_ON : public NMLmsg
{
public:

	//Constructor
    LOGICCMD_LEFT_LIGHT_ON();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOGICCMD_RIGHT_LIGHT_OFF : public NMLmsg
{
public:

	//Constructor
    LOGICCMD_RIGHT_LIGHT_OFF();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOGICCMD_RIGHT_LIGHT_ON : public NMLmsg
{
public:

	//Constructor
    LOGICCMD_RIGHT_LIGHT_ON();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOGICCMD_VEHICLE_CLOSING : public NMLmsg
{
public:

	//Constructor
    LOGICCMD_VEHICLE_CLOSING();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};

class LOGICCMD_VEHICLE_ON : public NMLmsg
{
public:

    //Constructor
    LOGICCMD_VEHICLE_ON();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};

class LOGICCMD_STARTER_ON : public NMLmsg
{
public:

	//Constructor
    LOGICCMD_STARTER_ON();

	// CMS Update Function
	void update(CMS *);

	// Place custom variables here.

};
class LOGICCMD_STARTER_OFF : public NMLmsg
{
public:

    //Constructor
    LOGICCMD_STARTER_OFF();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};

class LOGICCMD_HEADLIGHT_ON : public NMLmsg
{
public:

    //Constructor
    LOGICCMD_HEADLIGHT_ON();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};
class LOGICCMD_HEADLIGHT_OFF: public NMLmsg
{
public:

    //Constructor
    LOGICCMD_HEADLIGHT_OFF();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};

class LOGICCMD_HAND_BRAKE_RELEASE : public NMLmsg
{
public:

    //Constructor
    LOGICCMD_HAND_BRAKE_RELEASE();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};

class LOGICCMD_HAND_BRAKE_ON : public NMLmsg
{
public:

    //Constructor
    LOGICCMD_HAND_BRAKE_ON();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};

class LOGICCMD_ALTERNATOR_ON : public NMLmsg
{
public:

    //Constructor
    LOGICCMD_ALTERNATOR_ON();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};
class LOGICCMD_ALTERNATOR_OFF: public NMLmsg
{
public:

    //Constructor
    LOGICCMD_ALTERNATOR_OFF();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.

};

// Declare NML format function
extern int LogicCmdFormat(NMLTYPE, void *, CMS *);

#endif 	// LOGICCMDN_HH
