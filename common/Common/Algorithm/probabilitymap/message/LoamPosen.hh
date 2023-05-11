
// Prevent Multiple Inclusion
#ifndef LOAMPOSEN_HH
#define LOAMPOSEN_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "GlobalPositionInfon.hh"


// Define the integer type ids.
#define LOAMPOSE_MSG_TYPE 99788
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

class LOAMPOSE_MSG : public NMLmsgEx
{
public:
    //Constructor
    LOAMPOSE_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    double local_PC_time;
    double x;
    double y;
    double z;  // 
    double azimuth;   // 
    double pitch; //
    double roll;       
};

// Declare NML format function
extern int LoamPoseFormat(NMLTYPE, void *, CMS *);

#endif 	// TRAFFICSIGNN_HH
