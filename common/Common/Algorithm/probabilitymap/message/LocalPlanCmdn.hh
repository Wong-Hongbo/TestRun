#ifndef LOCALPLANCMDN_HH
#define LOCALPLANCMDN_HH


// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"
#include "LocalPathInfon.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define LOCALPLANCMD_MSG_TYPE 170450
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.

// Define the NML Message Classes


typedef struct {
    double x,y;
    double st,sp;
    double fuel,brake;
    double ddata[10];
    int    idata[20];
}FUTURE_POINT;
class LOCALPLANCMD_MSG : public NMLmsgEx
{
public:

    //Constructor
    LOCALPLANCMD_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    int MessageID;
   // int MessageSeqNum;
    int TimeFlag;

    int tag[20];
    int cmd[20];

    FUTURE_POINT futureData[50];

    // inert coordinate
//    PositionData Position;
};

// Declare NML format function
extern int LocalPlanCmdFormat(NMLTYPE, void *, CMS *);



#endif // LOCALPLANCMDN_HH
