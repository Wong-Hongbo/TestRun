#ifndef GENERATEDPATHN_HH
#define GENERATEDPATHN_HH


// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "usertype.hh"
#include "NMLmsgExn.hh"

// Trying to merge the type ids often results in redefinn the ID twice..
// RCS-Design-MERGE-DISABLE Edits to the following area will NOT be preserved by the RCS-Design tool.


// Define the integer type ids.
#define GENERATEDPATHINFO_MSG_TYPE 182000
// RCS-Design-MERGE-ENABLE Edits after this line will be preserved by the RCS-Design tool.
#define MAX_LEN 500
typedef struct
{
    int pts_num;
    double cnt_x[MAX_LEN];
    double cnt_y[MAX_LEN];
    double speed[MAX_LEN];

}SimplePath;

class GENERATEDPATHINFO_MSG : public NMLmsgEx
{
public:

    //Constructor
    GENERATEDPATHINFO_MSG();

    // CMS Update Function
    void update(CMS *);

    // Place custom variables here.
    SimplePath path;
};

// Declare NML format function
extern int GeneratedPathInfoFormat(NMLTYPE, void *, CMS *);



#endif // GENERATEDPATHN_HH
