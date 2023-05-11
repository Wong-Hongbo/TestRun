// Prevent Multiple Inclusion
#ifndef MAPPER_STATE_HH
#define MAPPER_STATE_HH

// Include Files
#include "rcs.hh" 	// Common RCS definitions
#include "CommonDefinitionX.hh"
#include "NMLmsgExn.hh"

// Define the integer type ids.
#define MAPPERSTATE_MSG_TYPE 071422

// Define the NML Message Classes
class MAPPERSTATE_MSG : public NMLmsgEx
{
public:
    //Constructor
    MAPPERSTATE_MSG();

    // CMS Update Function
    void update(CMS *);
    UINT8 Mapper_State;   // 1:正在建图 2:结束建图  3:保存地图
    long time;
};

// Declare NML format function
extern int MapperStateFormat(NMLTYPE, void *, CMS *);

#endif 	// MAPPER_STATE_HH
