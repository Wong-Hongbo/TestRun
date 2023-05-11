#ifndef USERTYPE_HH
#define USERTYPE_HH

#include "CommonDefinitionX.hh"

//#ifndef pi
//#define pi 3.141592653874
//#endif
//#ifndef Pi
//#define Pi 3.141592653874
//#endif
//#ifndef PI
//#define PI 3.141592653874
//#endif

#define 	MIN(a,b)   				((a)<=(b)?(a):(b))
#define 	MAX(a,b)   				((a)>=(b)?(a):(b))
#define 	_MIN(a,b)   				((a)<=(b)?(a):(b))
#define 	_MAX(a,b)   				((a)>=(b)?(a):(b))


//#ifndef bool
//  #define bool  int
//  #define true  1
//  #define false 0
//#endif

//const int  DIRECT_ACTUATOR=0,REMOTE_PILOT=1,AUTO_PILOT=2;
#define CMD_SOURCE int

//const int      OFF=0,ON=1;
#define LOGIC_STATE int

const int      TRANS_PARK=7,TRANS_BACKWARD=6,TRANS_NEURAL=5,TRANS_FORWARD=4,TRANS_UNKNOWN=3;
#define TRANS_POSITION int

const int    AUTONOMOUS = 0,SEMI_AUTO = 1, MASTER_SLAVE = 2,GLOBAL_PATH = 3;

#define REMOTE_ENGINE_RUN 0x03
#define REMOTE_ENGINE_STOP 0x00
#define REMOTE_ENGINE_MASK 0x03

#define REMOTE_VEHICLE_STOP 0x00
#define REMOTE_VEHICLE_FOREWARD 0x04
#define REMOTE_VEHICLE_BACKWARD 0x08
#define REMOTE_VEHICLE_PARK     0x0c
#define REMOTE_TRANS_MASK       0x0c

#define REMOTE_EMERGENCY_STOP_ON 0xf0
#define REMOTE_EMERGENCY_STOP_OFF 0x00
#define REMOTE_ENERGENCY_STOP_MASK 0xf0

#define REMOTE_LIGHT_ON 0x300
#define REMOTE_LIGHT_OFF 0x000
#define REMOTE_LIGHT_MASK 0x300

#define REMOTE_HORN_ON 0xc00
#define REMOTE_HORN_OFF 0x000
#define REMOTE_HORN_MASK 0xc00

#define REMOTE_LEFT_LAMP_ON  0x1000
#define REMOTE_LEFT_LAMP_OFF 0x0000
#define REMOTE_LEFT_LAMP_MASK 0x1000

#define REMOTE_RIGHT_LAMP_ON  0x2000
#define REMOTE_RIGHT_LAMP_OFF 0x0000
#define REMOTE_RIGHT_LAMP_MASK 0x2000

#endif // USERTYPE_HH
