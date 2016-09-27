#include "debug.h"
#include "system_config.h"
#include "system_definitions.h"

void dbgOutputVal(unsigned char outVal){
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, 0x00FF);
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, 0x8080);
    
    unsigned int outVal_hex = 0x00;
    outVal_hex |= outVal;
    
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, outVal_hex);    
}

void dbgOutputLoc(unsigned char outVal){
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_E, 0x00FF);
    PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, 0x8080);
    
    unsigned int outVal_hex = 0x00;
    outVal_hex |= outVal;
    
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, outVal_hex);
}

void halt(){
    while(1){
        
    }
}