#ifndef __PROXIMITYEVENTHANDLEALOG_H__
#define __PROXIMITYEVENTHANDLEALOG_H__

#include <linux/ProximityBasic.h>

struct ProximityEventHandleAlgo;

typedef struct ProximityEventHandleAlgo{
    void(*handle)(struct ProximityEventHandleAlgo *algo, 
        PROXIMITY_EVENT event,
        unsigned int ticket);
}ProximityEventHandleAlgo;

#endif //__PROXIMITYEVENTHANDLEALOG_H__

