#ifndef __PROXIMITYSOURCE_H__
#define __PROXIMITYSOURCE_H__

#include <linux/ProximityBasic.h>
#include <linux/list.h>
#include "ProximityEventHandleAlgo.h"

struct ProximitySource ;
struct ProximityEventWatcher;

typedef struct ProximityEventWatcher{
    void(*onEventChanged)(struct ProximitySource *apSource, PROXIMITY_EVENT event);
}ProximityEventWatcher;


typedef struct ProximitySource{
    void(*enable)(struct ProximitySource *_this, bool enabled);
    ProximityEventWatcher * (*getWatcher)(struct ProximitySource *_this);
    ProximityEventHandleAlgo * (*getAlgo)(struct ProximitySource *_this);    
    unsigned int  (*getIndex)(struct ProximitySource *_this);    
    struct list_head list; //should not be access by ProximitySource
}ProximitySource;

#endif //__PROXIMITYSOURCE_H__
