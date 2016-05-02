#ifndef __PROXIMITYFILESOURCE_H__
#define __PROXIMITYFILESOURCE_H__

#include <linux/ProximityBasic.h>
#include <linux/types.h>
#include <linux/list.h>
#include "../ProximitySource.h"
#include "../ProximityEventHandleAlgo.h"

#define TRUE  1
#define FALSE  0

struct BasicSource;

typedef struct BaiscSource{
    ProximitySource parent;
    ProximityEventHandler handler;
//operator member
    
//data member
    const char *name;
    bool enabled;
    PROXIMITY_EVENT event;
    ProximityEventWatcher *watcher;
    ProximityEventHandleAlgo * algo;
    unsigned int index;
    struct list_head list;
}BasicSource;

extern BasicSource *create_BasicSource(
    const char *name,
    ProximityEventWatcher *apWatcher, 
    PROXIMITY_EVENT initEvent,
    unsigned int index,
    ProximityEventHandleAlgo * algo);
#endif //__PROXIMITYFILESOURCE_H__

