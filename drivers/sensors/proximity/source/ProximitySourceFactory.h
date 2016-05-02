#ifndef __PROXIMITYSOURCEFACTORY_H__
#define __PROXIMITYSOURCEFACTORY_H__

#include <linux/ProximityBasic.h>
#include "../ProximitySource.h"
#include "../ProximityEventHandleAlgo.h"   

extern ProximitySource* getProximitySource(
    proximity_resource *resource,
    ProximityEventWatcher *watcher,
    ProximityEventHandleAlgo * algo
);

extern void create_ProximityTest_proc_file(
    const char *name, ProximityEventHandler *handler);

extern void setCap1106Handler(const char *name, ProximityEventHandler *handler);

#endif //__PROXIMITYSOURCEFACTORY_H__

