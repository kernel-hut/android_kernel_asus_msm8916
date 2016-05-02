#ifndef __PROXIMITYALGOFACTORY_H__
#define __PROXIMITYALGOFACTORY_H__

#include <linux/ProximityBasic.h>
#include "../ProximityEventHandleAlgo.h"   

extern ProximityEventHandleAlgo* getProximityAlgo(
    PROXIMITY_ALGO_TYPE type
);

#endif //__PROXIMITYALGOFACTORY_H__

