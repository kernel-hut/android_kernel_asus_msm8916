#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include "ProximityAlgoFactory.h"
#include "DebugInfoAlgo.h"
#include "BodySarNotifyAlgo.h"

ProximityEventHandleAlgo* getProximityAlgo(
    PROXIMITY_ALGO_TYPE type)
{
    ProximityEventHandleAlgo *algo = NULL;

    switch (type){
        case PROXIMITY_DEBUG_ALGO_TYPE:
            {
                    algo = getDebugInfoAlgo();

                    if(NULL == algo){
                        goto create_err;
                    }
            }
            break;
        case PROXIMITY_BODYSAR_NOTIFY_ALGO_TYPE:
            {
                    algo = getBodySarNotifyAlgo();

                    if(NULL == algo){
                        goto create_err;
                    }
            }            
            break;                
        default :
            goto create_err;
            break;
    }

    return algo;
create_err:
    printk(KERN_ERR "Failed to get proximity algo, when type =%d\n", type);               
    return algo;
}

