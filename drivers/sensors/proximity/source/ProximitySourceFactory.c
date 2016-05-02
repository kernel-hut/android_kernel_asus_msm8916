#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include "ProximitySourceFactory.h"
#include "BasicSource.h"

ProximitySource* getProximitySource(
    proximity_resource *resource,
    ProximityEventWatcher *watcher,
    ProximityEventHandleAlgo * algo)
{
    ProximitySource *source = NULL;

    switch (resource->type){
        case PROXIMITY_FILE_SOURCE:
            {
                    BasicSource *fileSource = create_BasicSource( \
                        resource->name,
                        watcher, 
                        resource->initEventState,
                        resource->index,
                        algo);

                    create_ProximityTest_proc_file(resource->name, &fileSource->handler);

                    if(NULL == fileSource){
                        goto create_err;
                    }

                    return &fileSource->parent;
            }
            break;

        case PROXIMITY_CAP1106_SOURCE:
            {
                    BasicSource *fileSource = create_BasicSource( \
                        resource->name,
                        watcher, 
                        resource->initEventState,
                        resource->index,
                        algo);

                    create_ProximityTest_proc_file(resource->name,&fileSource->handler);
                    setCap1106Handler(resource->name,&fileSource->handler);

                    if(NULL == fileSource){
                        goto create_err;
                    }

                    return &fileSource->parent;
            }

            break;                

        default :
            goto create_err;
            break;
    }

    return source;
create_err:
    printk(KERN_ERR "Failed to get proximity source, when type =%d\n", resource->type);               
    return source;
}

