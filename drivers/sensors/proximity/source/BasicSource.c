

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include "BasicSource.h"

static int readEvent(struct ProximityEventHandler *handler, char *client_name){
    BasicSource *__this = container_of(handler, BasicSource, handler);
//printk("!!! %s readEvent +++ !!!\n\n", __FUNCTION__);

    return __this->event;
}

static void onEvent(struct ProximityEventHandler *handler, const char *client_name, int event){
	BasicSource *__this = container_of(handler, BasicSource, handler);
//printk("!!! %s onEvent +++ !!!\n\n", __FUNCTION__);
	if(__this->event != event){
		__this->event = event;
		if(NULL != __this->watcher){
            
//printk(KERN_INFO"%s:Send Proximity Event %d\n", client_name, __this->event);
//printk("!!! %s onEvent --- !!!\n\n", __FUNCTION__);
			__this->watcher->onEventChanged(&__this->parent, __this->event);
		}
	}    
}

static void enable(ProximitySource *_this, bool enabled)
{
    BasicSource *__this = container_of(_this, BasicSource, parent);

    __this->enabled = enabled;

    return;
}
static ProximityEventWatcher * getWatcher(struct ProximitySource *_this)
{
    BasicSource *__this = container_of(_this, BasicSource, parent);

    return __this->watcher ;
}
static ProximityEventHandleAlgo * getAlgo(struct ProximitySource *_this)
{
    BasicSource *__this = container_of(_this, BasicSource, parent);

    return __this->algo;

}
static unsigned int  getIndex(struct ProximitySource *_this)
{
    BasicSource *__this = container_of(_this, BasicSource, parent);

    return __this->index;

}

static void construct(
    BasicSource *_this, 
    const char *name,
    ProximityEventWatcher *apWatcher, 
    PROXIMITY_EVENT initEvent,
    unsigned int index,
    ProximityEventHandleAlgo * algo)
{
    _this->name = name;

    _this->enabled = false;
    
    _this->event = initEvent;

    _this->watcher = apWatcher;

    _this->algo = algo;

    _this->handler.onEvent= onEvent;

    _this->handler.readEvent = readEvent;

    _this->index = index;

    _this->parent.enable = enable;

    _this->parent.getWatcher = getWatcher;

    _this->parent.getAlgo = getAlgo;

    _this->parent.getIndex = getIndex;

    return;
}


BasicSource *create_BasicSource(
    const char *name,
    ProximityEventWatcher *apWatcher, 
    PROXIMITY_EVENT initEvent,
    unsigned int index,
    ProximityEventHandleAlgo * algo)
{
    BasicSource *lpBasicSource = NULL;

    lpBasicSource = kzalloc(sizeof(BasicSource), GFP_KERNEL);

    BUG_ON(NULL == lpBasicSource);

    construct(lpBasicSource, 
        name,
        apWatcher, 
        initEvent,
        index,
        algo);

    return lpBasicSource;

}
