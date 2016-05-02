#include <linux/kernel.h>
#include <linux/switch.h>
#include <linux/mutex.h>
#include "BodySarNotifyAlgo.h"
#include "../util/TicketSystem.h"
#define LOGPREFIX "[BodySarNotifyAlgo]" 
static struct switch_dev body_sar;
static struct mutex gBodySarLock;
static TicketSystem *gTicketSystem = NULL;
static void handle(ProximityEventHandleAlgo *algo, 
    PROXIMITY_EVENT event,
    unsigned int ticket)
{

    bool current_event_status;

    bool previous_event_status ;

    mutex_lock(&gBodySarLock);
    
    previous_event_status = gTicketSystem->hasAnyReservation(gTicketSystem);
    
    if(PROXIMITY_EVNET_NEAR == event){

        gTicketSystem->reserveByTicket(gTicketSystem, ticket);
            
    }else{

        gTicketSystem->cancelByTicket(gTicketSystem, ticket);

    }

    current_event_status = 
       gTicketSystem->hasAnyReservation(gTicketSystem);

    if(current_event_status !=previous_event_status){

        printk(KERN_INFO"%s:Send BodySar Event %d\n",LOGPREFIX, event);

        if(current_event_status){

            switch_set_state(&body_sar, 1);

        }else{

            switch_set_state(&body_sar, 0);
        }
    }

    mutex_unlock(&gBodySarLock);
}
static ssize_t body_sar_switch_state(struct switch_dev *sdev, char *buf)
{
    bool current_event_status;

    int count=0;

    current_event_status = 
       gTicketSystem->hasAnyReservation(gTicketSystem);

    if(current_event_status){
        
        count+= sprintf(buf,"1\r\n");
        
    }else{

        count+= sprintf(buf,"0\r\n");

    }
    
    return count;

}
static ProximityEventHandleAlgo gBodySarNotifyAlgo=
{
    .handle = handle,
};
ProximityEventHandleAlgo * getBodySarNotifyAlgo(void)
{
    static bool inited = false;

    int ret=0;

    if(false == inited){
        
        body_sar.name="bodysar";

        body_sar.print_state=body_sar_switch_state;
        
        ret=switch_dev_register(&body_sar);        

        if (ret < 0){

            pr_err("%s: Unable to register switch dev! %d\n", __FUNCTION__,ret);

        }

        mutex_init(&gBodySarLock);

        gTicketSystem = getTicketSystem(TICKET_SYSTEM_BIT_IMPL_TYPE);

        inited = true;

    }

    return &gBodySarNotifyAlgo;
}

