#include <linux/kernel.h>
#include "DebugInfoAlgo.h"
#include "../util/TicketSystem.h"
#define LOGPREFIX "[DebugInfoAlgo]" 
static bool inited = false; 
static TicketSystem *gTicketSystem = NULL;
static void handle(ProximityEventHandleAlgo *algo, 
    PROXIMITY_EVENT event,
    unsigned int ticket)
{
    bool current_event_status;

    bool previous_event_status = 
        gTicketSystem->hasAnyReservation(gTicketSystem);
    
    if(PROXIMITY_EVNET_NEAR == event){

        gTicketSystem->reserveByTicket(gTicketSystem, ticket);
            
    }else{

        gTicketSystem->cancelByTicket(gTicketSystem, ticket);

    }

    current_event_status = 
       gTicketSystem->hasAnyReservation(gTicketSystem);

    if(current_event_status !=previous_event_status){
        
        printk(KERN_INFO"%s:Send Proximity Event %d\n",LOGPREFIX, event);

    }

}

static ProximityEventHandleAlgo gDebugInfoAlgo=
{
    .handle = handle,
};

ProximityEventHandleAlgo * getDebugInfoAlgo(void)
{
    if(false == inited){

        gTicketSystem = getTicketSystem(TICKET_SYSTEM_BIT_IMPL_TYPE);

        inited = true;
    }

    return &gDebugInfoAlgo;
}
