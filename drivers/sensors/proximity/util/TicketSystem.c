#include "TicketSystem.h"
#include "TicketSystemBitImpl.h"
TicketSystem * getTicketSystem(TICKET_SYSTEM_IMPL_TYPE type)
{

    switch(type){

        case TICKET_SYSTEM_BIT_IMPL_TYPE :
            {
                TicketSystemBitImpl *impl = create_TicketSystemBitImpl();

                if(NULL == impl){
                    goto create_err;
                }

                return &impl->parent;     
            }
            break;
        default:
            break;
    }
create_err:
    return NULL;
}
