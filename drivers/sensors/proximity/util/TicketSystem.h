#ifndef __TICKETSYSTEM_H__
#define __TICKETSYSTEM_H__
#include <linux/types.h>


struct TicketSystem;

typedef enum { 
    TICKET_SYSTEM_BIT_IMPL_TYPE = 0,
    TICKET_SYSTEM_IMPL_TYPE_MAX,
} TICKET_SYSTEM_IMPL_TYPE;

typedef struct TicketSystem{
    bool (*hasAnyReservation)(struct TicketSystem *_this);
    bool (*hasFullReservation)(struct TicketSystem *_this);
    void (*reserveByTicket)(struct TicketSystem *_this, unsigned int ticket);
    void (*cancelByTicket)(struct TicketSystem *_this, unsigned int ticket);
}TicketSystem;

extern TicketSystem * getTicketSystem(TICKET_SYSTEM_IMPL_TYPE type);

#endif //__TICKETSYSTEM_H__
