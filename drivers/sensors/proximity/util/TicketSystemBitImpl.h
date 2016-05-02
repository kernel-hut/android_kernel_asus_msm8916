#ifndef __TICKETSYSTEMBITIMPL_H__
#define __TICKETSYSTEMBITIMPL_H__
#include "TicketSystem.h"

typedef struct TicketSystemBitImpl{
    TicketSystem parent;
    unsigned long		flags;
}TicketSystemBitImpl;

extern TicketSystemBitImpl * create_TicketSystemBitImpl(void);

#endif //__TICKETSYSTEMBITIMPL_H__