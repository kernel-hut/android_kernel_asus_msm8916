#include <linux/kernel.h>
#include <linux/bitmap.h>
#include <linux/slab.h>
#include "TicketSystemBitImpl.h"

static bool hasAnyReservation(TicketSystem *_this)
{
    
    TicketSystemBitImpl *__this = container_of(_this, TicketSystemBitImpl, parent);

    return (!(find_first_bit(&__this->flags, BITS_PER_LONG)>=BITS_PER_LONG));
}
static bool hasFullReservation(TicketSystem *_this)
{
    TicketSystemBitImpl *__this = container_of(_this, TicketSystemBitImpl, parent);

    return (!(find_first_zero_bit(&__this->flags, BITS_PER_LONG)>=BITS_PER_LONG));

}
static void reserveByTicket(TicketSystem *_this, unsigned int ticket)
{
    TicketSystemBitImpl *__this = container_of(_this, TicketSystemBitImpl, parent);

    test_and_set_bit(ticket, &__this->flags);
}
static void cancelByTicket(TicketSystem *_this, unsigned int ticket)
{
    TicketSystemBitImpl *__this = container_of(_this, TicketSystemBitImpl, parent);

    test_and_clear_bit(ticket, &__this->flags);
}
static void construct(
    TicketSystemBitImpl *_this )
{
    _this->parent.hasAnyReservation = hasAnyReservation;
    
    _this->parent.hasFullReservation= hasFullReservation;
    
    _this->parent.reserveByTicket= reserveByTicket;
    
    _this->parent.cancelByTicket= cancelByTicket;

    return;
}
TicketSystemBitImpl * create_TicketSystemBitImpl(void)
{
    TicketSystemBitImpl *lpTicketSystemBitImpl = NULL;

    lpTicketSystemBitImpl = kzalloc(sizeof(TicketSystemBitImpl), GFP_KERNEL);

    BUG_ON(NULL == lpTicketSystemBitImpl);

    construct(lpTicketSystemBitImpl);

    return lpTicketSystemBitImpl;
}


