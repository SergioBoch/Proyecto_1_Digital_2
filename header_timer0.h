//timer 0

#ifndef header_timer0
#define header_timer0


#include <xc.h>
#include <stdint.h>
#include "header_timer0.h"

void tmr0_init (uint16_t prescaler, uint8_t valor_TMR0);
void tmr0_reload(uint8_t valor_TMR0);

#endif
