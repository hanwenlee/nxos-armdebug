/* Copyright (C) 2007 the NxOS developers
 *
 * See AUTHORS for a full list of the developers.
 *
 * Redistribution of this file is permitted under
 * the terms of the GNU Public License (GPL) version 2.
 */

.code 32
.text
.align 	0

/* What follows is the exception vectors for the NXT. They are placed at
 * the bottom of memory at system start up, and just call into exception
 * handlers in interrupts.S.
 *
 * LDR is used instead of plain branching because LDR jumps can be
 * relocated.
 */
        ldr   pc,v0 /* Reset */
        ldr   pc,v5 /* Illegal instruction */
        ldr   pc,v4 /* Software interrupt */
        ldr   pc,v2 /* Prefetch abort */
        ldr   pc,v3 /* Data abort */
        ldr   pc,v4 /* (reserved) */
        ldr   pc,v1 /* IRQ */
        ldr   pc,v4 /* FIQ */

v0:     .long nx_start
v1:     .long nx__irq_handler
#ifdef __DBGENABLE__
v2:     .long prefetch_abort_handler
v3:     .long data_abort_handler
#else
v2:     .long default_prefetch_abort_handler
v3:     .long default_data_abort_handler
#endif
v4:     .long nx__unhandled_exception
#ifdef __DBGENABLE__
v5:     .long undef_handler
#else
v5:     .long default_undef_handler
#endif
