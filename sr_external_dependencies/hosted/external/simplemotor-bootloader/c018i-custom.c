/* $Id: c018i.c,v 1.7 2006/11/15 22:53:12 moshtaa Exp $ */
/* Copyright (c)1999 Microchip Technology */
/* MPLAB-C18 startup code, including initialized data */
/* external reference to __init() function */

#warning We are using our own startup code


extern void main (void);                        // User's main()

void _entry (void);                             // Startup functions
void _startup (void);



#pragma code _entry_scn=0x000000                // Create goto instruction at 0x000000
void _entry (void)
{
    _asm goto _startup _endasm
}


#pragma code _startup_scn
void _startup (void)
{
    _asm
        lfsr 1, _stack                          // Initialize the stack pointer
        lfsr 2, _stack
        clrf TBLPTRU, 0                         // 1st silicon doesn't do this on POR
    _endasm 


    while(1)
        main ();                                // Call user's code.
}

