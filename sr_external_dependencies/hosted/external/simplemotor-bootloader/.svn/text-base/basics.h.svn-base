#ifndef BASICS_H_INCLUDED
#define BASICS_H_INCLUDED

/*   
	Authors:
	Hugo Elias
	Moritz "Morty" Struebe (morty@gmx.net)

	(c) Shadow Robot Company 2007
	This code is licensed under the GPL.

*/

#ifdef S_SPLINT_S
    #define rom 
    #define overlay 
    #define near
    #define __wparam 
#else
    #ifdef SDCC
        #define rom __code
        #define overlay 
        #define near
    #else
        #define __wparam 
    #endif
#endif

/* Data type abbreviations*/

#ifdef SDCC
    #define int8u  unsigned char
    #define int8s    signed char

    #define int16u unsigned int
    #define int16s   signed int

    #define int24u unsigned long
    #define int24s   signed long

    #define int32u unsigned long
    #define int32s   signed long
#else
    #define int8u  unsigned char
    #define int8s    signed char

    #define int16u unsigned short
    #define int16s   signed short

    #define int24u unsigned short long
    #define int24s   signed short long

    #define int32u unsigned long
    #define int32s   signed long
#endif


// Long long / 64 bit is not supported by C16 - C30 does, though.
// #define int64u unsigned long long
// #define int64s   signed long long

typedef struct int64
{
	int32u low;
	int32u high;	
} int64u;


//#include "this_node.h"

typedef union union16
{
	int16u  word;
	int8u   byte[2];
} union16_t;

typedef union union32
{
	int32u  dword;
	int16u  word[2];
	int8u   byte[4];
} union32_t;

typedef union union64
{
	int32u  dword[2];
	int16u  word[4];
	int8u   byte[8];
} union64_t;


// WARNING: Only works with overlay or global values. NOT VALUES ON THE STACK!
#ifdef SDCC
    #define shift_right_signed_32(x)    x >>= 1;
#else
    #define shift_right_signed_32(x)    x >>= 1;                \
                                        _asm                    \
                                            btfsc x+3, 6, 1     \
                                            bsf   x+3, 7, 1     \
                                        _endasm                        
#endif

#ifdef SDCC
    #define shift_right_signed_16(x)    x >>= 1;
#else
    #define shift_right_signed_16(x)    x >>= 1;                \
                                        _asm                    \
                                            btfsc x+1, 6, 1     \
                                            bsf   x+1, 7, 1     \
                                        _endasm                        
#endif

void ShiftLeft64(union union64 *p);

void Reset_func(void);                  //!< Use this instead of Reset to keep optimization.
void ClrWdt_func(void);                 //!< Clear Watchdog Timer

extern const int8u basic_lshift[];

/* The @sef tells splint these parameters must be side-effect free, because they may be invoked more
 * than once */

/*@notfunction@*/
#define abs(x ) ((x) > 0 ? (x) : -(x))
/*@notfunction@*/
#define abslimit(val, lim)  {                                               \
	                            if      ((val) >  (lim))  (val) =  (lim);   \
	                            else if ((val) < -(lim))  (val) = -(lim);   \
                            }

/*@notfunction@*/
#define rshift_sgn(val, shift) (((val) > 0) ? ((val) >> (shift)) : -((-(val))>>(shift))) 


#ifdef SDCC
    /* These defines map from non-SDCC bit variables to SDCC bit variables. */
    #define RXM0SIDL_EXIDEN RXM0SIDLbits.EXIDEN
    #define RXM1SIDL_EXIDEN RXM1SIDLbits.EXIDEN
    #define ECANCON_MDSEL1 ECANCONbits.MDSEL1
    #define ECANCON_MDSEL0 ECANCONbits.MDSEL0
    #define COMSTAT_FIFOEMPTY COMSTATbits.FIFOEMPTY
    #define PIR3_RXBnIF PIR3bits.RXBnIF
    #define COMSTAT_RXBnOVFL COMSTATbits.RXBnOVFL
    #define PIR3_IRXIF PIR3bits.IRXIF
    
    #define TXB0CON_TXREQ TXB0CONbits.TXREQ 
    #define TXB1CON_TXREQ TXB1CONbits.TXREQ
    #define TXB2CON_TXREQ TXB2CONbits.TXREQ
    #define BSEL0_B0TXEN BSEL0bits.B0TXEN
    #define B0CON_TXREQ B0CONbits.TXREQ
    
    #define BSEL0_B1TXEN BSEL0bits.B1TXEN
    #define B1CON_TXREQ B1CONbits.TXREQ
    #define BSEL0_B2TXEN BSEL0bits.B2TXEN
    #define B2CON_TXREQ B2CONbits.TXREQ
    #define BSEL0_B3TXEN BSEL0bits.B3TXEN
    #define B3CON_TXREQ B3CONbits.TXREQ
    #define BSEL0_B4TXEN BSEL0bits.B4TXEN
    #define B4CON_TXREQ B4CONbits.TXREQ
    #define BSEL0_B5TXEN BSEL0bits.B5TXEN
    #define B5CON_TXREQ B5CONbits.TXREQ
    #define RXB0CON_RXFUL RXB0CONbits.RXFUL
    #define PIR3_RXB0IF PIR3bits.RXB0IF
    #define COMSTAT_RXB0OVFL COMSTATbits.RXB0OVFL
    
    #define RXB0CON_FILHIT0 RXB0CONbits.FILHIT0
    #define RXB1CON_RXFUL RXB1CONbits.RXFUL
    #define PIR3_RXB1IF PIR3bits.RXB1IF
    #define COMSTAT_RXB1OVFL COMSTATbits.RXB1OVFL
    
    #define RXFCON0_RXF0EN RXFCON0bits.RXF0EN
    #define RXFCON0_RXF1EN RXFCON0bits.RXF1EN
    #define RXFCON0_RXF2EN RXFCON0bits.RXF2EN
    #define RXFCON0_RXF3EN RXFCON0bits.RXF3EN
    #define B0CON_RXFUL B0CONbits.RXFUL
    #define B1CON_RXFUL B1CONbits.RXFUL
    #define B2CON_RXFUL B2CONbits.RXFUL
    #define B3CON_RXFUL B3CONbits.RXFUL
    #define B4CON_RXFUL B4CONbits.RXFUL
    #define B5CON_RXFUL B5CONbits.RXFUL
    
    
    
    #define _asm __asm
    #define _endasm __endasm
    
    /* Can't copy aggregates! */
    #define msg_copy(dest, src) { dest.messageID=src.messageID; dest.d.dword[0]=src.d.dword[0]; dest.d.dword[1]=src.d.dword[1]; dest.length=src.length; dest.flags=src.flags; }
#else /* !SDCC */
    #define msg_copy(dest, src) dest=src
#endif /* SDCC */


/* Delays code */
#ifdef SDCC
    #include <delay.h>
    #define Delay1TCY Nop
    #define Delay10TCYx delay10tcy
    #define Delay100TCYx delay100tcy
    #define Delay10KTCYx delay10ktcy;
    #define Delay1KTCYx delay1ktcy;
#else /* !SDCC */
    #ifdef GCC
        #define Delay1TCY 
        #define Delay10TCYx(x)
        #define Delay100TCYx(x)
        #define Delay10KTCYx(x)
        #define Delay1KTCYx(x)
        #define delay1mtcy(x)
        #define CARRY_BIT 0
    #else /* !GCC */
        #include <delays.h>
        #define delay1mtcy(x) { int8u d; for (d=0;d<100; d++) Delay10KTCYx(d); }
        #define CARRY_BIT 0
    #endif /* GCC */
#endif /* SDCC */


#define FLASH(Led_State, n)    while (1)                                \
                               {                                        \
                                    int8u i;                             \
                                    for (i=0; i<n; i++)                 \
                                    {                                   \
                                        LED_set_state(Led_State);       \
                                        Delay10KTCYx(100);              \
                                        LED_set_state(0);               \
                                        Delay10KTCYx(100);              \
                                    }                                   \
                                    Delay10KTCYx(250);                  \
                               }


#define FLASH12(Led_State,a,b)  while (1)                           \
                                {                                   \
                                    LED_set_state(Led_State);       \
                                }

enum reset_flags
{
    reset_unknown     = 0,
    reset_brownout    = 1<<1,
    reset_poweron     = 1<<2,
    reset_watchdog    = 1<<3,
    reset_instruction = 1<<4
};
//extern int8u last_reset;


#endif
