#include "cc430x513x.h"
#include "RF1A.h"
#include "hal_pmm.h"
#include <msp430.h>
#include <stdio.h>

/**************************
*
*typedef for nice devclaration
***************************/
typedef  signed char ot_s8;
typedef signed short ot_int;
typedef unsigned char u8;



/*******************
 * Function Definition
 */
void Transmit(unsigned char *buffer, unsigned char length);
void ReceiveOn(void);
void ReceiveOff(void);

void InitButtonLeds(void);
void InitRadio(void);

void delayms(unsigned int n);
void InitUart(void); //raja adding initialize uart func
int putchar(int byte);
int getchar(void);
void uart_put_char( uint8_t );
void uart_write( uint8_t*, uint16_t );
void uart_write_escaped( uint8_t*, uint16_t );
u8 RF_ReadSingleReg (u8 addr);
u8 RF_GetRSSI();
ot_int radio_rssi();
#define __eint() asm("EINT") 
#define __nop() asm("NOP") 