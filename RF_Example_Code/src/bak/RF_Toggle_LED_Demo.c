/******************************************************************************
* CC430 RF Code Example - TX and RX (variable packet length =< FIFO size)
*
* Simple RF Link to Toggle Receiver's LED by pressing Transmitter's Button    
* Warning: This RF code example is setup to operate at either 868 or 915 MHz, 
* which might be out of allowable range of operation in certain countries.
* The frequency of operation is selectable as an active build configuration
* in the project menu. 
* 
* Please refer to the appropriate legal sources before performing tests with 
* this code example. 
* 
* This code example can be loaded to 2 CC430 devices. Each device will transmit 
* a small packet upon a button pressed. Each device will also toggle its LED 
* upon receiving the packet. 
* 
* The RF packet engine settings specify variable-length-mode with CRC check 
* enabled. The RX packet also appends 2 status bytes regarding CRC check, RSSI 
* and LQI info. For specific register settings please refer to the comments for 
* each register in RfRegSettings.c, the CC430x613x User's Guide, and/or 
* SmartRF Studio.
* 
* M. Morales
* Texas Instruments Inc.
* February 2010
* Built with IAR v4.21 and CCS v4.1
******************************************************************************/

#include "../inc/RF_Toggle_LED_Demo.h"

#define _EINT() asm("EINT") 
#define __nop() asm("NOP") 
#define  PACKET_LEN         (0x05)			// PACKET_LEN <= 61
#define  RSSI_IDX           (PACKET_LEN+1)  // Index of appended RSSI 
#define  CRC_LQI_IDX        (PACKET_LEN+2)  // Index of appended LQI, checksum
#define  CRC_OK             (BIT7)          // CRC_OK bit 
// 433MHz下的发射功率宏定义
#define  PATABLE_VAL        (0x50)          // 0XC4-- 10 dBm;
                                            // 0X50-- 0  dBm;
                                            // 0X2D-- -6 dBm;
                                            // 0X26-- -12dBm;
                                            // 0x05-- -30dBm;
                                            // 0xc0-- max 
#define  LED_RBIT           6
#define  LED_GBIT           7
#define  LED_DIR            P2DIR
#define  LED_OUT            P2OUT
#define  LED_SEL            P2SEL
#define  LEDR_ON()          LED_OUT|=(1<<LED_RBIT)
#define  LEDG_ON()          LED_OUT|=(1<<LED_GBIT)
#define  LEDR_OFF()         LED_OUT&=~(1<<LED_RBIT)
#define  LEDG_OFF()         LED_OUT&=~(1<<LED_GBIT)

#define  BUTTON_BIT         0
#define  BUTTON_DIR         PJDIR
#define  BUTTON_OUT         PJOUT
#define  BUTTON_REN         PJREN
#define  BUTTON_IN          PJIN
#define RF_CoreReg_RSSI             0x34
#define RF_RSSIOffset               74


extern RF_SETTINGS rfSettings;

unsigned char packetReceived;
unsigned char packetTransmit; 

unsigned char RxBuffer[64];
unsigned char RxBufferLength = 0;
//const unsigned char TxBuffer[6]= {PACKET_LEN, 0x27, 'L', 'S', 'D', '.'};
const unsigned char TxBuffer[6]= {'R','S','S','I',' ','='};
unsigned char buttonPressed = 0;
unsigned int i = 0; 

signed short rssi;
unsigned char transmitting = 0; 
unsigned char receiving =0;//= 0; //i am commenting for debug remove comment later

void main( void )
{  
  // Stop watchdog timer to prevent time out reset 
  //WDTCTL = WDTPW + WDTHOLD; 
  WDTCTL = WDT_ARST_250;     
  // Increase PMMCOREV level to 2 for proper radio operation
  SetVCore(2);                            
  
  ResetRadioCore();     
  InitRadio();
  InitButtonLeds();
  InitUart();
    receiving = 1; 
  ReceiveOn(); 
  
  _EINT();  
  while (1)
  { 
     WDTCTL = WDT_ARST_250;
     __bis_SR_register( LPM3_bits + GIE ); 
     __no_operation(); 

     if(!(BUTTON_IN & (1<<BUTTON_BIT)))
     {
       _NOP();
       delayms(30);
       if(!(BUTTON_IN & (1<<BUTTON_BIT)))
       {
         _NOP();
         LEDR_ON();
         ReceiveOff();
         receiving = 0;
         Transmit( (unsigned char*)TxBuffer,sizeof TxBuffer);
         transmitting = 1;
         while(!(BUTTON_IN & (1<<BUTTON_BIT)));
       }
      
     }
     else if(!transmitting)
     {
       ReceiveOn();
       receiving = 1;
     }
  }
}

void delayms(unsigned int n)
{
  unsigned int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<800;j++);
  }
}

void InitButtonLeds(void)
{
  // Initialize Port J
  PJOUT = 0x01;
  PJDIR = 0xFF;
  
  // Set up the button as interruptible
  BUTTON_DIR&=~(1<<BUTTON_BIT);   // 按键设置为输入
  BUTTON_REN|=BIT0;      // 上拉
   
  // Set up LEDs 
  LED_OUT&=~((1<<LED_RBIT)|(1<<LED_GBIT));    // LED端口输出0
  LED_DIR|=(1<<LED_RBIT)|(1<<LED_GBIT);       // LED端口方向设置为输出
}

void InitRadio(void)
{
  // Set the High-Power Mode Request Enable bit so LPM3 can be entered
  // with active radio enabled 
  PMMCTL0_H = 0xA5;
  PMMCTL0_L |= PMMHPMRE_L; 
  PMMCTL0_H = 0x00; 
  
  WriteRfSettings(&rfSettings);
  
  WriteSinglePATable(PATABLE_VAL);
}

void Transmit(unsigned char *buffer, unsigned char length)
{
  RF1AIES |= BIT9;                          
  RF1AIFG &= ~BIT9;                         // Clear pending interrupts
  RF1AIE |= BIT9;                           // Enable TX end-of-packet interrupt
  
  WriteBurstReg(RF_TXFIFOWR, buffer, length);     
  
  Strobe( RF_STX );                         // Strobe STX   
}

void ReceiveOn(void)
{  
  RF1AIES |= BIT9;                          // Falling edge of RFIFG9
  RF1AIFG &= ~BIT9;                         // Clear a pending interrupt
  RF1AIE  |= BIT9;                          // Enable the interrupt 
  
  // Radio is in IDLE following a TX, so strobe SRX to enter Receive Mode
  Strobe( RF_SRX );                      
}

void ReceiveOff(void)
{
  RF1AIE &= ~BIT9;                          // Disable RX interrupts
  RF1AIFG &= ~BIT9;                         // Clear pending IFG

  // It is possible that ReceiveOff is called while radio is receiving a packet.
  // Therefore, it is necessary to flush the RX FIFO after issuing IDLE strobe 
  // such that the RXFIFO is empty prior to receiving a packet.
  Strobe( RF_SIDLE );
  Strobe( RF_SFRX  );                       
}


void InitUart(void){
  
    PMAPPWD = 0x02D52;                        // Get write-access to port mapping regs  
    P1MAP5 = PM_UCA0RXD;                      // Map UCA0RXD output to P2.6 
    P1MAP6 = PM_UCA0TXD;                      // Map UCA0TXD output to P2.7 
    PMAPPWD = 0;                              // Lock port mapping registers 
  
    P1DIR |= BIT5;                            // Set P2.7 as TX output
    P1SEL |= BIT5 + BIT6;                     // Select P2.6 & P2.7 to UART function
  
    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL_1;                     // CLK = ACLK
    UCA0BR0 = 0x0D;                           // 2400 (see User's Guide)
    UCA0BR1 = 0x00;                           //
    UCA0MCTL |= UCBRS_6+UCBRF_0;              // Modulation UCBRSx=6, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA0IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
 
	
}

int putchar(int byte)
{
    //UCA0IFG & UCTXIFG
	while (!(UCA0IFG & UCTXIFG))
    {
    	;	// loop until the USCI_A0 TX buffer is ready
	}
    UCA0TXBUF = (unsigned char)byte;	// send the byte
    return 1;							// we sent one byte out
}

// int getchar(void)
// returns the character read from the serial port
int getchar(void)
{
    char c;

    while (!(UCA0IFG & UCRXIFG));

    c = UCA0RXBUF;
    return (int)c;
}



/*******************************************************************************
 * @fn     uart_put_char( uint8_t character )
 * @brief  transmit single character
 * ****************************************************************************/
void uart_put_char( uint8_t character )
{
  // Enable TX interrupt
  //UCA0IE |= UCRXIE;
  while (!(UCA0IFG&UCTXIFG));	// USCI_A0 TX buffer ready?
  UCA0TXBUF = character;
}

/*******************************************************************************
 * @fn     uart_write( uint8_t character )
 * @brief  transmit whole buffer
 * ****************************************************************************/
void uart_write( uint8_t* buffer, uint16_t length )
{
  uint16_t buffer_index;
  
  for( buffer_index = 0; buffer_index < length; buffer_index++ )
  {
    uart_put_char( buffer[buffer_index] );
  }
}

/*******************************************************************************
 * @fn     uart_write_escaped( uint8_t character )
 * @brief  transmit whole buffer while escaping characters
 * ****************************************************************************/
void uart_write_escaped( uint8_t* buffer, uint16_t length )
{
  uint16_t buffer_index;
    
  uart_put_char( 0x7e );
  for( buffer_index = 0; buffer_index < length; buffer_index++ )
  {
    if( (buffer[buffer_index] == 0x7e) | (buffer[buffer_index] == 0x7d) )
    {
      uart_put_char( 0x7d ); // Escape byte
      uart_put_char( buffer[buffer_index] ^ 0x20 );
    }
    else
    {
      uart_put_char( buffer[buffer_index] );
    }
  }
  uart_put_char( 0x7e );
}

//read the value of the single register, defined as the address
u8 RF_ReadSingleReg (u8 addr) {
    u8 data_out;
  
  // Check for valid configuration register address, 0x3E refers to PATABLE 
  if ((addr <= 0x2E) || (addr == 0x3E))
    // Send address + Instruction + 1 dummy byte (auto-read)
    RF1AINSTR1B = (addr | RF_SNGLREGRD);    
  else
    // Send address + Instruction + 1 dummy byte (auto-read)
    RF1AINSTR1B = (addr | RF_STATREGRD);    
  
  while (!(RF1AIFCTL1 & RFDOUTIFG) );       ///@todo there is a bug somewhere in here
  data_out = RF1ADOUTB;                    // Read data and clears the RFDOUTIFG

  return data_out;
}


//read the RSSI value from the register of CC430.
u8 RF_GetRSSI() {
    return RF_ReadSingleReg( RF_CoreReg_RSSI );
}


/// Transceiver implementation dependent
/// CC430 stores the RSSI in a special register, as a 2's complement number, of
/// offset 0.5 dBm units.  This function translates it into normal dBm units.
ot_int radio_rssi() {

    signed char  rssi_raw; //signed char - ot_s8
    signed short rssi_val; //signed short - ot_int
    
    rssi_raw    = (signed char)RF_GetRSSI();      // CC430 RSSI is 0.5 dBm units, signed byte
    rssi_val    = (signed short)rssi_raw;         // Convert to signed 16 bit (1 instr on MSP)
    rssi_val   += 128;                      // Make it positive...
    rssi_val  >>= 1;                        // ...So division to 1 dBm units can be a shift...
    rssi_val   -= (64 + RF_RSSIOffset);     // ...and then rescale it, including offset
    
    return rssi_val;
}




// Timer A0 interrupt service routine
// #pragma vector=TIMER1_A0_VECTOR
// __interrupt// 
// __attribute__((interrupt(TIMER1_A0_VECTOR)))
// void TIMER1_A0_ISR(void)
// code that is present already is as follows
//// #pragma vector=CC1101_VECTOR
// __interrupt void CC1101_ISR(void)
__attribute__((interrupt(CC1101_VECTOR)))
void CC1101_ISR(void)
{
  switch((RF1AIV))        // Prioritizing Radio Core Interrupt 
  {
    case  0: break;                         // No RF core interrupt pending                                            
    case  2: break;                         // RFIFG0 
    case  4: break;                         // RFIFG1
    case  6: break;                         // RFIFG2
    case  8: break;                         // RFIFG3
    case 10: break;                         // RFIFG4
    case 12: break;                         // RFIFG5
    case 14: break;                         // RFIFG6          
    case 16: break;                         // RFIFG7
    case 18: break;                         // RFIFG8
    case 20:                                // RFIFG9
      if(receiving)			    // RX end of packet
      {
        // Read the length byte from the FIFO       
        RxBufferLength = ReadSingleReg( RXBYTES );               
        ReadBurstReg(RF_RXFIFORD, RxBuffer, RxBufferLength); 
        
        // Stop here to see contents of RxBuffer
        __no_operation(); 		   
		rssi=radio_rssi();
		printf("REceived RSSSI is .. %i", rssi);
		//print the raw buffer without CRC check - raja code
        uart_write( RxBuffer, RxBufferLength );
        // Check the CRC results
        if(RxBuffer[CRC_LQI_IDX] & CRC_OK)
        {                     
          LED_OUT |= (1<<LED_GBIT);         // Toggle LED1
		  //print the CRC checked buffer and say CRC check ok -- raja code
		  printf("Hello, welcome to serial port example\n");
		  printf("REceived RSSSI is .. %i", rssi);
		  printf("\n");
		  uart_write( RxBuffer, RxBufferLength );
          delayms(10);
          LED_OUT &=~(1<<LED_GBIT);
        }
      }
      else if(transmitting)		    // TX end of packet
      {
        RF1AIE &= ~BIT9;                    // Disable TX end-of-packet interrupt                         
        LEDR_OFF();                         // Turn off LED after Transmit
        transmitting = 0; 
      }
      else while(1); 			    // trap 
      break;
    case 22: break;                         // RFIFG10
    case 24: break;                         // RFIFG11
    case 26: break;                         // RFIFG12
    case 28: break;                         // RFIFG13
    case 30: break;                         // RFIFG14
    case 32: break;                         // RFIFG15
  }  
  __bic_SR_register_on_exit(LPM3_bits);     
}
__attribute__((interrupt(USCI_A0_VECTOR)))
void USCI_A0_ISR(void)
{
  switch((UCA0IV))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while (!(UCA0IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
    UCA0TXBUF = UCA0RXBUF;                  // TX -> RXed character
    UCA0TXBUF = 'a';
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }  
}
