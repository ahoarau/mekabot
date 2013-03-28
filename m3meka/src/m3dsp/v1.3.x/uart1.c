/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef USE_UART

#include "p33fxxxx.h"
#include "setup.h"
#include "uart1.h"

//#include "m3serial.h"

//Note, there is a bug with using the TX INT
//Should be easy to fix, but no need for it at the 
//moment
//#define USE_TX_INT

#define EOF -1
#define TX1_BUF_LEN 68
#define RX1_BUF_LEN 16

char uart_str[64];

unsigned char tx1_buf[TX1_BUF_LEN];
unsigned char rx1_buf[RX1_BUF_LEN];

unsigned char tx1_head, tx1_tail;
unsigned char rx1_head, rx1_tail;

volatile unsigned char tx1_cnt;
volatile unsigned char rx1_cnt;


void setup_uart(void) {

  tx1_head = 0;
  tx1_tail = 0;
  tx1_cnt = 0;
  rx1_head = rx1_tail = rx1_cnt = 0;

#ifdef M3_FB_DEV_0_0
  // BRG = Fcy / (16 * baudrate) -1
  // -1 + 40e6 /16 / 57600

 // U1BRG = 21;  // (20.7014) 115.2Kb at 40 Mhz
	U1BRG = 10;  // (9.85) 230.400Kb at 40 Mhz
#else
  // BRG = Fcy / (16 * baudrate) -1
  // -1 + 40e6 /16 / 115200
  U1BRG = 21;  // (20.7014) 115.2Kb at 40 Mhz
#endif



  U1MODEbits.UARTEN = 0;			//Disable U1ART module

  //=================================================================
  // Configure Interrupt Priority
  _U1RXIF = 0;	//Clear Rx interrupt flags
  _U1TXIF = 0;	//Clear Tx interrupt flags
  _U1RXIE = 1;	//Receive interrupt: 0 disable, 1 enable 
  _U1TXIE = 1;	//Transmit interrupt: 0 disable, 1 enable

  U1STAbits.UTXISEL0 = 0;
  U1STAbits.UTXISEL1 = 0;

  U1MODEbits.UEN0 = 0;
  U1MODEbits.UEN1 = 0;
  
  //=================================================================
  // Configure Mode
  //  +--Default: 8N1, no loopback, no wake in sleep mode, continue in idle mode
  //  +--Diable autobaud detect
  //  +--Enable U1ART module
  U1MODEbits.ABAUD = 0;	//Disable Autobaud detect from U1RX 
  U1MODEbits.UARTEN = 1;	//U1ART enable
  
  //=================================================================
  // Configure Status
  //  +--Default: TxInt when a char is transmitted, no break char
  //  +--Default: RxInt when a char is received, no address detect, clear overflow
  //  +--Enable Transmit
  U1STAbits.UTXEN = 1;	//Tx enable
}


void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void) {
  unsigned char ch;
  _U1RXIF = 0;		//Clear the flag

#ifdef M3_FB_DEV_0_0
ch = (unsigned char) U1RXREG; //Read the data from buffer
serial_eth_receive(ch); //Saving the reception byte in a buffer for the decoding function (look in m3serial.c)
#else
   if ( U1STAbits.URXDA ){
    ch = (unsigned char) U1RXREG; //Read the data from buffer
    if (rx1_cnt < RX1_BUF_LEN) {
      rx1_buf[rx1_head++] = ch;
      ++rx1_cnt;

      if (rx1_head >= RX1_BUF_LEN) rx1_head -= RX1_BUF_LEN;

    } else {      // overrun

    } // if rx1_cnt else

  } // if U1STAbits
#endif
}


void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void) {
  unsigned char ch;
  _U1TXIF = 0;	//Clear Interrupt Flag

  //  PORTD = LATD ^ B1;
#ifdef USE_TX_INT
  while(U1STAbits.UTXBF==0 && tx1_cnt) {
    ch = tx1_buf[tx1_tail++];
    U1TXREG = ch;
    tx1_cnt--;
    
    if (tx1_tail >= TX1_BUF_LEN) tx1_tail -= TX1_BUF_LEN;
  }
#endif
}


int getchar1_cnt(void) {
  int i;
  _U1RXIE = 0;
  i = rx1_cnt;
  _U1RXIE = 1;
  return i;
}

int getchar1(void) {
  int ch = EOF;
  _U1RXIF = 0;
  _U1RXIE = 0;
  if (rx1_cnt) {
    --rx1_cnt;
    ch = rx1_buf[rx1_tail++];
    _U1RXIE = 1;
    if (rx1_tail >= RX1_BUF_LEN) rx1_tail -= RX1_BUF_LEN;

  } else   _U1RXIE = 1;
  return ch;
}




int puts1(char *s) {
  int n=0;
  int r;
  char ch;

  if (!s) return EOF;

  while(*s) {
    //putchar(*s++);
    ch = *s++;
    do {
      r = putchar1(ch);
    } while (r == EOF);
    ++n;
  }
  return n;
}


int putchar1(unsigned char ch) {
#ifdef USE_TX_INT

  //DisableInterrupts(5);
  _U1TXIE = 0;	
  if (tx1_cnt >= TX1_BUF_LEN) {
    _U1TXIE = 1;	
    // LATD = LATD ^ B5;
    return EOF;
  }
  ++tx1_cnt;

  tx1_buf[tx1_head++] = ch;

  if (tx1_head >= TX1_BUF_LEN) tx1_head -= TX1_BUF_LEN;

  _U1TXIE = 1;	
  if (!U1STAbits.UTXBF) _U1TXIF = 1;

  return ch;
#else
  _U1TXIE = 0;	
  while(U1STAbits.UTXBF) ;
  U1TXREG = ch;
  return 0;
#endif
}

#endif
