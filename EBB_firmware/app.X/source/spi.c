#include <p18cxxx.h>
#include <ctype.h>
#include "spi.h"


void spi_init(void)
{
  // SCK1 is RB4
  // SDI1 is RB5
  // SDO1 is RC7
  // SS1 is RA5
  
  // Set up SPI2 and RP pins
  // Inputs
  // SDI2 RPINR21    RC6 (RP17)
  // Outputs
  // SDO2 9          RC0 (RP11)
  // SCK2 10         RC2 (RP13)
  // SS              RD1
  
  LATCbits.LATC2 = 0;           // Init SCK2 to idle state (low)
  TRISCbits.TRISC2 = 0;         // Set SCK2 to be an output
  TRISCbits.TRISC6 = 1;         // Set SDI2 to be an input
  LATDbits.LATD1 = 1;           // Init SS2 to idle state (high)
  TRISDbits.TRISD1 = 0;         // Set SS2 to be an output
  TRISCbits.TRISC0 = 0;         // Set SDO2 to be an output
  
  RPINR21 = 17;                 // Set SDI2 to use RP17 pin
  RPOR11 = 9;                   // Set SDO2 to use RP11 pin
  RPOR13 = 10;                  // Set SCK2 to use RP13 pin
  
  SSP2STATbits.SMP = 0;         // Input data is sampled at middle of data output time
  SSP2STATbits.CKE = 0;         // Transmit occurs on idle to active clock edge
  SSP2CON1bits.SSPM = 0b0001;   // SPI clock = Fosc/16 or 3MHz
  SSP2CON1bits.CKP = 1;         // Clock idles high
  SSP2CON1bits.SSPEN = 1;       // Turn MSSP2 (SPI2) peripheral on
  
  spi_send(0x80, 0x00000008);
  spi_send(0xEC, 0x000100C5);
  spi_send(0xB0, 0x00011F05);
  spi_send(0x90, 0x000401C8);
  spi_send(0xB2, 0x00061A80);
  spi_send(0xB1, 0x00007530);
  
  spi_send(0xA4, 0x000003E8);
  spi_send(0xA5, 0x0000C350);
  spi_send(0xA6, 0x000001F4);
  spi_send(0xA7, 0x000304D0);
  spi_send(0xA8, 0x000002BC);
  spi_send(0xAA, 0x00000578);
  spi_send(0xAB, 0x0000000A);
 
  spi_send(0xA0, 0x00000000);
  spi_send(0xAD, 0xFFFF3800);
}

/// TODO: Add timeout?
void spi_send(unsigned char command, unsigned long int sendData)
{
  volatile unsigned char temp;
  unsigned char sd1, sd2, sd3, sd4;
  
  sd1 = sendData >> 24;
  sd2 = sendData >> 16;
  sd3 = sendData >> 8;
  sd4 = sendData;
  
  LATDbits.LATD1 = 0;           // Chip select goes low
  
  temp = SSP2BUF;
  SSP2BUF = command;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  temp = SSP2BUF;
  SSP2BUF = sd1;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  temp = SSP2BUF;
  SSP2BUF = sd2;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  temp = SSP2BUF;
  SSP2BUF = sd3;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  temp = SSP2BUF;
  SSP2BUF = sd4;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  
  LATDbits.LATD1 = 1;           // Chip select goes high
}