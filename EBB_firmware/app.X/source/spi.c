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
  
  spi_send(0x80 + 0x00, 0x00000008);   // GCONF=8: Enable PP and INT outputs
  spi_send(0x80 + 0x6C, 0x000100C5);   // CHOPCONF: TOFF=5, HSRTR=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
  spi_send(0x80 + 0x7C, 0x000100C5);   // CHOPCONF: TOFF=5, HSRTR=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
  spi_send(0x80 + 0x30, 0x00011F05);   // IHOLD_IRUN: IHOLD=5, IRUN=31 (max current), IHOLDDELAY=1
  spi_send(0x80 + 0x50, 0x00011F05);   // IHOLD_IRUN: IHOLD=5, IRUN=31 (max current), IHOLDDELAY=1
  spi_send(0x80 + 0x10, 0x000401C8);   // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
  spi_send(0x80 + 0x18, 0x000401C8);   // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
  spi_send(0x80 + 0x32, 0x00061A80);   // VHIGH=400 000: Set VHIGH to a high value to allow stealthChop
  spi_send(0x80 + 0x52, 0x00061A80);   // VHIGH=400 000: Set VHIGH to a high value to allow stealthChop
  spi_send(0x80 + 0x31, 0x00007530);   // VCOOLTHRS=30000: Set upper limit for stealthChop to about 30RPM
  spi_send(0x80 + 0x51, 0x00007530);   // VCOOLTHRS=30000: Set upper limit for stealthChop to about 30RPM
  
//  spi_send(0x80 + 0x24, 0x000003E8);   // A1 1 000 First acceleration
//  spi_send(0x80 + 0x44, 0x000003E8);   // A1 1 000 First acceleration
  spi_send(0x80 + 0x24, 0x00000001);   // A1 0 First acceleration
  spi_send(0x80 + 0x44, 0x00000001);   // A1 0 First acceleration
  //spi_send(0x80 + 0x25, 0x0000C350);   // V1 = 50 000 Acceleration threshold velocity V1
  //spi_send(0x80 + 0x45, 0x0000C350);   // V1 = 50 000 Acceleration threshold velocity V1
  spi_send(0x80 + 0x25, 0x00000000);   // V1 = 0 Acceleration threshold velocity V1 (disables A1 and D1 phase)
  spi_send(0x80 + 0x45, 0x00000000);   // V1 = 0 Acceleration threshold velocity V1 (disables A1 and D1 phase)
  spi_send(0x80 + 0x25, 0x00004E20);   // VSTART = 200 000
  spi_send(0x80 + 0x45, 0x00004E20);   // VSTART = 200 000
  //spi_send(0x80 + 0x26, 0x000001F4);   // AMAX = 500 Acceleration above V1
  //spi_send(0x80 + 0x46, 0x000001F4);   // AMAX = 500 Acceleration above V1
  spi_send(0x80 + 0x26, 0x00000001);   // AMAX = 0 Acceleration above V1
  spi_send(0x80 + 0x46, 0x00000001);   // AMAX = 0 Acceleration above V1
  spi_send(0x80 + 0x27, 0x00004E20);   // VMAX = 200 000
  spi_send(0x80 + 0x47, 0x00004E20);   // VMAX = 200 000
  //spi_send(0x80 + 0x28, 0x000002BC);   // DMAX = 700 Deceleration above V1
  //spi_send(0x80 + 0x48, 0x000002BC);   // DMAX = 700 Deceleration above V1
  spi_send(0x80 + 0x28, 0x00000001);   // DMAX = 1 Deceleration above V1
  spi_send(0x80 + 0x48, 0x00000001);   // DMAX = 1 Deceleration above V1
  //spi_send(0x80 + 0x2A, 0x00000578);   // D1 = 1400 Deceleration below V1
  //spi_send(0x80 + 0x4A, 0x00000578);   // D1 = 1400 Deceleration below V1
  spi_send(0x80 + 0x2A, 0x00000001);   // D1 = 1 Deceleration below V1
  spi_send(0x80 + 0x4A, 0x00000001);   // D1 = 1 Deceleration below V1
  spi_send(0x80 + 0x2B, 0x00004E20);   // VSTOP = 10 Stop velocity (Near to zero)
  spi_send(0x80 + 0x4B, 0x00004E20);   // VSTOP = 10 Stop velocity (Near to zero)
  spi_send(0x80 + 0x20, 0x00000000);   // RAMPMODE = 0 (Target position move)
  spi_send(0x80 + 0x40, 0x00000000);   // RAMPMODE = 0 (Target position move)
  spi_send(0x80 + 0x21, 0x00000000);   // XACTUAL = 0
  spi_send(0x80 + 0x41, 0x00000000);   // XACTUAL = 0
  spi_send(0x80 + 0x2D, 0xFFFF3800);   // XTARGET = -51200 (move one rotation left (200*256 microsteps))
  spi_send(0x80 + 0x4D, 0xFFFF3800);   // XTARGET = -51200 (move one rotation left (200*256 microsteps))
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