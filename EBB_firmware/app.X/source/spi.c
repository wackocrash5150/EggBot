#include <p18cxxx.h>
#include <ctype.h>
#include <GenericTypeDefs.h>
#include "spi.h"
#include "ubw.h"

void spi_init(void)
{
  UINT8 temp;
  
#if defined(EBB_2_3)
//// FOR EBB V2.3 CONNECTED TO TRINAMIC DEV BOARD
  // Set up SPI2 and RP pins
  // Inputs
  // SDI2 RPINR21    RC6 (RP17)
  // Outputs
  // SDO2 9          RC0 (RP11)
  // SCK2 10         RC2 (RP13)
  // SS2             RD1
  // DRV_ENN         RB3
  
  LATCbits.LATC2 = 0;           // Init SCK2 to idle state (low)
  TRISCbits.TRISC2 = 0;         // Set SCK2 to be an output
  TRISCbits.TRISC6 = 1;         // Set SDI2 to be an input
  LATDbits.LATD1 = 1;           // Init SS2 to idle state (high)
  TRISDbits.TRISD1 = 0;         // Set SS2 to be an output
  TRISCbits.TRISC0 = 0;         // Set SDO2 to be an output
  LATBbits.LATB3 = 0;           // Set DRV_ENN low to turn on drivers
  TRISBbits.TRISB3 = 0;         // And make it an output
  
  RPINR21 = 17;                 // Set SDI2 to use RC6 pin
  RPOR11 = 9;                   // Set SDO2 to use RC0 pin
  RPOR13 = 10;                  // Set SCK2 to use RC2 pin
#else
//// FOR EBB V4.0  
  // Set up SPI2 and RP pins
  // Inputs
  // SDI2 RPINR21    RB4 (RP7)
  // Outputs
  // SDO2 9          RB5 (RP8)
  // SCK2 10         RA0 (RP0)
  // SS2             RA2
  // DRV_ENN         RB3
  
  LATAbits.LATA0 = 0;           // Init SCK2 to idle state (low)
  TRISAbits.TRISA0 = 0;         // Set SCK2 to be an output
  TRISBbits.TRISB4 = 1;         // Set SDI2 to be an input
  LATAbits.LATA2 = 1;           // Init SS2 to idle state (high)
  TRISAbits.TRISA2 = 0;         // Set SS2 to be an output
  TRISBbits.TRISB5 = 0;         // Set SDO2 to be an output
  LATBbits.LATB3 = 0;           // Set DRV_ENN low to turn on drivers
  TRISBbits.TRISB3 = 0;         // And make it an output
  
  RPINR21 = 7;                  // Set SDI2 to use RP7 pin
  RPOR8 = 9;                    // Set SDO2 to use RP8 pin
  RPOR0 = 10;                   // Set SCK2 to use RP0 pin
#endif
  
  SSP2STATbits.SMP = 0;         // Input data is sampled at middle of data output time
  SSP2STATbits.CKE = 0;         // Transmit occurs on idle to active clock edge
  SSP2CON1bits.SSPM = 0b0010;   // SPI clock = Fosc/64
  SSP2CON1bits.CKP = 1;         // Clock idles high
  SSP2CON1bits.SSPEN = 1;       // Turn MSSP2 (SPI2) peripheral on

  spi_send_receive(&temp, 0x00 + 0x01, 0x00000000);   // Read GStat
  spi_send_receive(&temp, 0x00 + 0x01, 0x00000000);   // Read GStat
  spi_send_receive(&temp, 0x00 + 0x01, 0x00000000);   // Read GStat
  
//  spi_send(0x80 + 0x00, 0x00000008);   // GCONF=8: Enable PP and INT outputs
//  spi_send(0x80 + 0x6C, 0x000100C5);   // CHOPCONF: TOFF=5, HSRTR=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
//  spi_send(0x80 + 0x7C, 0x000100C5);   // CHOPCONF: TOFF=5, HSRTR=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
//  spi_send(0x80 + 0x30, 0x00011F05);   // IHOLD_IRUN: IHOLD=5, IRUN=31 (max current), IHOLDDELAY=1
//  spi_send(0x80 + 0x50, 0x00011F05);   // IHOLD_IRUN: IHOLD=5, IRUN=31 (max current), IHOLDDELAY=1
//  spi_send(0x80 + 0x10, 0x000401C8);   // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
//  spi_send(0x80 + 0x18, 0x000401C8);   // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
//  spi_send(0x80 + 0x32, 0x00061A80);   // VHIGH=400 000: Set VHIGH to a high value to allow stealthChop
//  spi_send(0x80 + 0x52, 0x00061A80);   // VHIGH=400 000: Set VHIGH to a high value to allow stealthChop
//  spi_send(0x80 + 0x31, 0x00007530);   // VCOOLTHRS=30000: Set upper limit for stealthChop to about 30RPM
//  spi_send(0x80 + 0x51, 0x00007530);   // VCOOLTHRS=30000: Set upper limit for stealthChop to about 30RPM
  
//  spi_send(0x80 + 0x24, 0x000003E8);   // A1 1 000 First acceleration
//  spi_send(0x80 + 0x44, 0x000003E8);   // A1 1 000 First acceleration
//  //spi_send(0x80 + 0x24, 0x00000001);   // A1 0 First acceleration
//  //spi_send(0x80 + 0x44, 0x00000001);   // A1 0 First acceleration
//  spi_send(0x80 + 0x25, 0x0000C350);   // V1 = 50 000 Acceleration threshold velocity V1
//  spi_send(0x80 + 0x45, 0x0000C350);   // V1 = 50 000 Acceleration threshold velocity V1
//  //spi_send(0x80 + 0x25, 0x00000000);   // V1 = 0 Acceleration threshold velocity V1 (disables A1 and D1 phase)
//  //spi_send(0x80 + 0x45, 0x00000000);   // V1 = 0 Acceleration threshold velocity V1 (disables A1 and D1 phase)
//  //spi_send(0x80 + 0x25, 0x00004E20);   // VSTART = 200 000
//  //spi_send(0x80 + 0x45, 0x00004E20);   // VSTART = 200 000
//  spi_send(0x80 + 0x26, 0x000001F4);   // AMAX = 500 Acceleration above V1
//  spi_send(0x80 + 0x46, 0x000001F4);   // AMAX = 500 Acceleration above V1
//  spi_send(0x80 + 0x26, 0x00000001);   // AMAX = 0 Acceleration above V1
//  spi_send(0x80 + 0x46, 0x00000001);   // AMAX = 0 Acceleration above V1
//  spi_send(0x80 + 0x27, 0x00004E20);   // VMAX = 200 000
//  spi_send(0x80 + 0x47, 0x00004E20);   // VMAX = 200 000
//  spi_send(0x80 + 0x28, 0x000002BC);   // DMAX = 700 Deceleration above V1
//  spi_send(0x80 + 0x48, 0x000002BC);   // DMAX = 700 Deceleration above V1
//  spi_send(0x80 + 0x28, 0x00000001);   // DMAX = 1 Deceleration above V1
//  spi_send(0x80 + 0x48, 0x00000001);   // DMAX = 1 Deceleration above V1
//  spi_send(0x80 + 0x2A, 0x00000578);   // D1 = 1400 Deceleration below V1
//  spi_send(0x80 + 0x4A, 0x00000578);   // D1 = 1400 Deceleration below V1
//  spi_send(0x80 + 0x2A, 0x00000001);   // D1 = 1 Deceleration below V1
//  spi_send(0x80 + 0x4A, 0x00000001);   // D1 = 1 Deceleration below V1
//  spi_send(0x80 + 0x2B, 0x00004E20);   // VSTOP = 10 Stop velocity (Near to zero)
//  spi_send(0x80 + 0x4B, 0x00004E20);   // VSTOP = 10 Stop velocity (Near to zero)
//  spi_send(0x80 + 0x20, 0x00000000);   // RAMPMODE = 0 (Target position move)
//  spi_send(0x80 + 0x40, 0x00000000);   // RAMPMODE = 0 (Target position move)
//  spi_send(0x80 + 0x21, 0x00000000);   // XACTUAL = 0
//  spi_send(0x80 + 0x41, 0x00000000);   // XACTUAL = 0
//  spi_send(0x80 + 0x2D, 0xFFFF3800);   // XTARGET = -51200 (move one rotation left (200*256 microsteps))
//  spi_send(0x80 + 0x4D, 0xFFFF3800);   // XTARGET = -51200 (move one rotation left (200*256 microsteps))
}

/// TODO: Add timeout?
UINT32 spi_send_receive(UINT8 * recieved_command, UINT8 send_command, UINT32 sendData)
{
  volatile UINT8 temp;
  UINT8 sd1, sd2, sd3, sd4;
  UINT32 result = 0;
  UINT8 dummy;
  
  /// TODO! Find out why this is necessary. What other code is overwriting this between
  /// booup and first TS command?
  RPOR0 = 10;                   // Set SCK2 to use RP0 pin

  sd1 = (sendData >> 24) & 0xFF;
  sd2 = (sendData >> 16) & 0xFF;
  sd3 = (sendData >> 8) & 0xFF;
  sd4 = sendData & 0xFF;

#if defined(EBB_2_3)
  LATDbits.LATD1 = 0;           // Chip select goes low
#else  
  LATAbits.LATA2 = 0;           // Chip select goes low
#endif
  
  dummy = SSP2BUF;
  SSP2BUF = send_command;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  *recieved_command = SSP2BUF;
  SSP2BUF = sd1;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  result = SSP2BUF;
  SSP2BUF = sd2;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  result = (result << 8) | SSP2BUF;
  SSP2BUF = sd3;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  result = (result << 8) | SSP2BUF;
  SSP2BUF = sd4;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  result = (result << 8) | SSP2BUF;

#if defined(EBB_2_3)
  LATDbits.LATD1 = 1;           // Chip select goes high
#else  
  LATAbits.LATA2 = 1;           // Chip select goes high
#endif
  return(result);
}

// The SPI Send/Receive
// Usage: SS,<send_command>,<send_data>><CR>
// <send_comamnd> : hex value (0 to FF) which is sent as the command
// <send_data>    : hex value (0 to FFFFFFFF) which is sent as the data
// Reply:
// <received_command>,<received_data>,<cr><lf>
// <received_command> : hex value (00 to FF)
// <recevied_data>    : hex value (00000000 to FFFFFFFF)
void parse_TS_packet (void)
{
  UINT32 temp;
  UINT8 send_cmd;
  UINT8 rec_cmd;
  UINT32 send_data;
  UINT32 rec_data;

  if (extract_number(kHEX_VALUE, &temp, kREQUIRED) != kEXTRACT_OK)
  {
    return;
  }
  if (extract_number(kHEX_VALUE, &send_data, kREQUIRED) != kEXTRACT_OK)
  {
    return;
  }

  send_cmd = temp;
  
  rec_data = spi_send_receive(&rec_cmd, send_cmd, send_data);

  printf((far rom char *)"%02X,%08lX\r\n", 
    rec_cmd,
    rec_data
  );
}
