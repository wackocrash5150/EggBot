#include <p18cxxx.h>
#include <ctype.h>
#include <GenericTypeDefs.h>
#include "spi.h"
#include "ubw.h"

void spi_init(void)
{
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
  
  SSP2STATbits.SMP = 0;         // Input data is sampled at middle of data output time
  SSP2STATbits.CKE = 0;         // Transmit occurs on idle to active clock edge
  SSP2CON1bits.SSPM = 0b0010;   // SPI clock = Fosc/64
  SSP2CON1bits.CKP = 1;         // Clock idles high
  SSP2CON1bits.SSPEN = 1;       // Turn MSSP2 (SPI2) peripheral on

//  spi_send(0x00 + 0x01, 0x00000000);   // Read GStat
//  spi_send(0x00 + 0x01, 0x00000000);   // Read GStat
//  spi_send(0x00 + 0x01, 0x00000000);   // Read GStat
  
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
///  spi_send(0x80 + 0x24, 0x00000001);   // A1 0 First acceleration
///  spi_send(0x80 + 0x44, 0x00000001);   // A1 0 First acceleration
  //spi_send(0x80 + 0x25, 0x0000C350);   // V1 = 50 000 Acceleration threshold velocity V1
  //spi_send(0x80 + 0x45, 0x0000C350);   // V1 = 50 000 Acceleration threshold velocity V1
///  spi_send(0x80 + 0x25, 0x00000000);   // V1 = 0 Acceleration threshold velocity V1 (disables A1 and D1 phase)
///  spi_send(0x80 + 0x45, 0x00000000);   // V1 = 0 Acceleration threshold velocity V1 (disables A1 and D1 phase)
///  spi_send(0x80 + 0x25, 0x00004E20);   // VSTART = 200 000
///  spi_send(0x80 + 0x45, 0x00004E20);   // VSTART = 200 000
  //spi_send(0x80 + 0x26, 0x000001F4);   // AMAX = 500 Acceleration above V1
  //spi_send(0x80 + 0x46, 0x000001F4);   // AMAX = 500 Acceleration above V1
///  spi_send(0x80 + 0x26, 0x00000001);   // AMAX = 0 Acceleration above V1
///  spi_send(0x80 + 0x46, 0x00000001);   // AMAX = 0 Acceleration above V1
///  spi_send(0x80 + 0x27, 0x00004E20);   // VMAX = 200 000
///  spi_send(0x80 + 0x47, 0x00004E20);   // VMAX = 200 000
  //spi_send(0x80 + 0x28, 0x000002BC);   // DMAX = 700 Deceleration above V1
  //spi_send(0x80 + 0x48, 0x000002BC);   // DMAX = 700 Deceleration above V1
///  spi_send(0x80 + 0x28, 0x00000001);   // DMAX = 1 Deceleration above V1
///  spi_send(0x80 + 0x48, 0x00000001);   // DMAX = 1 Deceleration above V1
  //spi_send(0x80 + 0x2A, 0x00000578);   // D1 = 1400 Deceleration below V1
  //spi_send(0x80 + 0x4A, 0x00000578);   // D1 = 1400 Deceleration below V1
///  spi_send(0x80 + 0x2A, 0x00000001);   // D1 = 1 Deceleration below V1
///  spi_send(0x80 + 0x4A, 0x00000001);   // D1 = 1 Deceleration below V1
///  spi_send(0x80 + 0x2B, 0x00004E20);   // VSTOP = 10 Stop velocity (Near to zero)
///  spi_send(0x80 + 0x4B, 0x00004E20);   // VSTOP = 10 Stop velocity (Near to zero)
///  spi_send(0x80 + 0x20, 0x00000000);   // RAMPMODE = 0 (Target position move)
///  spi_send(0x80 + 0x40, 0x00000000);   // RAMPMODE = 0 (Target position move)
///  spi_send(0x80 + 0x21, 0x00000000);   // XACTUAL = 0
///  spi_send(0x80 + 0x41, 0x00000000);   // XACTUAL = 0
///  spi_send(0x80 + 0x2D, 0xFFFF3800);   // XTARGET = -51200 (move one rotation left (200*256 microsteps))
///  spi_send(0x80 + 0x4D, 0xFFFF3800);   // XTARGET = -51200 (move one rotation left (200*256 microsteps))
}

/// TODO: Add timeout?
UINT32 spi_send_receive(UINT8 * recieved_command, unsigned char send_command, unsigned long int sendData)
{
  volatile unsigned char temp;
  unsigned char sd1, sd2, sd3, sd4;
  UINT32 result = 0;
  
  sd1 = sendData >> 24;
  sd2 = sendData >> 16;
  sd3 = sendData >> 8;
  sd4 = sendData;
  
  LATAbits.LATA2 = 0;           // Chip select goes low
  
  *recieved_command = SSP2BUF;
  SSP2BUF = send_command;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  result = SSP2BUF;
  SSP2BUF = sd1;
  while (SSP2STATbits.BF == 0)
  {
    ;
  }
  result = (result << 8) | SSP2BUF;
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
  
  LATAbits.LATA2 = 1;           // Chip select goes high
  return(result);
}

UINT8 extractHex(void)
{
  UINT8 temp = 0;
  UINT8 c1, c2;

  extract_number (kASCII_CHAR, &c1, kREQUIRED);
  extract_number (kASCII_CHAR, &c2, kREQUIRED);

  if (c1 >= 48 && c1 <= 57)
  {
    c1 = c1 - 48;
  }
  else if (c1 >= 65 && c1 <= 70)
  {
    c1 = c1 - 75;
  }
  else
  {
    c1 = 0;
  }
  if (c2 >= 48 && c2 <= 57)
  {
    c2 = c2 - 48;
  }
  else if (c2 >= 65 && c2 <= 70)
  {
    c2 = c2 - 75;
  }
  else
  {
    c2 = 0;
  }
  temp = c1 << 4 | c2;
  return(temp);
}

// The SPI Send/Receive
// Usage: SS,<byte1>,<byte2>,<byte3>,<byte4>,<byte5><CR>
// <byte1> .. <byte5> Bytes sent out, starting with byte1
// Reply:
// <reply_byte1>,<reply_byte2>,<reply_byte3>,<reply_byte4>,<reply_byte5>,<cr><lf>
void parse_TS_packet (void)
{
  UINT8 send_cmd;
  UINT8 rec_cmd;
  UINT32 send_data;
  UINT32 rec_data;

  send_cmd = extractHex();
  send_data = extractHex();
  send_data = (send_data << 8) || extractHex();
  send_data = (send_data << 8) || extractHex();
  send_data = (send_data << 8) || extractHex();

  // Bail if we got a conversion error
  if (error_byte)
  {
    return;
  }
  rec_data = spi_send_receive(&rec_cmd, send_cmd, send_data);

  printf((far rom char *)"%0X,%0X,%0X,%0X,%0X\r\n", 
    rec_cmd,
    rec_data >> 24,
    (rec_data >> 16) & 0xFF,
    (rec_data >> 8) & 0xFF,
    rec_data & 0xFF
  );
}
