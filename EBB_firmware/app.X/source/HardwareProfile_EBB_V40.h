/*********************************************************************
 *
 *                EiBotBoard Firmware
 *
 *********************************************************************
 * FileName:        HardwareProfile_EBB_V13_and_above.h
 * Company:         Schmalz Haus LLC
 * Author:          Brian Schmalz
 *
 * Based on original files by Microchip Inc. in MAL USB example.
 *
 * Software License Agreement
 *
 * Copyright (c) 2014, Brian Schmalz of Schmalz Haus LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials
 * provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of
 * its contributors may be used to endorse or promote products
 * derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef HARDWARE_PROFILE_EBB_V40_H
#define HARDWARE_PROFILE_EBB_V40_H

  /*******************************************************************/
  /******** USB stack hardware selection options *********************/
  /*******************************************************************/
  //This section is the set of definitions required by the MCHPFSUSB
  //  framework.  These definitions tell the firmware what mode it is
  //  running in, and where it can find the results to some information
  //  that the stack needs.
  //These definitions are required by every application developed with
  //  this revision of the MCHPFSUSB framework.  Please review each
  //  option carefully and determine which options are desired/required
  //  for your application.

  //The PICDEM FS USB Demo Board platform supports the USE_SELF_POWER_SENSE_IO
  //and USE_USB_BUS_SENSE_IO features.  Uncomment the below line(s) if
  //it is desirable to use one or both of the features.
  //#define USE_SELF_POWER_SENSE_IO
  #define tris_self_power     TRISAbits.TRISA2    // Input
  #if defined(USE_SELF_POWER_SENSE_IO)
  #define self_power          PORTAbits.RA2
  #else
  #define self_power          1
  #endif

  #define USE_USB_BUS_SENSE_IO
  #define tris_usb_bus_sense  TRISCbits.TRISC1    // Input
  #if defined(USE_USB_BUS_SENSE_IO)
  #define USB_BUS_SENSE       PORTCbits.RC1
  #else
  #define USB_BUS_SENSE       1
  #endif

  //Uncomment the following line to make the output HEX of this  
  //  project work with the MCHPUSB Bootloader    
  //#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER

  //Uncomment the following line to make the output HEX of this 
  //  project work with the HID Bootloader
  //#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER

  /*******************************************************************/
  /*******************************************************************/
  /*******************************************************************/
  /******** Application specific definitions *************************/
  /*******************************************************************/
  /*******************************************************************/
  /*******************************************************************/

  /** Board definition ***********************************************/
  //These definitions will tell the main() function which board is
  //  currently selected.  This will allow the application to add
  //  the correct configuration bits as wells use the correct
  //  initialization functions for the board.  These definitions are only
  //  required in the stack provided demos.  They are not required in
  //  final application design.
  #define CLOCK_FREQ 48000000

	/** L E D ***********************************************************/
	/* On EBB v40, LED1 (USB) = RC6, LED2 (USR) = RD3, SW = RC7			*/
	#define mInitAllLEDs()      LATDbits.LATD3 = 0; LATCbits.LATC6 = 0; TRISDbits.TRISD3 = 0; TRISCbits.TRISC6 = 0;
	#define mLED_1              LATCbits.LATC6
	#define mLED_2              LATDbits.LATD3

	/** S W I T C H *****************************************************/
	#define mInitSwitch()       TRISCbits.TRISC7 = INPUT_PIN;
	#define swProgram           PORTCbits.RC7

	/** R E F   A N A L O G   I N P U T *********************************/
	#define RefRA0_IO_TRIS      TRISAbits.TRISA0

	/** P E N   U P  D O W N *******************************************/
	#define PenUpDownIO         LATDbits.LATD4
  #define PEN_UP_DOWN_RPN     21
	#define PenUpDownIO_TRIS    TRISDbits.TRISD4
    
  /** R C   S E R V O  ************************************************/
  #define RCServoPowerIO_TRIS TRISAbits.TRISA3
  #define RCServoPowerIO      LATAbits.LATA3
  #define RCServoPowerIO_PORT PORTAbits.RA3
  #define RCSERVO_POWER_ON    1
  #define RCSERVO_POWER_OFF   0
    
	/** G E N E R I C ***************************************************/
	
	#define mLED_USB_Toggle()   mLED_1 = !mLED_1;
	
	#define mLED_1_On()         mLED_1 = 1;
	#define mLED_2_On()         mLED_2 = 1;
	
	#define mLED_1_Off()        mLED_1 = 0;
	#define mLED_2_Off()        mLED_2 = 0;
	
	#define mLED_1_Toggle()     mLED_1 = !mLED_1;
	#define mLED_2_Toggle()     mLED_2 = !mLED_2;
	
	#define mLED_Both_Off()     {mLED_1_Off(); mLED_2_Off();}
	#define mLED_Both_On()      {mLED_1_On(); mLED_2_On();}
	#define mLED_Only_1_On()    {mLED_1_On(); mLED_2_Off();}
	#define mLED_Only_2_On()    {mLED_1_Off(); mLED_2_On();}
	
#endif  //HARDWARE_PROFILE_EBB_V13_H
