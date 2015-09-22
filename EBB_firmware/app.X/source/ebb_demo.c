#include "ebb_demo.h"
#include "flash.h"
#include "HardwareProfile.h"
#include "ebb.h"

/* Path format in Flash. 
 * Each command consists of a command byte, followed by zero to four data
 * bytes depending upon the command.
 * 
 * End
 *   Command Byte = 0xFF
 *   Data bytes : none
 *   This command indicates that the path is complete and playback can stop.
 *
 * Pen Up
 *   Command Byte = 0xFE
 *   Data bytes : none
 *   This command simply raises the pen
 * 
 * Pen Down
 *   Command Byte = 0xFD
 *   Data bytes : none
 *   This command simply lowers the pen
 * 
 * Short Delay
 *   Command Byte = 0xFC
 *   Data byte 0 : Delay in milliseconds (from 0 to 255ms)
 *   This command executes a short delay
 * 
 * Long Delay
 *   Command Byte = 0xFB
 *   Data byte 0 : High byte of delay in milliseconds
 *   Data byte 1 : Low byte of delay in milliseconds
 *   This command executes a long delay (from 0 to 65535 ms)
 * 
 * Store Timings
 *   Command Bytes = 0xFA
 *   Data byte 0 : High byte of "Speed when pen is down (steps/s)"
 *   Data byte 1 : Low byte of "Speed when pen is down (steps/s)"
 *   Data byte 2 : High byte of "Speed when pen is up (steps/s)"
 *   Data byte 3 : Low byte of "Speed when pen is up (steps/s)"
 *   Data byte 4 : Pen raising speed (%)
 *   Data byte 5 : High byte of "Delay after raising pen (ms)"
 *   Data byte 6 : Low byte of "Delay after raising pen (ms)"
 *   Data byte 7 : "Pen lowering speed (%)"
 *   Data byte 8 : High byte of "Delay after lowering pen (ms)"
 *   Data byte 9 : Low byte of "Delay after lowering pen (ms)"
 *   Data byte 10 : "Pen up position, 0-100%"
 *   Data byte 11 : "Pen down position, 0-100%"
 *   This command is put in the Flash once at the beginning of the recording
 *   and captures all of the settings for that recording.
 * 
 * Very Long Move
 *   Command Byte = 0xF2
 *   Data byte 0 : High byte of move duration
 *   Data byte 1 : Mid byte of move duration
 *   Data byte 2 : Low byte of move duration
 *   Data byte 3 : High byte of motor 1 steps (signed)
 *   Data byte 4 : Mid byte of motor 1 steps
 *   Data byte 5 : Low byte of motor 1 steps
 *   Data byte 6 : High byte of motor 2 steps (signed)
 *   Data byte 7 : Mid byte of motor 2 steps
 *   Data byte 8 : Low byte of motor 2 steps 
 *   This command is a move command where duration, motor 1 steps and motor 2
 *   steps are all 24 bits, and is used when a Long Move command can't store
 *   the values needed.
 *  
 * Long Move
 *   Command Byte = 0xF1
 *   Data byte 0 : High byte of move duration
 *   Data byte 1 : Low byte of move duration
 *   Data byte 2 : High byte of motor 1 steps (signed)
 *   Data byte 3 : Low byte of motor 1 steps
 *   Data byte 4 : High byte of motor 2 steps (signed)
 *   Data byte 5 : Low byte of motor 2 steps
 *   This command is a move command where duration, motor 1 steps and motor 2 
 *   steps are all 16 bits, and is used when a Short move command can't store
 *   the values needed.
 * 
 * Short Move
 *   Command Byte = 0x01 to 0xF0 (duration of move in ms)
 *   Data byte 0 : motor 1 steps (signed)
 *   Data byte 1 : motor 2 steps (signed)
 *   This command is for moves that can fit into the following limits:
 *     Duration: From 1 to 240ms
 *     Step 1: From -128 to 127
 *     Step 2: From -128 to 127
 * 
 */

#define DEMO_COMMAND_END                    0xFF
#define DEMO_COMMAND_END_LENGTH             0x01
#define DEMO_COMMAND_PEN_UP                 0xFE
#define DEMO_COMMAND_PEN_UP_LENGTH          0x01
#define DEMO_COMMAND_PEN_DOWN               0xFD
#define DEMO_COMMAND_PEN_DOWN_LENGTH        0x01
#define DEMO_COMMAND_SHORT_DELAY            0xFC
#define DEMO_COMMAND_SHORT_DELAY_LENGTH     0x02
#define DEMO_COMMAND_LONG_DELAY             0xFB
#define DEMO_COMMAND_LONG_DELAY_LENGTH      0x03
#define DEMO_COMMAND_STORE_TIMINGS          0xFA
#define DEMO_COMMAND_STORE_TIMINGS_LENGTH   0x0D
#define DEMO_COMMAND_VERY_LONG_MOVE         0xF2
#define DEMO_COMMAND_VERY_LONG_MOVE_LENGTH  0x0A
#define DEMO_COMMAND_LONG_MOVE              0xF1
#define DEMO_COMMAND_LONG_MOVE_LENGTH       0x07
#define DEMO_COMMAND_SHORT_MOVE_LENGTH      0x03



#define PF_START_ADDR       0xC000
#define PF_END_ADDR         0xFFFF
#define CONFIG_START_ADDR   0xFFF8
#define CONFIG_LENGTH_BYTES 8

/* These are the states that LED2 can be in. */
typedef enum
{
    LED2_OFF = 0,
    LED2_ON,
    LED2_FAST_BLINK,
    LED2_SLOW_BLINK,
    LED2_SHORT_PIP
} DemoLED2StateType;

/* Main state variable for demo module state. */
DemoStateType demo_state;
/* LED2 state machine state variable. */
DemoLED2StateType demo_led2_state;
/* Current read or write position in Flash */
UINT16 FlashAddr;
    
void EBBWriteBytesFlash(unsigned int num_bytes, unsigned char *flash_array, BOOL finish_up, BOOL starting_up);


/* This function erases the section of Flash that we have set aside for
* storing our image data. Note that this involves saving off and restoring
* the config bits at the last eight bytes of Flash. */
void erase_path_flash(void)
{
    UINT8 config_bits[CONFIG_LENGTH_BYTES];
    
    // Read out config bits before we erase
    ReadFlash(CONFIG_START_ADDR, CONFIG_LENGTH_BYTES, config_bits);
    
    // Erase our whole block for the path flash
    EraseFlash(PF_START_ADDR, PF_END_ADDR);
    
    // Restore config bits
    WriteBytesFlash(CONFIG_START_ADDR, CONFIG_LENGTH_BYTES, config_bits);
}

void dump_flash(void)
{
    UINT8 array[32];
    UINT16 ReadAddr = PF_START_ADDR;
    UINT8 i;
    
    printf ((far rom char *)"Dumping Flash:\r\n");
    for (i=0; i < 4; i++)
    {
        ReadFlash(ReadAddr, 32, array);
        printf((far rom char *)"%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                array[0],
                array[1],
                array[2],
                array[3],
                array[4],
                array[5],
                array[6],
                array[7],
                array[8],
                array[9],
                array[10],
                array[11],
                array[12],
                array[13],
                array[14],
                array[15],
                array[16],
                array[17],
                array[18],
                array[19],
                array[20],
                array[21],
                array[22],
                array[23],
                array[24],
                array[25],
                array[26],
                array[27],
                array[28],
                array[29],
                array[30],
                array[31]
        );
        ReadAddr += 32;
    }  
}

/* Write a simple path into flash. */
void write_simple_path_flash(void)
{   
    UINT8 Array[0x10];
    
    // Store a set of settings
    dump_flash();

    // Always start of initializing the flash write routine 
    EBBWriteBytesFlash(0, NULL, FALSE, TRUE);
    
    // And a couple very long moves
    Array[0] = DEMO_COMMAND_VERY_LONG_MOVE;
    Array[1] = 0x01;
    Array[2] = 0xD4;
    Array[3] = 0xC0;    // 120,000
    Array[4] = 0x01;
    Array[5] = 0x86;
    Array[6] = 0xA0;    // 100,000
    Array[7] = 0xFE;
    Array[8] = 0x79;
    Array[9] = 0x60;    // -100,000
    EBBWriteBytesFlash(DEMO_COMMAND_VERY_LONG_MOVE_LENGTH, Array, FALSE, FALSE);

    // Do a couple short moves
    Array[0] = 0xC8;
    Array[1] = 0xC8;
    Array[2] = 0x7F;
    EBBWriteBytesFlash(DEMO_COMMAND_SHORT_MOVE_LENGTH, Array, FALSE, FALSE); 
    
    // The a couple long moves
    Array[0] = DEMO_COMMAND_LONG_MOVE;
    Array[1] = 0x03;
    Array[2] = 0xE8;
    Array[3] = 0x03;
    Array[4] = 0xE8;
    Array[5] = 0x03;
    Array[6] = 0xE8;
    EBBWriteBytesFlash(DEMO_COMMAND_LONG_MOVE_LENGTH, Array, FALSE, FALSE); 

    Array[0] = DEMO_COMMAND_LONG_MOVE;
    Array[1] = 0x07;
    Array[2] = 0xD0;
    Array[3] = 0x0F;
    Array[4] = 0xA0;
    Array[5] = 0xF0;
    Array[6] = 0x60;
    EBBWriteBytesFlash(DEMO_COMMAND_LONG_MOVE_LENGTH, Array, FALSE, FALSE); 
    
    // Put the pen down
    Array[0] = DEMO_COMMAND_PEN_DOWN;
    EBBWriteBytesFlash(DEMO_COMMAND_PEN_DOWN_LENGTH, Array, FALSE, FALSE); 
    
    // Do a long delay
    Array[0] = DEMO_COMMAND_LONG_DELAY;
    Array[1] = 0x13;
    Array[2] = 0x88;
    EBBWriteBytesFlash(DEMO_COMMAND_LONG_DELAY_LENGTH, Array, FALSE, FALSE); 

    // Then do a long move
    Array[0] = DEMO_COMMAND_LONG_MOVE;
    Array[1] = 0x07;
    Array[2] = 0xD0;
    Array[3] = 0x0F;
    Array[4] = 0xA0;
    Array[5] = 0xF0;
    Array[6] = 0x60;
    EBBWriteBytesFlash(DEMO_COMMAND_LONG_MOVE_LENGTH, Array, FALSE, FALSE); 
            
    // Put the pen up
    Array[0] = DEMO_COMMAND_PEN_UP;
    EBBWriteBytesFlash(DEMO_COMMAND_PEN_UP_LENGTH, Array, FALSE, FALSE); 

    // Do a long delay
    Array[0] = DEMO_COMMAND_LONG_DELAY;
    Array[1] = 0x13;
    Array[2] = 0x88;
    EBBWriteBytesFlash(DEMO_COMMAND_LONG_DELAY_LENGTH, Array, FALSE, FALSE); 
    
    // Then do a short move
    Array[0] = 0xC8;
    Array[1] = 0x50;
    Array[2] = 0xA0;
    EBBWriteBytesFlash(DEMO_COMMAND_SHORT_MOVE_LENGTH, Array, FALSE, FALSE); 
    
    // Do a short delay
    Array[0] = DEMO_COMMAND_SHORT_DELAY;
    Array[1] = 0xC8;
    EBBWriteBytesFlash(DEMO_COMMAND_SHORT_DELAY_LENGTH, Array, FALSE, FALSE); 
    
    // Then do a short move
    Array[0] = 0xC8;
    Array[1] = 0x50;
    Array[2] = 0xA0;
    EBBWriteBytesFlash(DEMO_COMMAND_SHORT_MOVE_LENGTH, Array, FALSE, FALSE); 
    
    // Do a long delay
    Array[0] = DEMO_COMMAND_LONG_DELAY;
    Array[1] = 0x13;
    Array[2] = 0x88;
    EBBWriteBytesFlash(DEMO_COMMAND_LONG_DELAY_LENGTH, Array, FALSE, FALSE); 
    
    // Then do a short move
    Array[0] = 0xC8;
    Array[1] = 0x50;
    Array[2] = 0xA0;
    EBBWriteBytesFlash(DEMO_COMMAND_SHORT_MOVE_LENGTH, Array, FALSE, FALSE); 
    
    // And end
    Array[0] = DEMO_COMMAND_END;
    EBBWriteBytesFlash(DEMO_COMMAND_END_LENGTH, Array, FALSE, FALSE); 

    // And finish writing to flash
    EBBWriteBytesFlash(0, NULL, TRUE, FALSE); 
}

/* Restart playing at location zero. */
void demo_play_init(void)
{
    // Start our address pointer at the beginning of our demo Flash
    FlashAddr = PF_START_ADDR;
}

/* See if it's time for a new move/command and send it on it's way.*/
void demo_play(void)
{
    UINT8 Command = 0;
    UINT8 Array[0x10];
    UINT32 Duration;
    INT32 Step1;
    INT32 Step2;
    
    // If there is room in the buffer
    if (FIFOEmpty)
    {
        // Then load up the next command from Flash 
        ReadFlash(FlashAddr, 1, &Command);
        printf ((far rom char *)"CMD: 0x%02X Addr: 0x%04X\r\n", Command, FlashAddr);
        
        // Based on the Command, read out more bytes
        switch (Command)
        {
            case DEMO_COMMAND_END:
                demo_state = DEMO_IDLE;
                demo_led2_state = LED2_OFF;
                printf ((far rom char *)"Command: End\r\n");                
                FlashAddr += DEMO_COMMAND_END_LENGTH;
                break;
            case DEMO_COMMAND_PEN_UP:
                printf ((far rom char *)"Command: Pen Up\r\n");                
        		process_SP(PEN_UP, 0);
                FlashAddr += DEMO_COMMAND_PEN_UP_LENGTH;
                break;
            case DEMO_COMMAND_PEN_DOWN:
                printf ((far rom char *)"Command: Pen Down\r\n");                
        		process_SP(PEN_DOWN, 0);
                FlashAddr += DEMO_COMMAND_PEN_DOWN_LENGTH;
                break;
            case DEMO_COMMAND_SHORT_DELAY:
                printf ((far rom char *)"Command: Short Delay\r\n");                
                ReadFlash(FlashAddr, DEMO_COMMAND_SHORT_DELAY_LENGTH, Array);
                printf ((far rom char *)"%02X %02X %02X %02X %02X %02X %02X\r\n", Array[0], Array[1], Array[2], Array[3], Array[4], Array[5], Array[6]);
                Duration = Array[1];
                process_SM(Duration, 0, 0);
                FlashAddr += DEMO_COMMAND_SHORT_DELAY_LENGTH;
                break;
            case DEMO_COMMAND_LONG_DELAY:
                printf ((far rom char *)"Command: Long Delay\r\n");                
                ReadFlash(FlashAddr, DEMO_COMMAND_LONG_DELAY_LENGTH, Array);
                printf ((far rom char *)"%02X %02X %02X %02X %02X %02X %02X\r\n", Array[0], Array[1], Array[2], Array[3], Array[4], Array[5], Array[6]);
                Duration = (((UINT32)Array[1]) << 8) + Array[2];
                process_SM(Duration, 0, 0);
                FlashAddr += DEMO_COMMAND_LONG_DELAY_LENGTH;
                break;
            case DEMO_COMMAND_STORE_TIMINGS:
                printf ((far rom char *)"Command: Store Timings\r\n");                
                ReadFlash(FlashAddr, DEMO_COMMAND_STORE_TIMINGS_LENGTH, Array);
                printf ((far rom char *)"%02X %02X %02X %02X %02X %02X %02X\r\n", Array[0], Array[1], Array[2], Array[3], Array[4], Array[5], Array[6]);
                FlashAddr += DEMO_COMMAND_STORE_TIMINGS_LENGTH;
                break;
            case DEMO_COMMAND_VERY_LONG_MOVE:
                printf ((far rom char *)"Command: Very Long Move\r\n");                
                ReadFlash(FlashAddr, DEMO_COMMAND_VERY_LONG_MOVE_LENGTH, Array);
                printf ((far rom char *)"%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n", Array[0], Array[1], Array[2], Array[3], Array[4], Array[5], Array[6], Array[7], Array[8], Array[9]);
                Duration = (((UINT32)Array[1]) << 16) + ((UINT32)Array[2] << 8) + Array[3];
                Step1 = (((INT32)(INT8)Array[4]) << 16) + ((UINT32)Array[5] << 8) + Array[6];
                Step2 = (((INT32)(INT8)Array[7]) << 16) + ((UINT32)Array[8] << 8) + Array[9];
                process_SM(Duration, Step1, Step2);
                FlashAddr += DEMO_COMMAND_VERY_LONG_MOVE_LENGTH;
                break;
            case DEMO_COMMAND_LONG_MOVE:
                printf ((far rom char *)"Command: Long Move\r\n");                
                ReadFlash(FlashAddr, DEMO_COMMAND_LONG_MOVE_LENGTH, Array);
                printf ((far rom char *)"%02X %02X %02X %02X %02X %02X %02X\r\n", Array[0], Array[1], Array[2], Array[3], Array[4], Array[5], Array[6]);
                Duration = ((UINT32)Array[1] << 8) + Array[2];
                Step1 = (((INT32)(INT8)Array[3]) << 8) + Array[4];
                Step2 = (((INT32)(INT8)Array[5]) << 8) + Array[6];
                process_SM(Duration, Step1, Step2);
                FlashAddr += DEMO_COMMAND_LONG_MOVE_LENGTH;
                break;
            default:
                printf ((far rom char *)"Command: Short Move\r\n");                
                ReadFlash(FlashAddr, DEMO_COMMAND_SHORT_MOVE_LENGTH, Array);
                printf ((far rom char *)"%02X %02X %02X\r\n", Array[0], Array[1], Array[2]);
                Duration = Array[0];
                Step1 = ((INT32)(INT8)Array[1]);
                Step2 = ((INT32)(INT8)Array[2]);
                process_SM(Duration, Step1, Step2);
                FlashAddr += DEMO_COMMAND_SHORT_MOVE_LENGTH;
                break;
        }
    }
    
}

/* Handle the LED2 state machine. We can be off, on,
* fast blink (50ms on, 50ms off), slow blink (1s on, 1s off),
* Or fast 'pip' (50ms on, 1950ms off).
* Other functions can just set demo_led2_state directly. */
void demo_blink_led2(void)
{
    switch(demo_led2_state)
    {
        case LED2_OFF:
            mLED_2_Off();
            USR_LED_ms_timer = 0;
            break;
            
        case LED2_ON:
            mLED_2_On();
            USR_LED_ms_timer = 0;
            break;
            
        case LED2_FAST_BLINK:
            if (USR_LED_ms_timer == 0)
            {
                USR_LED_ms_timer = 100;
                mLED_2_On();
            }
            else if (USR_LED_ms_timer < 50)
            {
                mLED_2_Off();
            }
            break;
            
        case LED2_SLOW_BLINK:
            if (USR_LED_ms_timer == 0)
            {
                USR_LED_ms_timer = 2000;
                mLED_2_On();
            }
            else if (USR_LED_ms_timer < 1000)
            {
                mLED_2_Off();
            }
            break;
            
        case LED2_SHORT_PIP:
            if (USR_LED_ms_timer == 0)
            {
                USR_LED_ms_timer = 2000;
                mLED_2_On();
            }
            else if (USR_LED_ms_timer < 1950)
            {
                mLED_2_Off();
            }
            break;
            
        default:
            break;
    }
}

/* Initialize the demo module to power on state. */
void demo_init(void)
{
    demo_state = DEMO_IDLE;
    demo_led2_state = LED2_OFF;
    // This timer has to start all the way as high as it can go on boot
    PRG_ms_timer = 0xFFFF;
    FlashAddr = PF_START_ADDR;
    
}

/* Main demo module run function. Gets called from mainline code once
 * each time through main loop. Watch for PRG button presses to see if 
 * user wants to change states. Call playback function and LED2 handler.*/
void demo_run(void)
{
    // Check for a PRG switch closure
    if (!swProgram)
    {
        // Do nothing here - primarily don't clear PRG_ms_timer to zero;
        if (PRG_ms_timer > 2000)
        {
           demo_led2_state = LED2_ON;
        }
    }
    else
    {
        // If we have a button push between 100ms and 2000ms
        if (PRG_ms_timer > 100 && PRG_ms_timer < 2000)
        {
            // If we are not already playing, then start playback
            switch (demo_state)
            {
                case DEMO_IDLE:
                    printf ((far rom char *)"Playback started.\r\n");
                    // Now that we're doing this, let's blink the USER led slowly
                    demo_state = DEMO_PLAYING;
                    demo_led2_state = LED2_SLOW_BLINK;
                    demo_play_init();
                    break;
                    
                case DEMO_RECORDING:
                    printf ((far rom char *)"Recording Stopped.\r\n");
                    demo_state = DEMO_IDLE;
                    demo_led2_state = LED2_OFF;
                    break;
                    
                case DEMO_PLAYING:
                    printf ((far rom char *)"Playback paused.\r\n");
                    demo_state = DEMO_PLAYING_PAUSED;
                    demo_led2_state = LED2_SHORT_PIP;
                    break;
                    
                case DEMO_PLAYING_PAUSED:
                    printf ((far rom char *)"Playback continuing.\r\n");
                    demo_led2_state = LED2_SLOW_BLINK;
                    demo_state = DEMO_PLAYING;
                    break;
                    
                default:
                    break;
            }
        }
        // If we have a button push more than 2 seconds long
        else if (PRG_ms_timer > 2000 && PRG_ms_timer != 0xFFFF)
        {
             switch (demo_state)
            {
                case DEMO_IDLE:
                    printf ((far rom char *)"Recording.\r\n");
                    // Now that we're doing this, let's turn the USER led on
                    erase_path_flash();

                    /* Later we can make this record real data from the PC. For now
                     we just write a dummy pattern into Flash.*/
                    write_simple_path_flash();
                    
                    demo_state = DEMO_RECORDING;
                    demo_led2_state = LED2_FAST_BLINK;
                    break;
                    
                case DEMO_RECORDING:
                    printf ((far rom char *)"Recording Stopped.\r\n");
                    demo_state = DEMO_IDLE;
                    demo_led2_state = LED2_OFF;
                    break;
                    
                case DEMO_PLAYING:
                    printf ((far rom char *)"Playback Stopped.\r\n");
                    demo_state = DEMO_IDLE;
                    demo_led2_state = LED2_OFF;
                    break;
                    
                case DEMO_PLAYING_PAUSED:
                    printf ((far rom char *)"Playback Stopped.\r\n");
                    demo_state = DEMO_IDLE;
                    demo_led2_state = LED2_OFF;
                    break;
                    
                default:
                    break;
            }
        }
        // Always keep resetting this to zero if the button is not pressed
        PRG_ms_timer = 0;    
    }
    
    // Handle the blinking of LED2
    demo_blink_led2();
    
    // If we are playing back, then call the playback function to see if it's
    // time for the next move.
    if (demo_state == DEMO_PLAYING)
    {
        demo_play();
    }
}

/* This function simply performs a 64 byte (block) write.
 * It assumes that everything (TBLPTR) has all been set up already.*/
void EBBBlockWrite(void)
{
    // Flag to remember if interrupts were enabled or not when we start writing to flash
    BOOL ISRflag = FALSE;

    //*********** Flash write sequence ***********************************
    EECON1bits.WREN = 1;
    if(INTCONbits.GIE)
    {
        INTCONbits.GIE = 0;
        ISRflag = TRUE;
    }		  
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR =1;
    EECON1bits.WREN = 0 ; 
    if(ISRflag)
    {
        INTCONbits.GIE = 1;	
        ISRflag = 0;
    }
}

/* This routine queues up byte writes to Flash, doing a 64 byte block write
 * whenever needed. Call with finish_up = TRUE to do the final write (no matter
 * if it's on a block boundary or not. 
 * Set starting_up = TRUE on the first call to this routine to reset the 
 * address pointer to the beginning of our path flash. */
void EBBWriteBytesFlash(unsigned int num_bytes, unsigned char *flash_array, BOOL finish_up, BOOL starting_up)
{
//    unsigned char write_byte=0;
    // 32 bit value used to transfer starting address to TBLPTR
	DWORD_VAL flash_addr;
    static UINT8 block_byte_count;
    
    // If this is the first call to the function, start our pointer at the right location
    if (starting_up == TRUE)
    {
        // We are assuming that CONFIG_START_ADDR is always on a block boundary
        FlashAddr = PF_START_ADDR;
        block_byte_count = FLASH_WRITE_BLOCK;
    }

    // Always refresh this value before we start writing
    flash_addr.Val = FlashAddr;

    // Load the address to Address pointer registers
    TBLPTRU = flash_addr.byte.UB;
    TBLPTRH = flash_addr.byte.HB;	
    TBLPTRL	= flash_addr.byte.LB;

    while(num_bytes)
    {
        // Put data byte in TABLAT write register
        TABLAT = *flash_array++;
        
        block_byte_count--;
        // Count this byte in our master pointer into flash
        FlashAddr++;
        
        // Are we on the last byte of the block?
        if (block_byte_count == 0)
        {
            // Yes, so don't increment the TBLPTR
            // Load the data byte into the holding register
            _asm  TBLWT _endasm
            
            // Now perform write of the block
//            EBBBlockWrite();
            
            //
            block_byte_count = FLASH_WRITE_BLOCK;

            // Reload the TBLPTR with the new block address
            flash_addr.Val = FlashAddr;

            // Load the address to Address pointer registers
            TBLPTRU = flash_addr.byte.UB;
            TBLPTRH = flash_addr.byte.HB;	
            TBLPTRL	= flash_addr.byte.LB;
        }
        else
        {
            // Load the data byte into the holding registers and increment
            _asm  TBLWTPOSTINC 	_endasm
        }        
    }
    
    // If finish_up is true, caller wants to close out this block even if its
    // not actually full.
    if (finish_up == TRUE && block_byte_count != FLASH_WRITE_BLOCK)
    {
//        EBBBlockWrite();
    }

    // For debugging, we print out the state of flash after every operation
    dump_flash();	
}

