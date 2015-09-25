#ifndef BB_DEMO_H
#define EBB_DEMO_H

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "ebb.h"

typedef enum {
    DEMO_IDLE = 0,
    DEMO_RECORDING,
    DEMO_PLAYING,
    DEMO_PLAYING_PAUSED
} DemoStateType;


extern DemoStateType demo_state;

extern void erase_path_flash(void);
extern void write_simple_path_flash(void);
extern void demo_play(void);
extern void demo_run(void);
extern void demo_init(void);
extern BOOL demo_is_recording(void);
extern void demo_write_delay(UINT32 delay_ms);
extern void demo_write_pen_state(PenStateType NewState, UINT16 CommandDuration);
extern void demo_write_move(UINT32 Duration, INT32 A1Stp, INT32 A2Stp);

#endif