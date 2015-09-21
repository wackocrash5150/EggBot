#ifndef BB_DEMO_H
#define EBB_DEMO_H

#include "GenericTypeDefs.h"
#include "Compiler.h"

typedef enum {
	COMD_END = 0,
	COMD_SM, 
	COMD_SP
} comd_type;

typedef struct {
	comd_type comd;
	unsigned int duration;
	signed int A1steps;
	signed int A2steps;
} packet_type;

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

#endif