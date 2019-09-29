#ifndef _NODE_TYPES_H
#define _NODE_TYPES_H

#include <stdint.h>

#define STRUCT_TYPE_ELEVATOR_CONTROL 0x13

typedef struct elevator_t
{
    uint8_t stype = STRUCT_TYPE_ELEVATOR_CONTROL;
    uint8_t battery = 0;
    uint8_t touch_status = 0;
    uint8_t velocity = 0;
    uint8_t intensity = 0;
} elevator_t;

#endif
