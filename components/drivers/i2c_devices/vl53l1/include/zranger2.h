/**
 *
 * ESP-Drone Firmware
 * 
 * Copyright 2019-2020  Espressif Systems (Shanghai) 
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * zranger.h: Z-Ranger deck driver
 */

#ifndef _ZRANGER2_H_
#define _ZRANGER2_H_

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"
//#include "deck_core.h"

typedef struct {
    bool seen;    /* At least one measurement attempt has completed. */
    bool readOk;  /* The latest attempt returned sensor data. */
    bool valid;   /* The latest sample was accepted for range/estimation. */
    bool fresh;   /* Latest attempt is no more than 300 ms old. */
    uint8_t status; /* VL53L1 RangeStatus, 255 when readOk is false. */
    int16_t rangeMm;
    float signalRateMcps;
    float ambientRateMcps;
    float sigmaMm;
    uint32_t ageMs; /* UINT32_MAX before the first completed attempt. */
} zRanger2Diagnostics_t;

/* Read-only copy of the latest attempt, including invalid sensor values.
 * readOk must be checked before interpreting rangeMm and quality fields.
 * This is independent of rangeGet(), which retains the last valid range. */
void zRanger2GetDiagnostics(zRanger2Diagnostics_t *snapshot);

void zRanger2Init(void);

bool zRanger2Test(void);
void zRanger2Task(void* arg);

#endif /* _ZRANGER2_H_ */
