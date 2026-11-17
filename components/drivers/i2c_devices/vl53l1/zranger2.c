/**
*    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * vl53l1x.c: Time-of-flight distance sensor driver
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "i2cdev.h"
#include "zranger2.h"
#include "vl53l1x.h"
#include "cf_math.h"
#define DEBUG_MODULE "ZR2"
#include "debug_cf.h"

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f; // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 4000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;

static VL53L1_Dev_t dev;

static portMUX_TYPE rangeDiagnosticsMux = portMUX_INITIALIZER_UNLOCKED;
static zRanger2Diagnostics_t rangeDiagnostics;
static TickType_t lastRangeSampleTick;

void zRanger2GetDiagnostics(zRanger2Diagnostics_t *snapshot)
{
    if (snapshot == NULL) return;

    portENTER_CRITICAL(&rangeDiagnosticsMux);
    *snapshot = rangeDiagnostics;
    const TickType_t sampleTick = lastRangeSampleTick;
    portEXIT_CRITICAL(&rangeDiagnosticsMux);

    const TickType_t age = xTaskGetTickCount() - sampleTick;
    snapshot->ageMs = snapshot->seen ?
        (uint32_t)(age * portTICK_PERIOD_MS) : UINT32_MAX;
    snapshot->fresh = snapshot->seen && age <= pdMS_TO_TICKS(300);
    if (!snapshot->readOk) snapshot->status = UINT8_MAX;
}

static void zRanger2RecordDiagnostics(
    const VL53L1_RangingMeasurementData_t *rangingData,
    bool readOk, bool valid)
{
    const zRanger2Diagnostics_t sample = {
        .seen = true,
        .readOk = readOk,
        .valid = valid,
        .status = readOk ? rangingData->RangeStatus : UINT8_MAX,
        .rangeMm = readOk ? rangingData->RangeMilliMeter : 0,
        .signalRateMcps = readOk ?
            (float)rangingData->SignalRateRtnMegaCps / 65536.0f : 0.0f,
        .ambientRateMcps = readOk ?
            (float)rangingData->AmbientRateRtnMegaCps / 65536.0f : 0.0f,
        .sigmaMm = readOk ?
            (float)rangingData->SigmaMilliMeter / 65536.0f : 0.0f,
    };

    portENTER_CRITICAL(&rangeDiagnosticsMux);
    rangeDiagnostics = sample;
    lastRangeSampleTick = xTaskGetTickCount();
    portEXIT_CRITICAL(&rangeDiagnosticsMux);
}

static bool zRanger2GetMeasurementAndRestart(
    VL53L1_Dev_t *dev,
    VL53L1_RangingMeasurementData_t *rangingData,
    bool *readOk)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    uint8_t dataReady = 0;
    *readOk = false;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        if (status != VL53L1_ERROR_NONE) {
            return false;
        }
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, rangingData);
    *readOk = status == VL53L1_ERROR_NONE;

    VL53L1_StopMeasurement(dev);
    VL53L1_Error restartStatus = VL53L1_StartMeasurement(dev);

    return status == VL53L1_ERROR_NONE &&
           restartStatus == VL53L1_ERROR_NONE;
}

void zRanger2Init(void)
{
  if (isInit)
    return;

  if (vl53l1xInit(&dev, I2C1_DEV))
  {
    DEBUG_PRINTI("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINTW("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void* arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Restart sensor
  VL53L1_StopMeasurement(&dev);
  
  VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_MEDIUM);
  VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000); 

  VL53L1_StartMeasurement(&dev);

  lastWakeTime = xTaskGetTickCount();

  while (1) 
  { 
    vTaskDelayUntil(&lastWakeTime, M2T(25)); 

    VL53L1_RangingMeasurementData_t rangingData = {0};
    bool readOk;
    bool measurementOk = zRanger2GetMeasurementAndRestart(&dev, &rangingData, &readOk);
    range_last = rangingData.RangeMilliMeter;

    const bool valid = measurementOk &&
        rangingData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID &&
        range_last > 0 &&
        range_last < RANGE_OUTLIER_LIMIT;
    zRanger2RecordDiagnostics(&rangingData, readOk, valid);

    // Publish and enqueue only valid measurements. Invalid samples must not
    // overwrite the last usable height exposed through rangeGet().
    if (valid) {
      rangeSet(rangeDown, range_last / 1000.0f);
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}

static uint8_t disable = 0;
#define PARAM_CORE (1<<5)
#define PARAM_PERSISTENT (1 << 8)
#define PARAM_ADD_CORE(TYPE, NAME, ADDRESS) \
  PARAM_ADD(TYPE | PARAM_CORE, NAME, ADDRESS)

PARAM_GROUP_START(deck)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcACS37800, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcActiveMarker, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcAI, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcBigQuad, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcCPPM, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, cpxOverUART2, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcGTGPS, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLedRing, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLhTester, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcDWM1000, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLoco, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &disable)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcServo, &disable)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcUSD, &disable)
PARAM_GROUP_STOP(deck)


static uint32_t effect = 0;
static uint32_t neffect = 0;
PARAM_GROUP_START(ring)
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_PERSISTENT, effect, &effect)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_GROUP_STOP(ring)
