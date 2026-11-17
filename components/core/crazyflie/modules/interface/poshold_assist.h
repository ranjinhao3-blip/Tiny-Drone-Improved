#pragma once

#include "stabilizer_types.h"

/* Optional final byte of a web RPYT packet. Legacy 14-byte packets stay valid. */
#define POSHOLD_BUTTON_TAKEOFF 0x02U
#define POSHOLD_BUTTON_LAND    0x04U
#define POSHOLD_BUTTON_STOP    0x08U
/* Persistent speed selection; omitted by legacy packets, so low is default. */
#define POSHOLD_FLAG_HIGH_SPEED 0x10U
#define POSHOLD_TARGET_HEIGHT_M 0.50f

/* 松杆后锁点策略：1 零速制动且速度稳定后锁点；0 立即锁定当前位置。 */
#ifndef POSHOLD_BRAKE_BEFORE_HOLD
#define POSHOLD_BRAKE_BEFORE_HOLD 1
#endif

typedef enum {
  posAssistIdle, posAssistSpooling, posAssistClimbing, posAssistHovering,
  posAssistFlying, posAssistLanding, posAssistStopped, posAssistError
} posAssistPhase_t;

typedef enum {
  posAssistOk, posAssistNeedSensors, posAssistNeedLevel, posAssistMotorLocked,
  posAssistLowBattery, posAssistNeedNeutral, posAssistAlreadyFlying,
  posAssistNoLiftoff, posAssistTimeout, posAssistLinkLost, posAssistInvalidState
} posAssistReason_t;

typedef struct {
  uint32_t nowMs;
  uint8_t buttons;
  float roll, pitch, yaw;
  uint16_t throttle;
  bool stateValid, rangeValid, flowValid, motorsAllowed, batteryOk;
  bool linkStale, shutdown, flightActive;
  float rangeM;
  state_t state;
} posAssistInput_t;

typedef struct {
  posAssistPhase_t phase;
  posAssistReason_t reason;
  uint8_t previousButtons;
  uint32_t startedMs, phaseMs, settledMs, landingTimeoutMs;
  bool settling, lifted, holdXY, holdZ;
  uint32_t xySettledMs;
  bool xySettling;
  uint32_t landingRampMs;
  bool landingRamping;
  point_t target;
  float groundZ, groundRange, thrustScale;
} posAssist_t;

void posHoldAssistReset(posAssist_t *assist);
bool posHoldAssistIsActive(const posAssist_t *assist);
posAssistReason_t posHoldAssistReadiness(const posAssistInput_t *input);
/* Returns true while this assistant owns the setpoint, including a latched stop. */
bool posHoldAssistUpdate(posAssist_t *assist, const posAssistInput_t *input,
                         setpoint_t *setpoint);
