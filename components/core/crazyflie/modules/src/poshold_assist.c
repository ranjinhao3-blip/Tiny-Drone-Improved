#include <math.h>
#include <string.h>
#include "poshold_assist.h"

#define CENTER 32767U
#define DEADZONE 5000U
#define SPOOL_MS 1500U
#define NO_LIFTOFF_MS 5000U
#define CLIMB_TIMEOUT_MS 12000U
#define LAND_MIN_TIMEOUT_MS 12000U
#define LAND_FAST_SPEED_MPS 0.20f
#define LAND_SLOW_SPEED_MPS 0.05f
#define LAND_FALLBACK_SPEED_MPS 0.08f
#define LAND_SLOW_HEIGHT_M 0.05f
#define LAND_FAST_HEIGHT_M 0.20f
#define LAND_SPINDOWN_HEIGHT_M 0.20f
#define LAND_SPINDOWN_MS 4000U
#define LAND_RELEASE_HEIGHT_M 0.25f
#define LAND_RELEASE_SPEED_MPS 0.10f
#define STICK_DEADZONE 0.6f
#define STICK_RESPONSE_EXPO 0.80f
#define MANUAL_XY_LOW_SPEED_MPS 0.70f
#define MANUAL_XY_HIGH_SPEED_MPS 1.20f
#define MANUAL_Z_LOW_SPEED_MPS 0.40f
#define MANUAL_Z_HIGH_SPEED_MPS 0.70f
#define WEB_FULL_STICK 15.0f
#define XY_STOP_SPEED_MPS 0.08f
#define XY_STOP_SETTLE_MS 200U

static float clamp(float value, float low, float high)
{
  return fminf(high, fmaxf(low, value));
}

static float stickToVelocity(float stick, float maxSpeed)
{
  if (!isfinite(stick) || fabsf(stick) <= STICK_DEADZONE) return 0.0f;

  const float magnitude = clamp((fabsf(stick) - STICK_DEADZONE) /
                                (WEB_FULL_STICK - STICK_DEADZONE), 0.0f, 1.0f);
  const float shaped = powf(magnitude, STICK_RESPONSE_EXPO);
  return copysignf(shaped * maxSpeed, stick);
}

static bool finiteState(const state_t *s)
{
  return isfinite(s->position.x) && isfinite(s->position.y) &&
      isfinite(s->position.z) && isfinite(s->velocity.x) &&
      isfinite(s->velocity.y) && isfinite(s->velocity.z) &&
      isfinite(s->attitude.roll) && isfinite(s->attitude.pitch) &&
      isfinite(s->attitude.yaw);
}

void posHoldAssistReset(posAssist_t *a)
{
  memset(a, 0, sizeof(*a));
  a->thrustScale = 1.0f;
}

bool posHoldAssistIsActive(const posAssist_t *a)
{
  return a->phase >= posAssistSpooling && a->phase <= posAssistLanding;
}

posAssistReason_t posHoldAssistReadiness(const posAssistInput_t *in)
{
  if (!in->motorsAllowed) return posAssistMotorLocked;
  if (!in->batteryOk) return posAssistLowBattery;
  if (!in->stateValid || !finiteState(&in->state)) return posAssistInvalidState;
  if (!in->rangeValid || !in->flowValid || !isfinite(in->rangeM) ||
      in->rangeM <= 0.0f) return posAssistNeedSensors;
  if (in->flightActive || in->rangeM > 0.20f) return posAssistAlreadyFlying;
  if (fabsf(in->state.attitude.roll) > 10.0f ||
      fabsf(in->state.attitude.pitch) > 10.0f ||
      fabsf(in->state.velocity.x) > 0.15f ||
      fabsf(in->state.velocity.y) > 0.15f ||
      fabsf(in->state.velocity.z) > 0.10f) return posAssistNeedLevel;
  if (fabsf(in->roll) > STICK_DEADZONE || fabsf(in->pitch) > STICK_DEADZONE ||
      fabsf(in->yaw) > 3.0f || in->throttle < CENTER - DEADZONE ||
      in->throttle > CENTER + DEADZONE) return posAssistNeedNeutral;
  return posAssistOk;
}

static void stop(posAssist_t *a, posAssistReason_t reason)
{
  a->phase = reason == posAssistOk ? posAssistStopped : posAssistError;
  a->reason = reason;
  a->thrustScale = 1.0f;
  a->xySettling = false;
  a->landingRamping = false;
}

static void land(posAssist_t *a, const posAssistInput_t *in,
                 posAssistReason_t reason)
{
  if (!a->lifted) {
    stop(a, reason);
    return;
  }
  a->phase = posAssistLanding;
  a->reason = reason;
  a->phaseMs = in->nowMs;
  a->settling = false;
  a->landingRamping = false;
  a->target.x = in->state.position.x;
  a->target.y = in->state.position.y;
  a->holdXY = true;
  a->xySettling = false;
  /* Manual ascent may put the aircraft above its original 0.5 m target.
   * Budget even the slow speed over the entire descent, plus six seconds
   * for braking, near-ground motor spindown and settling. */
  float descentM = fmaxf(0.0f, in->state.position.z - a->groundZ);
  if (in->rangeValid) descentM = fmaxf(descentM, in->rangeM - a->groundRange);
  a->landingTimeoutMs = (uint32_t)fmaxf((float)LAND_MIN_TIMEOUT_MS,
      6000.0f + 1000.0f * clamp(descentM, 0.0f, 10.0f) / LAND_SLOW_SPEED_MPS);
}

bool posHoldAssistUpdate(posAssist_t *a, const posAssistInput_t *in,
                         setpoint_t *sp)
{
  const uint8_t rising = in->buttons & ~a->previousButtons;
  a->previousButtons = in->buttons;
  if (in->buttons & POSHOLD_BUTTON_STOP) stop(a, posAssistOk);
  else if ((rising & POSHOLD_BUTTON_TAKEOFF) && !posHoldAssistIsActive(a)) {
    a->reason = posHoldAssistReadiness(in);
    if (a->reason == posAssistOk) {
      a->phase = posAssistSpooling;
      a->startedMs = a->phaseMs = in->nowMs;
      a->settling = a->lifted = false;
      a->holdXY = a->holdZ = true;
      a->xySettling = false;
      a->landingRamping = false;
      a->target = in->state.position;
      a->groundZ = in->state.position.z;
      a->groundRange = in->rangeM;
      /* Keep the fused-coordinate offset, but target 0.5 m over the floor
       * measured by ToF rather than adding 0.5 m to the landing-gear height. */
      a->target.z += POSHOLD_TARGET_HEIGHT_M - in->rangeM;
    } else if (a->phase != posAssistIdle) {
      a->phase = posAssistError;
    }
  }

  if (a->phase == posAssistIdle) return false;
  memset(sp, 0, sizeof(*sp));
  if (!posHoldAssistIsActive(a)) return true;

  if (in->shutdown) stop(a, posAssistLinkLost);
  else if (!in->motorsAllowed) stop(a, posAssistMotorLocked);
  else if (!in->stateValid || !finiteState(&in->state)) stop(a, posAssistInvalidState);
  if (!posHoldAssistIsActive(a)) return true;

  /* Both fused height and a live range change must show lift-off. This also
   * bounds the motor-on time of a propeller-free bench test. */
  if (in->rangeValid && in->rangeM - a->groundRange > 0.06f &&
      in->state.position.z - a->groundZ > 0.06f) a->lifted = true;

  if (in->linkStale && (a->phase == posAssistSpooling || a->phase == posAssistClimbing)) {
    if (!a->lifted) stop(a, posAssistLinkLost);
    else {
      a->phase = posAssistHovering;
      a->target.z = in->state.position.z;
      a->holdZ = true;
    }
  }
  if (!posHoldAssistIsActive(a)) return true;

  if (a->phase != posAssistLanding) {
    if (rising & POSHOLD_BUTTON_LAND) land(a, in, posAssistOk);
    else if (!in->batteryOk) land(a, in, posAssistLowBattery);
    else if (!in->rangeValid || !in->flowValid) land(a, in, posAssistNeedSensors);
    else if (fabsf(in->state.attitude.roll) > 25.0f ||
             fabsf(in->state.attitude.pitch) > 25.0f) land(a, in, posAssistNeedLevel);
  }
  if (!posHoldAssistIsActive(a)) return true;

  sp->mode.roll = sp->mode.pitch = modeDisable;
  sp->mode.yaw = modeVelocity;
  sp->attitudeRate.yaw = -in->yaw;
  sp->velocity_body = false;
  sp->mode.x = sp->mode.y = modeAbs;
  sp->position = a->target;
  sp->mode.z = modeVelocity;
  a->thrustScale = 1.0f;

  if (a->phase == posAssistSpooling) {
    a->thrustScale = clamp((float)(in->nowMs - a->startedMs) / SPOOL_MS, 0.0f, 1.0f);
    sp->assistedTakeoffRamp = true;
    sp->assistedThrustScale = a->thrustScale;
    sp->velocity.z = clamp((a->target.z - in->state.position.z) * 0.8f,
                           -0.08f, 0.15f) * a->thrustScale;
    if (a->thrustScale >= 1.0f) a->phase = posAssistClimbing;
  }
  if (a->phase == posAssistClimbing) {
    const float error = a->target.z - in->state.position.z;
    sp->velocity.z = clamp(error * 0.8f, -0.08f, 0.15f);
    const bool nearTarget = fabsf(error) < 0.04f &&
        fabsf(in->state.velocity.z) < 0.08f && a->lifted;
    if (nearTarget) {
      if (!a->settling) { a->settledMs = in->nowMs; a->settling = true; }
      if (in->nowMs - a->settledMs >= 600U) a->phase = posAssistHovering;
    } else a->settling = false;
    if (in->nowMs - a->startedMs >= CLIMB_TIMEOUT_MS)
      land(a, in, posAssistTimeout);
  }

  if (a->phase >= posAssistClimbing && a->phase <= posAssistFlying) {
    const bool moveXY = fabsf(in->roll) > STICK_DEADZONE ||
        fabsf(in->pitch) > STICK_DEADZONE;
    const bool moveZ = in->throttle < CENTER - DEADZONE ||
        in->throttle > CENTER + DEADZONE;
    if ((moveXY || moveZ || fabsf(in->yaw) > 3.0f) && a->phase == posAssistClimbing) {
      /* Moving a stick takes over immediately without retaining an old climb. */
      a->target.z = in->state.position.z;
      a->phase = posAssistFlying;
    }
    if (a->phase == posAssistHovering || a->phase == posAssistFlying) {
      const bool highSpeed = (in->buttons & POSHOLD_FLAG_HIGH_SPEED) != 0;
      const float maxXYSpeed = highSpeed ? MANUAL_XY_HIGH_SPEED_MPS : MANUAL_XY_LOW_SPEED_MPS;
      const float maxZSpeed = highSpeed ? MANUAL_Z_HIGH_SPEED_MPS : MANUAL_Z_LOW_SPEED_MPS;
      if (moveXY) {
        a->holdXY = false;
        a->xySettling = false;
        sp->mode.x = sp->mode.y = modeVelocity;
        sp->velocity_body = true;
        sp->velocity.x = stickToVelocity(-in->pitch, maxXYSpeed);
        sp->velocity.y = stickToVelocity(-in->roll, maxXYSpeed);
        const float speed = sqrtf(sp->velocity.x * sp->velocity.x +
                                  sp->velocity.y * sp->velocity.y);
        if (speed > maxXYSpeed) {
          sp->velocity.x *= maxXYSpeed / speed;
          sp->velocity.y *= maxXYSpeed / speed;
        }
      } else if (!a->holdXY) {
#if POSHOLD_BRAKE_BEFORE_HOLD
        /* Brake before capturing a position. Capturing on stick release
         * makes the position loop pull back after the aircraft coasts past
         * that point. Zero world velocity also stays valid during yaw. */
        sp->mode.x = sp->mode.y = modeVelocity;
        sp->velocity.x = sp->velocity.y = 0.0f;
        const float speedSquared = in->state.velocity.x * in->state.velocity.x +
            in->state.velocity.y * in->state.velocity.y;
        if (speedSquared <= XY_STOP_SPEED_MPS * XY_STOP_SPEED_MPS) {
          if (!a->xySettling) {
            a->xySettledMs = in->nowMs;
            a->xySettling = true;
          }
          if (in->nowMs - a->xySettledMs >= XY_STOP_SETTLE_MS) {
            a->target.x = in->state.position.x;
            a->target.y = in->state.position.y;
            a->holdXY = true;
            a->xySettling = false;
            sp->mode.x = sp->mode.y = modeAbs;
          }
        } else {
          a->xySettling = false;
        }
#else
        /* 临时跳过零速制动及 200 ms 等待，松杆当次捕获，后续不重复更新。 */
        a->target.x = in->state.position.x;
        a->target.y = in->state.position.y;
        a->holdXY = true;
        a->xySettling = false;
        sp->mode.x = sp->mode.y = modeAbs;
#endif
      }
      if (moveZ) {
        a->holdZ = false;
        sp->velocity.z = in->throttle > CENTER + DEADZONE ?
            maxZSpeed * (in->throttle - CENTER - DEADZONE) / (65535U - CENTER - DEADZONE) :
            -maxZSpeed * (CENTER - DEADZONE - in->throttle) / (CENTER - DEADZONE);
      } else {
        if (!a->holdZ) { a->target.z = in->state.position.z; a->holdZ = true; }
        sp->mode.z = modeAbs;
      }
      a->phase = moveXY || !a->holdXY || moveZ || fabsf(in->yaw) > 3.0f ?
          posAssistFlying : posAssistHovering;
      sp->position = a->target;
    }
  }

  if (a->phase == posAssistLanding) {
    sp->mode.z = modeVelocity;
    const bool rangeUsable = in->rangeValid && isfinite(in->rangeM) && in->rangeM > 0.0f;
    const float clearance = rangeUsable ? in->rangeM - a->groundRange : 0.0f;
    /* The reference is the sensor height with the landing gear on the floor.
     * Blend continuously to the final approach speed instead of cutting
     * collective a few centimetres above that reference. */
    const float approach = clamp((clearance - LAND_SLOW_HEIGHT_M) /
        (LAND_FAST_HEIGHT_M - LAND_SLOW_HEIGHT_M), 0.0f, 1.0f);
    sp->velocity.z = rangeUsable ? -(LAND_SLOW_SPEED_MPS +
        (LAND_FAST_SPEED_MPS - LAND_SLOW_SPEED_MPS) * approach) :
        -LAND_FALLBACK_SPEED_MPS;
    if (!in->flowValid) {
      sp->mode.x = sp->mode.y = modeDisable;
      sp->mode.roll = sp->mode.pitch = modeAbs;
    }
    if (!a->landingRamping) {
      /* Start as soon as range shows the aircraft is close to the floor;
       * do not wait for a touchdown or stable-contact confirmation. */
      if (rangeUsable && clearance <= LAND_SPINDOWN_HEIGHT_M) {
        a->landingRampMs = in->nowMs;
        a->landingRamping = true;
      }
    } else if (!rangeUsable || clearance > LAND_RELEASE_HEIGHT_M ||
               in->state.velocity.z > LAND_RELEASE_SPEED_MPS ||
               fabsf(in->state.attitude.roll) > 15.0f ||
               fabsf(in->state.attitude.pitch) > 15.0f) {
      /* A clear climb or sensor loss aborts spindown so landing can resume. */
      a->settling = false;
      a->landingRamping = false;
    }
    if (a->landingRamping) {
      /* Stop horizontal/yaw corrections and scale attitude mix with collective. */
      sp->mode.x = sp->mode.y = modeDisable;
      sp->mode.roll = sp->mode.pitch = modeAbs;
      sp->attitudeRate.yaw = 0.0f;
      a->thrustScale = 1.0f - clamp((float)(in->nowMs - a->landingRampMs) /
                                  LAND_SPINDOWN_MS, 0.0f, 1.0f);
      sp->assistedLandingRamp = true;
      sp->assistedThrustScale = a->thrustScale;
      if (in->nowMs - a->landingRampMs >= LAND_SPINDOWN_MS) stop(a, a->reason);
    }
    if (!a->landingRamping && posHoldAssistIsActive(a) &&
        in->nowMs - a->phaseMs >= a->landingTimeoutMs) stop(a, posAssistTimeout);
  }
  if (!a->lifted && a->phase != posAssistLanding &&
      in->nowMs - a->startedMs >= NO_LIFTOFF_MS) stop(a, posAssistNoLiftoff);
  if (!posHoldAssistIsActive(a)) memset(sp, 0, sizeof(*sp));
  return true;
}
