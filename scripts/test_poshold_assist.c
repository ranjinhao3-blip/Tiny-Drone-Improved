/* Tests the real assistant with synthetic state/sensor inputs; no motors. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "poshold_assist.h"
#include "position_controller.h"
#include "flowdeck_timing.h"

static posAssistInput_t input;
static posAssist_t assist;
static setpoint_t sp;

/* Platform queries only; the controller, PID and filters are the real code. */
uint8_t get_flight_mode(void) { return 2; } /* POSHOLD */
bool positionEstimatorAltitudeBarometerIsPrimary(void) { return false; }

static void fresh(void)
{
  input = (posAssistInput_t){0};
  input.stateValid = input.rangeValid = input.flowValid = true;
  input.motorsAllowed = input.batteryOk = true;
  input.throttle = 32767;
  input.rangeM = 0.04f;
  input.state.position.z = 0.04f;
  posHoldAssistReset(&assist);
}

static void update(uint32_t ms)
{
  input.nowMs = ms;
  posHoldAssistUpdate(&assist, &input, &sp);
}

static void start(void)
{
  input.buttons = POSHOLD_BUTTON_TAKEOFF;
  update(100);
  assert(assist.phase == posAssistSpooling);
  assert(sp.assistedTakeoffRamp && sp.assistedThrustScale == 0.0f);
  assert(fabsf(assist.target.z - 0.50f) < 0.0001f);
  input.buttons = 0;
}

static void hover(void)
{
  fresh(); start(); update(1600);
  input.state.position.z = input.rangeM = 0.50f;
  update(4500); update(5100);
  assert(assist.phase == posAssistHovering);
  assert(sp.mode.x == modeAbs && sp.mode.y == modeAbs && sp.mode.z == modeAbs);
}

#if POSHOLD_BRAKE_BEFORE_HOLD
static void assertBraking(void)
{
  assert(assist.phase == posAssistFlying && !assist.holdXY);
  assert(sp.mode.x == modeVelocity && sp.mode.y == modeVelocity);
  assert(!sp.velocity_body && sp.velocity.x == 0 && sp.velocity.y == 0);
}
#else
static void assertHolding(float x, float y)
{
  assert(assist.holdXY && !assist.xySettling);
  assert(sp.mode.x == modeAbs && sp.mode.y == modeAbs);
  assert(!sp.velocity_body);
  assert(sp.position.x == x && sp.position.y == y);
}
#endif

static void testReleaseDirections(bool highSpeed)
{
  const float directions[][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, -1}};
  for (unsigned i = 0; i < sizeof(directions) / sizeof(directions[0]); ++i) {
    hover();
    input.buttons = highSpeed ? POSHOLD_FLAG_HIGH_SPEED : 0;
    const float dx = directions[i][0], dy = directions[i][1];
    input.pitch = -15 * dx; input.roll = -15 * dy; update(5200);
    input.state.velocity.x = 0.4f * dx;
    input.state.velocity.y = 0.4f * dy;
#if POSHOLD_BRAKE_BEFORE_HOLD
    input.pitch = input.roll = 0; update(5220);
    assertBraking();
    /* A brief low-speed sample is not proof the craft has stopped. */
    input.state.velocity.x = 0.03f * dx;
    input.state.velocity.y = 0.03f * dy;
    update(5400); assertBraking();
    input.state.velocity.x = 0.10f * dx;
    input.state.velocity.y = 0.10f * dy;
    update(5500); assertBraking();
    input.state.velocity.x = 0.03f * dx;
    input.state.velocity.y = 0.03f * dy;
    update(5600); update(5799); assertBraking();
    input.state.position.x = 0.05f * dx;
    input.state.position.y = 0.05f * dy;
    update(5800);
    assert(assist.phase == posAssistHovering && assist.holdXY);
    assert(sp.mode.x == modeAbs && sp.mode.y == modeAbs);
    assert(sp.position.x == input.state.position.x && sp.position.y == input.state.position.y);
    /* Hold the settled point against later drift, not the release point. */
    input.state.position.x += 0.02f; input.state.position.y += 0.02f;
    update(5810);
    assert(sp.position.x == 0.05f * dx && sp.position.y == 0.05f * dy);
#else
    /* Capture the release point even while estimated speed remains high. */
    input.state.position.x = 0.20f * dx;
    input.state.position.y = 0.20f * dy;
    input.pitch = input.roll = 0; update(5220);
    assert(assist.phase == posAssistHovering);
    assertHolding(0.20f * dx, 0.20f * dy);
    /* Neither coasting nor 200 ms of zero speed moves the captured point. */
    input.state.position.x = 0.40f * dx;
    input.state.position.y = 0.40f * dy;
    update(5400); assertHolding(0.20f * dx, 0.20f * dy);
    input.state.velocity.x = input.state.velocity.y = 0;
    update(5600); update(5800);
    assertHolding(0.20f * dx, 0.20f * dy);
    input.yaw = 30; input.state.attitude.yaw = 90;
    input.throttle = 65535; update(5810);
    assertHolding(0.20f * dx, 0.20f * dy);
    assert(sp.attitudeRate.yaw == -30 && fabsf(sp.velocity.z - (highSpeed ? 0.70f : 0.40f)) < 0.001f);
#endif
  }
}

static void testSpeedModes(void)
{
  const float directions[][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, -1}};
  for (unsigned mode = 0; mode < 2; ++mode) {
    const uint8_t flags = mode ? POSHOLD_FLAG_HIGH_SPEED : 0;
    const float maxXY = mode ? 1.20f : 0.70f;
    const float maxZ = mode ? 0.70f : 0.40f;
    hover();
    input.buttons = flags;
    update(5110);
    assert(assist.phase == posAssistHovering && assist.holdXY && assist.holdZ);
    for (unsigned i = 0; i < sizeof(directions) / sizeof(directions[0]); ++i) {
      const float dx = directions[i][0], dy = directions[i][1];
      const float length = sqrtf(dx * dx + dy * dy);
      /* Oversized legacy stick values also stay inside the selected limit. */
      input.pitch = -30 * dx; input.roll = -30 * dy;
      input.throttle = 0; input.yaw = 45;
      update(5200 + i * 10);
      assert(sp.velocity_body && sp.mode.x == modeVelocity && sp.mode.y == modeVelocity);
      assert(fabsf(sp.velocity.x - dx * maxXY / length) < 0.001f);
      assert(fabsf(sp.velocity.y - dy * maxXY / length) < 0.001f);
      assert(fabsf(sp.velocity.z + maxZ) < 0.001f);
      assert(sp.attitudeRate.yaw == -45); /* Yaw speed is shared by both modes. */
    }
    input.throttle = 65535; update(5300);
    assert(fabsf(sp.velocity.z - maxZ) < 0.001f);
    input.buttons ^= POSHOLD_FLAG_HIGH_SPEED; update(5301);
    assert(fabsf(sp.velocity.z - (mode ? 0.40f : 0.70f)) < 0.001f);
    input.buttons = flags; update(5302);
    assert(fabsf(sp.velocity.z - maxZ) < 0.001f);

    /* Preserve proportional control on either side of the center dead zone. */
    input.throttle = 51651; update(5303);
    assert(fabsf(sp.velocity.z - maxZ * .5f) < 0.001f);
    input.throttle = 13883; update(5304);
    assert(fabsf(sp.velocity.z + maxZ * .5f) < 0.001f);

    /* Switching a held stick takes effect in the very next update. */
    input.roll = 0; input.pitch = -15; input.throttle = 0;
    input.buttons ^= POSHOLD_FLAG_HIGH_SPEED; update(5310);
    assert(fabsf(sp.velocity.x - (mode ? 0.70f : 1.20f)) < 0.001f);
    assert(fabsf(sp.velocity.z + (mode ? 0.40f : 0.70f)) < 0.001f);
    input.buttons = flags; update(5320);
    assert(fabsf(sp.velocity.x - maxXY) < 0.001f && fabsf(sp.velocity.z + maxZ) < 0.001f);

    /* The real PID must receive the full selected vertical command; release
     * captures the current height while keeping horizontal hold available. */
    input.roll = input.pitch = 0;
    for (unsigned up = 0; up < 2; ++up) {
      input.throttle = up ? 65535 : 0; update(5330 + up * 10);
      attitude_t attitude = {0}; float thrust;
      positionControllerInit(); positionControllerResetAllPID();
      positionController(&thrust, &attitude, &sp, &input.state);
      assert(fabsf(sp.velocity.z - (up ? maxZ : -maxZ)) < 0.001f);
    }
    input.state.position.z = .65f;
    input.throttle = 32767; update(5360);
    assert(sp.mode.z == modeAbs && fabsf(sp.position.z - .65f) < .001f);
  }

  /* A persistent speed bit must coexist with action edges and never start
   * flight by itself. Automatic takeoff and landing retain their speeds. */
  fresh(); input.buttons = POSHOLD_FLAG_HIGH_SPEED; update(0);
  assert(assist.phase == posAssistIdle);
  input.buttons |= POSHOLD_BUTTON_TAKEOFF; update(100);
  assert(assist.phase == posAssistSpooling);
  input.buttons = POSHOLD_FLAG_HIGH_SPEED; update(1600);
  assert(assist.phase == posAssistClimbing && fabsf(sp.velocity.z - 0.15f) < 0.001f);
  input.state.position.z = input.rangeM = 0.50f;
  update(4500); update(5100);
  assert(assist.phase == posAssistHovering);
  input.buttons |= POSHOLD_BUTTON_LAND; update(5200);
  assert(assist.phase == posAssistLanding && fabsf(sp.velocity.z + 0.20f) < 0.001f);
  input.buttons = POSHOLD_FLAG_HIGH_SPEED;
  input.state.position.z = input.rangeM = 0.08f; update(5300);
  assert(sp.assistedLandingRamp && fabsf(sp.velocity.z + 0.05f) < 0.001f);
  update(7300); assert(fabsf(sp.assistedThrustScale - 0.50f) < 0.001f);
  update(9300); assert(assist.phase == posAssistStopped && sp.thrust == 0);
  hover(); input.buttons = POSHOLD_FLAG_HIGH_SPEED | POSHOLD_BUTTON_STOP; update(5200);
  assert(assist.phase == posAssistStopped && sp.thrust == 0);
  input.buttons = POSHOLD_FLAG_HIGH_SPEED; update(5300);
  assert(assist.phase == posAssistStopped && sp.thrust == 0);

  /* The real controller's position-loop limit must not clip manual velocity
   * targets. A yaw rotation changes direction while preserving 1.2 m/s. */
  for (unsigned rotated = 0; rotated < 2; ++rotated) {
    hover();
    input.buttons = POSHOLD_FLAG_HIGH_SPEED;
    input.pitch = -15;
    input.state.attitude.yaw = rotated ? 90.0f : 0.0f;
    update(5200);
    attitude_t attitude = {0};
    float thrust = 0;
    positionControllerInit();
    positionControllerResetAllPID();
    positionController(&thrust, &attitude, &sp, &input.state);
    assert(fabsf(sp.velocity.x - (rotated ? 0.0f : 1.20f)) < 0.001f);
    assert(fabsf(sp.velocity.y - (rotated ? 1.20f : 0.0f)) < 0.001f);
  }
}

static void testReleaseTakeoverAndSafety(void)
{
  hover(); input.pitch = -15; update(5200);
  input.pitch = 0;
#if POSHOLD_BRAKE_BEFORE_HOLD
  /* The combined speed matters even when each axis is below 0.08 m/s. */
  input.state.velocity.x = input.state.velocity.y = 0.06f;
  update(5220); update(8000); assertBraking();
  input.state.velocity.x = input.state.velocity.y = 0;
  input.yaw = 30; input.state.attitude.yaw = 90;
  update(8010); assertBraking();
  assert(sp.attitudeRate.yaw == -30);
  input.pitch = 15; update(8100);
  assert(sp.velocity_body && sp.velocity.x < -0.69f); /* Immediate opposite input. */
  input.pitch = 0; input.yaw = 0; update(8110);
  update(8309); assertBraking(); /* Previous settling timer was cancelled. */
  update(8310); assert(assist.holdXY);
#else
  input.state.velocity.x = input.state.velocity.y = 0.06f;
  input.state.position.x = 0.80f; input.state.position.y = -0.40f;
  update(5220); assertHolding(0.80f, -0.40f);
  input.pitch = 15; update(8100);
  assert(!assist.holdXY && sp.velocity_body && sp.velocity.x < -0.69f);
  input.pitch = 0;
  input.state.position.x = 0.90f; input.state.position.y = -0.30f;
  update(8110); assertHolding(0.90f, -0.30f);
  input.state.position.x = 1.0f;
  update(8310); assertHolding(0.90f, -0.30f);
#endif

  for (int fault = 0; fault < 6; ++fault) {
    hover(); input.roll = 15; update(5200);
    input.roll = 0; input.state.velocity.y = -0.3f; update(5220);
#if POSHOLD_BRAKE_BEFORE_HOLD
    assertBraking();
#else
    assertHolding(input.state.position.x, input.state.position.y);
#endif
    if (fault == 0) input.buttons = POSHOLD_BUTTON_LAND;
    if (fault == 1) input.flowValid = false;
    if (fault == 2) input.batteryOk = false;
    if (fault == 3) input.buttons = POSHOLD_BUTTON_STOP;
    if (fault == 4) input.shutdown = true;
    if (fault == 5) input.state.velocity.x = NAN;
    update(5230);
    if (fault < 3) {
      assert(assist.phase == posAssistLanding && sp.velocity.z < 0);
      if (fault == 1) assert(sp.mode.x == modeDisable && sp.mode.y == modeDisable);
    } else {
      assert(!posHoldAssistIsActive(&assist) && sp.thrust == 0 && sp.mode.z == modeDisable);
    }
    assert(!assist.xySettling);
  }
}

static void testPositionPidTransfer(void)
{
  state_t state = {0};
  setpoint_t target = {0};
  attitude_t attitude = {0};
  float thrust;
  positionControllerInit(); positionControllerResetAllPID();
  target.mode.x = target.mode.y = modeAbs;
  target.mode.z = modeVelocity;
  target.position.x = 0.1f; target.position.y = -0.1f;
  state.velocity.z = -0.05f;
  for (int i = 0; i < 100; ++i) positionController(&thrust, &attitude, &target, &state);
  /* Build real outer- and inner-loop integrals before leaving a hold. */
  assert(target.velocity.x > 0.199f && target.velocity.y < -0.199f);
  target.mode.x = target.mode.y = modeVelocity;
  target.velocity.x = target.velocity.y = target.velocity.z = 0;
  positionController(&thrust, &attitude, &target, &state);
  const float brakeRoll = attitude.roll, brakePitch = attitude.pitch, brakeThrust = thrust;
  assert(fabsf(brakeRoll) > 0.1f && fabsf(brakePitch) > 0.1f);
  target.mode.x = target.mode.y = modeAbs;
  target.position.x = target.position.y = 0;
  positionController(&thrust, &attitude, &target, &state);
  assert(target.velocity.x == 0 && target.velocity.y == 0); /* No stale position I. */
  assert(fabsf(attitude.roll - brakeRoll) < 0.001f);
  assert(fabsf(attitude.pitch - brakePitch) < 0.001f); /* Inner-loop I survives. */
  assert(thrust >= brakeThrust && thrust <= brakeThrust + 10); /* No vertical reset. */

  /* Releasing X must not erase the still-active Y position integral. */
  target.mode.x = modeVelocity; target.position.y = 0.1f;
  for (int i = 0; i < 100; ++i) {
    target.velocity.x = 0;
    positionController(&thrust, &attitude, &target, &state);
  }
  assert(target.velocity.x == 0 && target.velocity.y > 0.199f);

  /* Manual high-speed targets bypass the outer position loop's 1.1 m/s
   * output limit; changing that limit would also change hold corrections. */
  target = (setpoint_t){0}; memset(&state, 0, sizeof(state));
  positionControllerResetAllPID();
  target.mode.x = target.mode.y = target.mode.z = modeVelocity;
  target.velocity_body = true; target.velocity.x = 1.20f;
  positionController(&thrust, &attitude, &target, &state);
  assert(fabsf(target.velocity.x - 1.20f) < 0.001f && target.velocity.y == 0);
}

static void beginLanding(void)
{
  hover();
  input.buttons = POSHOLD_BUTTON_LAND; update(5200);
  input.buttons = 0;
  assert(assist.phase == posAssistLanding);
}

static void testLandingApproachAndSpindown(void)
{
  beginLanding();
  assert(fabsf(sp.velocity.z + 0.20f) < 0.001f);
  /* Proximity alone starts spindown; no touchdown, velocity, attitude or
   * stable-range confirmation is required. */
  input.state.velocity.z = -0.20f;
  input.state.attitude.roll = 11.0f;
  input.state.position.z = input.rangeM = 0.3401f; update(5300);
  assert(fabsf(sp.velocity.z + 0.20f) < 0.001f && !sp.assistedLandingRamp);
  input.state.position.z = input.rangeM = 0.2401f; update(5410); const float above = sp.velocity.z;
  assert(!sp.assistedLandingRamp);
  input.state.position.z = input.rangeM = 0.2399f; update(5420); const float below = sp.velocity.z;
  assert(fabsf(above - below) < 0.001f);
  assert(assist.phase == posAssistLanding && sp.assistedLandingRamp);
  assert(!sp.assistedTakeoffRamp && sp.assistedThrustScale == 1.0f);
  assert(sp.mode.x == modeDisable && sp.mode.roll == modeAbs && sp.attitudeRate.yaw == 0);
  input.rangeM = 0.165f; update(5430);
  const float middleSpeed = sp.velocity.z;
  assert(middleSpeed > -0.20f && middleSpeed < -0.05f);
  /* Crossing the final-approach boundary does not step the velocity target. */
  input.rangeM = 0.0901f; update(5440); const float aboveSlow = sp.velocity.z;
  input.rangeM = 0.0899f; update(5450); const float near = sp.velocity.z;
  assert(fabsf(sp.velocity.z - near) < 0.001f);
  assert(fabsf(aboveSlow - near) < 0.001f);
  input.rangeM = 0.08f; update(5460);
  assert(fabsf(sp.velocity.z + 0.05f) < 0.001f);
  update(6420); assert(fabsf(sp.assistedThrustScale - 0.75f) < 0.001f);
  update(7420); assert(fabsf(sp.assistedThrustScale - 0.50f) < 0.001f);
  update(8420); assert(fabsf(sp.assistedThrustScale - 0.25f) < 0.001f);
  update(9419); assert(posHoldAssistIsActive(&assist) && sp.assistedThrustScale > 0);
  update(9420);
  assert(assist.phase == posAssistStopped && sp.thrust == 0 && sp.mode.z == modeDisable);
  input.throttle = 65535; update(11010);
  assert(assist.phase == posAssistStopped && sp.thrust == 0);
}

static void testLandingBounceAndFaults(void)
{
  for (int fault = 0; fault < 7; ++fault) {
    beginLanding();
    input.state.position.z = input.rangeM = 0.24f;
    update(6000); update(6200); update(6300);
    assert(sp.assistedLandingRamp && sp.assistedThrustScale < 1.0f);
    if (fault == 0) { input.rangeM = 0.30f; input.state.velocity.z = 0.12f; }
    if (fault == 1) input.rangeValid = false;
    if (fault == 2) input.rangeM = NAN;
    if (fault == 3) input.buttons = POSHOLD_BUTTON_STOP;
    if (fault == 4) input.shutdown = true;
    if (fault == 5) input.motorsAllowed = false;
    if (fault == 6) input.state.position.z = NAN;
    update(6310);
    if (fault < 3) {
      assert(assist.phase == posAssistLanding && !sp.assistedLandingRamp);
      assert(sp.mode.z == modeVelocity && sp.velocity.z < 0);
      if (fault > 0) assert(fabsf(sp.velocity.z + 0.08f) < 0.001f);
      /* Returning to the near-ground range restarts the ramp immediately. */
      input.rangeValid = true; input.rangeM = 0.24f; input.state.velocity.z = 0;
      update(6400);
      assert(sp.assistedLandingRamp && sp.assistedThrustScale == 1);
    } else {
      assert(!posHoldAssistIsActive(&assist) && sp.thrust == 0 && sp.mode.z == modeDisable);
    }
  }
  /* Loss of optical flow still levels the aircraft during its approach. */
  beginLanding(); input.flowValid = false; update(5300);
  assert(sp.mode.x == modeDisable && sp.mode.roll == modeAbs && sp.velocity.z < 0);
  /* Near-ground spindown just before timeout is allowed its full 4 s ramp. */
  beginLanding();
  const uint32_t deadline = assist.phaseMs + assist.landingTimeoutMs;
  input.state.position.z = input.rangeM = 0.24f;
  update(deadline - 2001);
  assert(sp.assistedLandingRamp && sp.assistedThrustScale == 1);
  update(deadline - 1);
  assert(assist.phase == posAssistLanding && fabsf(sp.assistedThrustScale - 0.50f) < 0.001f);
  update(deadline + 1998);
  assert(assist.phase == posAssistLanding && sp.assistedThrustScale > 0.0f);
  update(deadline + 1999);
  assert(assist.phase == posAssistStopped && assist.reason == posAssistOk);
}

static void testLandingThrustRamp(void)
{
  state_t state = {0};
  setpoint_t target = {0};
  attitude_t attitude = {0};
  float thrust;
  positionControllerInit(); positionControllerResetAllPID();
  target.mode.z = modeVelocity; target.velocity.z = -0.05f;
  for (int i = 0; i < 100; ++i) positionController(&thrust, &attitude, &target, &state);
  target.assistedLandingRamp = true; target.assistedThrustScale = 1;
  positionController(&thrust, &attitude, &target, &state);
  const float initial = thrust;
  assert(initial > 10000);
  /* Real velocity PID must not integrate against a stationary floor. */
  for (int i = 0; i < 80; ++i) positionController(&thrust, &attitude, &target, &state);
  assert(fabsf(thrust - initial) < 1.0f);
  target.assistedThrustScale = 0.5f;
  positionController(&thrust, &attitude, &target, &state);
  assert(fabsf(thrust - 0.5f * initial) < 1.0f);
  target.assistedThrustScale = 0;
  positionController(&thrust, &attitude, &target, &state);
  assert(thrust == 0); /* Ramp can go below the usual running motor minimum. */
}

int main(void)
{
  /* Bursts 2 and 3 are discarded, but burst 4 still covers only 5 ms. */
  uint64_t lastUs = 1000000;
  float dt;
  assert(flowdeckSampleDt(1005000, &lastUs, &dt));
  assert(fabsf(dt - 0.005f) < 0.000001f);
  assert(flowdeckSampleDt(1010000, &lastUs, &dt));
  assert(flowdeckSampleDt(1015000, &lastUs, &dt));
  assert(flowdeckSampleDt(1020000, &lastUs, &dt));
  assert(fabsf(dt - 0.005f) < 0.000001f);
  assert(!flowdeckSampleDt(1120000, &lastUs, &dt)); /* Stall: reject and rebase. */
  assert(flowdeckSampleDt(1125000, &lastUs, &dt));
  assert(fabsf(dt - 0.005f) < 0.000001f);
  assert(!flowdeckSampleDt(1125000, &lastUs, &dt)); /* No division by zero. */
  assert(!flowdeckSampleDt(1124000, &lastUs, &dt)); /* Clock discontinuity. */
  assert(flowdeckSampleDt(1129000, &lastUs, &dt));
  fresh(); assert(posHoldAssistReadiness(&input) == posAssistOk);
  input.flowValid = false; assert(posHoldAssistReadiness(&input) == posAssistNeedSensors);
  input.buttons = POSHOLD_BUTTON_TAKEOFF; update(0);
  assert(assist.phase == posAssistIdle && assist.reason == posAssistNeedSensors);
  fresh(); input.motorsAllowed = false;
  assert(posHoldAssistReadiness(&input) == posAssistMotorLocked);
  fresh(); input.batteryOk = false;
  assert(posHoldAssistReadiness(&input) == posAssistLowBattery);
  fresh(); input.state.attitude.roll = 15;
  assert(posHoldAssistReadiness(&input) == posAssistNeedLevel);
  fresh(); input.throttle = 65535;
  assert(posHoldAssistReadiness(&input) == posAssistNeedNeutral);
  fresh(); input.flightActive = true;
  assert(posHoldAssistReadiness(&input) == posAssistAlreadyFlying);
  fresh(); input.state.position.x = NAN;
  assert(posHoldAssistReadiness(&input) == posAssistInvalidState);

  fresh(); start(); update(850);
  assert(sp.assistedTakeoffRamp && fabsf(sp.assistedThrustScale - 0.5f) < 0.001f);
  input.buttons = POSHOLD_BUTTON_TAKEOFF; update(1000);
  assert(assist.startedMs == 100); /* No restart while already active. */
  update(1600); assert(assist.phase == posAssistClimbing && sp.velocity.z <= 0.15f);
  update(5100); assert(assist.phase == posAssistError && assist.reason == posAssistNoLiftoff);
  assert(sp.thrust == 0 && sp.mode.z == modeDisable);
  update(8000); assert(sp.thrust == 0); /* Held button cannot re-arm. */
  input.buttons = 0; update(8010);
  input.buttons = POSHOLD_BUTTON_TAKEOFF; update(8020);
  assert(assist.phase == posAssistSpooling);

  hover();
  input.roll = 0.6f; update(5200);
  assert(assist.phase == posAssistHovering && sp.mode.x == modeAbs);
  input.roll = 3.0f; update(5210);
  assert(assist.phase == posAssistFlying && sp.mode.x == modeVelocity);
  assert(fabsf(sp.velocity.y + 0.167f) < 0.003f);
  input.roll = 0.0f; update(5220);
  input.roll = 9; input.throttle = 65535; update(5300);
  assert(assist.phase == posAssistFlying && sp.mode.x == modeVelocity && sp.velocity_body);
  assert(fabsf(sp.velocity.y + 0.455f) < 0.003f && fabsf(sp.velocity.z - 0.40f) < 0.001f);
  input.roll = 0; input.throttle = 32767;
  input.state.position.x = 1.2f; input.state.position.y = 0.8f;
  input.state.velocity.x = 0.25f; input.state.velocity.y = -0.15f;
  input.state.position.z = 0.75f; update(5300);
#if POSHOLD_BRAKE_BEFORE_HOLD
  assertBraking();
#else
  assertHolding(1.2f, 0.8f);
#endif
  assert(sp.mode.z == modeAbs && sp.position.z == 0.75f);
  input.state.position.x = 1.25f; input.state.position.y = 0.77f;
  input.state.velocity.x = 0.02f; input.state.velocity.y = -0.03f;
  update(5400); update(5599);
#if POSHOLD_BRAKE_BEFORE_HOLD
  assertBraking();
#else
  assertHolding(1.2f, 0.8f);
#endif
  input.state.position.x = 1.27f; input.state.position.y = 0.76f; update(5600);
  assert(assist.phase == posAssistHovering && sp.mode.x == modeAbs);
#if POSHOLD_BRAKE_BEFORE_HOLD
  assert(sp.position.x == 1.27f && sp.position.y == 0.76f && sp.position.z == 0.75f);
#else
  assertHolding(1.2f, 0.8f);
  assert(sp.position.z == 0.75f);
#endif
  input.state.position.x = 1.32f; update(5700);
#if POSHOLD_BRAKE_BEFORE_HOLD
  assert(sp.position.x == 1.27f);
#else
  assertHolding(1.2f, 0.8f);
#endif

  /* Full stick and diagonal/legacy inputs stay within the horizontal limit. */
  hover();
  input.pitch = 15; input.throttle = 0; update(5410);
  assert(fabsf(sp.velocity.x + 0.70f) < 0.001f);
  assert(fabsf(sp.velocity.z + 0.40f) < 0.001f);
  input.roll = 30; input.pitch = -30; update(5420);
  assert(sp.velocity.x > 0.0f && sp.velocity.y < 0.0f);
  assert(fabsf(sqrtf(sp.velocity.x * sp.velocity.x + sp.velocity.y * sp.velocity.y) - 0.70f) < 0.001f);
  input.roll = input.pitch = 0; input.throttle = 32767; update(5430);

  input.buttons = POSHOLD_BUTTON_LAND; update(5500);
  assert(assist.phase == posAssistLanding && sp.velocity.z < 0);
  input.buttons = 0;
  input.state.position.z = input.rangeM = 0.24f;
  update(6000);
  assert(assist.phase == posAssistLanding && sp.assistedLandingRamp &&
         sp.assistedThrustScale == 1.0f);
  update(7000); assert(fabsf(sp.assistedThrustScale - 0.75f) < 0.001f);
  update(8000); assert(fabsf(sp.assistedThrustScale - 0.50f) < 0.001f);
  update(10000);
  assert(assist.phase == posAssistStopped && sp.thrust == 0 && sp.mode.z == modeDisable);

  hover(); input.flowValid = false; update(5200);
  assert(assist.phase == posAssistLanding && assist.reason == posAssistNeedSensors);
  assert(sp.mode.x == modeDisable && sp.mode.roll == modeAbs);
  input.rangeValid = false; update(assist.phaseMs + assist.landingTimeoutMs);
  assert(assist.phase == posAssistError && assist.reason == posAssistTimeout && sp.thrust == 0);

  fresh(); start(); input.flowValid = false; update(200);
  assert(assist.phase == posAssistError && sp.thrust == 0);
  hover(); input.shutdown = true; update(5200);
  assert(assist.phase == posAssistError && assist.reason == posAssistLinkLost && sp.thrust == 0);
  fresh(); start(); input.linkStale = true; update(650);
  assert(assist.phase == posAssistError && assist.reason == posAssistLinkLost && sp.thrust == 0);
  fresh(); start(); update(1600);
  input.state.position.z = input.rangeM = 0.2f; update(2000);
  input.linkStale = true; update(2600);
  assert(assist.phase == posAssistHovering && sp.mode.z == modeAbs && sp.position.z == 0.2f);
  hover(); input.buttons = POSHOLD_BUTTON_STOP; update(5200);
  assert(assist.phase == posAssistStopped && sp.thrust == 0);
  input.buttons = 0; input.throttle = 65535; update(5300);
  assert(assist.phase == posAssistStopped && sp.thrust == 0);

  fresh(); start(); update(1600);
  input.state.position.z = input.rangeM = 0.2f; update(4000);
  update(12100); assert(assist.phase == posAssistLanding && assist.reason == posAssistTimeout);
  hover(); input.batteryOk = false; update(5200);
  assert(assist.phase == posAssistLanding && assist.reason == posAssistLowBattery);
  fresh(); start(); update(1600); input.pitch = 6; update(1700);
  assert(assist.phase == posAssistFlying && sp.mode.z == modeAbs);
  assert(sp.position.z == input.state.position.z);
  /* Floor-relative target must not depend on the estimator's Z origin. */
  fresh(); input.state.position.z = 4.0f;
  input.buttons = POSHOLD_BUTTON_TAKEOFF; update(100);
  assert(fabsf(assist.target.z - 4.46f) < 0.0001f);
  /* Timeout must budget the slow approach even after a manual climb to 1.5 m. */
  hover(); input.state.position.z = input.rangeM = 1.50f;
  input.buttons = POSHOLD_BUTTON_LAND; update(5200);
  assert(assist.phase == posAssistLanding && assist.landingTimeoutMs > 24000U);
  input.buttons = 0;
  input.state.position.z = input.rangeM = 0.54f; update(17200);
  assert(assist.phase == posAssistLanding && sp.velocity.z < 0.0f);
  input.state.position.z = input.rangeM = 0.04f;
  update(24000); update(24600); update(26000); update(28000);
  assert(assist.phase == posAssistStopped && sp.thrust == 0.0f);
  testSpeedModes();
  testReleaseDirections(false);
  testReleaseDirections(true);
  testReleaseTakeoverAndSafety();
  testPositionPidTransfer();
  testLandingApproachAndSpindown();
  testLandingBounceAndFaults();
  testLandingThrustRamp();
  printf("PASS: release mode=%s; flow timing/recovery, takeoff, low/high speed modes and live switching, manual response/limits, release capture/takeover, PID transfer, landing approach/proximity spindown/bounce, sensor failure, link loss, stop latch.\n",
         POSHOLD_BRAKE_BEFORE_HOLD ? "brake then hold" : "immediate hold");
  return 0;
}
