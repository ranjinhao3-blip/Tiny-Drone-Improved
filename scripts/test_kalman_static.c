/* Runs the real estimator; no serial IO, actuator commands or sensor drivers. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "kalman_core.h"
#include "outlierFilter.h"
#include "physicalConstants.h"
#ifdef NDEBUG
#error Tests require active assertions, including matrix operations in cf_math.h
#endif
uint32_t testTick;

void outlierFilterReset(OutlierFilterLhState_t *filter, uint32_t now)
{ filter->openingTime = now; filter->openingWindow = 0; }
bool outlierFilterValidateTdoaSteps(const tdoaMeasurement_t *m, float e,
                                  const vector_t *j, const point_t *p)
{ (void)m; (void)e; (void)j; (void)p; assert(0); return false; }

static void finiteState(const kalmanCoreData_t *k)
{
  float norm2 = 0;
  for (int i = 0; i < 4; i++) {
    assert(isfinite(k->q[i]));
    norm2 += k->q[i]*k->q[i];
  }
  assert(fabsf(norm2 - 1) < 1e-5f);
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) assert(isfinite(k->R[i][j]));
  for (int i = 0; i < KC_STATE_DIM; i++) {
    assert(isfinite(k->S[i]));
    assert(k->P[i][i] > 0 && isfinite(k->P[i][i]));
    for (int j = 0; j < KC_STATE_DIM; j++) {
      assert(isfinite(k->P[i][j]));
      assert(fabsf(k->P[i][j] - k->P[j][i]) < 1e-5f);
    }
  }
}

static void stationary(const char *name, Axis3f gravity, float yaw)
{
  kalmanCoreData_t k;
  kalmanCoreInit(&k);
  k.q[0] = cosf(yaw / 2);
  k.q[3] = sinf(yaw / 2);
  kalmanCoreFinalize(&k, 0);
  const Axis3f gyro = {.axis = {0, 0, 0}};
  Axis3f acc = {.axis = {gravity.x * GRAVITY_MAGNITUDE,
                       gravity.y * GRAVITY_MAGNITUDE,
                       gravity.z * GRAVITY_MAGNITUDE}};
  float vx = 0, vy = 0;
  for (unsigned step = 1; step <= 12000; step++) {
    testTick = step * 5;
    if (step % 2 == 0) {
      Axis3f rates = gyro;
      kalmanCorePredict(&k, 0, &acc, &rates, 0.01f, false);
    }
    kalmanCoreAddProcessNoise(&k, 0.005f);
    if (step % 13 == 0) {
      tofMeasurement_t tof = {.timestamp = testTick, .distance = 0.165f, .stdDev = 0.005f};
      kalmanCoreUpdateWithTof(&k, &tof);
    }
    const flowMeasurement_t flow = {.timestamp = testTick, .dt = 0.005f, .stdDevX = 6, .stdDevY = 6};
    kalmanCoreUpdateWithFlow(&k, &flow, &gyro);
    kalmanCoreFinalize(&k, step * 5);
    finiteState(&k);
    if (step > 10000) { vx += k.S[KC_STATE_PX]; vy += k.S[KC_STATE_PY]; }
  }
  vx /= 2000; vy /= 2000;
  printf("%s: mean body vx=%.6f vy=%.6f m/s\n", name, vx, vy);
  fflush(stdout);
  assert(fabsf(vx) < 0.005f && fabsf(vy) < 0.005f);
  assert(k.ground.active && k.ground.updates > 5000);
  const float norm = sqrtf(gravity.x*gravity.x + gravity.y*gravity.y + gravity.z*gravity.z);
  for (int i = 0; i < 3; i++) assert(fabsf(k.R[2][i] - gravity.axis[i]/norm) < 0.002f);
  const float estimatedYaw = atan2f(k.R[1][0], k.R[0][0]);
  // Gravity observes tilt only: no return to initialYaw=0 for a nonzero yaw.
  assert(fabsf(estimatedYaw - yaw) < 0.01f);
}

static void measurements(kalmanCoreData_t *k, unsigned step, float vx, float vy,
                         bool flowEnabled, bool rangeEnabled, int ageMs)
{
  const Axis3f gyroDegrees = {.axis = {0}};
  if (rangeEnabled && step % 13 == 0) {
    tofMeasurement_t tof = {.timestamp = testTick - ageMs,
                           .distance = 0.165f, .stdDev = 0.005f};
    kalmanCoreUpdateWithTof(k, &tof);
  }
  if (flowEnabled) {
    const float scale = 0.005f * 30 / (DEG_TO_RAD * 4.2f) / 0.165f;
    const flowMeasurement_t flow = {.timestamp = testTick - ageMs,
        .dt = 0.005f, .stdDevX = 6, .stdDevY = 6,
        .dpixelx = scale * vx, .dpixely = scale * vy};
    kalmanCoreUpdateWithFlow(k, &flow, &gyroDegrees);
  }
}

static void tinyCorrection(void)
{
  kalmanCoreData_t k;
  kalmanCoreInit(&k);
  k.S[KC_STATE_D0] = 1e-6f;
  k.S[KC_STATE_D1] = -2e-6f;
  kalmanCoreFinalize(&k, 0);
  assert(fabsf(k.q[1] - 0.5e-6f) < 1e-9f);
  assert(fabsf(k.q[2] + 1e-6f) < 1e-9f);
  finiteState(&k);
}

static void noReversion(void)
{
  kalmanCoreData_t k;
  kalmanCoreInit(&k);
  // Nonzero yaw and tilt; no external measurements or ground alignment.
  k.q[0] = 0.49937513f; k.q[1] = 0.024989585f;
  k.q[2] = 0.04328323f; k.q[3] = 0.8649431f;
  kalmanCoreFinalize(&k, 0);
  float initial[4];
  for (int i = 0; i < 4; i++) initial[i] = k.q[i];
  Axis3f acc = {.axis = {k.R[2][0]*GRAVITY_MAGNITUDE,
                        k.R[2][1]*GRAVITY_MAGNITUDE,
                        k.R[2][2]*GRAVITY_MAGNITUDE}};
  Axis3f gyro = {.axis = {0}};
  for (int step = 0; step < 2000; step++) {
    kalmanCorePredict(&k, 0, &acc, &gyro, 0.01f, false);
    kalmanCoreAddProcessNoise(&k, 0.01f);
    kalmanCoreFinalize(&k, step*10);
  }
  for (int i = 0; i < 4; i++) assert(fabsf(k.q[i] - initial[i]) < 1e-5f);
  assert(!k.ground.active && k.ground.updates == 0);
  finiteState(&k);
}

enum GateCase { missingFlow, missingRange, oldFlow, oldRange, thrustOn, flying,
                rotating, badGravityNorm, changingAcceleration, moving,
                backAndForth, noNewWindow, staleFlow, staleRange, clockWrap,
                quietQuantization };

static void gate(enum GateCase scenario)
{
  kalmanCoreData_t k;
  kalmanCoreInit(&k);
  Axis3f acc = {.axis = {0.017788f*GRAVITY_MAGNITUDE,
      0.053252f*GRAVITY_MAGNITUDE, 0.998682f*GRAVITY_MAGNITUDE}};
  Axis3f gyro = {.axis = {0}};
  for (unsigned step = 1; step <= 600; step++) {
    testTick = scenario == clockWrap ? UINT32_MAX-500+step*5 : 10000+step*5;
    acc.z = (scenario == badGravityNorm ? 1.2f : 0.998682f) * GRAVITY_MAGNITUDE;
    gyro.z = scenario == rotating ? 4*DEG_TO_RAD : 0;
    if (scenario == changingAcceleration) acc.x = (step % 4 < 2 ? .05f : -.05f)*GRAVITY_MAGNITUDE;
    if (step % 2 == 0) kalmanCorePredict(&k, scenario == thrustOn ? 0.001f : 0,
        &acc, &gyro, 0.01f, scenario == flying);
    kalmanCoreAddProcessNoise(&k, 0.005f);
    const bool flow = scenario != missingFlow &&
        !(scenario == staleFlow && step > 10) && !(scenario == noNewWindow && step > 30);
    const bool range = scenario != missingRange && !(scenario == staleRange && step > 13);
    float vx = scenario == moving ? .10f : 0;
    if (scenario == backAndForth) vx = step % 2 ? .10f : -.10f;
    // One isolated pixel every 100ms must not block actual quiet sensors.
    if (scenario == quietQuantization && step % 20 == 0)
      vx = .165f*(DEG_TO_RAD*4.2f)/30/.005f;
    measurements(&k, step, vx, 0, flow && scenario != oldFlow,
                 range && scenario != oldRange, 0);
    if (scenario == oldFlow) measurements(&k, step, 0, 0, true, false, 100);
    if (scenario == oldRange) measurements(&k, step, 0, 0, false, true, 300);
    kalmanCoreFinalize(&k, testTick);
    finiteState(&k);
  }
  if (scenario == clockWrap || scenario == quietQuantization) {
    assert(k.ground.active && k.ground.updates > 100);
  } else {
    assert(!k.ground.active && k.ground.updates == 0);
  }
}

static void movingAfterRest(void)
{
  kalmanCoreData_t k;
  kalmanCoreInit(&k);
  Axis3f acc = {.axis = {0, 0, GRAVITY_MAGNITUDE}};
  Axis3f gyro = {.axis = {0}};
  float vx = 0, vy = 0;
  uint32_t updatesAfterMoving = 0;
  for (unsigned step = 1; step <= 3200; step++) {
    testTick = 20000 + step*5;
    if (step % 2 == 0) kalmanCorePredict(&k, 0, &acc, &gyro, .01f, false);
    kalmanCoreAddProcessNoise(&k, .005f);
    measurements(&k, step, step <= 1200 ? 0 : .15f,
                 step <= 1200 ? 0 : -.10f, true, true, 0);
    kalmanCoreFinalize(&k, testTick);
    if (step == 1200) assert(k.ground.active && k.ground.updates > 100);
    if (step == 1240) {
      assert(!k.ground.active);
      updatesAfterMoving = k.ground.updates;
    }
    if (step > 1240) assert(k.ground.updates == updatesAfterMoving);
    if (step > 2800) { vx += k.S[KC_STATE_PX]; vy += k.S[KC_STATE_PY]; }
    finiteState(&k);
  }
  vx /= 400; vy /= 400;
  printf("after rest, moving: body vx=%.6f vy=%.6f m/s\n", vx, vy);
  assert(fabsf(vx - .15f) < .02f && fabsf(vy + .10f) < .02f);
}

static void stopLearning(void)
{
  // Each independent trigger must revoke an already-active correction.
  for (int trigger = 0; trigger < 6; trigger++) {
    kalmanCoreData_t k;
    kalmanCoreInit(&k);
    Axis3f acc = {.axis = {0, 0, GRAVITY_MAGNITUDE}};
    Axis3f gyro = {.axis = {0}};
    for (unsigned step = 1; step <= 400; step++) {
      testTick = 30000 + step*5;
      if (step % 2 == 0) kalmanCorePredict(&k, 0, &acc, &gyro, .01f, false);
      kalmanCoreAddProcessNoise(&k, .005f);
      measurements(&k, step, 0, 0, true, true, 0);
      kalmanCoreFinalize(&k, testTick);
    }
    assert(k.ground.active);
    const uint32_t previousUpdates = k.ground.updates;
    if (trigger == 2) gyro.x = 4*DEG_TO_RAD;
    if (trigger == 3) acc.z = 1.2f*GRAVITY_MAGNITUDE;
    if (trigger == 4) k.ground.flowAge = 0.10f;
    if (trigger == 5) k.ground.rangeAge = 0.30f;
    kalmanCorePredict(&k, trigger == 0 ? .001f : 0, &acc, &gyro, .01f, trigger == 1);
    assert(!k.ground.active && k.ground.updates == previousUpdates);
    kalmanCoreFinalize(&k, testTick);
    finiteState(&k);
    kalmanCoreInit(&k);
    assert(!k.ground.active && k.ground.updates == 0 && !k.ground.accSeen);
  }
}

static void invalidMeasurements(void)
{
  kalmanCoreData_t k;
  const Axis3f gyro = {.axis = {0}};
  kalmanCoreInit(&k);
  for (int field = 0; field < 5; field++) {
    flowMeasurement_t flow = {.dt = .005f, .stdDevX = 6, .stdDevY = 6};
    if (field == 0) flow.dt = NAN;
    if (field == 1) flow.dt = 0;
    if (field == 2) flow.dpixelx = NAN;
    if (field == 3) flow.stdDevX = 0;
    if (field == 4) flow.stdDevY = NAN;
    kalmanCoreUpdateWithFlow(&k, &flow, &gyro);
    finiteState(&k);
  }
  for (int field = 0; field < 4; field++) {
    tofMeasurement_t tof = {.distance = .165f, .stdDev = .005f};
    if (field == 0) tof.distance = NAN;
    if (field == 1) tof.distance = 0;
    if (field == 2) tof.stdDev = NAN;
    if (field == 3) tof.stdDev = 0;
    kalmanCoreUpdateWithTof(&k, &tof);
    finiteState(&k);
  }
  assert(!k.ground.active && k.ground.updates == 0);
}

int main(void)
{
  stationary("logged gravity", (Axis3f){.axis = {.017788f, .053252f, .998682f}}, 0);
  stationary("level", (Axis3f){.axis = {0, 0, 1}}, 0);
  stationary("opposite tilt, yaw=90", (Axis3f){.axis = {-.04f, -.08f, sqrtf(1-.04f*.04f-.08f*.08f)}}, PI/2);
  stationary("mixed tilt, yaw=-60", (Axis3f){.axis = {.07f, -.02f, sqrtf(1-.07f*.07f-.02f*.02f)}}, -PI/3);
  tinyCorrection();
  noReversion();
  for (int scenario = missingFlow; scenario <= quietQuantization; scenario++) gate(scenario);
  movingAfterRest();
  stopLearning();
  invalidMeasurements();
  puts("Kalman stationary regression passed");
  return 0;
}
