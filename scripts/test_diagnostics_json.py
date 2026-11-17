"""Host-test the real /api/poshold JSON handler without aircraft hardware.

Usage: python scripts/test_diagnostics_json.py --cc path/to/zig.exe
Compiles the production serialization code with passive snapshot/HTTP stubs.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
WIFI = ROOT / 'components/drivers/general/wifi/wifi_esp32.c'
PRELUDE = r'''
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sensors.h"
#include "zranger2.h"
#include "flowdeck_v1v2.h"

typedef int esp_err_t;
typedef struct { int unused; } httpd_req_t;
#define HTTPD_500_INTERNAL_SERVER_ERROR 500
#define portTICK_PERIOD_MS 1U
enum { rangeDown, kalmanEstimator };
static state_t fixture_state;
static sensorDiagnostics_t fixture_sensors;
static flowdeckDiagnostics_t fixture_flow;
static zRanger2Diagnostics_t fixture_tof;
static float fixture_range, fixture_inputs[4];
static bool fixture_seen, allocation_fails;
static int allocations, frees, send_result, response_status;
static char response_body[8192];
static unsigned char *guarded_allocation;
static size_t guarded_size;

static bool stabilizerGetStateSnapshot(state_t *out) {
    *out = fixture_state; return fixture_seen;
}
static bool rangeGetFresh(int direction, uint32_t max_age, float *out) {
    (void)direction; (void)max_age; *out = fixture_range; return fixture_seen;
}
void flowdeckGetDiagnostics(flowdeckDiagnostics_t *out) { *out = fixture_flow; }
void zRanger2GetDiagnostics(zRanger2Diagnostics_t *out) { *out = fixture_tof; }
bool sensorsGetDiagnostics(sensorDiagnostics_t *out) {
    *out = fixture_sensors; return fixture_seen;
}
static uint32_t xTaskGetTickCount(void) { return UINT32_MAX; }
static int getStateEstimator(void) { return kalmanEstimator; }
static void crtpCommanderRpytGetPosHoldAssistTelemetry(
    uint8_t *phase, uint8_t *reason, bool *ready, float *height,
    float *roll, float *pitch, float *x, float *y, bool *hold) {
    *phase = 7; *reason = 11; *ready = fixture_seen; *height = 0;
    *roll = fixture_inputs[0]; *pitch = fixture_inputs[1];
    *x = fixture_inputs[2]; *y = fixture_inputs[3]; *hold = fixture_seen;
}
static void *fixture_malloc(size_t size) {
    if (allocation_fails) return NULL;
    assert(!guarded_allocation);
    guarded_allocation = malloc(size + 2);
    assert(guarded_allocation);
    guarded_size = size;
    guarded_allocation[0] = guarded_allocation[size + 1] = 0xA5;
    ++allocations;
    return guarded_allocation + 1;
}
static void fixture_free(void *data) {
    assert(data == guarded_allocation + 1);
    assert(guarded_allocation[0] == 0xA5);
    assert(guarded_allocation[guarded_size + 1] == 0xA5);
    free(guarded_allocation); guarded_allocation = NULL; ++frees;
}
static int httpd_resp_send_err(httpd_req_t *req, int code, const char *message) {
    (void)req; response_status = code;
    snprintf(response_body, sizeof(response_body), "%s", message); return -1;
}
static int httpd_resp_set_type(httpd_req_t *req, const char *type) {
    (void)req; assert(strcmp(type, "application/json") == 0); return 0;
}
static int httpd_resp_set_hdr(httpd_req_t *req, const char *key, const char *value) {
    (void)req; assert(strcmp(key, "Cache-Control") == 0);
    assert(strcmp(value, "no-store") == 0); return 0;
}
static int httpd_resp_send(httpd_req_t *req, const char *body, size_t length) {
    (void)req; assert(length < 4096); assert(length == strlen(body));
    assert(length < sizeof(response_body));
    memcpy(response_body, body, length + 1); response_status = 200;
    return send_result;
}
#define malloc fixture_malloc
#define free fixture_free
'''

EPILOGUE = r'''
#undef malloc
#undef free
int main(int argc, char **argv) {
    assert(argc == 2);
    const int mode = atoi(argv[1]);
    fixture_flow.ageMs = fixture_flow.acceptedAgeMs = UINT32_MAX;
    fixture_tof.ageMs = UINT32_MAX; fixture_tof.status = UINT8_MAX;
    fixture_sensors.imu.ageMs = fixture_sensors.barometer.ageMs =
        fixture_sensors.magnetometer.ageMs = UINT32_MAX;
    if (mode != 0) {
        fixture_seen = true;
        fixture_state.position.x = 1.25f; fixture_state.velocity.y = -2.5f;
        fixture_state.attitude.yaw = 45.0f;
        fixture_range = 0.4321f;
        fixture_inputs[0] = 1; fixture_inputs[1] = -2;
        fixture_inputs[2] = 3; fixture_inputs[3] = -4;
        fixture_flow.seen = fixture_flow.accepted = true;
        fixture_flow.acceptedSamples = 123; fixture_flow.samples = 456;
        fixture_flow.acceptedAgeMs = 5; fixture_flow.ageMs = 1;
        fixture_flow.dx = 11; fixture_flow.dy = -12;
        fixture_flow.rawDx = 12; fixture_flow.rawDy = -11;
        fixture_flow.dt = 0.005f; fixture_flow.squal = 42;
        fixture_flow.motion = 0xB0; fixture_flow.shutter = 1234;
        fixture_flow.observation = 31; fixture_flow.rawDataSum = 32;
        fixture_flow.maxRawData = 33; fixture_flow.minRawData = 34;
        fixture_sensors.imu.present = fixture_sensors.imu.seen = true;
        fixture_sensors.imu.fresh = fixture_sensors.imu.calibrated = true;
        fixture_sensors.imu.accRaw = (Axis3i16){.x=101,.y=-102,.z=103};
        fixture_sensors.imu.gyroRaw = (Axis3i16){.x=-201,.y=202,.z=-203};
        fixture_sensors.imu.tempRaw = -301; fixture_sensors.imu.tempC = 25.5f;
        fixture_sensors.imu.accG = (Axis3f){.x=.1f,.y=-.2f,.z=1.0f};
        fixture_sensors.imu.gyroDps = (Axis3f){.x=-4,.y=5,.z=-6};
        fixture_sensors.barometer.pressureRaw = 1234567;
        fixture_sensors.barometer.temperatureRaw = -7654321;
        fixture_sensors.barometer.pressureHpa = 1001.25f;
        fixture_sensors.barometer.tempC = 26.5f;
        fixture_sensors.barometer.altitudeM = 123.5f;
        fixture_sensors.magnetometer.raw = (Axis3i16){.x=401,.y=-402,.z=403};
        fixture_sensors.magnetometer.gauss = (Axis3f){.x=.4f,.y=-.5f,.z=.6f};
        fixture_tof.seen = fixture_tof.readOk = fixture_tof.valid = fixture_tof.fresh = true;
        fixture_tof.status = 0; fixture_tof.rangeMm = 432;
        fixture_tof.signalRateMcps = 1.25f; fixture_tof.ambientRateMcps = 2.5f;
        fixture_tof.sigmaMm = 3.75f;
    }
    if (mode == 2) {
        fixture_state.position.x = fixture_state.position.y = fixture_state.position.z = FLT_MAX;
        fixture_state.velocity.x = fixture_state.velocity.y = fixture_state.velocity.z = -FLT_MAX;
        fixture_state.attitude.roll = fixture_state.attitude.pitch = fixture_state.attitude.yaw = FLT_MAX;
        fixture_range = FLT_MAX;
        for (int i=0;i<4;++i) fixture_inputs[i] = -FLT_MAX;
        fixture_flow.dt = FLT_MAX;
        fixture_flow.rawDx = INT16_MIN; fixture_flow.rawDy = INT16_MAX;
        fixture_flow.samples = fixture_flow.acceptedSamples = UINT32_MAX;
        fixture_sensors.imu.tempC = FLT_MAX;
        fixture_sensors.imu.accG = (Axis3f){.x=FLT_MAX,.y=-FLT_MAX,.z=FLT_MAX};
        fixture_sensors.imu.gyroDps = fixture_sensors.imu.accG;
        fixture_sensors.barometer.pressureRaw = INT32_MIN;
        fixture_sensors.barometer.temperatureRaw = INT32_MAX;
        fixture_sensors.barometer.pressureHpa = FLT_MAX;
        fixture_sensors.barometer.tempC = -FLT_MAX;
        fixture_sensors.barometer.altitudeM = FLT_MAX;
        fixture_sensors.magnetometer.gauss = fixture_sensors.imu.accG;
        fixture_tof.rangeMm = INT16_MIN;
        fixture_tof.signalRateMcps = fixture_tof.ambientRateMcps = fixture_tof.sigmaMm = FLT_MAX;
    }
    if (mode == 3) {
        fixture_state.velocity.x = NAN; fixture_range = INFINITY;
        for (int i=0;i<4;++i) fixture_inputs[i] = NAN;
        fixture_flow.dt = NAN;
        fixture_sensors.imu.tempC = NAN;
        fixture_sensors.imu.accG = (Axis3f){.x=NAN,.y=INFINITY,.z=-INFINITY};
        fixture_sensors.imu.gyroDps = fixture_sensors.imu.accG;
        fixture_sensors.barometer.pressureHpa = INFINITY;
        fixture_sensors.barometer.tempC = NAN;
        fixture_sensors.barometer.altitudeM = -INFINITY;
        fixture_sensors.magnetometer.gauss = fixture_sensors.imu.accG;
        fixture_tof.signalRateMcps = NAN; fixture_tof.ambientRateMcps = INFINITY;
        fixture_tof.sigmaMm = -INFINITY;
    }
    if (mode == 4) allocation_fails = true;
    if (mode == 5) send_result = -77;
    if (mode == 6) {
        char big[WEB_DIAGNOSTICS_CAPACITY + 1]; memset(big, 'x', sizeof(big)-1);
        big[sizeof(big)-1] = 0;
        web_diagnostics_json_t json = {.data=fixture_malloc(WEB_DIAGNOSTICS_CAPACITY), .ok=true};
        web_diagnostics_append(&json, "%s", big);
        assert(!json.ok); web_diagnostics_append(&json, "%s", big);
        fixture_free(json.data);
        assert(allocations == frees); puts("overflow guarded"); return 0;
    }
    httpd_req_t request = {0};
    const int result = web_poshold_diagnostics_handler(&request);
    assert(!guarded_allocation); assert(allocations == frees);
    if (mode == 4) assert(result == -1 && allocations == 0);
    else assert(result == send_result && allocations == 1);
    printf("%d\t%s\n", response_status, response_body);
    return 0;
}
'''


def strict_json(body):
    def reject(value):
        raise AssertionError(f'Non-finite JSON token: {value}')
    return json.loads(body, parse_constant=reject)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=(shutil.which('gcc') or shutil.which('clang') or
                                        shutil.which('zig')))
    args = parser.parse_args()
    if not args.cc:
        parser.error('Specify a native GCC, Clang or Zig compiler with --cc.')
    source = WIFI.read_text(encoding='utf-8')
    begin = source.index('#define WEB_DIAGNOSTICS_CAPACITY ')
    end = source.index('\nstatic bool web_queue_control', begin)
    with tempfile.TemporaryDirectory(prefix='tinydrone-json-test-') as temporary:
        directory = Path(temporary)
        fixture = directory / 'test.c'
        fixture.write_text(PRELUDE + source[begin:end] + EPILOGUE, encoding='utf-8')
        binary = directory / ('test.exe' if sys.platform == 'win32' else 'test')
        compiler = [args.cc]
        if Path(args.cc).stem.lower() == 'zig':
            compiler += ['cc']
            if sys.platform == 'win32':
                compiler += ['-target', 'x86_64-windows-gnu']
        includes = [
            'components/core/crazyflie/modules/interface',
            'components/core/crazyflie/hal/interface',
            'components/drivers/spi_devices/pmw3901/include',
            'components/drivers/i2c_devices/vl53l1/include',
        ]
        include_flags = [item for path in includes for item in ('-I', str(ROOT / path))]
        # GCC 4.x warns on valid zero/designated initializers in anonymous
        # aggregate types used by the firmware; retain format/error checks.
        subprocess.run([*compiler, '-std=c99', '-Wall', '-Wextra', '-Werror',
                        '-Wno-missing-braces', '-Wno-missing-field-initializers',
                        *include_flags, str(fixture), '-o', str(binary), '-lm'], check=True)
        documents = []
        for mode in range(6):
            output = subprocess.check_output([str(binary), str(mode)], text=True).strip()
            status, body = output.split('\t', 1)
            if mode == 4:
                assert status == '500' and body == 'diagnostic allocation failed'
                documents.append(None)
                continue
            assert status == '200'
            assert len(body.encode()) < 4096
            documents.append(strict_json(body))
        absent, normal, extreme, nonfinite, _, send_failure = documents
        assert not absent['stateValid'] and not absent['sensors']['available']
        assert not absent['flow']['seen'] and absent['flow']['ageMs'] == 4294967295
        assert not absent['sensors']['tof']['readOk']
        assert absent['sensors']['tof']['status'] == 255
        assert absent['sensors']['imu']['ageMs'] == 4294967295
        assert normal['position'] == [1.25, 0, 0]
        assert normal['posHold']['target'] == [3, -4]
        assert normal['flow']['rawDx'] == 12 and normal['flow']['rawDy'] == -11
        assert normal['flow']['dx'] == 11 and normal['flow']['dy'] == -12
        assert [normal['flow'][key] for key in
                ('observation', 'rawDataSum', 'maxRawData', 'minRawData')] == [31, 32, 33, 34]
        imu = normal['sensors']['imu']
        assert imu['accRaw'] == [101, -102, 103] and imu['gyroRaw'] == [-201, 202, -203]
        assert imu['temperatureRaw'] == -301 and imu['temperatureC'] == 25.5
        assert normal['sensors']['barometer']['temperatureRaw'] == -7654321
        assert normal['sensors']['magnetometer']['raw'] == [401, -402, 403]
        assert normal['sensors']['tof']['rangeMm'] == 432
        assert normal['sensors']['tof']['sigmaMm'] == 3.75
        assert extreme['flow']['dtMs'] > 3e41
        assert extreme['sensors']['barometer']['pressureRaw'] == -2147483648
        assert not nonfinite['stateValid'] and nonfinite['velocity'] == [0, 0, 0]
        assert nonfinite['sensors']['imu']['accG'] == [None, None, None]
        assert nonfinite['sensors']['tof']['sigmaMm'] is None
        assert send_failure == normal
        assert subprocess.check_output([str(binary), '6'], text=True).strip() == 'overflow guarded'
        print('PASS: real diagnostics JSON handler, mapping, missing samples, extreme/non-finite '
              'values, allocation/send failure, bounded writes and allocation cleanup.')


if __name__ == '__main__':
    main()
