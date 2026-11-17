"""Run the real POSHOLD state machine and PID controller with GCC, Clang or Zig.

Usage: python scripts/test_poshold_assist.py [--cc path/to/compiler]
Synthetic measurements verify decisions, not real aircraft flight performance.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', default=(shutil.which('gcc') or shutil.which('clang') or
                                   shutil.which('zig')))
args = parser.parse_args()
if not args.cc:
    parser.error('Specify a native GCC, Clang or Zig compiler with --cc.')
core = root / 'components/core/crazyflie'
with tempfile.TemporaryDirectory(prefix='tinydrone-poshold-test-') as temp:
    # Old Windows TCC ships x87-only inline math in its bundled math.h. Use
    # portable float operations for this harness; firmware uses ESP-IDF libm.
    compat = []
    if 'tcc' in Path(args.cc).stem.lower():
        (Path(temp) / 'math.h').write_text('''#ifndef TINYDRONE_TEST_MATH_H
#define TINYDRONE_TEST_MATH_H
#include <float.h>
#define NAN (0.0f/0.0f)
#define isfinite(x) ((x) == (x) && (x) <= FLT_MAX && (x) >= -FLT_MAX)
#define isnan(x) ((x) != (x))
static inline float fabsf(float x) { return x < 0 ? -x : x; }
static inline float fminf(float x,float y) { return x < y ? x : y; }
static inline float fmaxf(float x,float y) { return x > y ? x : y; }
double sqrt(double x);
static inline float sqrtf(float x) { return (float)sqrt(x); }
double pow(double x,double y);
static inline float powf(float x,float y) { return (float)pow(x,y); }
static inline float copysignf(float x,float y) { return y < 0 ? -fabsf(x) : fabsf(x); }
double sin(double x), cos(double x), tan(double x);
static inline float sinf(float x) { return (float)sin(x); }
static inline float cosf(float x) { return (float)cos(x); }
static inline float tanf(float x) { return (float)tan(x); }
#endif
''', encoding='utf-8')
        compat = ['-I', temp]
    # Replace platform logging/parameter registration and flight-mode IO only.
    stubs = {
        'commander.h': '#pragma once\n',
        'log.h': '#define LOG_GROUP_START(...)\n#define LOG_GROUP_STOP(...)\n#define LOG_ADD(...)\n',
        'param.h': '#define PARAM_GROUP_START(...)\n#define PARAM_GROUP_STOP(...)\n#define PARAM_ADD(...)\n',
        'debug_cf.h': '#define DEBUG_PRINTI(...) ((void)0)\n',
        'crtp_commander.h': '#include <stdint.h>\n#define ALTHOLD_MODE 1\nuint8_t get_flight_mode(void);\n',
    }
    for name, contents in stubs.items():
        (Path(temp) / name).write_text(contents, encoding='utf-8')
    compiler = [args.cc]
    if Path(args.cc).stem.lower() == 'zig':
        compiler += ['cc']
        if sys.platform == 'win32':
            compiler += ['-target', 'x86_64-windows-gnu']
    for mode_name, mode_flags in [
            ('default', []),
            ('immediate_hold', ['-DPOSHOLD_BRAKE_BEFORE_HOLD=0'])]:
        binary = Path(temp) / f'test_{mode_name}.exe'
        subprocess.run([*compiler, '-std=c99', '-Wall', '-Wextra', *mode_flags,
                        '-I', temp,
                        '-I', str(core / 'modules/interface'),
                        '-I', str(core / 'hal/interface'),
                        '-I', str(core / 'utils/interface'),
                        '-I', str(root / 'components/drivers/spi_devices/pmw3901/include'),
                        str(root / 'scripts/test_poshold_assist.c'),
                        str(core / 'modules/src/poshold_assist.c'),
                        str(core / 'modules/src/position_controller_pid.c'),
                        str(core / 'modules/src/pid.c'),
                        str(core / 'utils/src/filter.c'),
                        str(core / 'utils/src/num.c'),
                        '-o', str(binary), *([] if compat else ['-lm'])], check=True)
        subprocess.run([str(binary)], check=True)
