"""Exercise the real Kalman core on synthetic stationary and moving measurements.

Usage: python scripts/test_kalman_static.py --cc path/to/zig
Only platform IO, DSP matrix primitives and unused UWB rejection are replaced.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
core = root / 'components/core/crazyflie'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--cc', default=shutil.which('gcc') or shutil.which('clang') or shutil.which('zig'))
parser.add_argument('--source', type=Path, default=core / 'modules/src/kalman_core.c')
args = parser.parse_args()
if not args.cc:
    parser.error('Specify a native GCC, Clang or Zig compiler with --cc.')

with tempfile.TemporaryDirectory(prefix='tinydrone-kalman-') as directory:
    temp = Path(directory)
    stubs = {
        'cfassert.h': '#pragma once\n#include <assert.h>\n#define __CFASSERT_H__\n#define ASSERT(x) assert(x)\n',
        'log.h': '#define LOG_GROUP_START(...)\n#define LOG_GROUP_STOP(...)\n#define LOG_ADD(...)\n',
        'param.h': '#define PARAM_GROUP_START(...)\n#define PARAM_GROUP_STOP(...)\n#define PARAM_ADD(...)\n',
        'debug_cf.h': '#define DEBUG_PRINTI(...) ((void)0)\n',
        'static_mem.h': '#include <string.h>\n#define NO_DMA_CCM_SAFE_ZERO_INIT\n',
        'FreeRTOS.h': '#include <stdint.h>\n#define pdMS_TO_TICKS(ms) (ms)\n',
        'task.h': '#pragma once\n#include <stdint.h>\nextern uint32_t testTick;\nstatic inline uint32_t xTaskGetTickCount(void) { return testTick; }\n',
        'xtensa_math.h': '''#pragma once
#include <math.h>
#include <stdint.h>
#include <assert.h>
typedef float float32_t;
typedef int xtensa_status;
#define XTENSA_MATH_SUCCESS 0
#define PI 3.14159265358979323846f
typedef struct { uint16_t numRows, numCols; float *pData; } xtensa_matrix_instance_f32;
static inline int xtensa_mat_trans_f32(const xtensa_matrix_instance_f32 *a, xtensa_matrix_instance_f32 *b) {
  assert(a->numRows == b->numCols && a->numCols == b->numRows);
  for (int i=0;i<a->numRows;i++) for(int j=0;j<a->numCols;j++) b->pData[j*b->numCols+i]=a->pData[i*a->numCols+j];
  return 0;
}
static inline int xtensa_mat_mult_f32(const xtensa_matrix_instance_f32 *a, const xtensa_matrix_instance_f32 *b, xtensa_matrix_instance_f32 *c) {
  assert(a->numCols==b->numRows && c->numRows==a->numRows && c->numCols==b->numCols);
  for(int i=0;i<c->numRows;i++) for(int j=0;j<c->numCols;j++) {
    float sum=0; for(int k=0;k<a->numCols;k++) sum+=a->pData[i*a->numCols+k]*b->pData[k*b->numCols+j];
    c->pData[i*c->numCols+j]=sum;
  }
  return 0;
}
static inline int xtensa_mat_inverse_f32(const xtensa_matrix_instance_f32 *a, xtensa_matrix_instance_f32 *b) { (void)a; (void)b; assert(0); return -1; }
static inline int xtensa_sqrt_f32(float a, float *b) { *b=sqrtf(a); return 0; }
#define xtensa_sin_f32 sinf
#define xtensa_cos_f32 cosf
''',
    }
    for name, content in stubs.items():
        (temp / name).write_text(content, encoding='utf-8')
    compiler = [args.cc]
    if Path(args.cc).stem.lower() == 'zig':
        compiler += ['cc']
        if sys.platform == 'win32':
            compiler += ['-target', 'x86_64-windows-gnu']
    executable = temp / 'kalman-static.exe'
    subprocess.run([*compiler, '-std=c99', '-O1', '-UNDEBUG', '-Wall', '-Wextra',
                    '-Wno-unused-parameter', '-Wno-unused-variable',
                    '-Wno-pointer-to-int-cast', '-Wno-absolute-value',
                    '-include', str(temp / 'cfassert.h'), '-I', str(temp),
                    '-I', str(core / 'modules/interface'),
                    '-I', str(core / 'utils/interface'),
                    '-I', str(core / 'hal/interface'),
                    str(root / 'scripts/test_kalman_static.c'), str(args.source),
                    '-o', str(executable), '-lm'], check=True, timeout=180)
    subprocess.run([str(executable)], check=True, timeout=60)
