"""Host-test real web control validation/queueing, without an aircraft.

Usage: python scripts/test_web_control.py --cc path/to/gcc.exe
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[1]
PRELUDE = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "crtp.h"
#define M2T(ms) (ms)
#define pdTRUE 1
typedef struct { uint8_t size; uint8_t data[64]; } UDPPacket;
static UDPPacket queued;
static int udpDataRx, sends, takes, gives;
static int webControlMutex = 1;
static bool queueOk = true, lockOk = true, isUDPConnected;
static int xQueueSend(int queue, const UDPPacket *packet, int timeout) {
    (void)queue; assert(timeout == 10); ++sends;
    if (queueOk) queued = *packet;
    return queueOk;
}
static int xSemaphoreTake(int mutex, int timeout) {
    assert(mutex == 1 && timeout == 10); ++takes; return lockOk;
}
static void xSemaphoreGive(int mutex) { assert(mutex == 1); ++gives; }
'''
EPILOGUE = r'''
int main(void) {
    web_control_packet_t command = {.roll=15, .pitch=-15, .yaw=45, .thrust=32767};
    assert(sizeof(command) == 15);
    /* Every combination of mode and action bits must preserve the legacy
     * payload, separate the carefree bit, and retain the speed flag. */
    for (unsigned flags = 0; flags < 32; ++flags) {
        command.carefree = flags;
        const int before = sends;
        assert(web_apply_control(&command));
        assert(sends == before + 1 && takes == gives && isUDPConnected);
        assert(queued.size == 16);
        CRTPPacket packet = {0};
        packet.size = queued.size - 1;
        memcpy(packet.raw, queued.data, queued.size);
        assert(packet.port == CRTP_PORT_SETPOINT && packet.channel == 0);
        assert(packet.reserved == (flags & 1));
        assert(memcmp(packet.data, &command, 14) == 0);
        assert(packet.data[14] == (flags & 0x1E));
    }
    /* Undefined bits cannot accidentally become new flight commands. */
    const int before = sends;
    for (unsigned flags = 32; flags < 256; ++flags) {
        command.carefree = flags;
        assert(!web_apply_control(&command));
    }
    assert(sends == before && takes == gives);
    command.carefree = 0x10;
    command.roll = NAN; assert(!web_apply_control(&command));
    command.roll = 15; command.pitch = INFINITY; assert(!web_apply_control(&command));
    command.pitch = -15; command.yaw = 361; assert(!web_apply_control(&command));
    assert(sends == before);
    command.yaw = 45;
    queueOk = false; isUDPConnected = false;
    assert(!web_apply_control(&command));
    assert(takes == gives && !isUDPConnected);
    queueOk = true; lockOk = false;
    const int failedSend = sends;
    assert(!web_apply_control(&command) && sends == failedSend);
    puts("Web control flags, legacy payload, validation and queue failures passed");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', default=(shutil.which('gcc') or shutil.which('clang') or
                                        shutil.which('zig')))
    args = parser.parse_args()
    if not args.cc:
        parser.error('Specify a native GCC, Clang or Zig compiler with --cc.')
    source = (ROOT / 'components/drivers/general/wifi/wifi_esp32.c').read_text(encoding='utf-8')
    struct_start = source.index('typedef struct __attribute__((packed))')
    struct_end = source.index('} web_control_packet_t;', struct_start) + len('} web_control_packet_t;')
    begin = source.index('static bool web_queue_control(')
    end = source.index('\n#if WIFI_WEB_REMOTE_ENABLE', begin)
    with tempfile.TemporaryDirectory(prefix='tinydrone-web-control-') as temporary:
        directory = Path(temporary)
        fixture = directory / 'control.c'
        fixture.write_text(PRELUDE + source[struct_start:struct_end] +
                           source[begin:end] + EPILOGUE, encoding='utf-8')
        binary = directory / ('test.exe' if sys.platform == 'win32' else 'test')
        compiler = [args.cc]
        if Path(args.cc).stem.lower() == 'zig':
            compiler += ['cc']
            if sys.platform == 'win32':
                compiler += ['-target', 'x86_64-windows-gnu']
        subprocess.run([*compiler, '-std=c99', '-Wall', '-Wextra', '-Werror',
                        '-I', str(ROOT / 'components/core/crazyflie/modules/interface'),
                        str(fixture), '-o', str(binary), '-lm'], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
