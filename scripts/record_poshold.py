"""Record read-only POSHOLD sensor diagnostics to CSV (no control commands).

Connect the computer to the aircraft Wi-Fi before running this script.
Use with propellers removed for the direction/scale measurement in the docs.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time
from urllib.error import URLError
from urllib.request import ProxyHandler, Request, build_opener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://192.168.26.28/api/poshold')
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--hz', type=float, default=10)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    if args.seconds <= 0 or not 0 < args.hz <= 10:
        parser.error('--seconds must be positive; --hz must be in (0, 10].')
    output = args.out or Path(time.strftime('poshold-%Y%m%d-%H%M%S.csv'))
    # Use a direct connection to the onboard AP, even on a PC with a proxy.
    opener = build_opener(ProxyHandler({}))
    fields = ['elapsed_s', 'uptime_ms', 'state_valid', 'kalman',
              'x_m', 'y_m', 'z_m', 'vx_mps', 'vy_mps', 'vz_mps',
              'forward_mps', 'left_mps',
              'roll_deg', 'pitch_deg', 'yaw_deg', 'range_m', 'range_fresh',
              'assist_phase', 'assist_reason', 'poshold_ready', 'roll_input',
              'pitch_input', 'target_x_m', 'target_y_m', 'hold_xy',
              'flow_seen', 'flow_fresh', 'flow_accepted', 'squal', 'motion',
              'shutter', 'dx', 'dy', 'dt_ms', 'sample_age_ms',
              'accepted_age_ms', 'samples', 'accepted_samples']
    count = 0
    start = time.monotonic()
    print(f'Recording {args.url} for {args.seconds:g} s -> {output}')
    # Preserve earlier measurement files if an output name is reused.
    with output.open('x', newline='', encoding='utf-8-sig') as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        try:
            while time.monotonic() - start < args.seconds:
                before = time.monotonic()
                try:
                    request = Request(args.url, headers={'Cache-Control': 'no-cache'})
                    with opener.open(request, timeout=1.5) as response:
                        data = json.load(response)
                    flow = data['flow']
                    assist = data['posHold']
                    vx, vy, _ = data['velocity']
                    yaw = math.radians(data['attitude'][2])
                    forward = math.cos(yaw) * vx + math.sin(yaw) * vy
                    left = -math.sin(yaw) * vx + math.cos(yaw) * vy
                    writer.writerow([
                        round(time.monotonic() - start, 3), data['uptimeMs'],
                        data['stateValid'], data['kalman'], *data['position'],
                        *data['velocity'], forward, left, *data['attitude'], data['rangeM'],
                        data['rangeFresh'], assist['phase'], assist['reason'],
                        assist['ready'], assist['rollInput'], assist['pitchInput'],
                        *assist['target'], assist['holdXY'],
                        flow['seen'], flow['fresh'],
                        flow['accepted'], flow['squal'], flow['motion'],
                        flow['shutter'], flow['dx'], flow['dy'], flow['dtMs'],
                        flow['ageMs'], flow['acceptedAgeMs'], flow['samples'],
                        flow['acceptedSamples']])
                    stream.flush()
                    count += 1
                except (URLError, TimeoutError, ValueError, KeyError) as error:
                    print(f'Sample unavailable: {error}')
                time.sleep(max(0, 1 / args.hz - (time.monotonic() - before)))
        except KeyboardInterrupt:
            pass
    print(f'Saved {count} samples: {output.resolve()}')
    return 0 if count else 1


if __name__ == '__main__':
    raise SystemExit(main())
