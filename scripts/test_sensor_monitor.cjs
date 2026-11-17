/* No browser download needed: exercise the page's real rendering and polling. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, 'sensor_monitor.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const nodes = new Map();
for (const match of html.matchAll(/\bid="([^"]+)"([^>]*)/g)) {
  const classes = new Set();
  nodes.set(match[1], {
    textContent: '', className: '', reading: match[2].includes('data-reading'),
    classList: { add: name => classes.add(name), toggle: (name, on) => on ? classes.add(name) : classes.delete(name) },
    addEventListener() {}, width: 1150, height: 240,
    getContext: () => new Proxy({}, {get: () => () => {}})
  });
}
let fetches = 0, nextTimer = 0;
let pendingFetch;
const timers = new Map();
const context = vm.createContext({
  document: {getElementById: id => { assert(nodes.has(id), `Unknown element ${id}`); return nodes.get(id); },
    querySelectorAll: () => [...nodes.values()].filter(node => node.reading)},
  window: {addEventListener() {}}, Date, AbortController, performance: {now: () => 0},
  fetch: () => { fetches++; return new Promise((resolve, reject) => { pendingFetch = {resolve, reject}; }); },
  setTimeout: (fn, ms) => { const id = ++nextTimer; timers.set(id, {fn, ms}); return id; },
  clearTimeout: id => timers.delete(id)
});
vm.runInContext(script, context);
const monitor = vm.runInContext('SensorMonitor', context);
const value = id => nodes.get(id).textContent;
const flush = () => new Promise(resolve => setImmediate(resolve));
const sensor = {present: true, seen: true, fresh: true, ageMs: 8, samples: 123};
const snapshot = {
  stateValid: true, kalman: true, rangeFresh: true, rangeM: .321,
  velocity: [1, 0, 0], attitude: [2, 3, 90], position: [0, 0, .3],
  flow: {...sensor, rawDx: -12, rawDy: 4, dx: 4, dy: 12, dtMs: 10,
    squal: 80, motion: 176, accepted: true, shutter: 900, rawDataSum: 12,
    maxRawData: 230, minRawData: 1, observation: 127, acceptedAgeMs: 9, acceptedSamples: 100},
  sensors: {available: true,
    imu: {...sensor, accRaw: [1, -2, 16384], gyroRaw: [3, 4, -5], accG: [.001, -.002, 1],
      gyroDps: [.1, .2, -.3], temperatureRaw: -20, temperatureC: 25.12, calibrated: true},
    tof: {...sensor, readOk: true, status: 0, valid: true, rangeMm: 321,
      signalRateMcps: 2.5, ambientRateMcps: .01, sigmaMm: 1.25},
    barometer: {...sensor, pressureRaw: 345678, temperatureRaw: 456789,
      pressureHpa: 1013.25, temperatureC: 24.5, altitudeM: .15},
    magnetometer: {...sensor, raw: [1, 2, 3], gauss: [.01, -.02, .03]}}
};

(async () => {
  assert.equal(fetches, 0, 'Monitor must keep automatic polling paused');
  monitor.start();
  assert.equal(fetches, 1);
  monitor.start(); monitor.stop(); monitor.start();
  assert.equal(fetches, 1, 'Restart while a request is pending must not overlap');
  for (const missing of [null, undefined, '', false, '0', NaN, Infinity]) assert.equal(monitor.fmt(missing), '—');
  assert.equal(monitor.fmt(0), '0.000');

  monitor.render(snapshot);
  assert.equal(value('forward'), '0.000'); assert.equal(value('left'), '-1.000');
  assert.equal(value('height'), '0.300 / 0.321');
  assert.equal(value('flowRaw'), '-12 / 4'); assert.equal(value('flowMapped'), '4 / 12');
  assert.match(value('flowMotion'), /0xB0.*已接收/);
  assert.equal(value('flowObservation'), '127');
  assert.equal(value('imuAccRaw'), '1 / -2 / 16384'); assert.equal(value('imuTemp'), '-20 / 25.12');
  assert.equal(value('barometerRaw'), '345678 / 456789');
  assert.equal(value('barometerTemp'), '24.50'); assert.equal(value('tofRange'), '321');
  assert.match(value('status'), /Kalman 已启用/);

  monitor.render({...snapshot, stateValid: false, kalman: false});
  assert.equal(value('forward'), '—'); assert.equal(value('left'), '—'); assert.equal(value('att'), '—');
  assert.equal(value('height'), '— / 0.321', 'Valid independent range remains visible when fusion is invalid');
  assert.match(value('status'), /Kalman 未启用/);
  monitor.render({...snapshot, stateValid: false, rangeFresh: false});
  assert.equal(value('height'), '— / —');

  const failedTof = structuredClone(snapshot);
  Object.assign(failedTof.sensors.tof, {readOk: false, valid: false, rangeMm: 0,
    signalRateMcps: 0, ambientRateMcps: 0, sigmaMm: 0});
  monitor.render(failedTof);
  for (const id of ['tofRange', 'tofSignal', 'tofAmbient', 'tofSigma']) assert.equal(value(id), '—');
  assert.equal(value('tofValid'), '— / 失败'); assert.equal(value('tofStatus'), '0');
  assert.match(value('tofNote'), /读取失败/);
  failedTof.sensors.tof.readOk = true;
  monitor.render(failedTof);
  assert.equal(value('tofRange'), '0', 'Successful but invalid raw ToF result remains visible');
  assert.equal(value('tofValid'), '无效 / 成功');

  const invalid = structuredClone(snapshot);
  invalid.sensors.tof.valid = false; invalid.sensors.tof.rangeMm = null;
  invalid.sensors.imu.accRaw = [0, null]; invalid.velocity[0] = null;
  invalid.sensors.magnetometer.present = false;
  monitor.render(invalid);
  assert.equal(value('forward'), '—'); assert.equal(value('left'), '—');
  assert.equal(value('imuAccRaw'), '0 / — / —'); assert.equal(value('tofRange'), '—');
  assert.match(value('tofValid'), /无效/); assert.match(value('tofNote'), /不能作为有效高度/);
  assert.equal(value('magnetometerRaw'), '—'); assert.match(value('magnetometerState'), /未安装/);

  monitor.render({velocity: [0, 0, 0], attitude: [0, 0, 0], position: [0, 0, 0], flow: {dx: 0, dy: 0, fresh: true}});
  assert.equal(value('forward'), '0.000'); assert.equal(value('flowMapped'), '0 / 0');
  assert.equal(value('imuAccRaw'), '—'); assert.match(value('imuNote'), /当前固件未提供/);
  assert.match(value('flowNote'), /未提供光流原始帧/);
  monitor.render({...snapshot, sensors: {available: false, tof: snapshot.sensors.tof}});
  assert.equal(value('imuAccRaw'), '—'); assert.match(value('imuNote'), /暂不可用/);
  assert.equal(value('tofRange'), '321', 'Independent ToF remains visible when IMU backend is unavailable');

  pendingFetch.resolve({ok: true, json: async () => snapshot});
  await flush();
  assert.equal(fetches, 1);
  const next = [...timers.values()].find(timer => timer.ms === 100);
  assert(next, 'Next poll must be scheduled only after the response');
  next.fn(); assert.equal(fetches, 2);
  pendingFetch.reject(new Error('网络中断'));
  await flush();
  assert.match(value('status'), /未连接.*网络中断/);
  for (const node of nodes.values()) if (node.reading) assert.equal(node.textContent, '—', 'Disconnect must clear all readings');
  assert.match(value('stamp'), /已断连/);
  monitor.stop();
  console.log('PASS sensor monitor rendering, missing data, disconnect and non-overlapping polling');
})().catch(error => { console.error(error); process.exitCode = 1; });
