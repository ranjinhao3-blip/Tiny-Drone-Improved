/* Real browser rendering checks. Uses an existing Playwright + Chromium/Edge. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { chromium } = require(process.env.SENSOR_MONITOR_PLAYWRIGHT || 'playwright');
const html = fs.readFileSync(path.join(__dirname, 'sensor_monitor.html'), 'utf8');
const screenshotDir = process.env.SENSOR_MONITOR_SCREENSHOTS || path.join(os.tmpdir(), 'tiny-drone-sensor-monitor-qa');
fs.mkdirSync(screenshotDir, {recursive: true});
const sample = {seen: true, fresh: true, ageMs: 8, samples: 1234};
const current = {
  stateValid: true, kalman: true, rangeFresh: true, rangeM: .319,
  position: [0, 0, .318], velocity: [.008, -.002, 0], attitude: [.5, -.3, 0],
  flow: {...sample, rawDx: 0, rawDy: -1, dx: -1, dy: 0, dtMs: 10, squal: 80, motion: 176,
    accepted: true, shutter: 900, observation: 127, rawDataSum: 12, maxRawData: 230,
    minRawData: 1, acceptedAgeMs: 8, acceptedSamples: 1200},
  sensors: {available: true,
    imu: {...sample, present: true, accRaw: [20, -30, 4096], gyroRaw: [2, -3, 4],
      accG: [.0049, -.0073, 1], gyroDps: [.061, -.092, .122], temperatureRaw: -4060,
      temperatureC: 24.59, calibrated: true},
    tof: {...sample, readOk: true, status: 0, valid: true, rangeMm: 319,
      signalRateMcps: 2.5, ambientRateMcps: .01, sigmaMm: 1.25},
    barometer: {...sample, present: true, pressureRaw: 9085466, temperatureRaw: 8569150,
      pressureHpa: 1013.25, temperatureC: 24.5, altitudeM: .15},
    magnetometer: {present: false, seen: false, fresh: false, ageMs: null, samples: 0,
      raw: null, gauss: null}}
};

(async () => {
  const browser = await chromium.launch({headless: true,
    ...(process.env.SENSOR_MONITOR_BROWSER ? {executablePath: process.env.SENSOR_MONITOR_BROWSER} : {})});
  try {
    const page = await browser.newPage({viewport: {width: 1280, height: 1000}});
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    let mode = 'current', active = 0, peak = 0, snapshots = 0;
    await page.route('http://sensor-monitor.test/**', async route => {
      assert.equal(route.request().method(), 'GET');
      if (new URL(route.request().url()).pathname !== '/snapshot') {
        await route.fulfill({body: html, contentType: 'text/html; charset=utf-8'}); return;
      }
      active++; peak = Math.max(peak, active); snapshots++;
      const selected = mode;
      try {
        if (selected === 'slow') await new Promise(resolve => setTimeout(resolve, 250));
        if (selected === 'offline') await route.fulfill({status: 502, json: {error: '无人机连接超时'}});
        else if (selected === 'legacy') await route.fulfill({json: {
          position: [0, 0, .318], velocity: [0, 0, 0], attitude: [0, 0, 0], stateValid: true,
          kalman: true, rangeFresh: true, rangeM: .319,
          flow: {...sample, dx: 0, dy: 0, squal: 80, dtMs: 10}}});
        else await route.fulfill({json: current});
      } finally { active--; }
    });
    await page.goto('http://sensor-monitor.test/');
    await page.waitForTimeout(200);
    assert.equal(snapshots, 0, 'Automatic polling must remain paused on page load');
    await page.evaluate(() => SensorMonitor.start());
    await page.waitForFunction(() => document.querySelector('#imuAccRaw').textContent === '20 / -30 / 4096');
    assert.equal(await page.locator('#barometerRaw').textContent(), '9085466 / 8569150');
    assert.match(await page.locator('#magnetometerState').textContent(), /未安装/);
    await page.screenshot({path: path.join(screenshotDir, 'current-desktop.png'), fullPage: true});

    await page.setViewportSize({width: 390, height: 844});
    assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'Mobile page must fit viewport');
    await page.screenshot({path: path.join(screenshotDir, 'current-mobile.png'), fullPage: true});
    await page.setViewportSize({width: 1280, height: 1000});
    mode = 'legacy';
    await page.waitForFunction(() => document.querySelector('#imuNote').textContent.includes('当前固件未提供'));
    assert.equal(await page.locator('#imuAccRaw').textContent(), '—');
    assert.equal(await page.locator('#flowMapped').textContent(), '0 / 0');
    await page.screenshot({path: path.join(screenshotDir, 'legacy-desktop.png'), fullPage: true});

    mode = 'offline';
    await page.waitForFunction(() => document.querySelector('#status').textContent.includes('未连接'));
    assert.deepEqual(await page.locator('[data-reading]').allTextContents(),
      Array(await page.locator('[data-reading]').count()).fill('—'));
    await page.screenshot({path: path.join(screenshotDir, 'offline-desktop.png'), fullPage: true});
    mode = 'slow';
    await page.waitForFunction(() => document.querySelector('#imuAccRaw').textContent === '20 / -30 / 4096');
    await page.waitForTimeout(850);
    assert.equal(peak, 1, 'Slow replies must never overlap');
    await page.evaluate(() => SensorMonitor.stop());
    assert.deepEqual(errors, []);
    console.log(`PASS real browser new/old firmware, offline, mobile layout and sequential polling (${snapshots} requests)`);
    console.log(`Screenshots: ${screenshotDir}`);
  } finally { await browser.close(); }
})().catch(error => {console.error(error); process.exitCode = 1;});
