/* Browser regression: real page and control Worker, mocked aircraft endpoints.
 * Run with Node and Playwright available (POSHOLD_PLAYWRIGHT may name its path).
 */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { chromium } = require(process.env.POSHOLD_PLAYWRIGHT || 'playwright');

(async () => {
  const html = fs.readFileSync(path.join(__dirname, '../components/drivers/general/wifi/web_remote.html'), 'utf8');
  const browser = await chromium.launch({ headless: true,
    ...(process.env.POSHOLD_BROWSER ? { executablePath: process.env.POSHOLD_BROWSER } : {}) });
  try {
    const page = await browser.newPage({ viewport: { width: 844, height: 390 } });
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    let telemetry = { battery: 3.9, relativeHeight: 0.04, flightMode: 0,
      altitudeHolding: false, ridState: 1, posHoldState: 0, posHoldReason: 0, posHoldReady: false };
    // Keep the real control Worker and its timer. Only its aircraft socket is
    // replaced; Playwright's WebSocket routing does not cover Worker sockets.
    await page.addInitScript(() => {
      const blobs = new Map();
      const originalUrl = URL.createObjectURL.bind(URL);
      URL.createObjectURL = blob => { const url = originalUrl(blob); blobs.set(url, blob); return url; };
      const OriginalWorker = Worker;
      window.__packets = [];
      window.Worker = class extends OriginalWorker {
        constructor(url) {
          const prelude = `const testSockets = [];
          class WebSocket {
            static OPEN = 1;
            constructor() { this.readyState=0; this.bufferedAmount=0;
              this.testId = testSockets.push(this);
              setTimeout(() => {this.readyState=1; if(this.onopen)this.onopen();},0); }
            send(data) { postMessage({type:'testpacket', data:data.slice(0), socket:this.testId}); }
            close() { this.readyState=3; if(this.onclose)this.onclose(); }
          }
          addEventListener('message', event => {
            if (event.data.type === 'testclose') {
              for (const socket of testSockets) if (socket.readyState === 1) socket.close();
            }
          });`;
          super(blobs.has(url) ? originalUrl(new Blob([prelude, blobs.get(url)], { type: 'text/javascript' })) : url);
          this.addEventListener('message', event => {
            if (event.data.type !== 'testpacket') return;
            const v = new DataView(event.data.data);
            window.__packets.push({ roll:v.getFloat32(0,true), pitch:v.getFloat32(4,true),
              yaw:v.getFloat32(8,true), throttle:v.getUint16(12,true), flags:v.getUint8(14),
              socket:event.data.socket });
          });
        }
      };
    });
    await page.route('http://drone.test/**', route => {
      const url = new URL(route.request().url());
      if (url.pathname === '/api/telemetry') return route.fulfill({ json: telemetry });
      if (url.pathname === '/api/features') return route.fulfill({ json: { version: 'test', camera: false } });
      return route.fulfill({ contentType: 'text/html; charset=utf-8', body: html });
    });
    await page.goto('http://drone.test/control');
    await page.locator('#connectToggle').click();
    await page.waitForFunction(() => controlConnected);
    assert.equal(await page.locator('#flightActions').isVisible(), false);
    assert.equal(await page.locator('#speedMode').isVisible(), false);
    telemetry = { ...telemetry, flightMode: 2, posHoldReady: true };
    await page.evaluate(() => pollBattery());
    assert.equal(await page.locator('#takeoffButton').isEnabled(), true);
    assert.equal(await page.locator('#speedMode').isVisible(), true);
    assert.equal(await page.locator('#highSpeed').isChecked(), false);
    assert.equal(await page.locator('#highSpeed').getAttribute('role'), 'switch');
    assert.equal(await page.locator('#speedModeText').textContent(), '低速模式');
    await page.locator('#highSpeed').check({ force: true });
    assert.equal(await page.locator('#speedModeText').textContent(), '高速模式');

    // Stored angle trim must never become an unwanted POSHOLD velocity.
    await page.evaluate(() => { rollTrim.value = '4'; pitchTrim.value = '-3'; });
    await page.locator('#takeoffButton').click();
    await page.waitForFunction(() => flightButtons === 2);
    await page.waitForTimeout(150);
    assert(await page.evaluate(() => __packets.some(p => (p.flags & 0x12) === 0x12 && p.throttle === 32767 && p.roll === 0 && p.pitch === 0)),
      'High speed must coexist with takeoff without changing the neutral sticks');
    assert.equal(await page.locator('#takeoffButton').isEnabled(), false);
    telemetry = { ...telemetry, posHoldState: 1, posHoldReady: false };
    await page.evaluate(() => pollBattery());
    assert.equal(await page.evaluate(() => flightButtons), 0);
    assert.match(await page.locator('#altitudeHoldStatus').textContent(), /逐步启动/);
    telemetry = { ...telemetry, posHoldState: 3, relativeHeight: 0.50, ridState: 2 };
    await page.evaluate(() => pollBattery());
    assert.match(await page.locator('#altitudeHoldStatus').textContent(), /定点悬停/);

    const stick = await page.locator('#leftStick').boundingBox();
    assert(stick.height > stick.width * 2, 'Left stick must be a vertical bar');
    assert(stick.width <= 72, 'Left throttle slider should stay narrow');
    const yawLeftBox = await page.locator('#yawLeft').boundingBox();
    const yawRightBox = await page.locator('#yawRight').boundingBox();
    assert(yawLeftBox.x + yawLeftBox.width < stick.x, 'Left turn button must flank the left side of the throttle slider');
    assert(yawRightBox.x > stick.x + stick.width, 'Right turn button must flank the right side of the throttle slider');
    // POSHOLD: 4% stays neutral; 7% commands climb/descent (old deadzone 10%).
    for (const [input, active] of [[.04, false], [.07, true], [-.07, true]]) {
      await page.mouse.move(stick.x + stick.width / 2,
        stick.y + stick.height / 2 - input * (stick.height / 2 - 30));
      await page.mouse.down();
      const throttle = await page.evaluate(() => state.throttle);
      if (!active) assert.equal(throttle, 32767);
      else assert(input > 0 ? throttle > 37767 : throttle < 27767);
      await page.mouse.up();
      assert.equal(await page.evaluate(() => state.throttle), 32767);
    }
    await page.mouse.move(stick.x + stick.width / 2, stick.y + stick.height * .22);
    await page.mouse.down();
    assert((await page.evaluate(() => state.throttle)) > 37767);
    await page.mouse.up();
    assert.equal(await page.evaluate(() => state.throttle), 32767);
    assert.equal(await page.locator('#leftKnob').evaluate(e => e.style.getPropertyValue('--throttle-position')), '0');
    // The full bar reaches both throttle endpoints and lateral movement adds no yaw.
    for (const [offset, throttle] of [[30, 65535], [stick.height - 30, 0]]) {
      await page.mouse.move(stick.x + stick.width * .8, stick.y + offset);
      await page.mouse.down();
      assert.equal(await page.evaluate(() => state.throttle), throttle);
      assert.equal(await page.evaluate(() => state.yaw), 0);
      await page.mouse.up();
    }
    // Switches latch after pointer release, remain exclusive and command 45 deg/s.
    assert.equal(await page.locator('#yawLeft').isDisabled(), true);
    assert.equal(await page.locator('#yawLeft').getAttribute('role'), 'switch');
    assert.equal(await page.locator('#yawRight').getAttribute('role'), 'switch');
    await page.locator('#yawLocked').uncheck({ force: true });
    const assertYawOff = async () => {
      assert.equal(await page.evaluate(() => state.yaw), 0);
      assert.equal(await page.locator('#yawLeft').isChecked(), false);
      assert.equal(await page.locator('#yawRight').isChecked(), false);
    };
    for (const useCarefree of [false, true]) {
      await page.locator('#carefree').setChecked(useCarefree, { force: true });
      for (const [id, sign] of [['yawLeft', -1], ['yawRight', 1]]) {
        await page.locator(`#${id}`).check();
        assert.equal(await page.evaluate(() => state.yaw), sign * 45);
        await page.waitForFunction(yaw => __packets.at(-1)?.yaw === yaw, sign * 45);
        const first = await page.evaluate(() => __packets.length);
        // Longer than 2 s: the switch must keep turning until explicitly closed.
        await page.waitForTimeout(2150);
        const packets = await page.evaluate(first => __packets.slice(first), first);
        assert(packets.length >= 70, 'Latched rotation must continue on the control Worker');
        assert(packets.every(p => p.yaw === sign * 45 && (p.flags & 1) === Number(useCarefree) && (p.flags & 0x10)),
          'Both yaw directions must command 45 deg/s in either heading mode');
        assert.equal(await page.locator(`#${id}`).isChecked(), true);
        await page.locator(`#${id}`).uncheck();
        await assertYawOff();
        await page.waitForFunction(() => __packets.at(-1)?.yaw === 0);
      }
    }
    await page.locator('#yawLeft').check();
    await page.locator('#yawRight').check();
    assert.equal(await page.locator('#yawLeft').isChecked(), false);
    assert.equal(await page.evaluate(() => state.yaw), 45);
    await page.locator('#yawRight').focus();
    await page.keyboard.press('Space');
    await assertYawOff();
    await page.keyboard.press('Space');
    assert.equal(await page.evaluate(() => state.yaw), 45);
    await page.locator('#yawLocked').check({ force: true });
    await assertYawOff();
    assert.equal(await page.locator('#yawRight').isDisabled(), true);
    await page.waitForFunction(() => __packets.at(-1)?.yaw === 0);
    await page.locator('#yawLocked').uncheck({ force: true });
    await page.locator('#yawLeft').check();
    await page.locator('#carefree').check({ force: true });
    await assertYawOff();

    // A short socket outage must not replay a latched turn on its first new packet.
    await page.locator('#yawLeft').check();
    await page.waitForFunction(() => __packets.at(-1)?.yaw === -45);
    const oldSocket = await page.evaluate(() => __packets.at(-1).socket);
    await page.evaluate(() => controlWorker.postMessage({ type: 'testclose' }));
    await page.waitForFunction(id => __packets.some(p => p.socket > id), oldSocket);
    await assertYawOff();
    assert.equal(await page.evaluate(id => __packets.find(p => p.socket > id).yaw, oldSocket), 0,
      'Reconnect must clear cached yaw before sending its first packet');
    assert(await page.evaluate(id => __packets.filter(p => p.socket > id).every(p => p.flags & 0x10), oldSocket),
      'Reconnect must preserve the speed mode shown by the switch');
    assert.equal(await page.locator('#highSpeed').isChecked(), true);
    await page.locator('#yawRight').check();
    await page.evaluate(() => {
      Object.defineProperty(document, 'hidden', { configurable: true, value: true });
      document.dispatchEvent(new Event('visibilitychange'));
      delete document.hidden;
    });
    await assertYawOff();

    const right = await page.locator('#rightStick').boundingBox();
    await page.mouse.move(right.x + right.width * .75, right.y + right.height / 2);
    await page.mouse.down();
    assert((await page.evaluate(() => state.roll)) > 0);
    const heldRoll = await page.evaluate(() => Math.fround(state.roll));
    // Switch the live speed setting without moving or releasing the stick.
    for (const high of [false, true]) {
      await page.evaluate(high => {
        highSpeed.checked = high;
        highSpeed.dispatchEvent(new Event('change'));
      }, high);
      await page.waitForFunction(({high, roll}) => {
        const p = __packets.at(-1);
        return p && Boolean(p.flags & 0x10) === high && p.roll === roll;
      }, {high, roll: heldRoll});
      assert.equal(await page.locator('#speedModeText').textContent(), high ? '高速模式' : '低速模式');
    }
    const packetCount = await page.evaluate(() => __packets.length);
    await page.waitForTimeout(300);
    assert((await page.evaluate(() => __packets.length)) - packetCount >= 10,
      'Held stick should send at approximately 50 Hz without pointer movement');
    await page.mouse.up();
    assert.equal(await page.evaluate(() => state.roll), 0);
    assert.equal(await page.evaluate(() => state.pitch), 0);

    await page.locator('#yawLeft').check();
    await page.locator('#landButton').click();
    await assertYawOff();
    assert.equal(await page.evaluate(() => flightButtons), 4);
    await page.waitForFunction(() => __packets.some(p => (p.flags & 0x14) === 0x14));
    telemetry = { ...telemetry, posHoldState: 5 };
    await page.evaluate(() => pollBattery());
    assert.equal(await page.evaluate(() => flightButtons), 0);
    assert.equal(await page.locator('#landButton').isEnabled(), false);
    await page.locator('#stopButton').click();
    await assertYawOff();
    await page.waitForTimeout(100);
    assert(await page.evaluate(() => __packets.some(p => (p.flags & 0x18) === 0x18)),
      'High speed must not mask the emergency stop action');
    telemetry = { ...telemetry, posHoldState: 6, relativeHeight: 0, ridState: 1 };
    await page.evaluate(() => pollBattery());
    assert.equal(await page.evaluate(() => flightButtons), 0);

    // Retry after a previous stop must survive the old stopped telemetry until
    // a new active phase acknowledges the command.
    telemetry = { ...telemetry, posHoldReady: true };
    await page.evaluate(() => pollBattery());
    await page.locator('#takeoffButton').click();
    await page.evaluate(() => pollBattery());
    assert.equal(await page.evaluate(() => flightButtons), 2);
    telemetry = { ...telemetry, posHoldState: 2, posHoldReady: false };
    await page.evaluate(() => pollBattery());
    assert.equal(await page.evaluate(() => flightButtons), 0);
    telemetry = { ...telemetry, posHoldState: 3, relativeHeight: .5 };
    await page.evaluate(() => pollBattery());

    // Returning from a different flight mode must start in low speed again.
    telemetry = { ...telemetry, flightMode: 1 };
    await page.evaluate(() => pollBattery());
    assert.equal(await page.locator('#speedMode').isVisible(), false);
    assert.equal(await page.locator('#highSpeed').isChecked(), false);
    telemetry = { ...telemetry, flightMode: 2 };
    await page.evaluate(() => pollBattery());
    assert.equal(await page.locator('#speedModeText').textContent(), '低速模式');
    await page.waitForFunction(() => (__packets.at(-1).flags & 0x10) === 0);

    assert.equal(await page.locator('#takeoffButton').textContent(), '一键起飞');
    telemetry = { ...telemetry, posHoldState: 0, posHoldReason: 0, posHoldReady: true,
      relativeHeight: .04, ridState: 1 };
    await page.evaluate(() => pollBattery());
    const output = process.env.POSHOLD_SCREENSHOTS;
    if (output) fs.mkdirSync(output, { recursive: true });
    for (const viewport of [{width:844,height:390}, {width:667,height:375},
                            {width:740,height:320}, {width:932,height:430}]) {
      await page.setViewportSize(viewport);
      const left = await page.locator('#leftStick').boundingBox();
      const right = await page.locator('#rightStick').boundingBox();
      const video = await page.locator('#video').boundingBox();
      assert(left.x + left.width <= video.x && video.x + video.width <= right.x,
        'Video must fit between the sticks');
      assert(Math.abs(video.x + video.width / 2 - viewport.width / 2) < 1,
        'Video must be centered in the screen');
      assert(right.width >= 190 && right.height === right.width, 'Right stick must be enlarged and round');
      for (const id of ['flightActions', 'leftStick', 'rightStick', 'video', 'status', 'yawLeft', 'speedMode']) {
        const box = await page.locator(`#${id}`).boundingBox();
        assert(box.x >= 0 && box.y >= 0 && box.x + box.width <= viewport.width + 1 &&
          box.y + box.height <= viewport.height + 1, `${id} outside viewport ${JSON.stringify(viewport)}`);
      }
      const leftYaw = await page.locator('#yawLeft').boundingBox();
      const throttle = await page.locator('#leftStick').boundingBox();
      const rightYaw = await page.locator('#yawRight').boundingBox();
      assert(leftYaw.x + leftYaw.width <= throttle.x && rightYaw.x >= throttle.x + throttle.width,
        'Turn buttons must remain on either side of the throttle slider');
      assert(leftYaw.x >= 0 && rightYaw.x + rightYaw.width <= viewport.width,
        'Turn buttons must remain inside the viewport');
      for (const id of ['leftStick', 'rightStick']) {
        const a = await page.locator('#flightActions').boundingBox();
        const b = await page.locator(`#${id}`).boundingBox();
        assert(a.x + a.width <= b.x || b.x + b.width <= a.x ||
          a.y + a.height <= b.y || b.y + b.height <= a.y, 'Buttons overlap a stick');
      }
      if (output) await page.screenshot({ path: path.join(output, `poshold-${viewport.width}x${viewport.height}.png`) });
    }
    await page.setViewportSize({ width: 390, height: 844 });
    assert.equal(await page.locator('#landscapeGate').isVisible(), false,
      'Rotation while connected must not cover the emergency stop');
    await page.locator('#yawRight').check();
    await page.locator('#connectToggle').click();
    await assertYawOff();
    assert.equal(await page.locator('#landscapeGate').isVisible(), true);
    if (output) await page.screenshot({ path: path.join(output, 'poshold-portrait.png') });
    await page.setViewportSize({ width: 844, height: 390 });
    await page.evaluate(() => { telemetryAt = performance.now() - 2000; updateFlightButtons(); });
    assert.equal(await page.locator('#takeoffButton').isEnabled(), false);
    assert.deepEqual(errors, []);
    console.log('PASS: real page/Worker packets, default low/live high speed, reconnect speed retention and flight-mode reset, speed flag with takeoff/land/stop/carefree, mode gate, neutral takeoff, status acknowledgement, throttle layout, exclusive yaw switches, continuous +/-45 deg/s with/without carefree, keyboard, lock, hidden-page and reconnect yaw reset, manual sticks, land/stop, retry, stale telemetry, landscape layouts, portrait prompt.');
  } finally {
    await browser.close();
  }
})().catch(error => { console.error(error); process.exitCode = 1; });
