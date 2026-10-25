// Browser regression checks against a local, synthetic direct-HTTP device.
// Requires Playwright; never connects to or commands an actual car.
const assert = require("node:assert/strict");
const http = require("node:http");
const fs = require("node:fs");
const path = require("node:path");
const {chromium, webkit} = require("playwright");

const root = path.resolve(__dirname, "../..");
const assets = path.join(root, "main/web");
const baseline = require("./fixtures/monitor_state.json");
let telemetry, unavailable, rejectConfig, requests;
function reset() {
  telemetry = structuredClone(baseline);
  unavailable = rejectConfig = false;
  requests = [];
}
// Model device responses after configuration changes; production C has separate
// boundary/property tests, and wire-format tests check this field's schema.
function updateMockToeRange() {
  const c = telemetry.config;
  const bounds = c.servos.map((servo, side) => {
    const center = servo.center_us + servo.trim_us;
    return ["left", "right"].map((direction, index) => {
      const span = Math.abs(servo[`${direction}_us`] - center);
      const shaft = c.steering_linkage[side === index ? "inside_deg" : "outside_deg"];
      const allowed = Math.min(span, Math.floor(Math.round(shaft * 10) * 2000 / 1800));
      const pulse = center + Math.sign(servo[`${direction}_us`] - center) * allowed;
      for (let p = 1; p < servo.wheel_points.length; p++) {
        const a = servo.wheel_points[p - 1], b = servo.wheel_points[p];
        if (pulse >= Math.min(a.pwm_us, b.pwm_us) && pulse <= Math.max(a.pwm_us, b.pwm_us)) {
          return Math.floor(Math.abs(a.angle_deg + (pulse - a.pwm_us) / (b.pwm_us - a.pwm_us) * (b.angle_deg - a.angle_deg)) * 10 + 1e-6) / 10;
        }
      }
      return Math.floor(Math.round(servo[`${direction}_angle_deg`] * 10) * allowed / span) / 10;
    });
  });
  c.steering_toe_range = {min_deg: -Math.min(bounds[0][0], bounds[1][1]), max_deg: Math.min(bounds[0][1], bounds[1][0])};
}
const server = http.createServer(async (req, res) => {
  res.setHeader("Cache-Control", "no-store");
  const json = (value, code = 200) => { res.writeHead(code, {"Content-Type": "application/json"}); res.end(JSON.stringify(value)); };
  if (req.url === "/api/state") {
    if (unavailable) return json({message: "Synthetic connection loss"}, 503);
    return json(telemetry);
  }
  if (req.url === "/api/config" && req.method === "GET") return json({ok: true, source: "nvs", config: telemetry.config,
    drive_mode: telemetry.drive_mode});
  if (req.method === "POST") {
    let body = "";
    for await (const chunk of req) body += chunk;
    const command = body ? JSON.parse(body).command : null;
    requests.push({url: req.url, command});
    if (req.url === "/api/config" && rejectConfig) return json({ok: false, message: "Synthetic validation rejection"}, 409);
    const modeCommand = command?.match(/^config drivetrain (awd|fwd|rwd|tank)$/);
    if (req.url === "/api/config" && modeCommand) telemetry.drive_mode = modeCommand[1].toUpperCase();
    const tankCommand = command?.match(/^config tank limit (\d+)$/);
    if (req.url === "/api/config" && tankCommand) telemetry.config.tank_limit_percent = Number(tankCommand[1]);
    const toeCommand = command?.match(/^config steering toe ([+-]?[\d.]+)$/);
    if (req.url === "/api/config" && toeCommand) telemetry.config.steering_toe_deg = Number(toeCommand[1]);
    const trimCommand = command?.match(/^config steering trim (left|right) (-?\d+)$/);
    if (req.url === "/api/config" && trimCommand) {
      telemetry.config.servos[trimCommand[1] === "left" ? 0 : 1].trim_us = Number(trimCommand[2]);
    }
    const linkageCommand = command?.match(/^config steering linkage ([\d.]+) ([\d.]+)$/);
    if (req.url === "/api/config" && linkageCommand) {
      telemetry.config.steering_linkage = {inside_deg: Number(linkageCommand[1]), outside_deg: Number(linkageCommand[2])};
    }
    const servoCommand = command?.match(/^config steering servo (left|right) ([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+)$/);
    if (req.url === "/api/config" && servoCommand) {
      const servo = telemetry.config.servos[servoCommand[1] === "left" ? 0 : 1];
      ["left_us", "center_us", "right_us", "left_angle_deg", "right_angle_deg"].forEach((key, i) => {
        servo[key] = Number(servoCommand[i + 2]);
      });
    }
    const wheelCommand = command?.match(/^config steering wheel (left|right) (.+)$/);
    if (req.url === "/api/config" && wheelCommand) {
      const servo = telemetry.config.servos[wheelCommand[1] === "left" ? 0 : 1];
      if (wheelCommand[2] === "clear") servo.wheel_points = [];
      else {
        const [left, right, ...points] = wheelCommand[2].split(" ");
        servo.wheel_points = points.map(p => { const [pwm, angle] = p.split(":").map(Number); return {pwm_us: pwm, angle_deg: angle}; });
        servo.left_us = servo.wheel_points[0].pwm_us; servo.left_angle_deg = -servo.wheel_points[0].angle_deg;
        servo.right_us = servo.wheel_points.at(-1).pwm_us; servo.right_angle_deg = servo.wheel_points.at(-1).angle_deg;
        telemetry.config.steering_geometry.max_left_deg = Number(left);
        telemetry.config.steering_geometry.max_right_deg = Number(right);
      }
    }
    if (req.url === "/api/config") updateMockToeRange();
    if (req.url === "/api/calibration") {
      if (command === "cal cancel") { telemetry.state = "DISARMED"; telemetry.calibration.active = false; }
      else if (command === "cal capture") {
        if (telemetry.calibration.kind.startsWith("wheel-")) {
          telemetry.calibration.values_us[0] = telemetry.steering[`${telemetry.calibration.kind.slice(6)}_us`];
          telemetry.calibration.wheel_capture_sequence++;
        } else {
          telemetry.calibration.values_us[telemetry.calibration.step] = 1500;
          if (++telemetry.calibration.step >= telemetry.calibration.total_steps) {
            telemetry.calibration.active = false;
            telemetry.state = "DISARMED";
          }
        }
      } else if (command === "cal esc max") telemetry.state = "ESC_CAL_MAX";
      else if (command === "cal esc min") telemetry.state = "ESC_CAL_MIN";
      else {
        const kind = command.startsWith("cal wheel ") ? `wheel-${command.split(" ")[2]}` : command.split(" ")[1];
        telemetry.state = kind.startsWith("wheel-") ? "DISARMED" : kind === "esc" ? "ESC_CAL_ARMED" : "CALIBRATING";
        telemetry.calibration = {active: true, sampling: false, kind, step: 0, total_steps: 3,
          values_us: [0, 0, 0], wheel_capture_sequence: telemetry.calibration.wheel_capture_sequence,
          prompt: "Hold the requested position, then capture.", message: "Synthetic workflow active"};
      }
    }
    if (req.url === "/api/disarm") { telemetry.state = "DISARMED"; telemetry.armed = false; }
    return json({ok: true, message: `Accepted ${command || req.url}`});
  }
  const filename = {"/": "index.html", "/style.css": "style.css", "/app.js": "app.js"}[req.url];
  if (!filename) { res.writeHead(404); return res.end(); }
  res.writeHead(200, {"Content-Type": filename.endsWith(".css") ? "text/css" : filename.endsWith(".js") ? "text/javascript" : "text/html"});
  res.end(fs.readFileSync(path.join(assets, filename)));
});

async function checkNavigationAtScrollEnd(page, label) {
  await page.evaluate(() => window.scrollTo(0, document.documentElement.scrollHeight));
  const reachable = await page.locator(".app-nav a").evaluateAll(nodes => nodes.every(node => {
    const box = node.getBoundingClientRect();
    const hit = document.elementFromPoint(box.x + box.width / 2, box.y + box.height / 2);
    return box.y >= 0 && box.bottom <= innerHeight && (hit === node || node.contains(hit));
  }));
  assert.equal(reachable, true, `${label}: navigation must remain reachable after scrolling`);
}

async function checkDesktop(browser, url, name, errors) {
  const context = await browser.newContext({viewport: {width: 1280, height: 720}, isMobile: false, hasTouch: false});
  try {
    const page = await context.newPage();
    page.on("pageerror", error => errors.push(error.message));
    page.setDefaultTimeout(7000);
    await page.goto(url);
    await page.locator("#connection.online").waitFor();
    for (const viewport of [
      {width: 640, height: 720}, {width: 1024, height: 768}, {width: 1280, height: 720},
      {width: 1440, height: 900}, {width: 1920, height: 1080}, {width: 2560, height: 1440}
    ]) {
      await page.setViewportSize(viewport);
      for (const view of ["live", "calibrate", "settings"]) {
        await page.locator(`[data-view="${view}"]`).click();
        await page.locator(`[data-page="${view}"]`).waitFor({state: "visible"});
        await page.evaluate(() => window.scrollTo(0, 0));
        assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false,
          `${name}: desktop overflow at ${viewport.width} in ${view}`);
        if (view === "live" && viewport.width >= 1024) {
          for (const selector of [".receiver-panel", ".steering-panel", ".esc-panel", ".rpm-panel"]) {
            const box = await page.locator(selector).boundingBox();
            assert.ok(box.y >= 0 && box.y + box.height <= viewport.height,
              `${name}: ${selector} must fit on desktop at ${viewport.width}`);
          }
        }
      }
      await checkNavigationAtScrollEnd(page, `${name}: desktop ${viewport.width}`);
    }
    await page.setViewportSize({width: 1280, height: 720});
    await checkNavigationAtScrollEnd(page, `${name}: desktop keyboard`);
    const drivetrain = page.locator('[data-builder="drivetrain"] select');
    await drivetrain.focus();
    // macOS WebKit uses Option-Tab to include buttons in keyboard navigation.
    await page.keyboard.press(name === "webkit" ? "Alt+Tab" : "Tab");
    assert.equal(await page.locator('[data-builder="drivetrain"] button').first().evaluate(node => node === document.activeElement), true,
      `${name}: keyboard reaches Save after the drivetrain field`);
    const focused = await page.locator(":focus").boundingBox();
    const header = await page.locator(".app-header").boundingBox();
    assert.ok(focused.y >= header.y + header.height && focused.y + focused.height <= 720,
      `${name}: keyboard focus must not be covered by the sticky header`);
    if (process.env.MONITOR_SCREENSHOTS && name === "webkit") {
      const dir = process.env.MONITOR_SCREENSHOTS;
      fs.mkdirSync(dir, {recursive: true});
      for (const [view, file, fullPage] of [
        ["live", "monitor-desktop.png", false], ["settings", "monitor-settings.png", true],
        ["calibrate", "monitor-calibration.png", false]
      ]) {
        await page.locator(`[data-view="${view}"]`).click();
        await page.locator(`[data-page="${view}"]`).waitFor({state: "visible"});
        await page.evaluate(() => window.scrollTo(0, 0));
        await page.mouse.move(1270, 710);
        await page.screenshot({path: path.join(dir, file), fullPage, animations: "disabled"});
      }
      await page.setViewportSize({width: 1440, height: 1000});
      await page.locator('[data-view="settings"]').click();
      await page.locator("#ackermann-form").scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-ackermann.png"), animations: "disabled"});
      await page.setViewportSize({width: 1920, height: 1080});
      await page.locator('[data-view="live"]').click();
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.screenshot({path: path.join(dir, "monitor-desktop-wide.png"), animations: "disabled"});
    }
  } finally { await context.close(); }
}

async function run(engine, launchOptions, name) {
  reset();
  const browser = await engine.launch({headless: true, ...launchOptions});
  try {
    const context = await browser.newContext({viewport: {width: 390, height: 844}, isMobile: true, hasTouch: true, deviceScaleFactor: 2});
    const page = await context.newPage();
    const errors = [];
    page.on("pageerror", error => errors.push(error.message));
    let accept = true;
    page.on("dialog", dialog => accept ? dialog.accept() : dialog.dismiss());
    page.setDefaultTimeout(7000);
    const url = `http://127.0.0.1:${server.address().port}`;
    await checkDesktop(browser, url, name, errors);
    const navigate = async view => {
      await page.locator(`[data-view="${view}"]`).click();
      await page.locator(`[data-page="${view}"]`).waitFor({state: "visible"});
    };
    const enabled = id => page.waitForFunction(id => !document.getElementById(id).disabled, id);
    const submit = async (locator, endpoint, command) => {
      const response = page.waitForResponse(r => r.url().endsWith(endpoint) && r.request().method() === "POST");
      await locator.click();
      await response;
      assert.deepEqual(requests.at(-1), {url: endpoint, command});
    };
    const reloadConfig = async () => {
      const response = page.waitForResponse(r => r.url().endsWith("/api/config") && r.request().method() === "GET");
      await page.locator("#reload-config").click();
      await response;
      await page.waitForFunction(() => document.getElementById("message").textContent === "Loaded saved NVS config.");
    };
    await page.goto(url);
    await page.locator("#connection.online").waitFor();
    assert.equal(await page.locator(".app-nav a").count(), 3);
    assert.equal(await page.locator("#receiver-grid .valid").count(), 3);
    assert.equal(await page.locator("#rpm-body tr").count(), 2);
    assert.equal(await page.locator("#output-body tr").count(), 4);
    assert.equal(await page.locator("#speed").textContent(), "24.3");

    assert.equal(await page.locator("#servo-left-us").textContent(), "1500 µs");
    assert.equal(await page.locator("#servo-right-us").textContent(), "1500 µs");
    assert.equal(await page.locator("#servo-left-angle").textContent(), "0.0");
    assert.equal(await page.locator("#servo-right-angle").textContent(), "0.0");
    assert.equal(await page.locator("#ackermann-live").textContent(), "25.0%");
    telemetry.steering.left_angle_deg = -12.5;
    telemetry.steering.right_angle_deg = -10;
    await page.waitForFunction(() => document.getElementById("servo-left-angle").textContent === "-12.5");
    assert.equal(await page.locator("#servo-right-angle").textContent(), "-10.0");
    telemetry.steering.left_angle_deg = telemetry.steering.right_angle_deg = 0;
    // Keep the engineering readings together without shrinking touch targets.
    for (const viewport of [{width: 1280, height: 720}, {width: 1024, height: 768}, {width: 390, height: 844}]) {
      await page.setViewportSize(viewport);
      const selectors = viewport.width >= 1024
        ? [".receiver-panel", ".steering-panel", ".esc-panel", ".rpm-panel"]
        : [".receiver-panel", ".steering-panel"];
      for (const selector of selectors) {
        const box = await page.locator(selector).boundingBox();
        assert.ok(box.y >= 0 && box.y + box.height <= viewport.height,
          `${name}: ${selector} must fit the first screen at ${viewport.width}x${viewport.height}`);
      }
    }
    // Touch layouts cover portrait, landscape and a reduced-height viewport.
    for (const viewport of [
      {width: 320, height: 568}, {width: 375, height: 667}, {width: 390, height: 844},
      {width: 430, height: 932}, {width: 390, height: 400}, {width: 568, height: 320},
      {width: 667, height: 375}, {width: 844, height: 390}, {width: 932, height: 430},
      {width: 620, height: 844}, {width: 768, height: 1024}, {width: 1024, height: 768}
    ]) {
      const {width, height} = viewport;
      await page.setViewportSize(viewport);
      for (const view of ["live", "calibrate", "settings"]) {
        await navigate(view);
        const overflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
        assert.equal(overflow, false, `${name}: overflow at ${width} in ${view}`);
        const undersized = await page.locator("[data-page]:not([hidden]) button, [data-page]:not([hidden]) input, [data-page]:not([hidden]) select, [data-page]:not([hidden]) summary, [data-page]:not([hidden]) a, .app-nav a, .brand").evaluateAll(nodes =>
          nodes.filter(node => node.getBoundingClientRect().height < 44 || node.getBoundingClientRect().width < 44).map(node => node.outerHTML));
        assert.deepEqual(undersized, [], `${name}: small touch targets at ${width}x${height}`);
        const smallInputs = await page.locator('[data-page]:not([hidden]) input, [data-page]:not([hidden]) select').evaluateAll(nodes =>
          nodes.filter(node => parseFloat(getComputedStyle(node).fontSize) < 16).map(node => node.outerHTML));
        assert.deepEqual(smallInputs, [], `${name}: touch input text below 16px at ${width}x${height}`);
      }
      await checkNavigationAtScrollEnd(page, `${name}: touch ${width}x${height}`);
    }
    await page.setViewportSize({width: 390, height: 844});
    await navigate("settings");
    const draft = page.locator('form[data-builder="smoothing"] input');
    const geometryDraft = page.locator('[data-builder="geometry"] [name="wheelbase"]');
    const inputDraft = page.locator('[data-builder="steering-input"] [name="deadband"]');
    const trimDraft = page.locator('[data-builder="trim-left"] [name="value"]');
    const linkageDraft = page.locator('[data-builder="linkage"] [name="outside"]');
    const toeDraft = page.locator('[data-builder="toe"] input');
    const tankDraft = page.locator('[data-builder="tank-limit"] input');
    await draft.fill("12");
    await geometryDraft.fill("445.5");
    await inputDraft.fill("31");
    await trimDraft.fill("25");
    await linkageDraft.fill("66.6");
    await toeDraft.fill("-1.5"); await tankDraft.fill("37");
    await page.setViewportSize({width: 844, height: 390});
    await page.waitForTimeout(400);
    await navigate("live");
    await navigate("settings");
    await page.setViewportSize({width: 390, height: 844});
    assert.equal(await draft.inputValue(), "12");
    assert.equal(await geometryDraft.inputValue(), "445.5", "Rotation and polling preserve geometry drafts");
    assert.equal(await inputDraft.inputValue(), "31", "Rotation and polling preserve CH1 drafts");
    assert.equal(await trimDraft.inputValue(), "25", "Rotation and polling preserve trim drafts");
    assert.equal(await linkageDraft.inputValue(), "66.6", "Rotation and polling preserve linkage drafts");
    assert.equal(await toeDraft.inputValue(), "-1.5", "Rotation and polling preserve toe drafts");
    assert.equal(await tankDraft.inputValue(), "37", "Rotation and polling preserve tank limit drafts");
    assert.equal(await page.locator("#trim-left-neutral").textContent(), "Neutral preview: 1525 µs");
    await reloadConfig();
    await page.waitForFunction(() => document.querySelector('[data-builder="smoothing"] input').value === "10");
    assert.equal(await geometryDraft.inputValue(), "445");
    assert.equal(await inputDraft.inputValue(), "30");
    assert.equal(await trimDraft.inputValue(), "0");
    assert.equal(await linkageDraft.inputValue(), "68");
    assert.equal(await toeDraft.inputValue(), "0"); assert.equal(await tankDraft.inputValue(), "25");
    assert.equal(await page.locator('[data-builder="linkage"] [name="inside"]').inputValue(), "90");
    for (const side of ["left", "right"]) {
      const form = page.locator(`[data-builder="servo-${side}"]`);
      assert.equal(await page.locator(`#servo-${side}-direction`).textContent(), "Endpoint direction matches your mounting: wheel left CCW, wheel right CW.");
      for (const field of ["left", "center", "right"]) {
        assert.equal(await form.locator(`[name="${field}"]`).getAttribute("min"), "500");
        assert.equal(await form.locator(`[name="${field}"]`).getAttribute("max"), "2500");
      }
      await form.locator('[name="left"]').fill("1000");
      await form.locator('[name="right"]').fill("2000");
      assert.equal(await page.locator(`#servo-${side}-direction`).evaluate(node => node.classList.contains("warning")), true);
      assert.match(await page.locator(`#servo-${side}-direction`).textContent(), /reversed for your mounting/);
      await reloadConfig();
    }
    // Wider servo support must not widen receiver calibration or confuse shaft and wheel angles.
    assert.equal(await page.locator('[data-builder="steering-input"] [name="left"]').getAttribute("min"), "800");
    assert.equal(await page.locator('[data-builder="steering-input"] [name="left"]').getAttribute("max"), "2200");
    assert.equal(await page.locator('[data-builder="servo-left"] [name="left_angle"]').getAttribute("max"), "85");
    const servoReference = page.locator('[aria-labelledby="servo-settings-heading"] details');
    await servoReference.locator("summary").click();
    for (const width of [320, 390, 568]) {
      await page.setViewportSize({width, height: 844});
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false, "Expanded servo specs fit phone screens");
    }
    assert.match(await servoReference.textContent(), /50–330 Hz/);
    assert.match(await servoReference.textContent(), /12.4 A/);
    await servoReference.locator("summary").click();
    await page.setViewportSize({width: 390, height: 844});

    const commands = {
      drivetrain: "config drivetrain awd", "arm-latch": "config arm-latch on", reverse: "config reverse 100",
      "tank-limit": "config tank limit 25", toe: "config steering toe 0",
      "drive-smoothing": "config drive smoothing 100", failsafe: "config failsafe 1565 10",
      smoothing: "config steering smoothing 10",
      "servo-left": "config steering servo left 2000 1500 1000 30 30",
      "servo-right": "config steering servo right 2000 1500 1000 30 30",
      "trim-left": "config steering trim left 0", "trim-right": "config steering trim right 0",
      linkage: "config steering linkage 90 68",
      geometry: "config steering geometry 445 320 107 30 30",
      "steering-input": "config steering input 2050 1518 987 30",
      ackermann: "config steering ackermann-strength 0:50 50:0",
      ppr: "config rpm ppr 7", poles: "config rpm poles 14"
    };
    assert.equal(await page.locator("form[data-builder]").count(), Object.keys(commands).length);
    for (const [builder, command] of Object.entries(commands)) {
      await submit(page.locator(`form[data-builder="${builder}"] button:not([type="button"])`).first(), "/api/config", command);
    }
    await page.waitForTimeout(400);
    for (const [builder, input, invalidValues, valid, command, key] of [
      ["toe", toeDraft, ["30.1", "-30.1", "0.01", ""], "-20.5", "config steering toe -20.5", "steering_toe_deg"],
      ["tank-limit", tankDraft, ["101", "-1", "12.5", ""], "37", "config tank limit 37", "tank_limit_percent"]
    ]) {
      const button = page.locator(`[data-builder="${builder}"] button`);
      const beforeInvalid = requests.length;
      for (const invalid of invalidValues) {
        await input.fill(invalid); await button.click();
        assert.equal(requests.length, beforeInvalid, `${builder}: reject '${invalid}' before submission`);
      }
      await input.fill(valid);
      await submit(button, "/api/config", command); await reloadConfig();
      assert.equal(await input.inputValue(), valid); assert.equal(telemetry.config[key], Number(valid));
      rejectConfig = true;
      await input.fill("2");
      await submit(button, "/api/config", builder === "toe" ? "config steering toe 2" : "config tank limit 2");
      await page.locator("#message.error").waitFor();
      assert.equal(telemetry.config[key], Number(valid)); assert.equal(await input.inputValue(), "2");
      rejectConfig = false; await reloadConfig();
      assert.equal(await input.inputValue(), valid);
    }
    await toeDraft.fill("30");
    await submit(page.locator('[data-builder="toe"] button'), "/api/config", "config steering toe 30");
    await reloadConfig(); assert.equal(await toeDraft.inputValue(), "30");
    await toeDraft.fill("0");
    await submit(page.locator('[data-builder="toe"] button'), "/api/config", "config steering toe 0");
    await reloadConfig();
    // Device-derived bounds change with calibration, linkage travel and trims.
    assert.equal(await toeDraft.getAttribute("min"), "-30");
    assert.equal(await toeDraft.getAttribute("max"), "30");
    assert.equal(await toeDraft.getAttribute("step"), "0.1");
    const beforeToeRange = structuredClone(telemetry.config);
    for (const side of ["left", "right"]) {
      const servoForm = page.locator(`[data-builder="servo-${side}"]`);
      await servoForm.locator('[name="left"]').fill("2500");
      await servoForm.locator('[name="right"]').fill("500");
      await submit(servoForm.locator("button"), "/api/config", `config steering servo ${side} 2500 1500 500 30 30`);
      await reloadConfig();
    }
    assert.equal(await toeDraft.getAttribute("min"), "-30");
    assert.equal(await toeDraft.getAttribute("max"), "22.6");
    assert.match(await page.locator("#toe-range").textContent(), /-30.0° to \+22.6°/);
    await page.waitForTimeout(400);
    await toeDraft.fill("20.5");
    // Simulate a second client's saved linkage change. Polling must update the
    // bounds and preserve the first client's now-invalid draft for correction.
    telemetry.config.steering_linkage.outside_deg = 45; updateMockToeRange();
    await page.waitForFunction(() => document.querySelector('[data-builder="toe"] input').max === "15");
    assert.equal(await toeDraft.inputValue(), "20.5");
    const beforeToeInvalid = requests.length;
    await page.locator('[data-builder="toe"] button').click();
    assert.equal(requests.length, beforeToeInvalid);
    await toeDraft.fill("15");
    await submit(page.locator('[data-builder="toe"] button'), "/api/config", "config steering toe 15");
    await reloadConfig(); assert.equal(await toeDraft.inputValue(), "15");
    telemetry.config.steering_linkage.inside_deg = 0; updateMockToeRange();
    await page.waitForFunction(() => document.querySelector('[data-builder="toe"] input').min === "0");
    assert.equal(await toeDraft.getAttribute("max"), "15");
    assert.equal(await toeDraft.evaluate(node => node.checkValidity()), true);
    telemetry.config.steering_toe_deg = -30;
    telemetry.config.steering_linkage = {inside_deg: 90, outside_deg: 0}; updateMockToeRange();
    await reloadConfig();
    assert.equal(await toeDraft.getAttribute("min"), "-30");
    assert.equal(await toeDraft.getAttribute("max"), "0");
    await submit(page.locator('[data-builder="toe"] button'), "/api/config", "config steering toe -30");
    telemetry.config.steering_toe_deg = 0;
    telemetry.config.steering_linkage = {inside_deg: 90, outside_deg: 68};
    telemetry.config.servos[0].trim_us = 200; updateMockToeRange();
    await reloadConfig();
    assert.equal(await toeDraft.getAttribute("max"), "18.8");
    telemetry.config = beforeToeRange;
    await reloadConfig();
    const driveSelect = page.locator('[data-builder="drivetrain"] select');
    await driveSelect.selectOption("tank");
    await submit(page.locator('[data-builder="drivetrain"] button').first(), "/api/config", "config drivetrain tank");
    await reloadConfig(); assert.equal(await driveSelect.inputValue(), "tank");
    await navigate("live");
    await page.waitForFunction(() => document.getElementById("drive-mode").textContent === "TANK");
    assert.equal(await page.locator("#speed-label").textContent(), "Wheel surface speed");
    assert.match(await page.locator("#speed-state").textContent(), /not vehicle speed/);
    assert.equal(await page.locator("#ackermann-live").textContent(), "Bypassed · tank");
    assert.match(await page.locator("#steering-target-note").textContent(), /hold toe/);
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
    await navigate("settings");
    await submit(page.locator("#cycle-drivetrain"), "/api/config", "config drivetrain awd");
    await reloadConfig();
    const linkageForm = page.locator('[data-builder="linkage"]');
    const beforeInvalidLinkage = requests.length;
    for (const field of ["inside", "outside"]) {
      const input = linkageForm.locator(`[name="${field}"]`);
      assert.equal(await input.getAttribute("min"), "0");
      assert.equal(await input.getAttribute("max"), "90");
      assert.equal(await input.getAttribute("step"), "0.1");
      for (const invalid of ["90.1", "-0.1", "68.01", ""]) {
        await input.fill(invalid);
        await linkageForm.locator("button").click();
        assert.equal(requests.length, beforeInvalidLinkage, `${field} linkage travel '${invalid}' cannot be submitted`);
      }
      await input.fill(field === "inside" ? "85.5" : "66.6");
    }
    await submit(linkageForm.locator("button"), "/api/config", "config steering linkage 85.5 66.6");
    await reloadConfig();
    assert.equal(await linkageForm.locator('[name="inside"]').inputValue(), "85.5");
    assert.equal(await linkageDraft.inputValue(), "66.6");
    assert.deepEqual(telemetry.config.steering_linkage, {inside_deg: 85.5, outside_deg: 66.6});
    rejectConfig = true;
    await linkageDraft.fill("40");
    await submit(linkageForm.locator("button"), "/api/config", "config steering linkage 85.5 40");
    await page.locator("#message.error").waitFor();
    assert.equal(telemetry.config.steering_linkage.outside_deg, 66.6);
    assert.equal(await linkageDraft.inputValue(), "40");
    rejectConfig = false;
    await reloadConfig();
    assert.equal(await linkageDraft.inputValue(), "66.6");
    await linkageForm.locator('[name="inside"]').fill("0");
    await linkageDraft.fill("90");
    await submit(linkageForm.locator("button"), "/api/config", "config steering linkage 0 90");
    await reloadConfig();
    assert.equal(await linkageForm.locator('[name="inside"]').inputValue(), "0");
    assert.equal(await linkageDraft.inputValue(), "90");
    // Signed trims use the saved center, preserve endpoints, and survive reload.
    for (const [side, value, neutral] of [["left", "25", 1525], ["right", "-35", 1465]]) {
      const form = page.locator(`[data-builder="trim-${side}"]`);
      const input = form.locator("input");
      assert.equal(await input.getAttribute("min"), "-450");
      assert.equal(await input.getAttribute("max"), "450");
      const beforeInvalidTrim = requests.length;
      for (const invalid of ["451", "-451", "1.5", ""]) {
        await input.fill(invalid);
        await form.locator("button").click();
        assert.equal(requests.length, beforeInvalidTrim, `${side} trim '${invalid}' outside endpoint clearance or integer precision cannot be submitted`);
      }
      await input.fill(value);
      assert.equal(await page.locator(`#trim-${side}-neutral`).textContent(), `Neutral preview: ${neutral} µs`);
      await submit(form.locator("button"), "/api/config", `config steering trim ${side} ${value}`);
      await reloadConfig();
      await page.waitForFunction(([side, value]) => document.querySelector(`[data-builder="trim-${side}"] input`).value === value, [side, value]);
    }
    assert.deepEqual(telemetry.config.servos.map(s => [s.left_us, s.center_us, s.right_us]), [[2000,1500,1000],[2000,1500,1000]]);
    // A rejected save leaves the active configuration intact and the draft editable.
    rejectConfig = true;
    await trimDraft.fill("40");
    await submit(page.locator('[data-builder="trim-left"] button'), "/api/config", "config steering trim left 40");
    await page.locator("#message.error").waitFor();
    assert.equal(telemetry.config.servos[0].trim_us, 25);
    assert.equal(await trimDraft.inputValue(), "40");
    rejectConfig = false;
    await reloadConfig();
    await page.waitForFunction(() => document.querySelector('[data-builder="trim-left"] input').value === "25");
    const leftServo = page.locator('[data-builder="servo-left"]');
    await leftServo.locator('[name="left"]').fill("2500");
    await leftServo.locator('[name="right"]').fill("500");
    const beforeInvalidEndpoint = requests.length;
    for (const [field, value] of [["left", "2501"], ["right", "499"]]) {
      const input = leftServo.locator(`[name="${field}"]`);
      await input.fill(value);
      await leftServo.locator("button").click();
      assert.equal(requests.length, beforeInvalidEndpoint, "Servo commands outside the rated PWM range cannot be submitted");
      await input.fill(field === "left" ? "2500" : "500");
    }
    await submit(leftServo.locator("button"), "/api/config", "config steering servo left 2500 1500 500 30 30");
    await reloadConfig();
    assert.equal(await trimDraft.getAttribute("min"), "-950");
    assert.equal(await trimDraft.getAttribute("max"), "950");
    await trimDraft.fill("900");
    await submit(page.locator('[data-builder="trim-left"] button'), "/api/config", "config steering trim left 900");
    await reloadConfig();
    assert.equal(await page.locator("#trim-left-neutral").textContent(), "Neutral preview: 2400 µs");
    const pointPercent = page.locator('#ackermann-points [data-point="percent"]').first();
    await pointPercent.fill("65.5");
    await page.waitForTimeout(600);
    await navigate("live"); await navigate("settings");
    assert.equal(await pointPercent.inputValue(), "65.5", "Polling/navigation preserve curve drafts");
    await page.locator("#ackermann-add").click();
    assert.equal(await page.locator("#ackermann-points tr").count(), 3);
    const speedPoint = page.locator('#ackermann-points [data-point="speed"]').nth(2);
    await speedPoint.fill("40"); // deliberately unordered
    const beforeInvalid = requests.length;
    await page.locator('#ackermann-form button[type="submit"]').click();
    assert.equal(requests.length, beforeInvalid, "Invalid curve cannot be submitted");
    await speedPoint.fill("70");
    const strengthPoint = page.locator('#ackermann-points [data-point="percent"]').nth(2);
    for (const invalidStrength of ["-10", "100.1"]) {
      await strengthPoint.fill(invalidStrength);
      await page.locator('#ackermann-form button[type="submit"]').click();
      assert.equal(requests.length, beforeInvalid, "Ackermann strength outside 0–100 cannot be submitted");
    }
    await strengthPoint.fill("10");
    assert.equal(await page.locator("#ackermann-preview circle").count(), 3);
    await submit(page.locator('#ackermann-form button[type="submit"]'), "/api/config", "config steering ackermann-strength 0:65.5 50:0 70:10");
    await page.waitForTimeout(400);
    telemetry.steering.speed_valid = false; telemetry.steering.speed_held = true;
    await page.waitForFunction(() => document.getElementById("speed-state").textContent.includes("HELD"));
    assert.equal(await page.locator("#speed-state").evaluate(node => getComputedStyle(node).color), "rgb(135, 85, 18)");
    telemetry.steering.speed_valid = true; telemetry.steering.speed_held = false;
    await submit(page.locator("#cycle-drivetrain"), "/api/config", "config drivetrain fwd");
    await submit(page.locator('[data-command="config failsafe off"]'), "/api/config", "config failsafe off");
    rejectConfig = true;
    await page.locator('form[data-builder="smoothing"] button').click();
    await page.locator("#message.error").waitFor();
    rejectConfig = false;

    telemetry.state = "DRIVE_ARMED"; telemetry.armed = true;
    await enabled("disarm-for-config");
    assert.equal(await page.locator('form button:not([type="button"])').evaluateAll(nodes => nodes.every(node => node.disabled)), true,
      "Every configuration form is locked while armed");
    accept = false;
    const before = requests.length;
    await page.locator("#disarm-for-config").click();
    assert.equal(requests.length, before);
    accept = true;
    await submit(page.locator("#disarm-for-config"), "/api/disarm", null);
    await page.waitForFunction(() => document.getElementById("global-state").textContent === "DISARMED");

    await navigate("calibrate");
    for (const kind of ["receiver", "steering", "arm"]) {
      const start = page.locator(`[data-cal-command="cal ${kind}"]`);
      await submit(start, "/api/calibration", `cal ${kind}`);
      await enabled("cal-capture");
      assert.equal(await page.locator("#cal-nav-indicator").isVisible(), true);
      await navigate("settings");
      assert.equal(await page.locator('form button:not([type="button"])').evaluateAll(nodes => nodes.every(node => node.disabled)), true,
        "Every configuration form is locked during calibration");
      await navigate("calibrate");
      for (let step = 0; step < 3; step++) {
        await enabled("cal-capture");
        await submit(page.locator("#cal-capture"), "/api/calibration", "cal capture");
        if (step < 2) await page.waitForFunction(step => document.getElementById("cal-step").textContent.startsWith(`${step + 2}/`), step);
      }
      await page.waitForFunction(() => document.getElementById("cal-capture").disabled);
    }
    await submit(page.locator("#cal-esc-arm"), "/api/calibration", "cal esc arm");
    await enabled("cal-esc-max");
    assert.equal(await page.locator("#cal-esc-min").isDisabled(), true);
    await submit(page.locator("#cal-esc-max"), "/api/calibration", "cal esc max");
    await enabled("cal-esc-min");
    assert.equal(await page.locator("#state-detail").textContent(), "ESC CAL MAX");
    await submit(page.locator("#cal-esc-min"), "/api/calibration", "cal esc min");
    await enabled("cal-cancel");
    await submit(page.locator("#cal-cancel"), "/api/calibration", "cal cancel");
    await enabled("cal-manual");
    await submit(page.locator("#cal-manual"), "/api/calibration", "cal manual");
    await enabled("cal-cancel");
    await submit(page.locator("#cal-cancel"), "/api/calibration", "cal cancel");

    // Guided wheel-angle measurement captures PWM while angles remain user input.
    const beforeWheelWorkflow = structuredClone(telemetry);
    telemetry = structuredClone(baseline);
    await navigate("calibrate");
    await enabled("wheel-cal-start");
    assert.match(await page.locator("#wheel-cal-status").textContent(), /Left: uncalibrated.*Right: uncalibrated/);
    await submit(page.locator("#wheel-cal-start"), "/api/calibration", "cal wheel left");
    await enabled("wheel-cal-capture");
    assert.equal(await page.locator("#wheel-cal-save").isDisabled(), true);
    assert.equal(await page.locator("#wheel-cal-side").isDisabled(), true);
    for (const [pwm, angle] of [[1500, 0], [2000, -25], [1750, -10.5], [1000, 28]]) {
      telemetry.steering.left_us = pwm;
      const count = await page.locator("#wheel-cal-points tr").count();
      await submit(page.locator("#wheel-cal-capture"), "/api/calibration", "cal capture");
      await page.waitForFunction(count => document.querySelectorAll("#wheel-cal-points tr").length === count + 1, count);
      await page.locator('#wheel-cal-points [data-wheel-point="angle"]').last().fill(String(angle));
    }
    const wheelDraft = await page.locator("#wheel-cal-points input").evaluateAll(inputs => inputs.map(n => n.value));
    await navigate("live");
    assert.equal(await page.locator("#servo-left-angle").textContent(), "--");
    assert.equal(await page.locator("#ackermann-live").textContent(), "Bypassed · measuring");
    await navigate("calibrate"); await page.waitForTimeout(400);
    assert.deepEqual(await page.locator("#wheel-cal-points input").evaluateAll(inputs => inputs.map(n => n.value)), wheelDraft);
    await submit(page.locator("#wheel-cal-capture"), "/api/calibration", "cal capture");
    await page.waitForFunction(() => document.getElementById("wheel-cal-message").textContent.includes("already in the draft"));
    assert.equal(await page.locator("#wheel-cal-points tr").count(), 4);
    await submit(page.locator("#wheel-cal-finish"), "/api/calibration", "cal cancel");
    await enabled("wheel-cal-save");
    assert.match(await page.locator("#wheel-cal-preview").textContent(), /25° left \/ 28° right/);
    const wheelCommand = "config steering wheel left 25 28 2000:-25 1750:-10.5 1500:0 1000:28";
    rejectConfig = true;
    await submit(page.locator("#wheel-cal-save"), "/api/config", wheelCommand);
    assert.equal(telemetry.config.servos[0].wheel_points.length, 0);
    assert.match(await page.locator("#wheel-cal-message").textContent(), /Draft retained/);
    rejectConfig = false;
    await submit(page.locator("#wheel-cal-save"), "/api/config", wheelCommand);
    await page.waitForFunction(() => document.getElementById("wheel-cal-status").textContent.includes("Left: 4 measured points saved"));
    assert.equal(telemetry.config.steering_geometry.max_left_deg, 25);
    assert.deepEqual(telemetry.config.servos[0].wheel_points.map(p => p.angle_deg), [-25, -10.5, 0, 28]);
    // Nonmonotonic / missing-neutral drafts must not leave the browser.
    const angleInputs = page.locator('#wheel-cal-points [data-wheel-point="angle"]');
    const beforeInvalidWheel = requests.length;
    await angleInputs.first().fill("1"); await page.locator("#wheel-cal-save").click();
    assert.equal(requests.length, beforeInvalidWheel); assert.match(await page.locator("#wheel-cal-message").textContent(), /straight ahead/);
    await angleInputs.first().fill("0");
    await page.locator('#wheel-cal-points [data-wheel-point="pwm"]').nth(2).fill("2100");
    await page.locator("#wheel-cal-save").click(); assert.equal(requests.length, beforeInvalidWheel);
    await page.locator("#wheel-cal-load").click();
    assert.deepEqual(await page.locator('#wheel-cal-points [data-wheel-point="pwm"]').evaluateAll(inputs => inputs.map(n => n.value)), ["2000", "1750", "1500", "1000"]);
    await page.locator("#wheel-cal-side").selectOption("right");
    assert.equal(await page.locator("#wheel-cal-points tr").count(), 0);
    for (const [pwm, angle] of [[2000, -30], [1500, 0], [1000, 30]]) {
      await page.locator("#wheel-cal-add").click();
      await page.locator('#wheel-cal-points [data-wheel-point="pwm"]').last().fill(String(pwm));
      await page.locator('#wheel-cal-points [data-wheel-point="angle"]').last().fill(String(angle));
    }
    await submit(page.locator("#wheel-cal-save"), "/api/config", "config steering wheel right 25 28 2000:-30 1500:0 1000:30");
    await page.waitForFunction(() => document.getElementById("wheel-cal-status").textContent.includes("Right: 3 measured points saved"));
    await page.locator("#wheel-cal-side").selectOption("left");
    assert.equal(await page.locator("#wheel-cal-points tr").count(), 4);
    telemetry.state = "DRIVE_ARMED"; telemetry.armed = true;
    await page.waitForFunction(() => document.getElementById("wheel-cal-save").disabled);
    assert.equal(await page.locator("#wheel-cal-clear").isDisabled(), true);
    telemetry.state = "DISARMED"; telemetry.armed = false;
    await enabled("wheel-cal-save");
    unavailable = true; await page.locator("#connection.offline").waitFor();
    assert.equal(await page.locator("#wheel-cal-save").isDisabled(), true);
    assert.equal(await page.locator("#wheel-cal-start").isDisabled(), true);
    unavailable = false; await page.locator("#connection.online").waitFor();
    assert.equal(await page.locator("#wheel-cal-points tr").count(), 4);
    for (const width of [320, 390, 568]) {
      await page.setViewportSize({width, height: 844});
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false, "Wheel measurement form fits phones");
    }
    if (process.env.MONITOR_SCREENSHOTS && name === "webkit") {
      await page.setViewportSize({width: 390, height: 844});
      await page.locator("#wheel-cal-form").scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(process.env.MONITOR_SCREENSHOTS, "monitor-wheel-calibration-phone.png"), animations: "disabled"});
    }
    await submit(page.locator("#wheel-cal-clear"), "/api/config", "config steering wheel left clear");
    await page.waitForFunction(() => document.getElementById("wheel-cal-status").textContent.includes("Left: uncalibrated"));
    assert.equal(await page.locator("#wheel-cal-points tr").count(), 4);
    // A page reload retrieves saved measurements; drafts never masquerade as saved.
    await page.reload(); await page.locator("#connection.online").waitFor();
    await page.locator("#wheel-cal-side").selectOption("right");
    assert.equal(await page.locator("#wheel-cal-points tr").count(), 3);
    telemetry = beforeWheelWorkflow;
    await page.setViewportSize({width: 390, height: 844});

    unavailable = true;
    await page.locator("#connection.offline").waitFor();
    assert.equal(await page.locator("#freshness-notice").isVisible(), true);
    for (const id of ["cal-capture", "cal-cancel", "disarm-for-config"]) assert.equal(await page.locator(`#${id}`).isDisabled(), true);
    assert.equal(await page.locator("#global-state").textContent(), "Status unavailable");
    unavailable = false;
    await page.locator("#connection.online").waitFor();
    assert.equal(await page.locator("#freshness-notice").isVisible(), false);
    assert.equal(await page.locator("#global-state").textContent(), "DISARMED");
    telemetry.protocol = 2;
    await page.locator("#connection.offline").waitFor();
    assert.equal(await page.locator("#cal-capture").isDisabled(), true);
    assert.equal(await page.locator("#disarm-for-config").isDisabled(), true);
    telemetry.protocol = 3;
    await page.locator("#connection.online").waitFor();
    const linkage = telemetry.config.steering_linkage;
    delete telemetry.config.steering_linkage;
    await page.locator("#connection.offline").waitFor();
    assert.equal(await linkageForm.locator("button").isDisabled(), true);
    telemetry.config.steering_linkage = linkage;
    await page.locator("#connection.online").waitFor();
    for (const key of ["steering_toe_deg", "steering_toe_range", "tank_limit_percent"]) {
      const value = telemetry.config[key]; delete telemetry.config[key];
      await page.locator("#connection.offline").waitFor();
      assert.equal(await page.locator('[data-builder="tank-limit"] button').isDisabled(), true);
      assert.equal(await page.locator('[data-builder="toe"] button').isDisabled(), true);
      telemetry.config[key] = value;
      await page.locator("#connection.online").waitFor();
    }
    const toeRange = telemetry.config.steering_toe_range;
    for (const invalidRange of [{min_deg: 0}, {min_deg: "-30", max_deg: 30}, {min_deg: 1, max_deg: 0}]) {
      telemetry.config.steering_toe_range = invalidRange;
      await page.locator("#connection.offline").waitFor();
      assert.equal(await page.locator('[data-builder="toe"] button').isDisabled(), true);
      telemetry.config.steering_toe_range = toeRange;
      await page.locator("#connection.online").waitFor();
    }
    await page.goto(url + "/#analysis");
    await page.locator('[data-page="live"]').waitFor({state: "visible"});
    await navigate("settings");
    await page.goBack();
    await page.locator('[data-page="live"]').waitFor({state: "visible"});
    assert.deepEqual(errors, [], `${name}: browser errors`);

    if (process.env.MONITOR_SCREENSHOTS && name === "webkit") {
      reset();
      await page.goto(url);
      await page.locator("#connection.online").waitFor();
      const dir = process.env.MONITOR_SCREENSHOTS;
      fs.mkdirSync(dir, {recursive: true});
      await page.screenshot({path: path.join(dir, "monitor-iphone.png"), animations: "disabled"});
      await page.setViewportSize({width: 844, height: 390});
      await page.screenshot({path: path.join(dir, "monitor-iphone-landscape.png"), animations: "disabled"});
      await page.setViewportSize({width: 390, height: 844});
      await navigate("settings");
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.screenshot({path: path.join(dir, "monitor-settings-phone.png"), animations: "disabled"});
      await page.locator('[aria-labelledby="tank-settings-heading"]').scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-tank-settings-phone.png"), animations: "disabled"});
      await page.locator('[aria-labelledby="toe-heading"]').scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-toe-phone.png"), animations: "disabled"});
      await page.locator('[aria-labelledby="linkage-heading"]').scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-linkage-phone.png"), animations: "disabled"});
      await page.locator('[data-builder="trim-left"]').scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-servo-trim-phone.png"), animations: "disabled"});
      await page.locator("#ackermann-form").scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-ackermann-phone.png"), animations: "disabled"});
      telemetry.drive_mode = "TANK";
      telemetry.steering = {...telemetry.steering, left_us:1500, right_us:1500, left_angle_deg:0, right_angle_deg:0,
        command:0, speed_valid:false, speed_held:false, speed_kmh:0, ackermann_strength_percent:0};
      await navigate("live");
      await page.waitForFunction(() => document.getElementById("drive-mode").textContent === "TANK");
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.screenshot({path: path.join(dir, "monitor-tank-live-phone.png"), animations: "disabled"});
    }
    console.log(`${name}: desktop/touch layouts, navigation, 18 configuration forms, toe, tank mode/limit, linkage limits, servo range/directions, signed trims, strength curve, calibration commands, and disconnect/recovery passed`);
  } finally { await browser.close(); }
}

(async () => {
  await new Promise(resolve => server.listen(0, "127.0.0.1", resolve));
  try {
    await run(chromium, {channel: "chrome"}, "chrome");
    await run(webkit, {}, "webkit");
  } finally { server.close(); server.closeAllConnections(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
