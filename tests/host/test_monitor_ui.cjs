// Browser regression checks against a local, synthetic direct-HTTP device.
// Requires Playwright; never connects to or commands an actual car.
const assert = require("node:assert/strict");
const http = require("node:http");
const fs = require("node:fs");
const path = require("node:path");
const {execFileSync} = require("node:child_process");
const {chromium, webkit} = require("playwright");

const root = path.resolve(__dirname, "../..");
const assets = path.join(root, "tools/wifi_bridge/web");
const baseline = JSON.parse(execFileSync(process.env.PYTHON || "python", ["-c",
  "import json; from tools.wifi_bridge.mock_esp import telemetry; print(json.dumps(telemetry(1)))"
], {cwd: root, encoding: "utf8"}));
let telemetry, unavailable, legacy, rejectConfig, requests;
const csvKeys = ["schema", "boot_id", "uptime_ms", "vehicle_speed_kmh", "gps_speed_kmh",
  "rpm_fl", "rpm_fr", "rpm_rl", "rpm_rr", "rc_throttle_us", "rc_steering_us", "rc_arm_us", "rc_tv_us",
  "steering_requested_deg", "steering_max_deg", "steering_servo_deg", "wheel_left_deg", "wheel_right_deg",
  "imu_yaw_dps", "tv_target_yaw_dps", "tv_yaw_error_dps", "out_fl_throttle_us", "out_fr_throttle_us",
  "out_rl_throttle_us", "out_rr_throttle_us", "tv_corr_fl", "tv_corr_fr", "tv_corr_rl", "tv_corr_rr",
  "tv_front_relief", "imu_ax_mps2", "imu_ay_mps2", "imu_az_mps2", "tv_predicted_lateral_mps2"];
const csv = csvKeys.join(",") + "\n" + Array.from({length: 12}, (_, i) =>
  csvKeys.map(key => key === "uptime_ms" ? i * 1000 : key === "boot_id" ? 1 : i + 1).join(",")).join("\n");

function reset() {
  telemetry = structuredClone(baseline);
  unavailable = legacy = rejectConfig = false;
  requests = [];
}
const server = http.createServer(async (req, res) => {
  res.setHeader("Cache-Control", "no-store");
  const json = (value, code = 200) => { res.writeHead(code, {"Content-Type": "application/json"}); res.end(JSON.stringify(value)); };
  if (req.url === "/api/state") {
    if (unavailable) return json({message: "Synthetic connection loss"}, 503);
    return json(legacy ? {connected: true, transport_connected: true, telemetry_fresh: true,
      telemetry_age_ms: 10, telemetry_count: 12, telemetry_received: true, esp_host: "mock", telemetry} : telemetry);
  }
  if (req.url === "/api/config" && req.method === "GET") return json({ok: true, source: "nvs", config: telemetry.config,
    drive_mode: telemetry.drive_mode, tv_configured: telemetry.tv.configured, logging: telemetry.logging});
  if (req.url === "/api/log.csv") {
    res.writeHead(200, {"Content-Type": "text/csv", "Content-Disposition": 'attachment; filename="telemetry.csv"'});
    return res.end(csv);
  }
  if (req.method === "POST") {
    let body = "";
    for await (const chunk of req) body += chunk;
    const command = body ? JSON.parse(body).command : null;
    requests.push({url: req.url, command});
    if (req.url === "/api/config" && rejectConfig) return json({ok: false, message: "Synthetic validation rejection"}, 409);
    if (req.url === "/api/log/start") telemetry.logging.recording = true;
    if (req.url === "/api/log/stop" || req.url === "/api/log/clear") telemetry.logging.recording = false;
    if (req.url === "/api/calibration") {
      if (command === "cal cancel") { telemetry.state = "DISARMED"; telemetry.calibration.active = false; }
      else if (command === "cal capture") {
        telemetry.calibration.values_us[telemetry.calibration.step] = 1500;
        if (++telemetry.calibration.step >= telemetry.calibration.total_steps) {
          telemetry.calibration.active = false;
          telemetry.state = "DISARMED";
        }
      } else if (command === "cal esc max") telemetry.state = "ESC_CAL_MAX";
      else if (command === "cal esc min") telemetry.state = "ESC_CAL_MIN";
      else {
        const kind = command.split(" ")[1];
        telemetry.state = kind === "esc" ? "ESC_CAL_ARMED" : "CALIBRATING";
        telemetry.calibration = {active: true, sampling: false, kind, step: 0, total_steps: kind === "imu" ? 1 : 3,
          values_us: [0, 0, 0], prompt: "Hold the requested position, then capture.", message: "Synthetic workflow active"};
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
    await page.goto(url);
    await page.locator("#connection.online").waitFor();
    assert.equal(await page.locator("#receiver-grid .valid").count(), 4);
    assert.equal(await page.locator("#rpm-body tr").count(), 4);
    assert.equal(await page.locator("#output-body tr").count(), 4);
    assert.equal(await page.locator("#speed").textContent(), "24.3");
    assert.equal(await page.locator("#gps-speed").textContent(), "24.6");
    assert.equal(await page.locator("#servo-us").textContent(), "1518 us");
    assert.equal(await page.locator("#tv-modes").textContent(), "OFF / inactive");

    // Every view stays inside small phones, portrait/landscape iPhones, and desktop.
    for (const width of [320, 375, 390, 430, 844, 1440]) {
      await page.setViewportSize({width, height: width === 844 ? 390 : 844});
      for (const view of ["live", "logs", "calibrate", "settings"]) {
        await navigate(view);
        const overflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
        assert.equal(overflow, false, `${name}: overflow at ${width} in ${view}`);
        const undersized = await page.locator("[data-page]:not([hidden]) button, [data-page]:not([hidden]) input, [data-page]:not([hidden]) select, .app-nav a").evaluateAll(nodes =>
          nodes.filter(node => node.getBoundingClientRect().height < 44).map(node => node.outerHTML));
        assert.deepEqual(undersized, [], `${name}: small touch targets at ${width}`);
      }
    }
    await page.setViewportSize({width: 390, height: 844});
    await navigate("settings");
    const draft = page.locator('form[data-builder="trim"] input');
    await draft.fill("1.2");
    await navigate("live");
    await navigate("settings");
    assert.equal(await draft.inputValue(), "1.2");
    await page.locator("#reload-config").click();
    await page.waitForFunction(() => document.querySelector('[data-builder="trim"] input').value === "0");

    const commands = {
      drivetrain: "config drivetrain awd", "arm-latch": "config arm-latch on", reverse: "config reverse 10",
      "drive-smoothing": "config drive smoothing 100", failsafe: "config failsafe 1565 10", trim: "config steering trim 0",
      smoothing: "config steering smoothing 60", "speed-limit": "config steering speed-limit on", "lateral-g": "config steering lateral-g 1",
      "tv-enabled": "tv disable", "tv-authority": "config tv authority 10", "front-relief": "config tv front-relief 20",
      "yaw-sign": "config imu yaw-sign 1", "tv-gains": "config tv gains 180 0.2 0.00025 0.00004 0.2",
      ppr: "config rpm ppr 7", poles: "config rpm poles 14", "logging-rate": "config logging rate 5"
    };
    assert.equal(await page.locator("form[data-builder]").count(), Object.keys(commands).length);
    for (const [builder, command] of Object.entries(commands)) {
      await submit(page.locator(`form[data-builder="${builder}"] button`).first(), "/api/config", command);
    }
    await submit(page.locator("#cycle-drivetrain"), "/api/config", "config drivetrain fwd");
    await submit(page.locator('[data-command="config failsafe off"]'), "/api/config", "config failsafe off");
    rejectConfig = true;
    await page.locator('form[data-builder="trim"] button').click();
    await page.locator("#message.error").waitFor();
    rejectConfig = false;

    telemetry.state = "DRIVE_ARMED"; telemetry.armed = true;
    await enabled("disarm-for-config");
    assert.equal(await page.locator('form[data-builder="trim"] button').isDisabled(), true);
    accept = false;
    const before = requests.length;
    await page.locator("#disarm-for-config").click();
    assert.equal(requests.length, before);
    accept = true;
    await submit(page.locator("#disarm-for-config"), "/api/disarm", null);
    await page.waitForFunction(() => document.getElementById("global-state").textContent === "DISARMED");

    await navigate("calibrate");
    for (const kind of ["receiver", "steering", "arm", "tv", "imu"]) {
      const start = page.locator(`[data-cal-command="cal ${kind}"]`);
      await submit(start, "/api/calibration", `cal ${kind}`);
      await enabled("cal-capture");
      assert.equal(await page.locator("#cal-nav-indicator").isVisible(), true);
      await navigate("settings");
      assert.equal(await page.locator('form[data-builder="trim"] button').isDisabled(), true);
      await navigate("calibrate");
      for (let step = 0; step < (kind === "imu" ? 1 : 3); step++) {
        await enabled("cal-capture");
        await submit(page.locator("#cal-capture"), "/api/calibration", "cal capture");
        if (step < (kind === "imu" ? 0 : 2)) await page.waitForFunction(step => document.getElementById("cal-step").textContent.startsWith(`${step + 2}/`), step);
      }
      await page.waitForFunction(() => document.getElementById("cal-capture").disabled);
    }
    await submit(page.locator("#cal-esc-arm"), "/api/calibration", "cal esc arm");
    await enabled("cal-esc-max");
    assert.equal(await page.locator("#cal-esc-min").isDisabled(), true);
    await submit(page.locator("#cal-esc-max"), "/api/calibration", "cal esc max");
    await enabled("cal-esc-min");
    await submit(page.locator("#cal-esc-min"), "/api/calibration", "cal esc min");
    await enabled("cal-cancel");
    await submit(page.locator("#cal-cancel"), "/api/calibration", "cal cancel");
    await enabled("cal-manual");
    await submit(page.locator("#cal-manual"), "/api/calibration", "cal manual");
    await enabled("cal-cancel");
    await submit(page.locator("#cal-cancel"), "/api/calibration", "cal cancel");

    await navigate("logs");
    await enabled("start-recording");
    assert.equal(await page.locator("#stop-recording").isDisabled(), true);
    assert.ok((await page.locator("#log-state").textContent()).startsWith("Stopped"));
    await submit(page.locator("#start-recording"), "/api/log/start", null);
    await enabled("stop-recording");
    assert.equal(await page.locator("#start-recording").isDisabled(), true);
    // Recording controls remain independent of the drive arm state.
    telemetry.state = "DRIVE_ARMED"; telemetry.armed = true;
    await submit(page.locator("#stop-recording"), "/api/log/stop", null);
    await enabled("start-recording");
    await submit(page.locator("#start-recording"), "/api/log/start", null);
    await enabled("stop-recording");
    await submit(page.locator("#stop-recording"), "/api/log/stop", null);
    await enabled("start-recording");
    telemetry.state = "DISARMED"; telemetry.armed = false;
    telemetry.logging.faulted = true;
    await page.waitForFunction(() => document.getElementById("start-recording").disabled);
    telemetry.logging.faulted = false;
    telemetry.logging.manual_control = false;
    await page.waitForFunction(() => document.getElementById("start-recording").disabled);
    telemetry.logging.manual_control = true;
    await enabled("start-recording");
    await enabled("load-device-log");
    const downloadPromise = page.waitForEvent("download");
    await page.locator("#download-log").click();
    assert.equal((await downloadPromise).suggestedFilename(), "telemetry.csv");
    await page.locator("#load-device-log").click();
    await page.waitForFunction(() => document.getElementById("analysis-samples").textContent === "12");
    for (const group of ["speed", "rpm", "receiver", "steering", "yaw", "outputs", "vectoring", "accel"]) {
      await page.locator("#analysis-group").selectOption(group);
      assert.ok(await page.locator("#chart-legend span").count() >= 2);
      assert.equal(await page.locator("#analysis-message").getAttribute("class"), "message success");
    }
    await page.locator("#analysis-chart").tap({position: {x: 180, y: 100}});
    assert.equal(await page.locator("#chart-tooltip").isVisible(), true);
    await page.locator("#analysis-chart").focus();
    await page.keyboard.press("ArrowRight");
    await page.keyboard.press("Escape");
    assert.equal(await page.locator("#chart-tooltip").isVisible(), false);
    const range = () => page.locator("#chart-range").textContent();
    await page.locator("#chart-zoom-in").click();
    assert.equal(await range(), "2.750–8.250 s · 2.0×");
    await page.locator("#analysis-group").selectOption("rpm");
    assert.equal(await range(), "2.750–8.250 s · 2.0×", "Group changes preserve the time window");
    await page.locator("#chart-zoom-in").click();
    assert.equal(await range(), "4.125–6.875 s · 4.0×");
    await page.locator("#chart-pan-right").click();
    assert.equal(await range(), "5.500–8.250 s · 4.0×");
    await page.locator("#chart-pan-left").click();
    assert.equal(await range(), "4.125–6.875 s · 4.0×");
    const chart = page.locator("#analysis-chart");
    await chart.scrollIntoViewIfNeeded();
    let box = await chart.boundingBox();
    const center = {x: box.x + 70 + (box.width - 88) / 2, y: box.y + 90};
    await page.mouse.move(center.x, center.y);
    await page.mouse.down();
    await page.mouse.move(center.x + 30, center.y, {steps: 3});
    await page.mouse.up();
    assert.notEqual(await range(), "4.125–6.875 s · 4.0×", "Dragging pans the zoomed window");
    await page.locator("#chart-reset").click();
    assert.equal(await range(), "Full run");
    // Playwright does not provide mouse-wheel input in mobile WebKit.
    const wheelPage = name === "webkit" ? await browser.newPage({viewport: {width: 1280, height: 900}}) : page;
    if (wheelPage !== page) {
      await wheelPage.goto(url + "/#logs");
      await wheelPage.waitForFunction(() => !document.getElementById("load-device-log").disabled);
      await wheelPage.locator("#load-device-log").click();
      await wheelPage.waitForFunction(() => document.getElementById("analysis-samples").textContent === "12");
    }
    await wheelPage.locator("#analysis-chart").scrollIntoViewIfNeeded();
    box = await wheelPage.locator("#analysis-chart").boundingBox();
    await wheelPage.mouse.move(box.x + 160, box.y + 90);
    await wheelPage.mouse.wheel(0, -200);
    await wheelPage.waitForFunction(() => document.getElementById("chart-range").textContent !== "Full run");
    await wheelPage.locator("#chart-reset").click();
    if (wheelPage !== page) await wheelPage.close();
    if (name === "chrome") {
      // Real multi-touch input through Chromium, rather than untrusted DOM events.
      await chart.scrollIntoViewIfNeeded();
      box = await chart.boundingBox();
      const cdp = await context.newCDPSession(page);
      const x = box.x + 70 + (box.width - 88) / 2, y = box.y + 100;
      const fingers = distance => [{x: x - distance, y, id: 1}, {x: x + distance, y, id: 2}];
      await cdp.send("Input.dispatchTouchEvent", {type: "touchStart", touchPoints: fingers(25)});
      await cdp.send("Input.dispatchTouchEvent", {type: "touchMove", touchPoints: fingers(50)});
      await cdp.send("Input.dispatchTouchEvent", {type: "touchEnd", touchPoints: []});
      await page.waitForFunction(() => document.getElementById("chart-range").textContent.includes("2.0×"));
      assert.equal(await range(), "2.750–8.250 s · 2.0×", "Pinch zoom is anchored at the fingers");
      await cdp.detach();
      await page.locator("#chart-reset").click();
    }
    await chart.focus();
    await page.keyboard.press("+");
    assert.equal(await range(), "2.750–8.250 s · 2.0×");
    await page.keyboard.press("Shift+ArrowRight");
    assert.equal(await range(), "5.500–11.000 s · 2.0×");
    assert.equal(await page.locator("#chart-pan-right").isDisabled(), true);
    await chart.tap({position: {x: 70, y: 100}});
    assert.equal(await page.locator("#chart-tooltip strong").textContent(), "6.00 s", "Inspection uses the visible time window");
    await chart.focus();
    await page.keyboard.press("0");
    assert.equal(await range(), "Full run");
    assert.equal(await page.locator("#chart-reset").isDisabled(), true);
    // Zoom is bounded and Reset always restores the full run.
    await chart.focus();
    for (let i = 0; i < 12; i++) await page.keyboard.press("+");
    assert.equal(await page.locator("#chart-zoom-in").isDisabled(), true);
    await page.locator("#chart-zoom-out").click();
    assert.equal(await page.locator("#chart-zoom-in").isDisabled(), false);
    await navigate("live");
    await navigate("logs");
    assert.notEqual(await range(), "Full run", "View changes preserve zoom");
    await page.locator("#analysis-file").setInputFiles({name: "saved-run.csv", mimeType: "text/csv", buffer: Buffer.from(csv)});
    await page.waitForFunction(() => document.getElementById("analysis-message").textContent.startsWith("saved-run.csv"));
    assert.equal(await range(), "Full run", "A new CSV resets zoom");
    await page.locator("#analysis-file").setInputFiles({name: "empty.csv", mimeType: "text/csv", buffer: Buffer.from(csvKeys.join(",") + "\n")});
    await page.waitForFunction(() => document.getElementById("analysis-samples").textContent === "0");
    assert.equal(await page.locator("#chart-legend span").count(), 0);
    assert.equal(await page.locator("#chart-zoom-in").isDisabled(), true);
    await page.locator("#analysis-file").setInputFiles({name: "invalid.csv", mimeType: "text/csv", buffer: Buffer.from("")});
    await page.locator("#analysis-message.error").waitFor();
    await submit(page.locator("#clear-log"), "/api/log/clear", null);
    await enabled("start-recording");
    assert.equal(await page.locator("#stop-recording").isDisabled(), true);
    telemetry.logging.samples = 0; telemetry.logging.full = true; telemetry.logging.free_bytes = 0;
    await page.waitForFunction(() => document.getElementById("clear-log").textContent === "Recover log storage");
    await submit(page.locator("#clear-log"), "/api/log/clear", null);

    unavailable = true;
    await page.locator("#connection.offline").waitFor();
    assert.equal(await page.locator("#freshness-notice").isVisible(), true);
    for (const id of ["clear-log", "load-device-log", "start-recording", "stop-recording", "cal-capture", "cal-cancel", "disarm-for-config"]) assert.equal(await page.locator(`#${id}`).isDisabled(), true);
    assert.equal(await page.locator("#download-log").getAttribute("class"), "button-link disabled");
    assert.equal(await page.locator("#global-state").textContent(), "Status unavailable");
    unavailable = false; legacy = true;
    await page.locator("#connection.online").waitFor();
    assert.equal(await page.locator("#freshness-notice").isVisible(), false);
    assert.equal(await page.locator("#global-state").textContent(), "DISARMED");
    await page.goto(url + "/#analysis");
    await page.locator('[data-page="logs"]').waitFor({state: "visible"});
    await navigate("settings");
    await page.goBack();
    await page.locator('[data-page="logs"]').waitFor({state: "visible"});
    assert.deepEqual(errors, [], `${name}: browser errors`);

    if (process.env.MONITOR_SCREENSHOTS && name === "webkit") {
      reset();
      await page.goto(url);
      await page.locator("#connection.online").waitFor();
      const dir = process.env.MONITOR_SCREENSHOTS;
      fs.mkdirSync(dir, {recursive: true});
      await page.screenshot({path: path.join(dir, "monitor-iphone.png"), animations: "disabled"});
      await navigate("logs");
      await page.locator("#load-device-log").click();
      await page.waitForFunction(() => document.getElementById("analysis-samples").textContent === "12");
      await page.locator("#analysis").scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-logs-iphone.png"), animations: "disabled"});
      await page.setViewportSize({width: 1440, height: 1000});
      await navigate("live");
      await page.mouse.move(1400, 950);
      await page.screenshot({path: path.join(dir, "monitor-desktop.png"), animations: "disabled"});
    }
    console.log(`${name}: layout, navigation, 17 configuration forms, calibration commands, logs, eight chart groups, zoom/pan/reset, touch, and disconnect/recovery passed`);
  } finally { await browser.close(); }
}

(async () => {
  await new Promise(resolve => server.listen(0, "127.0.0.1", resolve));
  try {
    await run(chromium, {channel: "chrome"}, "chrome");
    await run(webkit, {}, "webkit");
  } finally { server.close(); server.closeAllConnections(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
