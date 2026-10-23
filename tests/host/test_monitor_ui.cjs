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
        telemetry.calibration = {active: true, sampling: false, kind, step: 0, total_steps: 3,
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
    await draft.fill("12");
    await geometryDraft.fill("445.5");
    await inputDraft.fill("31");
    await page.setViewportSize({width: 844, height: 390});
    await page.waitForTimeout(400);
    await navigate("live");
    await navigate("settings");
    await page.setViewportSize({width: 390, height: 844});
    assert.equal(await draft.inputValue(), "12");
    assert.equal(await geometryDraft.inputValue(), "445.5", "Rotation and polling preserve geometry drafts");
    assert.equal(await inputDraft.inputValue(), "31", "Rotation and polling preserve CH1 drafts");
    await page.locator("#reload-config").click();
    await page.waitForFunction(() => document.querySelector('[data-builder="smoothing"] input').value === "10");
    assert.equal(await geometryDraft.inputValue(), "445");
    assert.equal(await inputDraft.inputValue(), "30");

    const commands = {
      drivetrain: "config drivetrain awd", "arm-latch": "config arm-latch on", reverse: "config reverse 100",
      "drive-smoothing": "config drive smoothing 100", failsafe: "config failsafe 1565 10",
      smoothing: "config steering smoothing 10",
      "servo-left": "config steering servo left 2000 1500 1000 30 30",
      "servo-right": "config steering servo right 1000 1500 2000 30 30",
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
      await page.locator("#ackermann-form").scrollIntoViewIfNeeded();
      await page.screenshot({path: path.join(dir, "monitor-ackermann-phone.png"), animations: "disabled"});
    }
    console.log(`${name}: desktop/touch layouts, navigation, 13 configuration forms, strength curve, calibration commands, and disconnect/recovery passed`);
  } finally { await browser.close(); }
}

(async () => {
  await new Promise(resolve => server.listen(0, "127.0.0.1", resolve));
  try {
    await run(chromium, {channel: "chrome"}, "chrome");
    await run(webkit, {}, "webkit");
  } finally { server.close(); server.closeAllConnections(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
