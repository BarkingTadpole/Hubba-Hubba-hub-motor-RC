const wheels = ["FL", "FR", "RL", "RR"];
const channels = [
  ["steering", "CH1 Steering"], ["throttle", "CH2 Throttle"],
  ["arm", "CH4 RUN/STOP"]
];
const el = id => document.getElementById(id);
const fmt = (value, digits = 1) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--";
const setText = (id, value) => { const node = el(id); if (node) node.textContent = value; };
const asArray = value => Array.isArray(value) ? value : [];
let editingAllowed = false;
let initialInputsLoaded = false;
let ackDraftLoaded = false;
let calibrationRequestPending = false;

// Switching views never reloads telemetry, form drafts, or an active workflow.
const viewScroll = {};
let activeView = null;
function showView() {
  const aliases = {calibration: "calibrate"};
  const requested = location.hash.slice(1);
  if (requested === "main" && activeView) {
    el("main").focus();
    return;
  }
  const view = aliases[requested] || requested || "live";
  const selected = ["live", "calibrate", "settings"].includes(view) ? view : "live";
  if (activeView === selected) return;
  if (activeView) viewScroll[activeView] = window.scrollY;
  document.querySelectorAll("[data-page]").forEach(page => { page.hidden = page.dataset.page !== selected; });
  document.querySelectorAll("[data-view]").forEach(link => {
    if (link.dataset.view === selected) link.setAttribute("aria-current", "page");
    else link.removeAttribute("aria-current");
  });
  activeView = selected;
  el("main").focus({preventScroll: true});
  window.scrollTo(0, viewScroll[selected] || 0);
}
window.addEventListener("hashchange", showView);

function renderChannels(rc = {}) {
  el("receiver-grid").innerHTML = channels.map(([key, label]) => {
    const channel = rc[key] || {};
    const valid = channel.valid === true;
    return `<tr class="${valid ? "valid" : "invalid"}">
      <th scope="row">${label}</th><td>${valid ? channel.us : "--"}</td>
      <td>${Number(channel.age_ms) >= 0 ? channel.age_ms : "--"}</td>
      <td class="${valid ? "ok" : "bad"}">${valid ? "VALID" : "MISSING"}</td>
    </tr>`;
  }).join("");
}

function renderTables(t) {
  const rpm = asArray(t.rpm?.wheels);
  const outputs = asArray(t.outputs);
  el("rpm-body").innerHTML = ["RL", "RR"].map((wheel, index) => {
    const value = rpm[index] || {};
    return `<tr><th scope="row">${wheel}</th><td>${fmt(value.rpm, 0)}</td><td>${fmt(value.hz, 1)}</td><td>${value.edges ?? "--"}</td><td class="${value.valid ? "ok" : "bad"}">${value.valid ? "VALID" : "INVALID"}</td></tr>`;
  }).join("");
  el("output-body").innerHTML = wheels.map((wheel, index) => {
    const output = outputs[index] || {};
    return `<tr><th scope="row">${wheel}</th><td>${output.throttle_us ?? "--"}</td><td>${output.reverse_us ?? "--"}</td></tr>`;
  }).join("");
}

function setFormValue(builder, name, value) {
  if (value === undefined || value === null) return;
  const input = document.querySelector(`form[data-builder="${builder}"] [name="${name}"]`);
  if (input && document.activeElement !== input) input.value = value;
}

function loadConfigInputs(t, force = false) {
  if (force && document.activeElement instanceof HTMLElement) document.activeElement.blur();
  const c = t.config || {};
  setFormValue("drivetrain", "value", String(t.drive_mode || "awd").toLowerCase());
  setFormValue("arm-latch", "value", (c.permanent_arm_latch ?? true) ? "on" : "off");
  setFormValue("reverse", "value", c.reverse_limit_percent);
  setFormValue("drive-smoothing", "value", c.drive_smoothing_percent);
  setFormValue("poles", "value", c.motor_poles);
  setFormValue("ppr", "value", c.rpm_ppr);
  setFormValue("smoothing", "value", c.steering_smoothing_ms);
  setFormValue("failsafe", "pulse", c.failsafe_us);
  setFormValue("failsafe", "window", c.failsafe_window_us);
  ["left", "right"].forEach((side, index) => {
    const servo = c.servos?.[index];
    if (servo) {
      ["left", "center", "right"].forEach(end => setFormValue(`servo-${side}`, end, servo[`${end}_us`]));
      ["left", "right"].forEach(end => setFormValue(`servo-${side}`, `${end}_angle`, servo[`${end}_angle_deg`]));
    }
  });
  for (const [field, key] of [["wheelbase", "wheelbase_mm"], ["track", "track_width_mm"], ["diameter", "wheel_diameter_mm"], ["max_left", "max_left_deg"], ["max_right", "max_right_deg"]]) {
    setFormValue("geometry", field, c.steering_geometry?.[key]);
  }
  ["left", "center", "right", "deadband"].forEach(field => setFormValue("steering-input", field, c.steering_input?.[`${field}_us`]));
  if (force || !ackDraftLoaded) {
    setAckermannPoints(c.ackermann_strength_points || [{speed_kmh: 0, percent: 0}, {speed_kmh: 100, percent: 0}]);
    ackDraftLoaded = true;
  }
  initialInputsLoaded = true;
  setText("config-source", c.loaded_from_nvs === true
    ? "Source: saved NVS"
    : (c.loaded_from_nvs === false ? "Source: compiled defaults" : "Source: device config"));
}

function renderLinkHealth(connected, error) {
  const connection = el("connection");
  connection.className = `connection ${connected ? "online" : "offline"}`;
  connection.textContent = connected ? "Connected" : "Offline";
  connection.title = connected ? location.host : error;
  setText("http-health", location.host);
  setText("telemetry-health", connected ? "Live" : "Unavailable");
  setText("protocol-health", connected ? "v3" : "--");
  ["http-health", "telemetry-health", "protocol-health"].forEach(id => {
    el(id).className = connected ? "ok" : "bad";
  });
}

function render(t = null, error = "Waiting for ESP32") {
  const snapshotFresh = Boolean(t);
  renderLinkHealth(snapshotFresh, error);
  document.body.dataset.stale = String(!snapshotFresh);
  el("freshness-notice").hidden = snapshotFresh;
  setText("freshness-notice", `No live telemetry. Readings are stale; controls locked. ${error}`);
  setText("global-state", snapshotFresh ? (t.inhibited ? "Output inhibited" : String(t.state || "Unknown state").replaceAll("_", " ")) : "Status unavailable");
  el("global-state").classList.toggle("armed", snapshotFresh && Boolean(t.armed || t.inhibited));
  el("cal-nav-indicator").hidden = !(snapshotFresh && t.calibration?.active);
  if (!snapshotFresh) initialInputsLoaded = false;
  if (!t) {
    editingAllowed = false;
    document.querySelectorAll("form button").forEach(button => button.disabled = true);
    document.querySelectorAll("[data-cal-command]").forEach(button => button.disabled = true);
    el("disarm-for-config").disabled = true;
    setText("config-lock", "Locked · no telemetry");
    el("config-lock").className = "warning";
    return;
  }
  const steering = t.steering || {};
  setText("state-detail", t.inhibited ? `Inhibited · flags ${t.inhibit_flags}`
    : (t.armed ? "Drive active" : String(t.state || "Unknown").replaceAll("_", " ")));
  setText("drive-mode", t.drive_mode ?? "--");
  setText("arm-cycle", `Initial arm cycle ${t.initial_arm_cycle_ready ? "ready" : "required"}`);
  setText("latch-state", (t.config?.permanent_arm_latch ?? true) ? "Latch ON · STOP inhibits" : "Latch OFF · STOP disarms");
  setText("speed", (steering.speed_valid || steering.speed_held) ? fmt(steering.speed_kmh, 1) : "--");
  setText("speed-state", steering.speed_valid ? "LIVE · rear mean" : steering.speed_held ? "HELD · rear RPM missing" : "Unavailable · curve at 0 km/h");
  el("speed-state").className = steering.speed_held ? "warning" : "";
  setText("servo-left-us", `${steering.left_us ?? "--"} µs`);
  setText("servo-right-us", `${steering.right_us ?? "--"} µs`);
  setText("servo-left-angle", fmt(steering.left_angle_deg, 1));
  setText("servo-right-angle", fmt(steering.right_angle_deg, 1));
  setText("steering-command", `${fmt(Number(steering.command) * 100, 1)}%`);
  setText("ackermann-live", `${fmt(steering.ackermann_strength_percent, 1)}%`);
  renderChannels(t.rc);
  renderTables(t);
  setText("steering-filter", `${steering.filter_active ? "Active" : "Warmup"} · ${steering.rejected_spikes ?? 0} rejected`);
  const calibration = t.calibration || {};
  const calibrationActive = calibration.active === true;
  const calibrationSampling = calibration.sampling === true;
  const calibrationKind = String(calibration.kind || "none");
  const calibrationTotal = Number(calibration.total_steps || 0);
  const calibrationStep = Number(calibration.step || 0);
  const capturedValues = asArray(calibration.values_us)
    .map(Number)
    .filter(value => Number.isFinite(value) && value > 0);
  setText("cal-kind", calibrationActive ? calibrationKind : "none");
  setText("cal-step", calibrationActive && calibrationTotal > 0
    ? `${Math.min(calibrationStep + 1, calibrationTotal)}/${calibrationTotal}${calibrationSampling ? " (sampling...)" : ""}`
    : "--");
  setText("cal-values", capturedValues.length ? `${capturedValues.join(" / ")} us` : "--");
  setText("cal-prompt", calibration.prompt || "Select a calibration.");
  const calibrationMessage = el("cal-message");
  if (!calibrationRequestPending) {
    calibrationMessage.textContent = calibration.message || "";
    calibrationMessage.className = `message ${calibrationActive ? "warning" : ""}`;
  }

  editingAllowed = Boolean(t.state === "DISARMED" && !calibrationActive);
  document.querySelectorAll("form button").forEach(button => button.disabled = !editingAllowed);
  document.querySelectorAll("[data-cal-start]").forEach(button => {
    button.disabled = !(t.state === "DISARMED" && !calibrationActive &&
      !calibrationRequestPending);
  });
  el("cal-capture").disabled = !(calibrationActive && !calibrationSampling &&
    !calibrationRequestPending && !["none", "esc"].includes(calibrationKind));
  el("cal-cancel").disabled = !(calibrationActive && calibrationKind !== "none" &&
    !calibrationRequestPending);
  el("cal-esc-max").disabled = !(!calibrationRequestPending && calibrationKind === "esc" &&
    ["ESC_CAL_ARMED", "ESC_CAL_MAX", "ESC_CAL_MIN"].includes(t.state));
  el("cal-esc-min").disabled = !(!calibrationRequestPending &&
    calibrationKind === "esc" && t.state === "ESC_CAL_MAX");
  el("disarm-for-config").disabled = !(t.state === "DRIVE_ARMED" && !calibrationActive);
  el("config-lock").textContent = editingAllowed
    ? "Editable · DISARMED"
    : (calibrationActive ? "Locked · calibration active" : "Locked · disarm to edit");
  el("config-lock").className = editingAllowed ? "ok" : "warning";
  if (!initialInputsLoaded) loadConfigInputs(t);
  updateCurveButtons();
}

function readAckermannPoints() {
  const points = [...el("ackermann-points").rows].map(row => ({
    speed_kmh: row.querySelector('[data-point="speed"]').valueAsNumber,
    percent: row.querySelector('[data-point="percent"]').valueAsNumber
  }));
  if (points.length < 2 || points.length > 8) throw new Error("Use 2–8 curve points.");
  points.forEach((p, i) => {
    if (!Number.isFinite(p.speed_kmh) || !Number.isFinite(p.percent) || p.speed_kmh < 0 || p.speed_kmh > 300 || p.percent < 0 || p.percent > 100 ||
        Math.abs(p.speed_kmh * 10 - Math.round(p.speed_kmh * 10)) > 0.001 || Math.abs(p.percent * 10 - Math.round(p.percent * 10)) > 0.001) {
      throw new Error("Use speeds 0–300 km/h and Ackermann strength 0–100%, with at most one decimal place.");
    }
    if ((i === 0 && p.speed_kmh !== 0) || (i > 0 && p.speed_kmh <= points[i - 1].speed_kmh)) {
      throw new Error("Start at 0 km/h and enter speeds in strictly increasing order.");
    }
  });
  return points;
}

function drawAckermannCurve() {
  const svg = el("ackermann-preview");
  try {
    const points = readAckermannPoints(), maxSpeed = Math.max(10, points.at(-1).speed_kmh);
    const x = speed => 60 + speed / maxSpeed * 440, y = percent => 210 - percent * 1.8;
    const grid = [0, 25, 50, 75, 100].map(p => `<line x1="60" y1="${y(p)}" x2="500" y2="${y(p)}" stroke="#d6dfda"/><text x="50" y="${y(p) + 4}" text-anchor="end">${p}%</text>`).join("");
    const ticks = [0, 0.25, 0.5, 0.75, 1].map(t => `<text x="${x(t * maxSpeed)}" y="230" text-anchor="middle">${fmt(t * maxSpeed, 1)}</text>`).join("");
    svg.innerHTML = `<title>Draft Ackermann strength curve</title>${grid}${ticks}<text x="280" y="253" text-anchor="middle">Estimated vehicle speed (km/h)</text><polyline fill="none" stroke="#087759" stroke-width="3" points="${points.map(p => `${x(p.speed_kmh)},${y(p.percent)}`).join(" ")}"/>${points.map(p => `<circle cx="${x(p.speed_kmh)}" cy="${y(p.percent)}" r="5" fill="#087759"/>`).join("")}`;
    setText("ackermann-preview-status", "Draft · save to apply");
  } catch (error) {
    svg.innerHTML = "";
    setText("ackermann-preview-status", error.message);
  }
}

function addAckermannRow(point) {
  const index = el("ackermann-points").rows.length;
  const row = document.createElement("tr");
  for (const [field, value, min, max, label] of [["speed", point.speed_kmh, 0, 300, "Speed km/h"], ["percent", point.percent, 0, 100, "Ackermann strength percent"]]) {
    const cell = row.insertCell(), input = document.createElement("input");
    input.type = "number"; input.min = min; input.max = max; input.step = "0.1"; input.required = true;
    input.dataset.point = field; input.value = Number.isFinite(Number(value)) ? Number(value) : 0;
    input.setAttribute("aria-label", `${label}, point ${index + 1}`);
    if (field === "speed" && index === 0) input.readOnly = true;
    input.addEventListener("input", drawAckermannCurve); cell.append(input);
  }
  const remove = document.createElement("button");
  remove.type = "button"; remove.textContent = "Remove"; remove.dataset.removePoint = "";
  remove.disabled = !editingAllowed;
  remove.addEventListener("click", () => {
    if (!editingAllowed || row.rowIndex === 1 || el("ackermann-points").rows.length <= 2) return;
    row.remove(); drawAckermannCurve(); updateCurveButtons();
  });
  row.insertCell().append(remove); el("ackermann-points").append(row);
}

function updateCurveButtons() {
  const rows = [...el("ackermann-points").rows];
  rows.forEach((row, i) => row.querySelector("button").disabled = !editingAllowed || i === 0 || rows.length <= 2);
  el("ackermann-add").disabled = !editingAllowed || rows.length >= 8;
}

function setAckermannPoints(points) {
  el("ackermann-points").replaceChildren();
  points.slice(0, 8).forEach(addAckermannRow);
  drawAckermannCurve(); updateCurveButtons();
}

el("ackermann-add").addEventListener("click", () => {
  if (!editingAllowed || el("ackermann-points").rows.length >= 8) return;
  const rows = [...el("ackermann-points").rows], last = rows.at(-1);
  const speed = last?.querySelector('[data-point="speed"]').valueAsNumber || 0;
  const percent = last?.querySelector('[data-point="percent"]').valueAsNumber || 0;
  addAckermannRow({speed_kmh: Math.min(300, speed + 10), percent});
  drawAckermannCurve(); updateCurveButtons();
});

const commandBuilders = {
  drivetrain: f => `config drivetrain ${f.value.value}`,
  "arm-latch": f => `config arm-latch ${f.value.value}`,
  reverse: f => `config reverse ${f.value.value}`,
  "drive-smoothing": f => `config drive smoothing ${f.value.value}`,
  poles: f => `config rpm poles ${f.value.value}`,
  ppr: f => `config rpm ppr ${f.value.value}`,
  "servo-left": f => `config steering servo left ${f.left.value} ${f.center.value} ${f.right.value} ${f.left_angle.value} ${f.right_angle.value}`,
  "servo-right": f => `config steering servo right ${f.left.value} ${f.center.value} ${f.right.value} ${f.left_angle.value} ${f.right_angle.value}`,
  geometry: f => `config steering geometry ${f.wheelbase.value} ${f.track.value} ${f.diameter.value} ${f.max_left.value} ${f.max_right.value}`,
  "steering-input": f => `config steering input ${f.left.value} ${f.center.value} ${f.right.value} ${f.deadband.value}`,
  ackermann: () => `config steering ackermann-strength ${readAckermannPoints().map(p => `${p.speed_kmh}:${p.percent}`).join(" ")}`,
  smoothing: f => `config steering smoothing ${f.value.value}`,
  failsafe: f => `config failsafe ${f.pulse.value} ${f.window.value}`,
};

async function sendCommand(command) {
  const message = el("message");
  message.className = "message";
  message.textContent = `Sending: ${command}`;
  try {
    const response = await fetch("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({command})});
    const result = await response.json();
    message.className = `message ${result.ok ? "success" : "error"}`;
    message.textContent = result.message;
    if (result.ok) {
      initialInputsLoaded = false;
      if (command.startsWith("config steering ackermann-strength ")) ackDraftLoaded = false;
    }
  } catch (error) {
    message.className = "message error";
    message.textContent = `Config request failed: ${error}`;
  }
}

const calibrationConfirmations = {
  "cal receiver": "Start receiver throttle calibration? The saved NVS values are replaced only after neutral, full throttle, and full reverse are all captured and validated.",
  "cal steering": "Start steering calibration? Keep the car DISARMED. The saved NVS values are replaced only after center, full left, and full right are all captured and validated.",
  "cal arm": "Start CH4 calibration? Keep throttle neutral. You will capture RUN, STOP position 1, and STOP position 2 before the saved NVS values are replaced.",
  "cal esc arm": "Arm ESC endpoint calibration? LIFT AND RESTRAIN THE CAR, disconnect ESC traction power, leave CH4 in STOP, and keep throttle neutral before continuing.",
  "cal esc max": "Output the ESC maximum-throttle endpoint on all four throttle signals? Confirm the car is lifted and restrained and follow the ESC power-up procedure exactly.",
  "cal esc min": "Output the ESC minimum-throttle endpoint? Continue only after the ESC maximum-endpoint beeps have completed.",
  "cal manual": "DANGER: relay receiver throttle directly to all four ESC throttle outputs for up to 30 seconds? Lift and restrain the car, keep CH4 in STOP, and maintain immediate traction-power isolation."
};

async function sendCalibrationCommand(command) {
  if (calibrationRequestPending) return;
  const confirmation = calibrationConfirmations[command];
  if (confirmation && !confirm(confirmation)) return;
  calibrationRequestPending = true;
  document.querySelectorAll("[data-cal-command]").forEach(button => button.disabled = true);
  const message = el("cal-message");
  message.className = "message";
  message.textContent = `Sending: ${command}`;
  try {
    const response = await fetch("/api/calibration", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({command})
    });
    const result = await response.json();
    message.className = `message ${result.ok ? "success" : "error"}`;
    message.textContent = result.message;
  } catch (error) {
    message.className = "message error";
    message.textContent = `Calibration request failed: ${error}`;
  } finally {
    calibrationRequestPending = false;
  }
}

document.querySelectorAll("form[data-builder]").forEach(form => form.addEventListener("submit", event => {
  event.preventDefault();
  if (!editingAllowed || !form.reportValidity()) return;
  try { sendCommand(commandBuilders[form.dataset.builder](form.elements)); }
  catch (error) { el("message").className = "message error"; setText("message", error.message); }
}));
document.querySelectorAll("button[data-command]").forEach(button => button.addEventListener("click", () => editingAllowed && sendCommand(button.dataset.command)));
document.querySelectorAll("[data-cal-command]").forEach(button => button.addEventListener("click", () => {
  if (!button.disabled) sendCalibrationCommand(button.dataset.calCommand);
}));

el("cycle-drivetrain").addEventListener("click", () => {
  if (!editingAllowed) return;
  const select = document.querySelector('form[data-builder="drivetrain"] [name="value"]');
  const next = {awd: "fwd", fwd: "rwd", rwd: "awd"}[select.value] || "awd";
  select.value = next;
  sendCommand(`config drivetrain ${next}`);
});

el("disarm-for-config").addEventListener("click", async () => {
  if (!confirm("Disarm the car for configuration? Motor outputs will go safe immediately, and rearming will require the physical CH4 STOP-to-RUN cycle.")) return;
  const message = el("message");
  message.className = "message";
  message.textContent = "Disarming for configuration...";
  try {
    const response = await fetch("/api/disarm", {method: "POST"});
    const result = await response.json();
    message.className = `message ${result.ok ? "success" : "error"}`;
    message.textContent = result.message;
    if (result.ok) initialInputsLoaded = false;
  } catch (error) {
    message.className = "message error";
    message.textContent = `Disarm request failed: ${error}`;
  }
});

el("reload-config").addEventListener("click", async () => {
  const message = el("message");
  message.className = "message";
  message.textContent = "Loading saved config…";
  try {
    const response = await fetch("/api/config", {cache: "no-store"});
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.message || `HTTP ${response.status}`);
    loadConfigInputs({
      config: result.config,
      drive_mode: result.drive_mode,
    }, true);
    message.className = "message success";
    message.textContent = result.source === "nvs"
      ? "Loaded saved NVS config."
      : (result.source === "compiled_defaults"
        ? "Loaded compiled defaults; no saved config."
        : "Loaded device config; NVS source unknown.");
  } catch (error) {
    message.className = "message error";
    message.textContent = `Configuration load failed: ${error}`;
  }
});

async function poll() {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 3000);
  try {
    const response = await fetch("/api/state", {cache: "no-store", signal: controller.signal});
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.message || `HTTP ${response.status}`);
    if (!payload || payload.protocol !== 3 || !payload.state || !payload.config?.steering_geometry ||
        !payload.config.steering_input || !Array.isArray(payload.config.ackermann_strength_points)) {
      throw new Error("Unsupported telemetry response");
    }
    render(payload);
  } catch (error) {
    render(null, `ESP32: ${error.message}`);
  } finally {
    clearTimeout(timeout);
    setTimeout(poll, 250);
  }
}
window.addEventListener("error", event => {
  const message = el("message");
  if (message) {
    message.className = "message error";
    message.textContent = `Dashboard JavaScript error: ${event.message}`;
  }
});
showView();
render();
poll();
