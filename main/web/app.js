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
let limitDraftLoaded = false;
let calibrationRequestPending = false;
let savedServos = [];
let wheelSnapshot = null;
const wheelDrafts = {left: null, right: null};
let wheelLastCapture = null;
let wheelRequestPending = false;
let backupData = null;
let backupBusy = false;

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

function updateToeRange(config) {
  const range = config.steering_toe_range;
  const input = document.querySelector('[data-builder="toe"] [name="value"]');
  input.min = range.min_deg;
  input.max = range.max_deg;
  const signed = value => `${value > 0 ? "+" : ""}${value.toFixed(1)}°`;
  setText("toe-range", `Available range: ${signed(range.min_deg)} to ${signed(range.max_deg)} per wheel, in 0.1° steps. Based on both wheels’ saved calibration, trims and linkage limits.`);
}

function updateServoTrimPreview(side, index) {
  const servo = savedServos[index];
  if (!servo) return;
  const input = document.querySelector(`[data-builder="trim-${side}"] [name="value"]`);
  // Trim commands use saved calibration, independent of endpoint form drafts.
  input.min = Math.min(servo.left_us, servo.right_us) + 50 - servo.center_us;
  input.max = Math.max(servo.left_us, servo.right_us) - 50 - servo.center_us;
  const neutral = input.value === "" ? "--" : servo.center_us + Number(input.value);
  setText(`trim-${side}-neutral`, `Neutral preview: ${neutral} µs`);
}

function updateServoDirection(side) {
  const fields = document.querySelector(`[data-builder="servo-${side}"]`).elements;
  const left = fields.left.valueAsNumber, center = fields.center.valueAsNumber, right = fields.right.valueAsNumber;
  const matches = left > center && right < center;
  const reversed = left < center && right > center;
  const hint = el(`servo-${side}-direction`);
  hint.className = `hint${matches ? "" : " warning"}`;
  hint.textContent = matches ? "Endpoint direction matches your mounting: wheel left CCW, wheel right CW."
    : reversed ? "Endpoint direction is reversed for your mounting. Wheel-left PWM should be above neutral; wheel-right below."
      : "Set wheel-left PWM above neutral and wheel-right below for your mounting.";
}

function loadConfigInputs(t, force = false) {
  if (force && document.activeElement instanceof HTMLElement) document.activeElement.blur();
  const c = t.config || {};
  setFormValue("drivetrain", "value", String(t.drive_mode || "awd").toLowerCase());
  setFormValue("arm-latch", "value", (c.permanent_arm_latch ?? true) ? "on" : "off");
  setFormValue("reverse", "value", c.reverse_limit_percent);
  setFormValue("tank-limit", "value", c.tank_limit_percent);
  setFormValue("toe", "value", c.steering_toe_deg);
  updateToeRange(c);
  setFormValue("drive-smoothing", "value", c.drive_smoothing_percent);
  setFormValue("poles", "value", c.motor_poles);
  setFormValue("ppr", "value", c.rpm_ppr);
  setFormValue("smoothing", "value", c.steering_smoothing_ms);
  setFormValue("failsafe", "pulse", c.failsafe_us);
  setFormValue("failsafe", "window", c.failsafe_window_us);
  savedServos = c.servos || [];
  ["left", "right"].forEach((side, index) => {
    const servo = c.servos?.[index];
    if (servo) {
      ["left", "center", "right"].forEach(end => setFormValue(`servo-${side}`, end, servo[`${end}_us`]));
      ["left", "right"].forEach(end => setFormValue(`servo-${side}`, `${end}_angle`, servo[`${end}_angle_deg`]));
      setFormValue(`trim-${side}`, "value", servo.trim_us ?? 0);
      updateServoTrimPreview(side, index);
      updateServoDirection(side);
    }
  });
  for (const [field, key] of [["wheelbase", "wheelbase_mm"], ["track", "track_width_mm"], ["diameter", "wheel_diameter_mm"], ["max_left", "max_left_deg"], ["max_right", "max_right_deg"]]) {
    setFormValue("geometry", field, c.steering_geometry?.[key]);
  }
  ["inside", "outside"].forEach(field => setFormValue("linkage", field, c.steering_linkage?.[`${field}_deg`]));
  ["left", "center", "right", "deadband"].forEach(field => setFormValue("steering-input", field, c.steering_input?.[`${field}_us`]));
  if (force || !ackDraftLoaded) {
    setCurvePoints("ackermann", c.ackermann_strength_points);
    ackDraftLoaded = true;
  }
  if (force || !limitDraftLoaded) {
    setCurvePoints("steering-limit", c.steering_limit_points);
    limitDraftLoaded = true;
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

function rememberWheelDraft() {
  wheelDrafts[el("wheel-cal-side").value] = [...el("wheel-cal-points").rows].map(row => ({
    pwm_us: row.querySelector('[data-wheel-point="pwm"]').value,
    angle_deg: row.querySelector('[data-wheel-point="angle"]').value
  }));
}

function drawWheelDraft() {
  const side = el("wheel-cal-side").value;
  const body = el("wheel-cal-points");
  body.replaceChildren();
  (wheelDrafts[side] || []).forEach((point, index) => {
    const row = body.insertRow();
    row.innerHTML = '<td><input data-wheel-point="pwm" type="number" min="500" max="2500" step="1" required></td><td><input data-wheel-point="angle" type="number" min="-85" max="85" step="0.1" required></td><td><button type="button">×</button></td>';
    const pwm = row.querySelector('[data-wheel-point="pwm"]'), angle = row.querySelector('[data-wheel-point="angle"]');
    pwm.value = point.pwm_us; angle.value = point.angle_deg;
    pwm.setAttribute("aria-label", `${side} measurement ${index + 1} PWM in microseconds`);
    angle.setAttribute("aria-label", `${side} measurement ${index + 1} wheel angle in degrees`);
    row.querySelector("button").setAttribute("aria-label", `Remove ${side} measurement ${index + 1}`);
    row.querySelector("button").addEventListener("click", () => {
      wheelDrafts[side].splice(index, 1); drawWheelDraft(); renderWheelCalibration(wheelSnapshot);
    });
  });
}

function wheelCalibrationPlan() {
  if (!wheelSnapshot) throw new Error("Connect to the device to review measurements.");
  const side = el("wheel-cal-side").value, index = side === "left" ? 0 : 1;
  const config = wheelSnapshot.config, servo = config.servos[index];
  const center = servo.center_us + servo.trim_us;
  const draft = wheelDrafts[side] || [];
  if (draft.length < 3 || draft.length > 9) throw new Error("Enter 3–9 measurements including straight ahead, left and right.");
  const points = draft.map(p => {
    const pwm = Number(p.pwm_us), angle = Number(p.angle_deg);
    if (p.pwm_us === "" || p.angle_deg === "" || !Number.isInteger(pwm) || pwm < 500 || pwm > 2500 ||
        !Number.isFinite(angle) || angle < -85 || angle > 85 || Math.abs(angle * 10 - Math.round(angle * 10)) > .001)
      throw new Error("Enter PWM 500–2500 µs and signed wheel angles −85° to +85°, with one decimal place.");
    return {pwm_us: pwm, angle_deg: angle};
  }).sort((a, b) => a.angle_deg - b.angle_deg);
  const first = points[0], last = points.at(-1);
  if (first.angle_deg > -1 || last.angle_deg < 1) throw new Error("Measure at least 1° of travel in each direction.");
  if (!points.some(p => p.angle_deg === 0 && p.pwm_us === center))
    throw new Error(`Verify straight ahead at trimmed neutral ${center} µs and enter 0°. Adjust trim first if needed.`);
  if ([center, servo.center_us].some(c => Math.abs(first.pwm_us - c) < 50 || Math.abs(last.pwm_us - c) < 50 ||
      (first.pwm_us - c) * (last.pwm_us - c) >= 0))
    throw new Error("Outer measurements must bracket base center and trimmed neutral by at least 50 µs.");
  for (let i = 1; i < points.length; i++) {
    const delta = points[i].pwm_us - points[i - 1].pwm_us;
    if (points[i].angle_deg <= points[i - 1].angle_deg || delta === 0 ||
        (delta > 0) !== (servo.right_us > servo.left_us))
      throw new Error("Angles and PWM must progress strictly in the saved wheel-left to wheel-right direction; remove duplicate positions.");
  }
  const other = config.servos[1 - index];
  const maxLeft = Math.min(config.steering_geometry.max_left_deg, -first.angle_deg, other.left_angle_deg);
  const maxRight = Math.min(config.steering_geometry.max_right_deg, last.angle_deg, other.right_angle_deg);
  return {side, points, maxLeft, maxRight, summary: `Save ${side}: ${points.length} measured points; left endpoint ${first.pwm_us} µs / ${first.angle_deg}°, right endpoint ${last.pwm_us} µs / +${last.angle_deg}°. Requested steering maxima will be ${maxLeft}° left / ${maxRight}° right. Center and trim stay fixed.`};
}

function renderWheelCalibration(t) {
  wheelSnapshot = t;
  const select = el("wheel-cal-side");
  const kind = t?.calibration?.kind || "none";
  const measuring = Boolean(t?.calibration?.active && ["wheel-left", "wheel-right"].includes(kind));
  const idle = Boolean(t?.state === "DISARMED" && !t.calibration.active);
  const editable = idle || measuring;
  const pending = wheelRequestPending || calibrationRequestPending;
  if (t) {
    let redraw = false;
    ["left", "right"].forEach((side, i) => {
      if (wheelDrafts[side] === null) {
        wheelDrafts[side] = t.config.servos[i].wheel_points.map(p => ({pwm_us: String(p.pwm_us), angle_deg: String(p.angle_deg)}));
        redraw = true;
      }
    });
    if (measuring && select.value !== kind.slice(6)) { select.value = kind.slice(6); redraw = true; }
    const sequence = t.calibration.wheel_capture_sequence;
    if (measuring && wheelLastCapture !== null && sequence !== wheelLastCapture && t.calibration.values_us[0] > 0) {
      const points = wheelDrafts[select.value], pulse = t.calibration.values_us[0];
      if (points.some(p => Number(p.pwm_us) === pulse)) {
        setText("wheel-cal-message", `PWM ${pulse} µs is already in the draft; update its measured angle.`);
      } else if (points.length < 9) {
        points.push({pwm_us: String(pulse), angle_deg: ""}); redraw = true;
        setText("wheel-cal-message", `Captured ${pulse} µs. Hold the wheel still and enter its measured angle.`);
      } else setText("wheel-cal-message", "Nine points already entered. Remove a point before capturing another.");
    }
    wheelLastCapture = sequence;
    if (redraw) drawWheelDraft();
    const index = select.value === "left" ? 0 : 1, servo = t.config.servos[index];
    setText("wheel-cal-live", `${select.value === "left" ? "Left" : "Right"} servo command: ${t.steering[`${select.value}_us`]} µs · trimmed neutral: ${servo.center_us + servo.trim_us} µs${measuring ? " · measurement mode, motors inhibited" : ""}`);
    const status = t.config.servos.map((s, i) => `${i ? "Right" : "Left"}: ${s.wheel_points.length ? `${s.wheel_points.length} measured points saved` : "uncalibrated"}`);
    setText("wheel-cal-status", status.join(" · "));
    el("wheel-cal-status").classList.toggle("warning", t.config.servos.some(s => !s.wheel_points.length));
  } else {
    wheelLastCapture = null;
    setText("wheel-cal-live", "No live servo position; actions locked. Draft retained.");
  }
  select.disabled = !idle || pending;
  el("wheel-cal-start").dataset.calCommand = `cal wheel ${select.value}`;
  el("wheel-cal-start").disabled = !idle || pending || t?.config.steering_toe_deg !== 0;
  el("wheel-cal-capture").disabled = !measuring || pending || wheelDrafts[select.value]?.length >= 9;
  el("wheel-cal-finish").disabled = !measuring || pending;
  el("wheel-cal-add").disabled = !editable || pending || wheelDrafts[select.value]?.length >= 9;
  el("wheel-cal-load").disabled = !idle || pending;
  el("wheel-cal-save").disabled = !idle || pending;
  el("wheel-cal-clear").disabled = !idle || pending || !t?.config.servos[select.value === "left" ? 0 : 1].wheel_points.length;
  el("wheel-cal-points").querySelectorAll("button,input").forEach(input => input.disabled = !editable || pending);
  try { setText("wheel-cal-preview", wheelCalibrationPlan().summary); }
  catch (error) { setText("wheel-cal-preview", error.message); }
}

async function saveWheelCalibration(command) {
  if (!wheelSnapshot || wheelSnapshot.state !== "DISARMED" || wheelSnapshot.calibration.active || wheelRequestPending) return;
  wheelRequestPending = true;
  renderWheelCalibration(wheelSnapshot);
  try {
    const response = await fetch("/api/config", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({command})});
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.message || "Save failed");
    initialInputsLoaded = false;
    el("wheel-cal-message").className = "message success";
    setText("wheel-cal-message", result.message);
  } catch (error) {
    el("wheel-cal-message").className = "message error";
    setText("wheel-cal-message", `${error.message}. Draft retained.`);
  } finally { wheelRequestPending = false; renderWheelCalibration(wheelSnapshot); }
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
    renderWheelCalibration(null);
    editingAllowed = false;
    document.querySelectorAll("form button").forEach(button => button.disabled = true);
    document.querySelectorAll("[data-cal-command]").forEach(button => button.disabled = true);
    el("disarm-for-config").disabled = true;
    setText("config-lock", "Locked · no telemetry");
    el("config-lock").className = "warning";
    updateBackupControls();
    return;
  }
  const steering = t.steering || {};
  // Refresh bounds without replacing an unsaved toe draft.
  updateToeRange(t.config);
  const tank = t.drive_mode === "TANK";
  setText("state-detail", t.inhibited ? `Inhibited · flags ${t.inhibit_flags}`
    : (t.armed ? "Drive active" : String(t.state || "Unknown").replaceAll("_", " ")));
  setText("drive-mode", t.drive_mode ?? "--");
  setText("speed-label", tank ? "Wheel surface speed" : "Rear speed");
  setText("arm-cycle", `Initial arm cycle ${t.initial_arm_cycle_ready ? "ready" : "required"}`);
  setText("latch-state", (t.config?.permanent_arm_latch ?? true) ? "Latch ON · STOP inhibits" : "Latch OFF · STOP disarms");
  setText("speed", (steering.speed_valid || steering.speed_held) ? fmt(steering.speed_kmh, 1) : "--");
  setText("speed-state", steering.speed_valid ? "LIVE · rear mean" : steering.speed_held ? "HELD · rear RPM missing" : "Unavailable · curve at 0 km/h");
  el("speed-state").className = steering.speed_held ? "warning" : "";
  if (tank) setText("speed-state", `${steering.speed_valid ? "LIVE" : steering.speed_held ? "HELD" : "Unavailable"} · not vehicle speed`);
  setText("servo-left-us", `${steering.left_us ?? "--"} µs`);
  setText("servo-right-us", `${steering.right_us ?? "--"} µs`);
  setText("servo-left-angle", fmt(steering.left_angle_deg, 1));
  setText("servo-right-angle", fmt(steering.right_angle_deg, 1));
  setText("steering-command", `${fmt(Number(steering.command) * 100, 1)}% → ${fmt(Number(steering.limited_command) * 100, 1)}%`);
  setText("steering-limit-live", tank ? "Bypassed · tank" : !steering.speed_valid
    ? "100% · no live speed" : `${fmt(steering.limit_percent, 1)}%`);
  el("steering-limit-live").className = !tank && !steering.speed_valid ? "warning" : "";
  setText("ackermann-live", tank ? "Bypassed · tank" : `${fmt(steering.ackermann_strength_percent, 1)}%`);
  setText("steering-target-note", tank ? "Tank: front wheels hold toe; CH1 selects spin." : "Angle targets include toe: + right; not measured.");
  if (t.config.servos.some(s => !s.wheel_points.length))
    setText("steering-target-note", `${tank ? "Tank: servos hold toe. " : ""}Wheel angles uncalibrated; displayed degrees are estimates.`);
  if (["wheel-left", "wheel-right"].includes(t.calibration.kind) && t.calibration.active) {
    setText("servo-left-angle", "--"); setText("servo-right-angle", "--");
    setText("ackermann-live", "Bypassed · measuring");
    setText("steering-limit-live", "Bypassed · measuring");
    el("steering-limit-live").className = "";
    setText("steering-target-note", "Wheel measurement mode: CH1 positions the selected servo; enter measured angles in Calibrate.");
  }
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
  renderWheelCalibration(t);
  updateBackupControls();
}

function updateBackupControls() {
  el("backup-export").disabled = !editingAllowed || backupBusy;
  el("backup-file").disabled = !editingAllowed || backupBusy;
  el("backup-import").disabled = !editingAllowed || backupBusy || !backupData;
}

function backupMessage(text, error = false) {
  setText("backup-message", text);
  el("backup-message").className = `message ${error ? "error" : "success"}`;
}

el("backup-export").addEventListener("click", async () => {
  if (!editingAllowed || backupBusy) return;
  backupBusy = true; updateBackupControls();
  try {
    const response = await fetch("/api/config/backup", {cache: "no-store"});
    if (!response.ok) throw new Error((await response.json()).message || "Export rejected");
    const blob = await response.blob();
    const url = URL.createObjectURL(blob), link = document.createElement("a");
    link.href = url; link.download = "rc-car-configuration.rcconfig";
    document.body.append(link); link.click(); link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
    backupMessage("Saved configuration exported. Keep this file to restore your setup.");
  } catch (error) { backupMessage(error.message, true); }
  finally { backupBusy = false; updateBackupControls(); }
});

el("backup-file").addEventListener("change", async () => {
  backupData = null;
  const file = el("backup-file").files[0];
  if (!file) { setText("backup-preview", "Choose a backup to validate and preview it before import."); updateBackupControls(); return; }
  backupBusy = true; updateBackupControls();
  setText("backup-preview", "Validating backup…");
  try {
    if (file.size !== 250) throw new Error("Invalid or unsupported configuration backup file.");
    const data = await file.arrayBuffer();
    const response = await fetch("/api/config/preview", {method: "POST", headers: {"Content-Type": "application/octet-stream"}, body: data});
    const preview = await response.json();
    if (!response.ok || !preview.ok) throw new Error(preview.message || "Invalid backup");
    backupData = data;
    const receivers = [preview.throttle_calibrated && "CH2", preview.steering_calibrated && "CH1", preview.arm_calibrated && "CH4"].filter(Boolean);
    setText("backup-preview", `${file.name}: ${preview.drive_mode}; receiver calibrations: ${receivers.join(", ") || "none saved"}; wheel measurements: left ${preview.left_wheel_points}, right ${preview.right_wheel_points}; Ackermann ${preview.ackermann_points} points; steering limit ${preview.steering_limit_points} points. Import replaces the complete saved vehicle setup.`);
    backupMessage("Backup validated. Review the preview, then import when ready.");
  } catch (error) {
    setText("backup-preview", "Backup rejected; the current configuration is unchanged.");
    backupMessage(error.message, true);
  } finally { backupBusy = false; updateBackupControls(); }
});

el("backup-import").addEventListener("click", async () => {
  if (!editingAllowed || backupBusy || !backupData) return;
  if (!confirm("Replace all saved vehicle settings and receiver/wheel calibrations with this backup? Servo alignment may change. The car stays disarmed and will require a physical CH4 STOP-to-RUN cycle.")) return;
  backupBusy = true; updateBackupControls();
  try {
    const response = await fetch("/api/config/import", {method: "POST", headers: {"Content-Type": "application/octet-stream"}, body: backupData});
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.message || "Import rejected");
    initialInputsLoaded = ackDraftLoaded = limitDraftLoaded = false;
    wheelDrafts.left = wheelDrafts.right = null;
    backupMessage(result.message);
  } catch (error) { backupMessage(error.message, true); }
  finally { backupBusy = false; updateBackupControls(); }
});

function readCurvePoints(id) {
  const points = [...el(`${id}-points`).rows].map(row => ({
    speed_kmh: row.querySelector('[data-point="speed"]').valueAsNumber,
    percent: row.querySelector('[data-point="percent"]').valueAsNumber
  }));
  if (points.length < 2 || points.length > 8) throw new Error("Use 2–8 curve points.");
  points.forEach((p, i) => {
    if (!Number.isFinite(p.speed_kmh) || !Number.isFinite(p.percent) || p.speed_kmh < 0 || p.speed_kmh > 300 || p.percent < 0 || p.percent > 100 ||
        Math.abs(p.speed_kmh * 10 - Math.round(p.speed_kmh * 10)) > 0.001 || Math.abs(p.percent * 10 - Math.round(p.percent * 10)) > 0.001) {
      throw new Error("Use speeds 0–300 km/h and percentages 0–100%, with at most one decimal place.");
    }
    if ((i === 0 && p.speed_kmh !== 0) || (i > 0 && p.speed_kmh <= points[i - 1].speed_kmh)) {
      throw new Error("Start at 0 km/h and enter speeds in strictly increasing order.");
    }
  });
  return points;
}

function drawCurve(id) {
  const svg = el(`${id}-preview`);
  try {
    const points = readCurvePoints(id), maxSpeed = Math.max(10, points.at(-1).speed_kmh);
    const x = speed => 60 + speed / maxSpeed * 440, y = percent => 210 - percent * 1.8;
    const grid = [0, 25, 50, 75, 100].map(p => `<line x1="60" y1="${y(p)}" x2="500" y2="${y(p)}" stroke="#d6dfda"/><text x="50" y="${y(p) + 4}" text-anchor="end">${p}%</text>`).join("");
    const ticks = [0, 0.25, 0.5, 0.75, 1].map(t => `<text x="${x(t * maxSpeed)}" y="230" text-anchor="middle">${fmt(t * maxSpeed, 1)}</text>`).join("");
    svg.innerHTML = `<title>Draft ${id === "ackermann" ? "Ackermann strength" : "steering limit"} curve</title>${grid}${ticks}<text x="280" y="253" text-anchor="middle">Estimated vehicle speed (km/h)</text><polyline fill="none" stroke="#087759" stroke-width="3" points="${points.map(p => `${x(p.speed_kmh)},${y(p.percent)}`).join(" ")}"/>${points.map(p => `<circle cx="${x(p.speed_kmh)}" cy="${y(p.percent)}" r="5" fill="#087759"/>`).join("")}`;
    setText(`${id}-preview-status`, "Draft · save to apply");
  } catch (error) {
    svg.innerHTML = "";
    setText(`${id}-preview-status`, error.message);
  }
}

function addCurveRow(id, point) {
  const index = el(`${id}-points`).rows.length;
  const row = document.createElement("tr");
  for (const [field, value, min, max, label] of [["speed", point.speed_kmh, 0, 300, "Speed km/h"], ["percent", point.percent, 0, 100, id === "ackermann" ? "Ackermann strength percent" : "Steering range percent"]]) {
    const cell = row.insertCell(), input = document.createElement("input");
    input.type = "number"; input.min = min; input.max = max; input.step = "0.1"; input.required = true;
    input.dataset.point = field; input.value = Number.isFinite(Number(value)) ? Number(value) : 0;
    input.setAttribute("aria-label", `${label}, point ${index + 1}`);
    if (field === "speed" && index === 0) input.readOnly = true;
    input.addEventListener("input", () => drawCurve(id)); cell.append(input);
  }
  const remove = document.createElement("button");
  remove.type = "button"; remove.textContent = "Remove"; remove.dataset.removePoint = "";
  remove.disabled = !editingAllowed;
  remove.addEventListener("click", () => {
    if (!editingAllowed || row.rowIndex === 1 || el(`${id}-points`).rows.length <= 2) return;
    row.remove(); drawCurve(id); updateCurveButtons();
  });
  row.insertCell().append(remove); el(`${id}-points`).append(row);
}

function updateCurveButtons() {
  for (const id of ["ackermann", "steering-limit"]) {
    const rows = [...el(`${id}-points`).rows];
    rows.forEach((row, i) => row.querySelector("button").disabled = !editingAllowed || i === 0 || rows.length <= 2);
    el(`${id}-add`).disabled = !editingAllowed || rows.length >= 8;
  }
}

function setCurvePoints(id, points) {
  el(`${id}-points`).replaceChildren();
  points.slice(0, 8).forEach(point => addCurveRow(id, point));
  drawCurve(id); updateCurveButtons();
}

for (const id of ["ackermann", "steering-limit"]) el(`${id}-add`).addEventListener("click", () => {
  if (!editingAllowed || el(`${id}-points`).rows.length >= 8) return;
  const rows = [...el(`${id}-points`).rows], last = rows.at(-1);
  const speed = last?.querySelector('[data-point="speed"]').valueAsNumber || 0;
  const percent = last?.querySelector('[data-point="percent"]').valueAsNumber || 0;
  addCurveRow(id, {speed_kmh: Math.min(300, speed + 10), percent});
  drawCurve(id); updateCurveButtons();
});

const commandBuilders = {
  drivetrain: f => `config drivetrain ${f.value.value}`,
  "arm-latch": f => `config arm-latch ${f.value.value}`,
  reverse: f => `config reverse ${f.value.value}`,
  "tank-limit": f => `config tank limit ${f.value.value}`,
  toe: f => `config steering toe ${f.value.value}`,
  "drive-smoothing": f => `config drive smoothing ${f.value.value}`,
  poles: f => `config rpm poles ${f.value.value}`,
  ppr: f => `config rpm ppr ${f.value.value}`,
  "servo-left": f => `config steering servo left ${f.left.value} ${f.center.value} ${f.right.value} ${f.left_angle.value} ${f.right_angle.value}`,
  "servo-right": f => `config steering servo right ${f.left.value} ${f.center.value} ${f.right.value} ${f.left_angle.value} ${f.right_angle.value}`,
  "trim-left": f => `config steering trim left ${f.value.value}`,
  "trim-right": f => `config steering trim right ${f.value.value}`,
  linkage: f => `config steering linkage ${f.inside.value} ${f.outside.value}`,
  geometry: f => `config steering geometry ${f.wheelbase.value} ${f.track.value} ${f.diameter.value} ${f.max_left.value} ${f.max_right.value}`,
  "steering-input": f => `config steering input ${f.left.value} ${f.center.value} ${f.right.value} ${f.deadband.value}`,
  ackermann: () => `config steering ackermann-strength ${readCurvePoints("ackermann").map(p => `${p.speed_kmh}:${p.percent}`).join(" ")}`,
  "steering-limit": () => `config steering limit ${readCurvePoints("steering-limit").map(p => `${p.speed_kmh}:${p.percent}`).join(" ")}`,
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
      if (command.startsWith("config steering limit ")) limitDraftLoaded = false;
    }
  } catch (error) {
    message.className = "message error";
    message.textContent = `Config request failed: ${error}`;
  }
}

const calibrationConfirmations = {
  "cal wheel left": "Start left wheel measurement? Disconnect traction power, power the servos, set toe to 0°, center CH1/CH2 and keep CH4 at STOP. CH1 will move only the left wheel within its saved travel limits.",
  "cal wheel right": "Start right wheel measurement? Disconnect traction power, power the servos, set toe to 0°, center CH1/CH2 and keep CH4 at STOP. CH1 will move only the right wheel within its saved travel limits.",
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

["left", "right"].forEach((side, index) => {
  document.querySelector(`[data-builder="trim-${side}"] input`).addEventListener("input", () => updateServoTrimPreview(side, index));
  document.querySelector(`[data-builder="servo-${side}"]`).addEventListener("input", () => updateServoDirection(side));
});

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
  const next = {awd: "fwd", fwd: "rwd", rwd: "tank", tank: "awd"}[select.value] || "awd";
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
        !payload.config.steering_input || !payload.config.steering_linkage ||
        !Number.isFinite(payload.config.steering_toe_deg) || !Number.isFinite(payload.config.tank_limit_percent) ||
        !Number.isFinite(payload.config.steering_toe_range?.min_deg) ||
        !Number.isFinite(payload.config.steering_toe_range?.max_deg) ||
        payload.config.steering_toe_range.min_deg > payload.config.steering_toe_range.max_deg ||
        !Array.isArray(payload.config.servos) || payload.config.servos.length !== 2 ||
        !payload.config.servos.every(s => Array.isArray(s.wheel_points)) ||
        !Number.isFinite(payload.calibration?.wheel_capture_sequence) ||
        !Array.isArray(payload.config.ackermann_strength_points) ||
        !Array.isArray(payload.config.steering_limit_points) ||
        !Number.isFinite(payload.steering?.limit_percent) ||
        !Number.isFinite(payload.steering?.limited_command)) {
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
el("wheel-cal-points").addEventListener("input", () => { rememberWheelDraft(); renderWheelCalibration(wheelSnapshot); });
el("wheel-cal-side").addEventListener("change", () => { drawWheelDraft(); renderWheelCalibration(wheelSnapshot); });
el("wheel-cal-add").addEventListener("click", () => {
  const draft = wheelDrafts[el("wheel-cal-side").value];
  if (!draft || draft.length >= 9) return;
  draft.push({pwm_us: "", angle_deg: ""}); drawWheelDraft(); renderWheelCalibration(wheelSnapshot);
});
el("wheel-cal-load").addEventListener("click", () => {
  if (!wheelSnapshot) return;
  const side = el("wheel-cal-side").value;
  wheelDrafts[side] = wheelSnapshot.config.servos[side === "left" ? 0 : 1].wheel_points.map(p => ({pwm_us: String(p.pwm_us), angle_deg: String(p.angle_deg)}));
  drawWheelDraft(); renderWheelCalibration(wheelSnapshot);
});
el("wheel-cal-form").addEventListener("submit", event => {
  event.preventDefault();
  try {
    const plan = wheelCalibrationPlan();
    saveWheelCalibration(`config steering wheel ${plan.side} ${plan.maxLeft} ${plan.maxRight} ${plan.points.map(p => `${p.pwm_us}:${p.angle_deg}`).join(" ")}`);
  } catch (error) { el("wheel-cal-message").className = "message error"; setText("wheel-cal-message", error.message); }
});
el("wheel-cal-clear").addEventListener("click", () => {
  const side = el("wheel-cal-side").value;
  if (confirm(`Clear ${side} wheel measurements? The endpoint approximation remains and this wheel is marked uncalibrated. Draft measurements remain in this browser.`))
    saveWheelCalibration(`config steering wheel ${side} clear`);
});
showView();
render();
poll();
