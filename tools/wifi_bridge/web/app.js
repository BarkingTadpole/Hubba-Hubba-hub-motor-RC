const wheels = ["FL", "FR", "RL", "RR"];
const channels = [
  ["steering", "CH1 Steering"], ["throttle", "CH2 Throttle"],
  ["arm", "CH4 Arm / inhibit"], ["tv", "CH5 TV mode"]
];
const el = id => document.getElementById(id);
const fmt = (value, digits = 1) => Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--";
const setText = (id, value) => { const node = el(id); if (node) node.textContent = value; };
const asArray = value => Array.isArray(value) ? value : [];
const bytesLabel = value => {
  const bytes = Number(value);
  if (!Number.isFinite(bytes)) return "--";
  if (bytes >= 1048576) return `${(bytes / 1048576).toFixed(2)} MiB`;
  if (bytes >= 1024) return `${(bytes / 1024).toFixed(1)} KiB`;
  return `${bytes} B`;
};
const durationLabel = seconds => {
  if (!Number.isFinite(seconds) || seconds < 0) return "--";
  if (seconds < 60) return `${seconds.toFixed(1)} s`;
  return `${Math.floor(seconds / 60)} min ${Math.round(seconds % 60)} s`;
};

let editingAllowed = false;
let initialInputsLoaded = false;
let analysisData = null;
let plottedSeries = [];
let calibrationRequestPending = false;

function renderChannels(rc = {}) {
  el("receiver-grid").innerHTML = channels.map(([key, label]) => {
    const channel = rc[key] || {};
    const valid = channel.valid === true;
    return `<article class="channel ${valid ? "valid" : "invalid"}">
      <header><span>${label}</span><small class="${valid ? "ok" : "bad"}">${valid ? "VALID" : "MISSING"}</small></header>
      <strong>${valid ? channel.us : "--"} <small>us</small></strong>
      <small>${Number(channel.age_ms) >= 0 ? `${channel.age_ms} ms old` : "no current frame"}</small>
    </article>`;
  }).join("");
}

function renderTables(t) {
  const rpm = asArray(t.rpm?.wheels);
  const outputs = asArray(t.outputs);
  const correction = asArray(t.tv?.correction);
  el("rpm-body").innerHTML = wheels.map((wheel, index) => {
    const value = rpm[index] || {};
    return `<tr><td>${wheel}</td><td>${fmt(value.rpm, 0)}</td><td>${fmt(value.hz, 1)}</td><td>${value.edges ?? "--"}</td><td class="${value.valid ? "ok" : "bad"}">${value.valid ? "VALID" : "INVALID"}</td></tr>`;
  }).join("");
  el("output-body").innerHTML = wheels.map((wheel, index) => {
    const output = outputs[index] || {};
    return `<tr><td>${wheel}</td><td>${output.throttle_us ?? "--"} us</td><td>${output.reverse_us ?? "--"} us</td><td>${fmt(Number(correction[index]) * 100, 2)}%</td></tr>`;
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
  setFormValue("logging-rate", "value", c.logging_rate_hz ?? t.logging?.rate_hz);
  setFormValue("drivetrain", "value", String(t.drive_mode || "awd").toLowerCase());
  setFormValue("arm-latch", "value", (c.permanent_arm_latch ?? true) ? "on" : "off");
  setFormValue("reverse", "value", c.reverse_limit_percent);
  setFormValue("drive-smoothing", "value", c.drive_smoothing_percent);
  setFormValue("poles", "value", c.motor_poles);
  setFormValue("ppr", "value", c.rpm_ppr);
  setFormValue("trim", "value", c.steering_trim_deg);
  setFormValue("smoothing", "value", c.steering_smoothing_ms);
  setFormValue("speed-limit", "value", c.steering_speed_limit ? "on" : "off");
  setFormValue("lateral-g", "value", c.steering_lateral_g);
  setFormValue("tv-enabled", "value", t.tv?.configured ? "enable" : "disable");
  setFormValue("tv-authority", "value", c.tv_authority_percent);
  setFormValue("front-relief", "value", c.tv_front_relief_percent);
  setFormValue("yaw-sign", "value", c.imu_yaw_sign);
  setFormValue("failsafe", "pulse", c.failsafe_us);
  setFormValue("failsafe", "window", c.failsafe_window_us);
  setFormValue("tv-gains", "yaw", c.tv_turn_yaw_gain_dps);
  setFormValue("tv-gains", "turn", c.tv_turn_rpm_gain);
  setFormValue("tv-gains", "kp", c.tv_yaw_kp);
  setFormValue("tv-gains", "ki", c.tv_yaw_ki);
  setFormValue("tv-gains", "rpm", c.tv_rpm_kp);
  initialInputsLoaded = true;
  setText("config-source", c.loaded_from_nvs === true
    ? "Loaded from ESP32 NVS"
    : (c.loaded_from_nvs === false ? "Using compiled defaults" : "Loaded current device config"));
}

function renderLinkHealth(state) {
  const connection = el("connection");
  if (state.direct_http) {
    connection.className = "connection online";
    connection.textContent = `ESP32 AP ${state.esp_host || location.host} / live`;
    setText("tcp-health", `connected to ${state.esp_host || location.host}`);
    el("tcp-health").className = "ok";
    setText("telemetry-health", "fresh HTTP snapshot");
    el("telemetry-health").className = "ok";
    setText("protocol-health", "on-device API active");
    el("protocol-health").className = "ok";
    return;
  }
  if (state.telemetry_fresh || state.connected) {
    connection.className = "connection online";
    connection.textContent = `ESP32 ${state.esp_host || ""} / ${state.telemetry_age_ms ?? "--"} ms`;
  } else if (state.transport_connected) {
    connection.className = "connection degraded";
    connection.textContent = state.telemetry_received ? "ESP32 telemetry stale" : "ESP32 TCP connected / no telemetry";
  } else {
    connection.className = "connection offline";
    connection.textContent = state.last_error || "ESP32 not connected";
  }
  const reconnectTarget = state.last_known_esp_host
    ? `reconnecting to ${state.last_known_esp_host}`
    : "not connected";
  setText("tcp-health", state.transport_connected ? `connected to ${state.esp_host}` : reconnectTarget);
  el("tcp-health").className = state.transport_connected ? "ok" : "bad";
  const streamText = state.telemetry_fresh
    ? `${state.telemetry_count ?? "--"} frames / ${state.telemetry_age_ms} ms old`
    : (state.telemetry_received ? `stale / ${state.telemetry_age_ms} ms old` : "no valid frames");
  setText("telemetry-health", streamText);
  el("telemetry-health").className = state.telemetry_fresh ? "ok" : "bad";
  setText("protocol-health", state.hello ? `handshake / ${state.json_error_count || 0} JSON errors` : (state.last_error || "waiting"));
}

function renderLogging(logging, stateName, fresh) {
  if (!logging) {
    setText("log-state", "Not reported by installed firmware");
    setText("retention-estimate", "Flash logger requires the updated firmware");
    el("download-log").classList.add("disabled");
    el("load-device-log").disabled = true;
    el("clear-log").disabled = true;
    return;
  }
  const samples = Number(logging.samples || 0), used = Number(logging.bytes || 0);
  const recoveryNeeded = samples <= 0 && Boolean(logging.full) &&
    Number(logging.capacity_bytes) > 0 && Number(logging.free_bytes) <= 16384;
  const logState = recoveryNeeded
    ? "STORAGE ALLOCATION FULL — recovery required"
    : logging.faulted
    ? `STORAGE FAULT${logging.last_errno ? ` (errno ${logging.last_errno})` : ""} — export readable data, then clear`
    : (logging.full ? "FULL — export and clear" : (logging.recording ? "Recording offline and online" : "Unavailable"));
  setText("log-state", logState);
  setText("log-samples", Number(logging.samples || 0).toLocaleString());
  setText("log-size", `${bytesLabel(logging.bytes)} / ${bytesLabel(logging.capacity_bytes)}`);
  setText("log-free", bytesLabel(logging.free_bytes));
  setText("log-interval", `${logging.rate_hz ?? "--"} Hz (${logging.interval_ms ?? "--"} ms)`);
  setText("log-errors", logging.write_errors ?? "--");
  const free = Math.max(0, Number(logging.free_bytes) - 16384);
  const interval = Number(logging.interval_ms) / 1000;
  const seconds = samples > 0 && used > 0 ? free / (used / samples) * interval : NaN;
  setText("retention-estimate", recoveryNeeded
    ? "The CSV contains only its header, but FAT reports no free clusters. Recover storage while DISARMED."
    : logging.faulted
    ? "Storage stopped after an I/O error; export before attempting recovery"
    : (Number.isFinite(seconds)
      ? `About ${durationLabel(seconds)} remains at the observed row size`
      : "Estimate available after samples arrive"));
  el("download-log").classList.toggle("disabled", !fresh || samples <= 0);
  el("load-device-log").disabled = !fresh || samples <= 0;
  el("clear-log").textContent = recoveryNeeded ? "Recover log storage" : "Clear after export";
  el("clear-log").dataset.recoveryNeeded = recoveryNeeded ? "true" : "false";
  el("clear-log").disabled = !(fresh && stateName === "DISARMED" && used > 0);
}

function render(state) {
  renderLinkHealth(state);
  if (!state.transport_connected) initialInputsLoaded = false;
  const t = state.telemetry;
  if (!t) {
    editingAllowed = false;
    document.querySelectorAll("form button").forEach(button => button.disabled = true);
    document.querySelectorAll("[data-cal-command]").forEach(button => button.disabled = true);
    el("disarm-for-config").disabled = true;
    return;
  }
  const steering = t.steering || {}, imu = t.imu || {}, tv = t.tv || {}, dragy = t.dragy || {};
  setText("state", t.state ?? "--");
  setText("state-detail", t.inhibited ? `Safe output inhibit ${t.inhibit_flags}` : (t.armed ? "Drive outputs active" : "Drive disarmed"));
  setText("drive-mode", t.drive_mode ?? "--");
  setText("arm-cycle", `${(t.config?.permanent_arm_latch ?? true) ? "Permanent latch" : "STOP-to-disarm"}; initial cycle ${t.initial_arm_cycle_ready ? "ready" : "required"}`);
  setText("speed", steering.speed_valid ? fmt(steering.speed_kmh, 1) : "--");
  setText("steering-limit", steering.limited ? "Steering actively LIMITED" : `Maximum ${fmt(steering.maximum_deg, 2)} deg`);
  setText("yaw", imu.valid ? fmt(imu.yaw_dps, 2) : "--");
  setText("imu-state", `${imu.valid ? "valid" : "invalid"}, bias ${imu.bias_calibrated ? "calibrated" : "missing"}`);
  setText("gps-speed", dragy.speed_valid ? fmt(dragy.speed_kmh, 1) : "--");
  setText("gps-state", dragy.fix_valid ? `${dragy.satellites ?? "--"} satellites / valid fix` : (dragy.nmea_recent ? "NMEA received / no fix" : "No recent NMEA"));
  renderChannels(t.rc || {});
  renderTables(t);
  setText("dragy-uart", dragy.initialized ? `${dragy.baud ?? "--"} baud / ${dragy.uart_bytes ?? 0} bytes` : "disabled or unavailable");
  setText("dragy-position", dragy.position_valid ? `${fmt(dragy.latitude_deg, 6)}, ${fmt(dragy.longitude_deg, 6)}` : "no valid position");
  setText("dragy-course-altitude", `${dragy.course_valid ? `${fmt(dragy.course_deg, 1)} deg` : "--"} / ${dragy.altitude_valid ? `${fmt(dragy.altitude_m, 1)} m` : "--"}`);
  setText("dragy-quality", `${dragy.satellites ?? "--"} / ${fmt(dragy.hdop, 2)}`);
  setText("dragy-nmea", `${dragy.valid_sentences ?? 0} valid, ${dragy.checksum_errors ?? 0} checksum, ${dragy.parse_errors ?? 0} parse errors`);
  const addresses = asArray(dragy.i2c_addresses).map(address => `0x${Number(address).toString(16).toUpperCase().padStart(2, "0")}`);
  setText("dragy-i2c-addresses", dragy.i2c_scan_complete ? (addresses.join(", ") || "none") : "scan unavailable");
  setText("dragy-compass-presence", dragy.compass_candidate_present ? "candidate detected" : "not identified");
  setText("dragy-compass-data", dragy.compass_data_supported ? "available" : "protocol not published");
  setText("servo-us", `${steering.servo_us ?? "--"} us`);
  setText("servo-deg", `${fmt(steering.servo_command_deg, 2)} deg`);
  setText("wheel-deg", `${fmt(steering.left_wheel_deg, 2)} deg / ${fmt(steering.right_wheel_deg, 2)} deg`);
  setText("requested-max", `${fmt(steering.requested_deg, 2)} deg / ${fmt(steering.maximum_deg, 2)} deg`);
  setText("steering-filter", `${steering.filter_active ? "active" : "warmup"}, ${steering.rejected_spikes ?? 0} spikes rejected`);
  setText("tv-modes", `${tv.requested ?? "--"} / ${tv.active ? tv.active_mode : "inactive"}`);
  setText("tv-yaw", `${fmt(tv.target_yaw_dps, 2)} / ${fmt(tv.yaw_error_dps, 2)} deg/s`);
  setText("tv-rpm-error", fmt(tv.side_rpm_error, 5));
  setText("tv-demand", `${fmt(Number(tv.lateral_demand) * 100, 1)}%, relief ${fmt(Number(tv.front_relief) * 100, 1)}%`);
  setText("tv-reason", tv.reason ?? "--");
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
  setText("cal-prompt", calibration.prompt || "Select a calibration below");
  const calibrationMessage = el("cal-message");
  if (!calibrationRequestPending) {
    calibrationMessage.textContent = calibration.message || "No calibration command sent.";
    calibrationMessage.className = `message ${calibrationActive ? "warning" : ""}`;
  }

  const fresh = Boolean((state.telemetry_fresh || state.connected) && Number(state.telemetry_age_ms) < 1500);
  renderLogging(t.logging, t.state, fresh);
  editingAllowed = Boolean(fresh && t.state === "DISARMED" && !calibrationActive);
  document.querySelectorAll("form button").forEach(button => button.disabled = !editingAllowed);
  document.querySelectorAll("[data-cal-start]").forEach(button => {
    button.disabled = !(fresh && t.state === "DISARMED" && !calibrationActive &&
      !calibrationRequestPending);
  });
  el("cal-capture").disabled = !(fresh && calibrationActive && !calibrationSampling &&
    !calibrationRequestPending && !["none", "esc"].includes(calibrationKind));
  el("cal-cancel").disabled = !(fresh && calibrationActive && calibrationKind !== "none" &&
    !calibrationRequestPending);
  el("cal-esc-max").disabled = !(fresh && !calibrationRequestPending && calibrationKind === "esc" &&
    ["ESC_CAL_ARMED", "ESC_CAL_MAX", "ESC_CAL_MIN"].includes(t.state));
  el("cal-esc-min").disabled = !(fresh && !calibrationRequestPending &&
    calibrationKind === "esc" && t.state === "ESC_CAL_MAX");
  el("disarm-for-config").disabled = !(fresh && t.state === "DRIVE_ARMED" && !calibrationActive);
  el("config-lock").textContent = editingAllowed
    ? "Editing enabled while DISARMED"
    : (calibrationActive ? "Configuration locked while calibration is active" : "Configuration locked: connect and remain DISARMED");
  el("config-lock").className = editingAllowed ? "ok" : "warning";
  if (!initialInputsLoaded) loadConfigInputs(t);
}

const commandBuilders = {
  "logging-rate": f => `config logging rate ${f.value.value}`,
  drivetrain: f => `config drivetrain ${f.value.value}`,
  "arm-latch": f => `config arm-latch ${f.value.value}`,
  reverse: f => `config reverse ${f.value.value}`,
  "drive-smoothing": f => `config drive smoothing ${f.value.value}`,
  poles: f => `config rpm poles ${f.value.value}`,
  ppr: f => `config rpm ppr ${f.value.value}`,
  trim: f => `config steering trim ${f.value.value}`,
  smoothing: f => `config steering smoothing ${f.value.value}`,
  "speed-limit": f => `config steering speed-limit ${f.value.value}`,
  "lateral-g": f => `config steering lateral-g ${f.value.value}`,
  "tv-enabled": f => `tv ${f.value.value}`,
  "tv-authority": f => `config tv authority ${f.value.value}`,
  "front-relief": f => `config tv front-relief ${f.value.value}`,
  "yaw-sign": f => `config imu yaw-sign ${f.value.value}`,
  failsafe: f => `config failsafe ${f.pulse.value} ${f.window.value}`,
  "tv-gains": f => `config tv gains ${f.yaw.value} ${f.turn.value} ${f.kp.value} ${f.ki.value} ${f.rpm.value}`
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
    if (result.ok) initialInputsLoaded = false;
  } catch (error) {
    message.className = "message error";
    message.textContent = `ESP32 viewer request failed: ${error}`;
  }
}

const calibrationConfirmations = {
  "cal receiver": "Start receiver throttle calibration? The saved NVS values are replaced only after neutral, full throttle, and full reverse are all captured and validated.",
  "cal steering": "Start steering calibration? Keep the car DISARMED. The saved NVS values are replaced only after center, full left, and full right are all captured and validated.",
  "cal arm": "Start CH4 calibration? Keep throttle neutral. You will capture RUN, STOP position 1, and STOP position 2 before the saved NVS values are replaced.",
  "cal tv": "Start CH5 torque-vectoring selector calibration? Keep throttle neutral. You will capture OFF, STRAIGHT, and FULL before the saved NVS values are replaced.",
  "cal imu": "Start IMU yaw-bias calibration? Put the car on a level surface and keep it completely stationary, then use Capture to sample for two seconds.",
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
  if (editingAllowed && form.reportValidity()) sendCommand(commandBuilders[form.dataset.builder](form.elements));
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
  message.textContent = "Loading saved configuration from ESP32…";
  try {
    const response = await fetch("/api/config", {cache: "no-store"});
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.message || `HTTP ${response.status}`);
    loadConfigInputs({
      config: result.config,
      drive_mode: result.drive_mode,
      tv: {configured: result.tv_configured},
      logging: result.logging
    }, true);
    message.className = "message success";
    message.textContent = result.source === "nvs"
      ? "Loaded saved configuration from ESP32 NVS."
      : (result.source === "compiled_defaults"
        ? "The ESP32 is using compiled defaults; no saved NVS drive configuration was reported."
        : "Loaded the current device configuration; this firmware does not report its NVS source.");
  } catch (error) {
    message.className = "message error";
    message.textContent = `Configuration load failed: ${error}`;
  }
});

el("clear-log").addEventListener("click", async () => {
  const recoveryNeeded = el("clear-log").dataset.recoveryNeeded === "true";
  const prompt = recoveryNeeded
    ? "Rebuild the dedicated telemetry storage? It currently contains no samples. This reformats that log partition and cannot be undone."
    : "Delete the on-device telemetry CSV? Export it first; this cannot be undone.";
  if (!confirm(prompt)) return;
  const message = el("log-message");
  message.className = "message";
  message.textContent = recoveryNeeded ? "Recovering telemetry storage…" : "Clearing CSV log…";
  try {
    const response = await fetch("/api/log/clear", {method: "POST"});
    const result = await response.json();
    message.className = `message ${result.ok ? "success" : "error"}`;
    message.textContent = result.message;
  } catch (error) {
    message.className = "message error";
    message.textContent = `Clear failed: ${error}`;
  }
});
el("download-log").addEventListener("click", event => {
  if (el("download-log").classList.contains("disabled")) event.preventDefault();
});

const analysisGroups = {
  speed: {unit: "km/h", series: [["vehicle_speed_kmh", "RPM-derived speed"], ["gps_speed_kmh", "Dragy GPS speed"]]},
  rpm: {unit: "RPM", series: [["rpm_fl", "Front left"], ["rpm_fr", "Front right"], ["rpm_rl", "Rear left"], ["rpm_rr", "Rear right"]]},
  receiver: {unit: "µs", series: [["rc_throttle_us", "CH2 throttle"], ["rc_steering_us", "CH1 steering"], ["rc_arm_us", "CH4 arm"], ["rc_tv_us", "CH5 TV"]]},
  steering: {unit: "degrees", series: [["steering_requested_deg", "Requested mean wheel"], ["steering_max_deg", "Speed limit"], ["steering_servo_deg", "Servo command"], ["wheel_left_deg", "Left wheel"], ["wheel_right_deg", "Right wheel"]]},
  yaw: {unit: "deg/s", series: [["imu_yaw_dps", "Measured yaw"], ["tv_target_yaw_dps", "Target yaw"], ["tv_yaw_error_dps", "Yaw error"]]},
  outputs: {unit: "µs", series: [["out_fl_throttle_us", "FL throttle"], ["out_fr_throttle_us", "FR throttle"], ["out_rl_throttle_us", "RL throttle"], ["out_rr_throttle_us", "RR throttle"]]},
  vectoring: {unit: "fraction", series: [["tv_corr_fl", "FL correction"], ["tv_corr_fr", "FR correction"], ["tv_corr_rl", "RL correction"], ["tv_corr_rr", "RR correction"], ["tv_front_relief", "Front relief"]]},
  accel: {unit: "m/s²", series: [["imu_ax_mps2", "IMU X"], ["imu_ay_mps2", "IMU Y"], ["imu_az_mps2", "IMU Z"], ["tv_predicted_lateral_mps2", "Predicted lateral"]]}
};
const chartColors = ["#70f0c0", "#ffb454", "#65b8ff", "#f279c6", "#c6f16d", "#ad8cff"];

function parseCsv(text) {
  const lines = text.replace(/^\uFEFF/, "").split(/\r?\n/).filter(line => line.trim() !== "");
  if (!lines.length) throw new Error("CSV is empty");
  const headers = lines[0].split(",");
  if (lines.length === 1) return {headers, rows: []};
  const rows = [];
  let elapsed = 0, previousUptime = null, previousBoot = null;
  for (const line of lines.slice(1)) {
    const values = line.split(",");
    if (values.length !== headers.length) continue;
    const row = Object.fromEntries(headers.map((header, index) => [header, values[index]]));
    const uptime = Number(row.uptime_ms), boot = row.boot_id;
    if (previousUptime !== null) {
      const delta = boot === previousBoot && uptime >= previousUptime ? (uptime - previousUptime) / 1000 : 0.1;
      elapsed += Math.min(Math.max(delta, 0), 5);
    }
    row.__time = elapsed;
    previousUptime = uptime;
    previousBoot = boot;
    rows.push(row);
  }
  if (!rows.length) throw new Error("No complete telemetry rows were found");
  return {headers, rows};
}
function numeric(row, key) {
  const value = Number(row[key]);
  return Number.isFinite(value) ? value : null;
}
function summarizeAnalysis() {
  const rows = analysisData.rows;
  const speeds = rows.flatMap(row => [numeric(row, "vehicle_speed_kmh"), numeric(row, "gps_speed_kmh")].filter(Number.isFinite));
  const rpms = rows.flatMap(row => ["rpm_fl", "rpm_fr", "rpm_rl", "rpm_rr"].map(key => numeric(row, key)).filter(Number.isFinite));
  const peakSpeed = speeds.length ? Math.max(...speeds) : NaN;
  const peakRpm = rpms.length ? Math.max(...rpms) : NaN;
  setText("analysis-samples", rows.length.toLocaleString());
  setText("analysis-duration", durationLabel(rows.at(-1).__time));
  setText("analysis-speed", Number.isFinite(peakSpeed) ? `${peakSpeed.toFixed(1)} km/h` : "--");
  setText("analysis-rpm", Number.isFinite(peakRpm) ? `${peakRpm.toFixed(0)} RPM` : "--");
}

function drawChart() {
  const canvas = el("analysis-chart"), context = canvas.getContext("2d");
  const ratio = window.devicePixelRatio || 1, width = canvas.clientWidth || 1280, height = 460;
  canvas.width = Math.round(width * ratio); canvas.height = Math.round(height * ratio);
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.fillStyle = "#09171c"; context.fillRect(0, 0, width, height);
  if (!analysisData) return;
  const group = analysisGroups[el("analysis-group").value];
  plottedSeries = group.series.map(([key, label], index) => ({key, label, color: chartColors[index]}))
    .filter(series => analysisData.headers.includes(series.key));
  const points = analysisData.rows;
  const values = points.flatMap(row => plottedSeries.map(series => numeric(row, series.key))).filter(Number.isFinite);
  if (!values.length) {
    el("analysis-message").className = "message error";
    setText("analysis-message", `No ${group.unit} columns are present in this CSV.`);
    return;
  }
  let minimum = Math.min(...values), maximum = Math.max(...values);
  if (minimum === maximum) { minimum -= 1; maximum += 1; }
  const pad = (maximum - minimum) * 0.08; minimum -= pad; maximum += pad;
  const left = 70, right = 18, top = 24, bottom = 48;
  const plotWidth = width - left - right, plotHeight = height - top - bottom;
  const endTime = Math.max(points.at(-1).__time, 0.001);
  const x = time => left + time / endTime * plotWidth;
  const y = value => top + (maximum - value) / (maximum - minimum) * plotHeight;
  context.strokeStyle = "#24404b"; context.fillStyle = "#87a2ab";
  context.lineWidth = 1; context.font = "12px ui-monospace, monospace";
  for (let tick = 0; tick <= 5; tick++) {
    const py = top + tick / 5 * plotHeight, value = maximum - tick / 5 * (maximum - minimum);
    context.beginPath(); context.moveTo(left, py); context.lineTo(width - right, py); context.stroke();
    context.fillText(value.toFixed(2), 8, py + 4);
    const px = left + tick / 5 * plotWidth;
    context.beginPath(); context.moveTo(px, top); context.lineTo(px, height - bottom); context.stroke();
    context.fillText(`${(endTime * tick / 5).toFixed(1)} s`, px - 15, height - 18);
  }
  const stride = Math.max(1, Math.ceil(points.length / Math.max(800, width)));
  plottedSeries.forEach(series => {
    context.strokeStyle = series.color; context.lineWidth = 2; context.beginPath();
    let started = false;
    for (let index = 0; index < points.length; index += stride) {
      const value = numeric(points[index], series.key);
      if (value === null) { started = false; continue; }
      const px = x(points[index].__time), py = y(value);
      if (!started) { context.moveTo(px, py); started = true; } else context.lineTo(px, py);
    }
    context.stroke();
  });
  el("chart-legend").innerHTML = plottedSeries.map(series => `<span><i style="background:${series.color}"></i>${series.label}</span>`).join("");
  el("analysis-message").className = "message success";
  setText("analysis-message", `${plottedSeries.length} project channels plotted in ${group.unit}. Hover for exact sample values.`);
}

function loadAnalysis(text, source) {
  try {
    analysisData = parseCsv(text);
    if (!analysisData.rows.length) {
      analysisData = null;
      setText("analysis-samples", "0");
      setText("analysis-duration", "--");
      setText("analysis-speed", "--");
      setText("analysis-rpm", "--");
      drawChart();
      setText("analysis-message", `${source}: this CSV contains its header but no recorded telemetry samples.`);
      el("analysis-message").className = "message warning-state";
      return;
    }
    summarizeAnalysis();
    drawChart();
    setText("analysis-message", `${source}: ${analysisData.rows.length.toLocaleString()} complete rows loaded.`);
    el("analysis-message").className = "message success";
  } catch (error) {
    analysisData = null;
    el("analysis-message").className = "message error";
    setText("analysis-message", `Could not load CSV: ${error.message}`);
    drawChart();
  }
}
el("analysis-file").addEventListener("change", async event => {
  const file = event.target.files[0];
  if (file) loadAnalysis(await file.text(), file.name);
});
el("load-device-log").addEventListener("click", async () => {
  el("analysis-message").className = "message";
  setText("analysis-message", "Downloading current on-device CSV…");
  try {
    const response = await fetch("/api/log.csv", {cache: "no-store"});
    if (!response.ok) {
      const body = await response.text();
      try {
        throw new Error(JSON.parse(body).message || `HTTP ${response.status}`);
      } catch (error) {
        if (error instanceof SyntaxError) throw new Error(body || `HTTP ${response.status}`);
        throw error;
      }
    }
    loadAnalysis(await response.text(), "On-device CSV");
  } catch (error) {
    el("analysis-message").className = "message error";
    setText("analysis-message", `Device log download failed: ${error}`);
  }
});
el("analysis-group").addEventListener("change", drawChart);
window.addEventListener("resize", () => analysisData && drawChart());
el("analysis-chart").addEventListener("mousemove", event => {
  if (!analysisData || !plottedSeries.length) return;
  const rect = event.target.getBoundingClientRect();
  const fraction = Math.min(1, Math.max(0, (event.clientX - rect.left - 70) / Math.max(1, rect.width - 88)));
  const target = fraction * analysisData.rows.at(-1).__time;
  let low = 0, high = analysisData.rows.length - 1;
  while (low < high) {
    const mid = Math.floor((low + high) / 2);
    if (analysisData.rows[mid].__time < target) low = mid + 1; else high = mid;
  }
  const row = analysisData.rows[low], tooltip = el("chart-tooltip");
  tooltip.innerHTML = `<strong>${row.__time.toFixed(2)} s</strong>${plottedSeries.map(series => `<span><i style="background:${series.color}"></i>${series.label}: ${fmt(numeric(row, series.key), 3)}</span>`).join("")}`;
  tooltip.hidden = false;
  tooltip.style.left = `${Math.min(rect.width - 230, Math.max(8, event.clientX - rect.left + 12))}px`;
  tooltip.style.top = `${Math.max(8, event.clientY - rect.top - 30)}px`;
});
el("analysis-chart").addEventListener("mouseleave", () => { el("chart-tooltip").hidden = true; });

async function poll() {
  try {
    const response = await fetch("/api/state", {cache: "no-store"});
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.message || `HTTP ${response.status}`);
    if (payload && payload.protocol === 1 && !payload.telemetry) {
      render({
        connected: true,
        transport_connected: true,
        telemetry_received: true,
        telemetry_fresh: true,
        direct_http: true,
        esp_host: location.host,
        telemetry_age_ms: 0,
        telemetry: payload
      });
    } else {
      render(payload);
    }
  } catch (error) {
    render({connected: false, transport_connected: false, last_error: `ESP32 viewer unavailable: ${error}`, telemetry: null});
  } finally {
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
poll();
