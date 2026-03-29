function meters(value) {
  return `${Number(value || 0).toFixed(2)} m`;
}

function speed(value) {
  return `${Number(value || 0).toFixed(2)} m/s`;
}

function seconds(value) {
  return `${Number(value || 0).toFixed(1)} s`;
}

function setText(id, value) {
  const element = document.getElementById(id);
  if (element) {
    element.textContent = value;
  }
}

function renderHeartbeats(heartbeats) {
  const container = document.getElementById("heartbeatStates");
  container.innerHTML = "";
  Object.entries(heartbeats || {}).forEach(([name, healthy]) => {
    const span = document.createElement("span");
    span.className = `pill ${healthy ? "good" : "bad"}`;
    span.textContent = `${name}: ${healthy ? "OK" : "FAILED"}`;
    container.appendChild(span);
  });
  if (!Object.keys(heartbeats || {}).length) {
    container.innerHTML = '<span class="pill">No heartbeat data yet</span>';
  }
}

function renderAreaTable(areaState) {
  const body = document.getElementById("areaTableBody");
  body.innerHTML = "";
  Object.entries((areaState && areaState.states) || {}).forEach(([name, metrics]) => {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${name}</td>
      <td>${meters(metrics.distance_m)}</td>
      <td>${seconds(metrics.total_time_sec)}</td>
      <td>${seconds(metrics.average_time_per_visit_sec)}</td>
      <td>${speed(metrics.average_speed_mps)}</td>
      <td>${metrics.visits}</td>
    `;
    body.appendChild(row);
  });
  if (!body.children.length) {
    body.innerHTML = '<tr><td colspan="6">No navigation area data yet</td></tr>';
  }
}

function renderEvents(events) {
  const body = document.getElementById("eventsTableBody");
  body.innerHTML = "";
  (events || []).forEach((event) => {
    const row = document.createElement("tr");
    row.innerHTML = `
      <td>${event.time || "--"}</td>
      <td><strong>${event.event_type || "--"}</strong></td>
      <td><code>${JSON.stringify(event.details || {})}</code></td>
    `;
    body.appendChild(row);
  });
  if (!body.children.length) {
    body.innerHTML = '<tr><td colspan="3">No events available</td></tr>';
  }
}

function renderLive(data) {
  const summary = data.summary || {};
  const mode = summary.mode || {};
  const motion = summary.motion || {};
  const interventions = summary.interventions || {};
  const robotState = summary.robot_state || {};
  const pathDeviation = summary.path_deviation || {};
  const navArea = summary.navigation_area || {};

  const isAutonomous = Boolean(mode.autonomous);
  const modeChip = document.getElementById("modeChip");
  modeChip.textContent = mode.label || "Waiting for summary";
  modeChip.className = `status-chip ${isAutonomous ? "good" : "bad"}`;

  setText("serverTimeLabel", data.server_time || "Server time unavailable");
  setText(
    "heartbeatLabel",
    data.heartbeat && data.heartbeat.seen
      ? `Heartbeat age ${Number(data.heartbeat.age_sec || 0).toFixed(1)} s`
      : "Heartbeat unknown"
  );
  setText("summaryTimestamp", summary.timestamp || "No summary received yet");
  setText("mdbiValue", summary.mdbi_m == null ? "N/A" : meters(summary.mdbi_m));
  setText("interventionsValue", interventions.count ?? 0);
  setText(
    "activeDisabledValue",
    robotState.active_to_disabled_in_autonomous_count ?? 0
  );
  setText("currentSpeedValue", speed(motion.current_speed_mps));
  setText("totalDistanceValue", meters(motion.total_distance_m));
  setText(
    "autoDistanceValue",
    meters(mode.distance_m ? mode.distance_m.Autonomous : 0)
  );
  setText(
    "manualDistanceValue",
    meters(mode.distance_m ? mode.distance_m.Manual : 0)
  );
  setText("autoTimeValue", seconds(mode.time_sec ? mode.time_sec.Autonomous : 0));
  setText("manualTimeValue", seconds(mode.time_sec ? mode.time_sec.Manual : 0));
  setText("avgSpeedValue", speed(motion.average_speed_mps));
  setText("areaCurrentValue", navArea.current || "Current area unknown");
  setText("pathCurrentValue", meters(pathDeviation.current_m));
  setText("pathAverageValue", meters(pathDeviation.average_m));
  setText("pathMaxValue", meters(pathDeviation.max_m));
  setText("pathSampleValue", `${pathDeviation.samples || 0} samples`);

  renderHeartbeats(summary.heartbeats || {});
  renderAreaTable(navArea);
}

function renderHistory(data) {
  if (data.session) {
    setText(
      "sessionInfo",
      `Session ${data.session._id} started ${data.session.session_start_time}`
    );
  }
  renderEvents(data.events || []);
}

async function loadLive() {
  const response = await fetch("/api/live");
  const data = await response.json();
  renderLive(data);
}

async function loadHistory() {
  const response = await fetch("/api/history?events=20&snapshots=10");
  const data = await response.json();
  renderHistory(data);
}

async function refresh() {
  try {
    await loadLive();
  } catch (error) {
    setText("serverTimeLabel", `Live refresh failed: ${error.message}`);
  }
}

async function refreshHistory() {
  try {
    await loadHistory();
  } catch (error) {
    setText("sessionInfo", `History refresh failed: ${error.message}`);
  }
}

refresh();
refreshHistory();
setInterval(refresh, 1000);
setInterval(refreshHistory, 10000);
