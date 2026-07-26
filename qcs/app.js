const flightModes = ["MISSION", "OFFBOARD", "POSITION", "HOLD", "RETURN", "LAND", "TAKEOFF"];
const flightModeLabels = {
  MISSION: "미션",
  OFFBOARD: "외부 제어",
  POSITION: "위치 유지",
  HOLD: "대기",
  RETURN: "복귀",
  LAND: "착륙",
  TAKEOFF: "이륙",
  MANUAL: "수동",
  ALTITUDE: "고도 유지",
  ACRO: "아크로",
  STABILIZED: "안정화",
};
let currentFlightMode = "";
let sitlConnected = false;
let selectedPlanWaypoints = [];
let lastSitlTelemetry = null;
let previousQgcFrameUrl;
let qgcMap;
let qgcVehicleMarker;
let qgcTrayMarker;
let qgcMissionLayer;
let qgcMissionKey = "";
const sensorHistory = {
  accel: [],
  gyro: [],
  rawGpsPosition: [],
  ekfGlobalPosition: [],
};
let lastEstimatorLogTime = 0;

const telemetry = [
  { id: "gps", title: "GPS 위성", value: "—", unit: "개", footer: "PX4 기체 GPS", icon: "gps" },
  { id: "throttle", title: "모터 출력", value: "—", unit: "%", footer: "MAVROS 모터 출력 평균", icon: "gauge" },
  { id: "altitude", title: "고도", value: "—", unit: "m", footer: "지역 위치 기준", icon: "altitude" },
  { id: "airspeed", title: "지상속도", value: "—", unit: "m/s", footer: "PX4 NED 수평속도", icon: "wind" },
];

const icons = {
  camera: '<rect x="3" y="6" width="14" height="12" rx="2"/><path d="m17 10 4-2v8l-4-2M8 6l1-2h4l1 2"/>',
  target: '<circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>',
  compass: '<circle cx="12" cy="12" r="9"/><path d="m15 9-2 5-5 2 2-5z"/>',
  grip: '<path d="M7 4v6M17 4v6M7 7h10M9 10v7l3 3 3-3v-7"/>',
  upload: '<path d="M12 16V4m0 0L7 9m5-5 5 5M5 15v5h14v-5"/>',
  box: '<path d="m4 7 8-4 8 4-8 4zM4 7v10l8 4 8-4V7M12 11v10"/>',
  gps: '<path d="M12 21s6-5.3 6-11a6 6 0 1 0-12 0c0 5.7 6 11 6 11z"/><circle cx="12" cy="10" r="2"/>',
  signal: '<path d="M4 18v2M8 14v6M12 10v10M16 6v14M20 3v17"/>',
  gauge: '<path d="M5 18a8 8 0 1 1 14 0M12 14l4-4"/><circle cx="12" cy="14" r="1"/>',
  altitude: '<path d="M12 21V3m0 0L7 8m5-5 5 5M5 20h14"/>',
  wind: '<path d="M3 8h11c4 0 4-5 1-5-2 0-3 1-3 2M3 12h16c3 0 3 5 0 5-2 0-2-1-2-2M3 16h9"/>',
};

const svg = (name) => `<svg viewBox="0 0 24 24" aria-hidden="true">${icons[name]}</svg>`;

function renderTelemetry() {
  const grid = document.querySelector("#telemetryGrid");
  grid.innerHTML = telemetry.map((item) => `
    <article class="telemetry-card ${item.good ? "good" : ""}">
      <header><span>${item.title}</span>${svg(item.icon)}</header>
      <div class="telemetry-value"><strong id="${item.id}Value">${item.value}</strong><span>${item.unit}</span></div>
      <footer>${item.footer}</footer>
    </article>
  `).join("");
}

function renderWaypoints() {
  document.querySelector("#waypoints").innerHTML = "";
}

let toastTimer;
function showToast(message) {
  const toast = document.querySelector("#toast");
  toast.textContent = message;
  toast.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("show"), 1800);
}

function showConfirm(message) {
  const backdrop = document.querySelector("#confirmBackdrop");
  const messageElement = document.querySelector("#confirmMessage");
  const accept = document.querySelector("#confirmAccept");
  const cancel = document.querySelector("#confirmCancel");
  messageElement.textContent = message;
  backdrop.hidden = false;
  accept.focus();

  return new Promise((resolve) => {
    const finish = (result) => {
      backdrop.hidden = true;
      accept.removeEventListener("click", acceptDialog);
      cancel.removeEventListener("click", cancelDialog);
      backdrop.removeEventListener("click", clickBackdrop);
      document.removeEventListener("keydown", pressEscape);
      resolve(result);
    };
    const acceptDialog = () => finish(true);
    const cancelDialog = () => finish(false);
    const clickBackdrop = (event) => {
      if (event.target === backdrop) finish(false);
    };
    const pressEscape = (event) => {
      if (event.key === "Escape") finish(false);
    };
    accept.addEventListener("click", acceptDialog);
    cancel.addEventListener("click", cancelDialog);
    backdrop.addEventListener("click", clickBackdrop);
    document.addEventListener("keydown", pressEscape);
  });
}

function bindControls() {
  document.querySelector("#modeCard").addEventListener("click", () => {
    const display = document.querySelector("#flightMode");
    if (!sitlConnected || !window.gcsBridge) {
      showToast("PX4 연결 후 비행 모드를 변경할 수 있습니다.");
      return;
    }
    const nextIndex = (flightModes.indexOf(currentFlightMode) + 1) % flightModes.length;
    const nextMode = flightModes[nextIndex];

    window.gcsBridge.sendCommand({ type: "flightMode", mode: nextMode });
    showToast(`PX4 모드 변경 요청: ${nextMode}`);
  });

  document.querySelector("#vtolControl").addEventListener("click", (event) => {
    const button = event.target.closest("button");
    if (!button) return;
    if (!sitlConnected || !window.gcsBridge) {
      showToast("PX4 연결 후 VTOL 상태를 변경할 수 있습니다.");
      return;
    }
    event.currentTarget.querySelectorAll("button").forEach((item) => item.classList.remove("active"));
    button.classList.add("active");
    if (button.dataset.state !== "TRANSITION") {
      window.gcsBridge.sendCommand({ type: "vtolState", state: button.dataset.state });
    }
    const stateLabels = { FW: "고정익", TRANSITION: "전환", MC: "멀티콥터" };
    showToast(`VTOL 상태: ${stateLabels[button.dataset.state]}`);
  });

  document.querySelector(".view-tabs").addEventListener("click", (event) => {
    const button = event.target.closest("button");
    if (!button) return;
    document.querySelectorAll(".view-tabs button").forEach((item) => item.classList.toggle("active", item === button));
    document.querySelectorAll(".view").forEach((view) => view.classList.toggle("active", view.dataset.panel === button.dataset.view));

    if (button.dataset.view === "qgc") {
      launchQGroundControl();
      setTimeout(() => qgcMap?.invalidateSize(), 50);
    }
  });

  document.querySelector("#launchQgcButton").addEventListener("click", launchQGroundControl);
  document.querySelector("#selectPlanButton").addEventListener("click", selectMissionPlan);
  document.querySelector("#trayPlanButton").addEventListener("click", createTrayLandingPlan);
  document.querySelector("#uploadPlanButton").addEventListener("click", uploadMissionPlan);
  document.querySelector("#startPlanButton").addEventListener("click", startMissionPlan);
  document.querySelector("#qgcMirrorToggle").addEventListener("click", toggleQgcMirror);
  document.querySelector("#minimizeWindow").addEventListener("click", () => {
    window.gcsBridge?.minimizeWindow();
  });
  document.querySelector("#maximizeWindow").addEventListener("click", () => {
    window.gcsBridge?.toggleMaximizeWindow();
  });
  document.querySelector("#closeWindow").addEventListener("click", () => {
    window.gcsBridge?.closeWindow();
  });
}

async function launchQGroundControl() {
  if (!window.gcsBridge) return;

  const button = document.querySelector("#launchQgcButton");
  button.disabled = true;
  button.textContent = "시작 중...";

  const result = await window.gcsBridge.launchQgc();

  if (!result.ok) {
    button.disabled = false;
    button.textContent = "QGroundControl 다시 시도";
    showToast(result.error);
  }
}

function updateQgcStatus({ status, detail }) {
  const statusLabel = document.querySelector("#qgcStatus");
  const detailLabel = document.querySelector("#qgcStatusDetail");
  const statusDot = document.querySelector("#qgcStatusDot");
  const button = document.querySelector("#launchQgcButton");
  const running = status === "running";

  const statusLabels = {
    running: "실행 중",
    starting: "시작 중",
    stopped: "종료됨",
    error: "오류",
    standby: "대기 중",
  };
  const detailLabels = {
    "Opening QGroundControl": "QGroundControl을 여는 중",
    "QGroundControl closed": "QGroundControl이 종료됨",
  };
  statusLabel.textContent = statusLabels[status] || status;
  detailLabel.textContent = detailLabels[detail] || detail;
  statusDot.classList.toggle("online", running);
  button.disabled = running || status === "starting";
  button.textContent = running
    ? "QGC 실행 중"
    : status === "starting"
      ? "시작 중..."
      : "QGroundControl 열기";
}

function updateQgcFrame(frame) {
  const mirror = document.querySelector("#qgcMirror");
  const frameUrl = URL.createObjectURL(
    new Blob([frame], { type: "image/jpeg" }),
  );
  mirror.src = frameUrl;

  if (previousQgcFrameUrl) {
    URL.revokeObjectURL(previousQgcFrameUrl);
  }
  previousQgcFrameUrl = frameUrl;
}

function toggleQgcMirror() {
  const mapView = document.querySelector(".map-view");
  const button = document.querySelector("#qgcMirrorToggle");
  const active = mapView.classList.toggle("mirror-active");
  button.textContent = active ? "계획 지도 보기" : "QGC 실시간 보기";
}

function setPlanBusy(busy) {
  document.querySelector("#selectPlanButton").disabled = busy;
  document.querySelector("#trayPlanButton").disabled = busy;
  document.querySelector("#uploadPlanButton").disabled = busy || selectedPlanWaypoints.length === 0;
  document.querySelector("#startPlanButton").disabled = busy || selectedPlanWaypoints.length === 0;
}

function updatePlanStatus({ status, message }) {
  const statusLabel = document.querySelector("#planStatus");
  const statusDot = document.querySelector("#planStatusDot");
  const busyStatuses = ["connecting", "uploading", "starting"];
  const error = status === "error";
  const ready = ["ready", "connected", "uploaded", "started"].includes(status);

  statusLabel.textContent = message;
  statusDot.className = error
    ? "error"
    : busyStatuses.includes(status)
      ? "busy"
      : ready
        ? "ready"
        : "";
  setPlanBusy(busyStatuses.includes(status));

  if (status === "started") {
    showPlanMap();
    showToast("QGC AUTO.MISSION이 시작되었습니다.");
  }
}

function applySelectedPlan(plan) {
  selectedPlanWaypoints = plan.waypoints.map((waypoint) => ({
    ...waypoint,
    label: `경유점 ${waypoint.sequence}`,
  }));
  document.querySelector("#planCount").textContent = `경유점 ${plan.count}개`;
  document.querySelector("#planFileName").textContent = plan.path.split("/").pop();
  updatePlanStatus(plan);
  setPlanBusy(false);

  if (lastSitlTelemetry) {
    renderMissionMap(lastSitlTelemetry);
  }
}

async function selectMissionPlan() {
  setPlanBusy(true);
  const response = await window.gcsBridge.selectMissionPlan();

  if (response.canceled) {
    setPlanBusy(false);
    return;
  }
  if (!response.ok) {
    updatePlanStatus({ status: "error", message: response.error });
    return;
  }

  applySelectedPlan(response.result);
}

async function createTrayLandingPlan() {
  setPlanBusy(true);
  const response = await window.gcsBridge.createTrayLandingPlan();
  if (!response.ok) {
    updatePlanStatus({ status: "error", message: response.error });
    return;
  }
  applySelectedPlan(response.result);
  showToast("트레이 착륙 계획을 생성했습니다.");
}

async function uploadMissionPlan() {
  setPlanBusy(true);
  const response = await window.gcsBridge.uploadMissionPlan();
  if (!response.ok) {
    updatePlanStatus({ status: "error", message: response.error });
  }
}

async function startMissionPlan() {
  setPlanBusy(true);
  const confirmed = await showConfirm(
    "선택한 미션으로 기체를 ARM하고 AUTO.MISSION을 시작할까요?",
  );
  if (!confirmed) {
    setPlanBusy(false);
    return;
  }

  const response = await window.gcsBridge.startMissionPlan();
  if (!response.ok) {
    updatePlanStatus({ status: "error", message: response.error });
    return;
  }

  showPlanMap();
}

function showPlanMap() {
  const qgcTab = document.querySelector('[data-view="qgc"]');
  document.querySelectorAll(".view-tabs button").forEach((button) => {
    button.classList.toggle("active", button === qgcTab);
  });
  document.querySelectorAll(".view").forEach((view) => {
    view.classList.toggle("active", view.dataset.panel === "qgc");
  });

  const mapView = document.querySelector(".map-view");
  mapView.classList.remove("mirror-active");
  document.querySelector("#qgcMirrorToggle").textContent = "QGC 실시간 보기";
  setTimeout(() => qgcMap?.invalidateSize(), 50);
}

function startClock() {
  const clock = document.querySelector("#clock");
  const update = () => {
    clock.textContent = new Intl.DateTimeFormat("ko-KR", {
      hour: "2-digit", minute: "2-digit", second: "2-digit", hour12: false,
    }).format(new Date());
  };
  update();
  setInterval(update, 1000);
}

function updateSitlTelemetry(data) {
  lastSitlTelemetry = data;
  sitlConnected = Boolean(data.connected);

  const connection = document.querySelector(".connection-pill");
  const connectionLabel = document.querySelector("#connectionLabel");
  connection.classList.toggle("sitl", sitlConnected);
  connectionLabel.textContent = sitlConnected ? "SITL 연결됨" : "연결 대기";

  if (!sitlConnected) {
    currentFlightMode = "";
    document.querySelector("#flightMode").textContent = "PX4 오프라인";
    ["gps", "throttle", "altitude", "airspeed"].forEach((id) => {
      document.querySelector(`#${id}Value`).textContent = "—";
    });
    document.querySelector("#hudAltitude").textContent = "—";
    document.querySelector("#hudSpeed").textContent = "—";
    return;
  }

  const altitude = Number(data.altitude || 0);
  const groundSpeed = Number(data.groundSpeed || 0);
  const throttle = Number(data.throttle || 0);
  const gpsSatellites = Number(data.gpsSatellites || 0);

  currentFlightMode = data.flightMode;
  document.querySelector("#flightMode").textContent =
    flightModeLabels[data.flightMode] || data.flightMode;
  document.querySelector("#altitudeValue").textContent = altitude.toFixed(1);
  document.querySelector("#airspeedValue").textContent = groundSpeed.toFixed(1);
  document.querySelector("#throttleValue").textContent = Math.round(throttle);
  document.querySelector("#gpsValue").textContent = gpsSatellites;
  document.querySelector("#hudAltitude").textContent = altitude.toFixed(1);
  document.querySelector("#hudSpeed").textContent = groundSpeed.toFixed(1);

  document.querySelectorAll("#vtolControl button").forEach((button) => {
    button.classList.toggle("active", button.dataset.state === data.vtolState);
  });

  renderMissionMap(data);
  updateSensorDisplay(data);
}

function updateSensorDisplay(data) {
  const sensor = data.sensor || {};
  const estimate = data.estimate || {};
  const missionControl = data.missionControl || {};
  const accelerometer = sensor.accelerometer || [0, 0, 0];
  const gyroscope = sensor.gyroscope || [0, 0, 0];
  const localPosition = estimate.localPosition || [0, 0, 0];
  const velocity = estimate.velocity || [0, 0, 0];

  ["X", "Y", "Z"].forEach((axis, index) => {
    document.querySelector(`#accel${axis}`).textContent = Number(accelerometer[index]).toFixed(5);
    document.querySelector(`#gyro${axis}`).textContent = Number(gyroscope[index]).toFixed(5);
  });

  const attitudeValues = {
    Roll: Number(estimate.roll || 0),
    Pitch: Number(estimate.pitch || 0),
    Yaw: Number(estimate.yaw || 0),
  };
  Object.entries(attitudeValues).forEach(([axis, value]) => {
    document.querySelector(`#estimate${axis}`).textContent = `${value.toFixed(3)}°`;
  });

  document.querySelector("#estimatePosition").textContent =
    `북 ${localPosition[0].toFixed(3)} · 동 ${localPosition[1].toFixed(3)} · 하 ${localPosition[2].toFixed(3)} m`;
  document.querySelector("#estimateVelocity").textContent =
    `북 ${velocity[0].toFixed(3)} · 동 ${velocity[1].toFixed(3)} · 하 ${velocity[2].toFixed(3)} m/s`;
  document.querySelector("#horizontalError").textContent =
    `${Number(estimate.horizontalError || 0).toFixed(2)} m`;
  document.querySelector("#verticalError").textContent =
    `${Number(estimate.verticalError || 0).toFixed(2)} m`;

  const clipping = Boolean(sensor.accelClipping || sensor.gyroClipping);
  const flags = {
    flagTilt: estimate.tiltAligned,
    flagYaw: estimate.yawAligned,
    flagGnss: estimate.gnssFusion,
    flagBaro: estimate.barometerFusion,
    flagDeadReckoning: estimate.deadReckoning,
    flagClipping: clipping,
  };
  Object.entries(flags).forEach(([id, active]) => {
    const element = document.querySelector(`#${id}`);
    element.classList.toggle("active", Boolean(active));
    element.classList.toggle(
      "warning",
      Boolean(active) && ["flagDeadReckoning", "flagClipping"].includes(id),
    );
  });

  const healthy = !estimate.inertialFault
    && !estimate.deadReckoning
    && !clipping
    && estimate.tiltAligned
    && estimate.yawAligned;
  const health = document.querySelector("#estimatorHealth");
  health.textContent = healthy ? "EKF 정상" : "추정기 확인";
  health.classList.toggle("healthy", healthy);
  const px4Timestamp = Number(
    data.sourceTimestamps?.sensorCombined
    || data.sourceTimestamps?.vehicleLocalPosition
    || 0,
  );
  document.querySelector("#px4SourceTime").textContent = px4Timestamp
    ? `PX4 시간 · ${(px4Timestamp / 1e6).toFixed(3)}초`
    : "PX4 시간 · 대기 중";
  const targetOffset = missionControl.targetOffset || [];
  const targetDistance = Number.isFinite(targetOffset[0])
    && Number.isFinite(targetOffset[1])
    ? Math.hypot(targetOffset[0], targetOffset[1])
    : Number.NaN;
  const yawError = missionControl.yawErrorDegrees;
  const missionDetails = [
    missionControl.state || "대기 중",
    Number.isFinite(targetDistance)
      ? `표적 ${targetDistance.toFixed(2)}m`
      : null,
    Number.isFinite(yawError)
      ? `Yaw ${yawError.toFixed(1)}°`
      : null,
  ].filter(Boolean);
  document.querySelector("#missionControlStatus").textContent =
    `Jetson 미션 · ${missionDetails.join(" · ")}`;

  updateSensorAnalysis(data, healthy);
}

function pushLimited(list, value, limit = 50) {
  list.push(value);
  if (list.length > limit) list.shift();
}

function pushTrackPoint(list, value, minimumDistance) {
  const previous = list[list.length - 1];
  if (
    previous
    && Math.hypot(...value.map((item, index) => item - previous[index]))
      < minimumDistance
  ) {
    return;
  }
  list.push(value);
}

function prepareCanvas(canvas) {
  const bounds = canvas.getBoundingClientRect();
  if (!bounds.width || !bounds.height) return null;
  const ratio = window.devicePixelRatio || 1;
  const width = Math.round(bounds.width * ratio);
  const height = Math.round(bounds.height * ratio);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { context, width: bounds.width, height: bounds.height };
}

function drawTimeChart(canvasId, samples) {
  const prepared = prepareCanvas(document.querySelector(`#${canvasId}`));
  if (!prepared) return;
  const { context, width, height } = prepared;
  context.clearRect(0, 0, width, height);
  const plot = { left: 46, right: width - 12, top: 12, bottom: height - 24 };
  const plotWidth = plot.right - plot.left;
  const plotHeight = plot.bottom - plot.top;
  context.strokeStyle = "#343b45";
  context.lineWidth = 1;
  [0.25, 0.5, 0.75].forEach((ratio) => {
    context.beginPath();
    context.moveTo(plot.left, plot.top + plotHeight * ratio);
    context.lineTo(plot.right, plot.top + plotHeight * ratio);
    context.stroke();
  });
  const extent = Math.max(
    0.01,
    ...samples.flat().map((value) => Math.abs(value)),
  );
  context.fillStyle = "#a0a0a0";
  context.font = "10px Consolas, monospace";
  context.textAlign = "right";
  context.fillText(extent.toFixed(2), plot.left - 7, plot.top + 4);
  context.fillText("0", plot.left - 7, plot.top + plotHeight / 2 + 4);
  context.fillText((-extent).toFixed(2), plot.left - 7, plot.bottom);
  context.textAlign = "left";
  context.fillText("-10s", plot.left, height - 6);
  context.textAlign = "right";
  context.fillText("NOW", plot.right, height - 6);
  if (samples.length < 2) return;
  ["#4daafc", "#73c991", "#f48771"].forEach((color, axis) => {
    context.beginPath();
    context.strokeStyle = color;
    context.lineWidth = 1.5;
    samples.forEach((sample, index) => {
      const x = plot.left + (index / (samples.length - 1)) * plotWidth;
      const y = plot.top + plotHeight / 2
        - (sample[axis] / extent) * (plotHeight * 0.46);
      if (index === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
  });
}

function globalToMeters(position, origin) {
  const earthRadius = 6378137;
  const latitudeRadians = origin[0] * Math.PI / 180;
  return [
    (position[0] - origin[0]) * Math.PI / 180 * earthRadius,
    (position[1] - origin[1]) * Math.PI / 180
      * earthRadius * Math.cos(latitudeRadians),
  ];
}

function drawGpsEstimateChart() {
  const prepared = prepareCanvas(document.querySelector("#gpsEstimateChart"));
  if (!prepared) return;
  const { context, width, height } = prepared;
  context.clearRect(0, 0, width, height);
  const globalPoints = [
    ...sensorHistory.rawGpsPosition,
    ...sensorHistory.ekfGlobalPosition,
  ];
  if (!globalPoints.length) return;

  const origin = globalPoints[0];
  const rawMeters = sensorHistory.rawGpsPosition.map((point) => (
    globalToMeters(point, origin)
  ));
  const ekfMeters = sensorHistory.ekfGlobalPosition.map((point) => (
    globalToMeters(point, origin)
  ));
  const allMeters = [...rawMeters, ...ekfMeters];
  const extent = Math.max(
    2,
    ...allMeters.flat().map((value) => Math.abs(value)),
  );
  const plot = { left: 48, right: width - 18, top: 18, bottom: height - 34 };
  const plotWidth = plot.right - plot.left;
  const plotHeight = plot.bottom - plot.top;
  const project = ([north, east]) => [
    plot.left + plotWidth / 2 + (east / extent) * plotWidth * 0.46,
    plot.top + plotHeight / 2 - (north / extent) * plotHeight * 0.46,
  ];

  context.strokeStyle = "#343b45";
  context.lineWidth = 1;
  context.beginPath();
  context.moveTo(plot.left + plotWidth / 2, plot.top);
  context.lineTo(plot.left + plotWidth / 2, plot.bottom);
  context.moveTo(plot.left, plot.top + plotHeight / 2);
  context.lineTo(plot.right, plot.top + plotHeight / 2);
  context.stroke();

  [
    [rawMeters, "#73c991", [5, 4]],
    [ekfMeters, "#4daafc", []],
  ].forEach(([points, color, dash]) => {
    if (points.length < 2) return;
    context.beginPath();
    context.strokeStyle = color;
    context.lineWidth = 2;
    context.setLineDash(dash);
    points.forEach((point, index) => {
      const [x, y] = project(point);
      if (index === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
    const [lastX, lastY] = project(points[points.length - 1]);
    context.setLineDash([]);
    context.beginPath();
    context.fillStyle = color;
    context.arc(lastX, lastY, 4, 0, Math.PI * 2);
    context.fill();
  });
  context.setLineDash([]);
  context.fillStyle = "#a0a0a0";
  context.font = "10px Consolas, monospace";
  context.fillText(`N ±${extent.toFixed(1)}m`, plot.left, plot.top - 5);
  context.textAlign = "right";
  context.fillText(`E ±${extent.toFixed(1)}m`, plot.right, plot.bottom + 15);
  context.textAlign = "left";
}

function appendEstimatorLog(data, healthy, trackingError) {
  const now = Date.now();
  if (now - lastEstimatorLogTime < 1000) return;
  lastEstimatorLogTime = now;
  const log = document.querySelector("#estimatorLog");
  log.querySelector(".empty-log")?.remove();
  const row = document.createElement("div");
  row.className = healthy ? "log-row" : "log-row warning";
  const px4Timestamp = Number(
    data.sourceTimestamps?.vehicleLocalPosition
    || data.sourceTimestamps?.sensorCombined
    || 0,
  );
  const time = px4Timestamp
    ? `${(px4Timestamp / 1e6).toFixed(3)}s`
    : "—";
  row.innerHTML = `
    <time>${time}</time>
    <span>${data.flightMode}</span>
    <strong>${healthy ? "EKF 정상" : "EKF 확인"}</strong>
    <code>${Number.isFinite(trackingError) ? `위치 Δ ${trackingError.toFixed(2)} m` : "목표 없음"}</code>
  `;
  log.prepend(row);
  while (log.children.length > 40) log.lastElementChild.remove();
}

function updateSensorAnalysis(data, healthy) {
  const sensor = data.sensor || {};
  const estimate = data.estimate || {};
  const setpoint = data.setpoint || {};
  const actual = estimate.localPosition || [0, 0, 0];
  const desired = setpoint.localPosition || [0, 0, 0];
  const gpsPosition = data.gpsPosition || {};
  const globalEstimate = estimate.globalPosition || [];
  const rawGpsValid = Number.isFinite(gpsPosition.latitude)
    && Number.isFinite(gpsPosition.longitude)
    && Number(data.gpsFix) >= 2;
  const globalEstimateValid = Number.isFinite(globalEstimate[0])
    && Number.isFinite(globalEstimate[1]);

  pushLimited(sensorHistory.accel, sensor.accelerometer || [0, 0, 0]);
  pushLimited(sensorHistory.gyro, sensor.gyroscope || [0, 0, 0]);
  if (rawGpsValid) {
    pushTrackPoint(
      sensorHistory.rawGpsPosition,
      [gpsPosition.latitude, gpsPosition.longitude],
      0.0000001,
    );
  }
  if (globalEstimateValid) {
    pushTrackPoint(
      sensorHistory.ekfGlobalPosition,
      [globalEstimate[0], globalEstimate[1]],
      0.0000001,
    );
  }

  const trackingError = setpoint.valid
    ? Math.hypot(
        Number(actual[0]) - Number(desired[0]),
        Number(actual[1]) - Number(desired[1]),
        Number(actual[2]) - Number(desired[2]),
      )
    : Number.NaN;
  const gpsEstimateSeparation = rawGpsValid && globalEstimateValid
    ? Math.hypot(
        ...globalToMeters(
          [globalEstimate[0], globalEstimate[1]],
          [gpsPosition.latitude, gpsPosition.longitude],
        ),
      )
    : Number.NaN;
  document.querySelector("#gpsEstimateError").textContent =
    Number.isFinite(gpsEstimateSeparation)
      ? `거리 ${gpsEstimateSeparation.toFixed(2)} m`
      : "거리 —";
  document.querySelector("#rawGpsPosition").textContent = rawGpsValid
    ? `${gpsPosition.latitude.toFixed(7)}, ${gpsPosition.longitude.toFixed(7)} · ${Number(gpsPosition.altitude).toFixed(2)}m`
    : "—";
  document.querySelector("#ekfGlobalPosition").textContent = globalEstimateValid
    ? `${Number(globalEstimate[0]).toFixed(7)}, ${Number(globalEstimate[1]).toFixed(7)} · ${Number(globalEstimate[2]).toFixed(2)}m`
    : "—";

  drawTimeChart("accelChart", sensorHistory.accel);
  drawTimeChart("gyroChart", sensorHistory.gyro);
  drawGpsEstimateChart();
  appendEstimatorLog(data, healthy, trackingError);
}

function renderMissionMap(data) {
  const activeMissionPoints = Array.isArray(data.missionWaypoints)
    ? data.missionWaypoints
    : [];
  const missionPoints = selectedPlanWaypoints.length
    ? selectedPlanWaypoints
    : activeMissionPoints;
  const vehicleValid = Number.isFinite(data.latitude)
    && Number.isFinite(data.longitude);

  if (!vehicleValid && missionPoints.length === 0) return;

  renderActualMap(missionPoints, data, vehicleValid);

  const geographicPoints = [
    ...missionPoints,
    ...(vehicleValid
      ? [{
          label: "VEHICLE",
          latitude: data.latitude,
          longitude: data.longitude,
        }]
      : []),
  ];
  const latitudes = geographicPoints.map((point) => point.latitude);
  const longitudes = geographicPoints.map((point) => point.longitude);
  const latitudePadding = Math.max(
    (Math.max(...latitudes) - Math.min(...latitudes)) * 0.15,
    0.00005,
  );
  const longitudePadding = Math.max(
    (Math.max(...longitudes) - Math.min(...longitudes)) * 0.15,
    0.00005,
  );
  const minLatitude = Math.min(...latitudes) - latitudePadding;
  const maxLatitude = Math.max(...latitudes) + latitudePadding;
  const minLongitude = Math.min(...longitudes) - longitudePadding;
  const maxLongitude = Math.max(...longitudes) + longitudePadding;

  const project = (point) => ({
    x: 70 + ((point.longitude - minLongitude) / (maxLongitude - minLongitude)) * 760,
    y: 430 - ((point.latitude - minLatitude) / (maxLatitude - minLatitude)) * 360,
  });
  const projectedMission = missionPoints.map((point) => ({
    ...point,
    ...project(point),
  }));

  document.querySelector("#missionRoute").setAttribute(
    "d",
    projectedMission.length > 1
      ? `M ${projectedMission.map((point) => `${point.x},${point.y}`).join(" L ")}`
      : "",
  );
  document.querySelector("#waypoints").innerHTML = projectedMission
    .map((point, index) => `
      <circle cx="${point.x}" cy="${point.y}" r="7"></circle>
      <text x="${point.x + 11}" y="${point.y + 4}">${index + 1} · ${point.label}</text>
    `)
    .join("");

  if (vehicleValid) {
    const vehicle = project({
      latitude: data.latitude,
      longitude: data.longitude,
    });
    const marker = document.querySelector(".drone-marker");
    marker.style.left = `${(vehicle.x / 900) * 100}%`;
    marker.style.top = `${(vehicle.y / 500) * 100}%`;
  }

  document.querySelector("#mapLabel").textContent = missionPoints.length
    ? `QGC 미션 · 경유점 ${missionPoints.length}개`
    : "QGC 연결됨 · 활성 미션 없음";
}

function initializeActualMap() {
  if (!window.L) return;

  qgcMap = L.map("qgcMap", {
    zoomControl: true,
    attributionControl: true,
    minZoom: 2,
    maxZoom: 22,
    worldCopyJump: true,
    preferCanvas: true,
  }).setView([47.39801, 8.546162], 17);

  L.tileLayer(
    "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    {
      maxNativeZoom: 19,
      maxZoom: 22,
      keepBuffer: 4,
      errorTileUrl: "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='256' height='256'%3E%3Crect width='256' height='256' fill='%2316191d'/%3E%3Cpath d='M0 0H256M0 64H256M0 128H256M0 192H256M0 256H256M0 0V256M64 0V256M128 0V256M192 0V256M256 0V256' stroke='%23262b31'/%3E%3C/svg%3E",
      attribution: "© OpenStreetMap contributors",
    },
  ).addTo(qgcMap);

  qgcMissionLayer = L.layerGroup().addTo(qgcMap);
}

function renderActualMap(missionPoints, data, vehicleValid) {
  if (!qgcMap) return;

  const reference = data.estimate?.referencePosition;
  const tray = data.simulation?.trayPosition;
  if (
    !qgcTrayMarker
    && Array.isArray(reference)
    && Array.isArray(tray)
    && Number.isFinite(reference[0])
    && Number.isFinite(reference[1])
  ) {
    const earthRadius = 6378137;
    const trayPosition = [
      reference[0] + (tray[0] / earthRadius) * (180 / Math.PI),
      reference[1]
        + (tray[1] / (
          earthRadius * Math.cos(reference[0] * Math.PI / 180)
        )) * (180 / Math.PI),
    ];
    qgcTrayMarker = L.marker(trayPosition, {
      icon: L.divIcon({
        className: "",
        html: '<div class="tray-map-marker"><i></i></div>',
        iconSize: [28, 22],
        iconAnchor: [14, 11],
      }),
      zIndexOffset: 900,
    }).addTo(qgcMap).bindTooltip("생존자 트레이 · 착륙 지점");
  }

  if (vehicleValid) {
    const vehiclePosition = [data.latitude, data.longitude];
    const heading = Number.isFinite(Number(data.heading))
      ? Number(data.heading)
      : 0;
    if (!qgcVehicleMarker) {
      qgcVehicleMarker = L.marker(vehiclePosition, {
        icon: L.divIcon({
          className: "",
          html: `<div class="vehicle-map-marker" style="transform:rotate(${heading}deg)"><i></i></div>`,
          iconSize: [22, 22],
          iconAnchor: [11, 11],
        }),
        zIndexOffset: 1000,
      }).addTo(qgcMap).bindTooltip("PX4 VEHICLE");
    } else {
      qgcVehicleMarker.setLatLng(vehiclePosition);
    }
    const markerElement = qgcVehicleMarker.getElement()
      ?.querySelector(".vehicle-map-marker");
    if (markerElement) {
      markerElement.style.transform = `rotate(${heading}deg)`;
    }
  }

  const missionKey = missionPoints
    .map((point) => `${point.latitude},${point.longitude},${point.altitude}`)
    .join("|");
  if (missionKey === qgcMissionKey) return;

  qgcMissionKey = missionKey;
  qgcMissionLayer.clearLayers();

  const route = missionPoints.map((point) => [
    point.latitude,
    point.longitude,
  ]);
  if (route.length > 1) {
    L.polyline(route, {
      color: "#007acc",
      weight: 3,
      opacity: 0.9,
    }).addTo(qgcMissionLayer);
  }

  missionPoints.forEach((point, index) => {
    L.marker([point.latitude, point.longitude], {
      icon: L.divIcon({
        className: "",
        html: `<div class="waypoint-map-marker">${index + 1}</div>`,
        iconSize: [22, 22],
        iconAnchor: [11, 11],
      }),
    })
      .addTo(qgcMissionLayer)
      .bindTooltip(
        `${point.label || `WP ${index + 1}`} · ${Number(point.altitude || 0).toFixed(1)}m`,
      );
  });

  if (route.length) {
    const bounds = L.latLngBounds(route);
    if (vehicleValid) {
      bounds.extend([data.latitude, data.longitude]);
    }
    qgcMap.fitBounds(bounds, { padding: [55, 55], maxZoom: 18 });
  }
}

let previousCameraUrl;
function updateCameraFrame(frame) {
  const camera = document.querySelector("#gazeboCamera");
  const blob = new Blob([frame], { type: "image/jpeg" });
  const frameUrl = URL.createObjectURL(blob);

  updateCameraStatus(true);
  camera.src = frameUrl;

  if (previousCameraUrl) {
    URL.revokeObjectURL(previousCameraUrl);
  }

  previousCameraUrl = frameUrl;
}

function updateCameraStatus(connected) {
  document.querySelector(".camera-view").classList.toggle(
    "camera-online",
    connected,
  );
  document.querySelector("#cameraMeta").textContent = connected
    ? "Gazebo 하향 카메라 · YOLO 트레이 탐지"
    : "시뮬레이션 · 대기 중";
}

/*
 * 실기체 연동 지점:
 * rosbridge_server를 실행한 뒤 WebSocket 메시지의 nav_state를 아래 모드로 변환하고,
 * #flightMode와 각 telemetry value를 갱신하면 됩니다.
 * 원본 qcs.py 토픽: /fmu/out/vehicle_status
 */
function navStateToMode(navState) {
  const px4Modes = { 3: "POSITION", 4: "MISSION", 5: "HOLD", 6: "RETURN", 12: "OFFBOARD", 17: "TAKEOFF", 18: "LAND" };
  return px4Modes[navState] ?? `MODE ${navState}`;
}

renderTelemetry();
renderWaypoints();
initializeActualMap();
bindControls();
startClock();

if (window.gcsBridge) {
  window.gcsBridge.onTelemetry(updateSitlTelemetry);
  window.gcsBridge.onCameraFrame(updateCameraFrame);
  window.gcsBridge.onCameraStatus(updateCameraStatus);
  window.gcsBridge.onQgcStatus(updateQgcStatus);
  window.gcsBridge.onQgcFrame(updateQgcFrame);
  window.gcsBridge.onMissionStatus(updatePlanStatus);
  window.gcsBridge.onWindowMaximized((maximized) => {
    document.body.classList.toggle("window-maximized", maximized);
  });
}

window.ArecadaGCS = { navStateToMode };
