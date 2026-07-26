const {
  app,
  BrowserWindow,
  desktopCapturer,
  dialog,
  ipcMain,
  shell,
} = require("electron");
const { spawn } = require("node:child_process");
const http = require("node:http");
const os = require("node:os");
const path = require("node:path");
const fs = require("node:fs");

const projectRoot = path.join(__dirname, "..");
const swaggerAssets = path.join(projectRoot, "node_modules", "swagger-ui-dist");
const koreanFontAssets = path.join(
  projectRoot,
  "node_modules",
  "@fontsource",
  "noto-sans-kr",
);
const jetsonGcsUrl = (
  process.env.JETSON_GCS_URL
  || "http://192.168.144.26:8765"
).replace(/\/+$/, "");

if (process.platform === "linux") {
  app.disableHardwareAcceleration();
  app.commandLine.appendSwitch(
    "ozone-platform-hint",
    process.env.ELECTRON_OZONE_PLATFORM_HINT || "auto",
  );
  app.commandLine.appendSwitch("disable-vulkan");
  app.commandLine.appendSwitch(
    "disable-features",
    "VaapiVideoDecoder,VaapiVideoEncoder,WebRTCPipeWireCapturer",
  );
}

const contentTypes = {
  ".css": "text/css; charset=utf-8",
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".png": "image/png",
  ".svg": "image/svg+xml",
  ".woff": "font/woff",
  ".woff2": "font/woff2",
  ".yaml": "application/yaml; charset=utf-8",
  ".yml": "application/yaml; charset=utf-8",
};

let server;
let rosBridge;
let cameraBridge;
let qgcProcess;
let mavrosProcess;
let missionProcess;
let selectedPlanPath;
let selectedTrayLandingTarget;
let qgcCaptureTimer;
let qgcCaptureBusy = false;
let qgcCaptureReady = false;
let qgcCaptureSourcesLogged = false;
let lastTelemetry;
let jetsonPollTimer;
let jetsonRequestActive = false;
let lastJetsonConnectionState;

function resolveRequestPath(requestUrl) {
  const url = new URL(requestUrl, "http://127.0.0.1");
  const decodedPath = decodeURIComponent(url.pathname);

  if (decodedPath.startsWith("/vendor/swagger-ui/")) {
    const fileName = decodedPath.slice("/vendor/swagger-ui/".length);
    return path.join(swaggerAssets, path.basename(fileName));
  }

  if (decodedPath.startsWith("/vendor/leaflet/")) {
    const fileName = decodedPath.slice("/vendor/leaflet/".length);
    const leafletAssets = path.join(projectRoot, "node_modules", "leaflet", "dist");
    return path.join(leafletAssets, path.basename(fileName));
  }

  if (decodedPath.startsWith("/vendor/noto-sans-kr/")) {
    const relativeFontPath = decodedPath.slice("/vendor/noto-sans-kr/".length);
    const requestedFontPath = path.resolve(koreanFontAssets, relativeFontPath);
    const relativeToFontRoot = path.relative(
      koreanFontAssets,
      requestedFontPath,
    );
    if (
      relativeToFontRoot.startsWith("..")
      || path.isAbsolute(relativeToFontRoot)
    ) {
      return null;
    }
    return requestedFontPath;
  }

  const relativePath = decodedPath === "/" ? "index.html" : decodedPath.slice(1);
  const requestedPath = path.resolve(projectRoot, relativePath);
  const relativeToRoot = path.relative(projectRoot, requestedPath);

  if (relativeToRoot.startsWith("..") || path.isAbsolute(relativeToRoot)) {
    return null;
  }

  return requestedPath;
}

function serveFile(request, response) {
  const filePath = resolveRequestPath(request.url);

  if (!filePath) {
    response.writeHead(403).end("Forbidden");
    return;
  }

  fs.stat(filePath, (statError, stat) => {
    if (statError || !stat.isFile()) {
      response.writeHead(404).end("Not found");
      return;
    }

    const contentType = contentTypes[path.extname(filePath)] || "application/octet-stream";
    response.writeHead(200, {
      "Content-Type": contentType,
      "Cache-Control": "no-store",
      "X-Content-Type-Options": "nosniff",
    });

    if (request.method === "HEAD") {
      response.end();
      return;
    }

    fs.createReadStream(filePath).pipe(response);
  });
}

function startServer() {
  return new Promise((resolve, reject) => {
    server = http.createServer(serveFile);
    server.once("error", reject);
    server.listen(0, "127.0.0.1", () => {
      const address = server.address();
      resolve(`http://127.0.0.1:${address.port}`);
    });
  });
}

function createWindow(baseUrl) {
  const window = new BrowserWindow({
    width: 1280,
    height: 760,
    minWidth: 920,
    minHeight: 620,
    backgroundColor: "#edf3f5",
    frame: false,
    autoHideMenuBar: true,
    title: "ARECADA GCS",
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: true,
      preload: path.join(__dirname, "preload.js"),
    },
  });

  window.loadURL(baseUrl);
  const sendMaximizedState = () => {
    window.webContents.send("window:maximized", window.isMaximized());
  };
  window.on("maximize", sendMaximizedState);
  window.on("unmaximize", sendMaximizedState);

  window.webContents.setWindowOpenHandler(({ url }) => {
    if (!url.startsWith(baseUrl)) {
      shell.openExternal(url);
      return { action: "deny" };
    }

    return { action: "allow" };
  });
}

function windowFromEvent(event) {
  return BrowserWindow.fromWebContents(event.sender);
}

ipcMain.on("window:minimize", (event) => {
  windowFromEvent(event)?.minimize();
});

ipcMain.on("window:toggle-maximize", (event) => {
  const window = windowFromEvent(event);
  if (!window) return;
  if (window.isMaximized()) {
    window.unmaximize();
  } else {
    window.maximize();
  }
});

ipcMain.on("window:close", (event) => {
  windowFromEvent(event)?.close();
});

function startRosBridge() {
  const bridgePath = path.join(projectRoot, "bridge", "px4_bridge.py");

  rosBridge = spawn("python3", [bridgePath], {
    cwd: projectRoot,
    stdio: ["pipe", "pipe", "pipe"],
    env: {
      ...process.env,
      PYTHONUNBUFFERED: "1",
    },
  });

  let bufferedOutput = "";

  rosBridge.stdout.on("data", (chunk) => {
    bufferedOutput += chunk.toString();
    const lines = bufferedOutput.split("\n");
    bufferedOutput = lines.pop();

    for (const line of lines) {
      try {
        const message = JSON.parse(line);
        lastTelemetry = message;
        BrowserWindow.getAllWindows().forEach((window) => {
          window.webContents.send("px4:telemetry", message);
        });
      } catch {
        continue;
      }
    }
  });

  rosBridge.stderr.on("data", (chunk) => {
    console.error(`[PX4 bridge] ${chunk.toString().trim()}`);
  });

  rosBridge.on("error", (error) => {
    console.error(`[PX4 bridge] Failed to start: ${error.message}`);
  });
}

function startMavros() {
  mavrosProcess = spawn(
    "ros2",
    [
      "run",
      "mavros",
      "mavros_node",
      "--ros-args",
      "--params-file",
      "/opt/ros/humble/share/mavros/launch/px4_config.yaml",
      "-p",
      "fcu_url:=udp://:14540@127.0.0.1:14580",
    ],
    {
      cwd: projectRoot,
      stdio: ["ignore", "ignore", "pipe"],
      env: process.env,
    },
  );

  mavrosProcess.stderr.on("data", (chunk) => {
    const message = chunk.toString();
    if (message.includes("ERROR") || message.includes("FATAL")) {
      console.error(`[MAVROS] ${message.trim()}`);
    }
  });

  mavrosProcess.on("error", (error) => {
    console.error(`[MAVROS] Failed to start: ${error.message}`);
  });
}

function broadcast(channel, payload) {
  BrowserWindow.getAllWindows().forEach((window) => {
    window.webContents.send(channel, payload);
  });
}

async function jetsonRequest(requestPath, options = {}) {
  const headers = {
    Accept: "application/json",
    ...(options.body ? { "Content-Type": "application/json" } : {}),
  };
  const response = await fetch(`${jetsonGcsUrl}${requestPath}`, {
    ...options,
    headers: {
      ...headers,
      ...(options.headers || {}),
    },
    signal: AbortSignal.timeout(options.timeout || 5000),
  });
  const payload = await response.json();
  if (!response.ok) {
    throw new Error(payload.error || `Jetson HTTP ${response.status}`);
  }
  return payload;
}

function startJetsonTelemetry() {
  if (!jetsonGcsUrl || jetsonPollTimer) return;

  const logConnectionState = (state, detail) => {
    if (lastJetsonConnectionState === state) return;
    lastJetsonConnectionState = state;

    if (state === "connected") {
      console.log(`[Jetson gateway] PX4 DDS connected via ${jetsonGcsUrl}`);
    } else if (state === "waiting") {
      console.warn(
        `[Jetson gateway] Gateway reachable at ${jetsonGcsUrl}, `
        + "waiting for PX4 DDS messages",
      );
    } else {
      console.error(`[Jetson gateway] Unreachable at ${jetsonGcsUrl}: ${detail}`);
    }
  };

  const poll = async () => {
    if (jetsonRequestActive) return;
    jetsonRequestActive = true;
    try {
      const telemetry = await jetsonRequest("/status", { timeout: 1500 });
      const enrichedTelemetry = {
        ...telemetry,
        gatewayConnected: true,
        gatewayUrl: jetsonGcsUrl,
      };
      lastTelemetry = enrichedTelemetry;
      logConnectionState(
        enrichedTelemetry.connected ? "connected" : "waiting",
      );
      broadcast("px4:telemetry", enrichedTelemetry);
    } catch (error) {
      logConnectionState("unreachable", error.message);
      broadcast("px4:telemetry", {
        type: "telemetry",
        connected: false,
        gatewayConnected: false,
        gatewayUrl: jetsonGcsUrl,
        gatewayError: error.message,
      });
    } finally {
      jetsonRequestActive = false;
    }
  };

  poll();
  jetsonPollTimer = setInterval(poll, 250);
}

function startCameraBridge() {
  const bridgePath = path.join(
    projectRoot,
    "bridge",
    "gazebo_camera_bridge.py",
  );
  const virtualEnvironmentPython = path.join(
    projectRoot,
    ".venv",
    "bin",
    "python",
  );
  const cameraPython = fs.existsSync(virtualEnvironmentPython)
    ? virtualEnvironmentPython
    : "python3";

  cameraBridge = spawn(cameraPython, [bridgePath], {
    cwd: projectRoot,
    stdio: ["ignore", "pipe", "pipe"],
    env: {
      ...process.env,
      MPLCONFIGDIR: path.join(projectRoot, ".generated", "matplotlib"),
      TRAY_YOLO_MODEL: path.join(projectRoot, "best.pt"),
    },
  });

  let frameBuffer = Buffer.alloc(0);
  let cameraConnected = false;

  cameraBridge.stdout.on("data", (chunk) => {
    frameBuffer = Buffer.concat([frameBuffer, chunk]);

    while (frameBuffer.length >= 4) {
      const frameLength = frameBuffer.readUInt32BE(0);

      if (frameBuffer.length < frameLength + 4) {
        break;
      }

      const frame = frameBuffer.subarray(4, frameLength + 4);
      frameBuffer = frameBuffer.subarray(frameLength + 4);

      if (!cameraConnected) {
        cameraConnected = true;
        console.log(
          `[Gazebo camera] Streaming JPEG frames (${frame.length} bytes first frame)`,
        );
        broadcast("gazebo:camera-status", true);
      }

      broadcast("gazebo:camera-frame", frame);
    }
  });

  cameraBridge.stderr.on("data", (chunk) => {
    console.log(`[Gazebo camera] ${chunk.toString().trim()}`);
  });

  cameraBridge.on("error", (error) => {
    console.error(`[Gazebo camera] Failed to start: ${error.message}`);
    broadcast("gazebo:camera-status", false);
  });

  cameraBridge.on("exit", () => {
    broadcast("gazebo:camera-status", false);
  });
}

function qgcExecutablePath() {
  if (process.env.QGC_PATH) {
    return process.env.QGC_PATH;
  }

  const homeDirectory = os.homedir();
  const candidates = [
    path.join(homeDirectory, "Downloads", "QGroundControl-x86_64.AppImage"),
    path.join(homeDirectory, "Downloads", "QGroundControl.AppImage"),
    path.join(homeDirectory, "apps", "QGroundControl.AppImage"),
  ];

  return candidates.find((candidate) => fs.existsSync(candidate))
    || candidates[0];
}

function launchQGroundControl() {
  if (qgcProcess && qgcProcess.exitCode === null) {
    broadcast("qgc:status", {
      status: "running",
      detail: "MAVLink UDP 14550",
    });
    return { ok: true, alreadyRunning: true };
  }

  const executable = qgcExecutablePath();

  if (!fs.existsSync(executable)) {
    const error = `QGroundControl not found: ${executable}`;
    broadcast("qgc:status", { status: "missing", detail: executable });
    return { ok: false, error };
  }

  broadcast("qgc:status", {
    status: "starting",
    detail: "Opening QGroundControl",
  });

  qgcProcess = spawn(executable, [], {
    cwd: path.dirname(executable),
    stdio: "ignore",
    env: {
      ...process.env,
      QT_QPA_PLATFORM: process.env.QT_QPA_PLATFORM || "xcb",
      QSG_RHI_BACKEND: process.env.QSG_RHI_BACKEND || "opengl",
    },
  });

  qgcProcess.once("spawn", () => {
    broadcast("qgc:status", {
      status: "running",
      detail: "MAVLink UDP 14550",
    });
  });

  qgcProcess.once("error", (error) => {
    broadcast("qgc:status", {
      status: "error",
      detail: error.message,
    });
  });

  qgcProcess.once("exit", () => {
    qgcProcess = null;
    broadcast("qgc:status", {
      status: "stopped",
      detail: "QGroundControl closed",
    });
  });

  return { ok: true, alreadyRunning: false };
}

function startQgcCapture() {
  if (qgcCaptureTimer) return;

  const isWaylandSession = (
    process.env.XDG_SESSION_TYPE === "wayland"
    || Boolean(process.env.WAYLAND_DISPLAY)
  );
  if (isWaylandSession && process.env.QGC_CAPTURE !== "1") {
    console.log(
      "[QGC capture] Disabled on Wayland to prevent repeated screen-share prompts. "
      + "Set QGC_CAPTURE=1 to enable it.",
    );
    return;
  }

  qgcCaptureTimer = setInterval(async () => {
    if (qgcCaptureBusy) {
      return;
    }

    qgcCaptureBusy = true;
    try {
      const sources = await desktopCapturer.getSources({
        types: ["window"],
        thumbnailSize: { width: 1280, height: 720 },
        fetchWindowIcons: false,
      });
      const qgcSource = sources.find((source) => (
        source.name.toLowerCase().includes("qgroundcontrol")
      ));

      if (!qgcCaptureSourcesLogged) {
        qgcCaptureSourcesLogged = true;
        console.log(
          `[QGC capture] Available windows: ${sources.map((source) => source.name).join(", ")}`,
        );
      }

      if (qgcSource && !qgcSource.thumbnail.isEmpty()) {
        if (!qgcCaptureReady) {
          qgcCaptureReady = true;
          console.log(`[QGC capture] Mirroring window: ${qgcSource.name}`);
        }
        broadcast(
          "qgc:frame",
          qgcSource.thumbnail.toJPEG(68),
        );
      }
    } catch (error) {
      console.error(`[QGC capture] ${error.message}`);
    } finally {
      qgcCaptureBusy = false;
    }
  }, 400);
}

ipcMain.handle("qgc:launch", () => launchQGroundControl());

function resolveTrayLandingTarget() {
  const reference = lastTelemetry?.estimate?.referencePosition;
  const tray = lastTelemetry?.simulation?.trayPosition || [5, 0, 0.04];
  if (
    !Array.isArray(reference)
    || !Number.isFinite(reference[0])
    || !Number.isFinite(reference[1])
  ) {
    throw new Error("PX4 로컬 위치 기준점을 아직 받지 못했습니다");
  }

  const earthRadius = 6378137;
  return {
    latitude: reference[0]
      + (tray[0] / earthRadius) * (180 / Math.PI),
    longitude: reference[1]
      + (tray[1] / (
        earthRadius * Math.cos(reference[0] * Math.PI / 180)
      )) * (180 / Math.PI),
    homeAltitude: Number.isFinite(reference[2]) ? reference[2] : 0,
    reference,
  };
}

function missionItem(command, sequence, latitude, longitude, altitude) {
  return {
    arecadaTrayLanding: true,
    Altitude: altitude,
    AltitudeMode: 1,
    autoContinue: true,
    command,
    doJumpId: sequence,
    frame: 3,
    params: [0, 0, 0, null, latitude, longitude, altitude],
    type: "SimpleItem",
  };
}

function saveTrayLandingPlan(plan, target) {
  const items = plan.mission?.items;
  if (!Array.isArray(items)) {
    throw new Error("QGC Plan에 미션 항목이 없습니다");
  }

  while (items.at(-1)?.arecadaTrayLanding) {
    items.pop();
  }
  if (items.at(-1)?.command === 21) {
    const landing = items.pop();
    const lowApproach = items.at(-1);
    const highApproach = items.at(-2);
    const sameTarget = (item) => (
      item?.command === 16
      && item.params?.[4] === landing.params?.[4]
      && item.params?.[5] === landing.params?.[5]
    );
    if (
      sameTarget(lowApproach)
      && sameTarget(highApproach)
      && lowApproach.params?.[6] === 2
      && highApproach.params?.[6] === 8
    ) {
      items.splice(-2);
    }
  }

  const sequence = items.length + 1;
  items.push(
    missionItem(16, sequence, target.latitude, target.longitude, 8),
    missionItem(16, sequence + 1, target.latitude, target.longitude, 2),
    missionItem(21, sequence + 2, target.latitude, target.longitude, 0),
  );
  items.forEach((item, index) => {
    item.doJumpId = index + 1;
  });

  const plansDirectory = path.join(projectRoot, "plans");
  fs.mkdirSync(plansDirectory, { recursive: true });
  selectedPlanPath = path.join(plansDirectory, "tray_landing.plan");
  fs.writeFileSync(
    selectedPlanPath,
    `${JSON.stringify(plan, null, 2)}\n`,
    "utf8",
  );
  selectedTrayLandingTarget = {
    latitude: target.latitude,
    longitude: target.longitude,
  };
}

function runMissionAdapter(action, planPath) {
  if (jetsonGcsUrl) {
    return fs.promises.readFile(planPath, "utf8")
      .then((encodedPlan) => jetsonRequest(
        `/mission/${action}`,
        {
          method: "POST",
          body: JSON.stringify({ plan: JSON.parse(encodedPlan) }),
          timeout: action === "inspect" ? 10000 : 125000,
        },
      ))
      .then((response) => {
        if (response.result) {
          broadcast("mission:status", response.result);
        }
        return response;
      })
      .catch((error) => ({
        ok: false,
        error: error.message,
      }));
  }

  return new Promise((resolve) => {
    if (missionProcess) {
      resolve({
        ok: false,
        error: "다른 미션 작업이 진행 중입니다",
      });
      return;
    }

    const adapterPath = path.join(
      projectRoot,
      "bridge",
      "mission_adapter.py",
    );
    let bufferedOutput = "";
    let lastMessage = null;

    missionProcess = spawn(
      "python3",
      [adapterPath, action, planPath],
      {
        cwd: projectRoot,
        stdio: ["ignore", "pipe", "pipe"],
        env: {
          ...process.env,
          ...(selectedTrayLandingTarget
            ? {
                ARECADA_TRAY_LANDING_TARGET: JSON.stringify(
                  selectedTrayLandingTarget,
                ),
              }
            : {}),
        },
      },
    );

    missionProcess.stdout.on("data", (chunk) => {
      bufferedOutput += chunk.toString();
      const lines = bufferedOutput.split("\n");
      bufferedOutput = lines.pop();

      for (const line of lines) {
        try {
          lastMessage = JSON.parse(line);
          broadcast("mission:status", lastMessage);
        } catch {
          continue;
        }
      }
    });

    let errorOutput = "";
    missionProcess.stderr.on("data", (chunk) => {
      errorOutput += chunk.toString();
    });

    missionProcess.on("exit", (exitCode) => {
      missionProcess = null;
      resolve({
        ok: exitCode === 0,
        result: lastMessage,
        error: exitCode === 0
          ? null
          : lastMessage?.message || errorOutput.trim() || "미션 작업 실패",
      });
    });
  });
}

ipcMain.handle("mission:select", async () => {
  const result = await dialog.showOpenDialog({
    title: "QGroundControl Plan 선택",
    defaultPath: path.join(projectRoot, "plans"),
    properties: ["openFile"],
    filters: [
      { name: "QGroundControl Plan", extensions: ["plan"] },
      { name: "JSON", extensions: ["json"] },
    ],
  });

  if (result.canceled || result.filePaths.length === 0) {
    return { ok: false, canceled: true };
  }

  try {
    const sourcePlan = JSON.parse(fs.readFileSync(result.filePaths[0], "utf8"));
    const target = resolveTrayLandingTarget();
    saveTrayLandingPlan(sourcePlan, target);
    return runMissionAdapter("inspect", selectedPlanPath);
  } catch (error) {
    return { ok: false, error: error.message };
  }
});

ipcMain.handle("mission:tray-plan", async () => {
  let target;
  try {
    target = resolveTrayLandingTarget();
  } catch (error) {
    return {
      ok: false,
      error: error.message,
    };
  }

  const plan = {
    fileType: "Plan",
    geoFence: { circles: [], polygons: [], version: 2 },
    groundStation: "QGroundControl",
    mission: {
      cruiseSpeed: 3,
      firmwareType: 12,
      globalPlanAltitudeMode: 1,
      hoverSpeed: 2,
      items: [
        missionItem(
          22,
          1,
          target.reference[0],
          target.reference[1],
          8,
        ),
      ],
      plannedHomePosition: [
        target.reference[0],
        target.reference[1],
        target.homeAltitude,
      ],
      vehicleType: 2,
      version: 2,
    },
    rallyPoints: { points: [], version: 2 },
    version: 1,
  };
  saveTrayLandingPlan(plan, target);
  return runMissionAdapter("inspect", selectedPlanPath);
});

ipcMain.handle("mission:upload", () => {
  if (!selectedPlanPath) {
    return { ok: false, error: "먼저 Plan 파일을 선택하세요" };
  }
  return runMissionAdapter("upload", selectedPlanPath);
});

ipcMain.handle("mission:start", () => {
  if (!selectedPlanPath) {
    return { ok: false, error: "먼저 Plan 파일을 선택하세요" };
  }
  return runMissionAdapter("start", selectedPlanPath);
});

ipcMain.on("px4:command", (_event, command) => {
  if (jetsonGcsUrl) {
    jetsonRequest(
      "/command",
      {
        method: "POST",
        body: JSON.stringify({ command }),
      },
    ).catch((error) => {
      console.error(`[Jetson command] ${error.message}`);
    });
    return;
  }

  if (rosBridge?.stdin.writable) {
    rosBridge.stdin.write(`${JSON.stringify(command)}\n`);
  }
});

app.whenReady().then(async () => {
  const baseUrl = await startServer();
  console.log(`[Jetson gateway] Remote mode: ${jetsonGcsUrl}`);
  startJetsonTelemetry();
  createWindow(baseUrl);
  setTimeout(launchQGroundControl, 1500);
  startQgcCapture();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow(baseUrl);
    }
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") {
    app.quit();
  }
});

app.on("before-quit", () => {
  server?.close();
  rosBridge?.kill("SIGTERM");
  mavrosProcess?.kill("SIGTERM");
  cameraBridge?.kill("SIGTERM");
  qgcProcess?.kill("SIGTERM");
  missionProcess?.kill("SIGTERM");
  if (qgcCaptureTimer) {
    clearInterval(qgcCaptureTimer);
  }
  if (jetsonPollTimer) {
    clearInterval(jetsonPollTimer);
  }
});
