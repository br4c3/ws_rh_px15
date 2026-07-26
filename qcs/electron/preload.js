const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("gcsBridge", {
  onTelemetry: (callback) => {
    ipcRenderer.on("px4:telemetry", (_event, telemetry) => callback(telemetry));
  },
  onCameraFrame: (callback) => {
    ipcRenderer.on("gazebo:camera-frame", (_event, frame) => callback(frame));
  },
  onCameraStatus: (callback) => {
    ipcRenderer.on("gazebo:camera-status", (_event, connected) => callback(connected));
  },
  onQgcStatus: (callback) => {
    ipcRenderer.on("qgc:status", (_event, status) => callback(status));
  },
  onQgcFrame: (callback) => {
    ipcRenderer.on("qgc:frame", (_event, frame) => callback(frame));
  },
  onMissionStatus: (callback) => {
    ipcRenderer.on("mission:status", (_event, status) => callback(status));
  },
  launchQgc: () => ipcRenderer.invoke("qgc:launch"),
  selectMissionPlan: () => ipcRenderer.invoke("mission:select"),
  createTrayLandingPlan: () => ipcRenderer.invoke("mission:tray-plan"),
  uploadMissionPlan: () => ipcRenderer.invoke("mission:upload"),
  startMissionPlan: () => ipcRenderer.invoke("mission:start"),
  minimizeWindow: () => ipcRenderer.send("window:minimize"),
  toggleMaximizeWindow: () => ipcRenderer.send("window:toggle-maximize"),
  closeWindow: () => ipcRenderer.send("window:close"),
  onWindowMaximized: (callback) => {
    ipcRenderer.on("window:maximized", (_event, maximized) => callback(maximized));
  },
  sendCommand: (command) => ipcRenderer.send("px4:command", command),
});
