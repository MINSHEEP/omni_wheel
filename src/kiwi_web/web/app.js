/* Same-origin local console. No CDN, rosbridge, video stream or browser ROS dependency. */
(() => {
  "use strict";
  const $ = (id) => document.getElementById(id),
    page = document.body.dataset.page;
  const modeNames = {
    idle: "已停止",
    mapping: "建图",
    navigation: "导航",
    remote: "独立遥控",
  };
  const navNames = {
    idle: "等待",
    running: "执行中",
    paused: "已暂停",
    completed: "已完成",
    canceled: "已取消",
    failed: "失败",
  };
  let socket,
    clientId = "",
    challenge = "",
    state = null,
    connected = false,
    reconnectTimer,
    lastState = 0;
  let enabled = false,
    sequence = 0,
    speed = 0.5,
    limits = [0.4, 0.4, 0.6],
    chassis = "",
    lateralEnabled = true,
    keys = new Set(),
    heldPointers = new Map();
  let mapBusy = false,
    liveVersion = -1,
    mapGeneration = 0,
    metadata = null,
    editingName = "",
    operations = [],
    redo = [],
    previewOld = new Map();
  let activeTool = "obstacle",
    activeShape = "brush",
    routePoints = [],
    routes = {},
    multi = false,
    interaction = "initial",
    target = null;
  let latency = 0,
    toastTimer,
    lastNavMessage = "",
    lastMode = "",
    lastModeId = "",
    restoreNavigation = true,
    showingExecution = false,
    editorSaving = false,
    routeDirty = false;
  const canvas = $("map-canvas")
    ? new window.MapCanvas($("map-canvas"), $("map-wrapper"))
    : null;
  const setText = (id, text) => {
    if ($(id)) $(id).textContent = text;
  };
  function applyChassis(value) {
    if (!["kiwi", "tracked_pwm"].includes(value)) return;
    if (chassis && chassis !== value) release();
    chassis = value;
    lateralEnabled = chassis === "kiwi" && limits[1] > 0;
    document.body.dataset.chassis = chassis;
    const name = chassis === "tracked_pwm" ? "履带底盘控制" : "Kiwi 底盘控制";
    const brand = document.querySelector(".brand");
    if (brand) brand.textContent = name;
    document.title = document.title.replace(/Kiwi 底盘控制|履带底盘控制/, name);
    document.querySelectorAll('[data-move="a"], [data-move="d"]').forEach((button) => {
      button.disabled = !lateralEnabled;
      button.title = lateralEnabled ? "" : "履带底盘不支持横移";
    });
    document.querySelectorAll("[data-lateral-hint]").forEach((hint) => {
      hint.hidden = !lateralEnabled;
    });
    const imuLabel = $("imu-display")?.closest(".status-card")?.querySelector(".label");
    if (imuLabel) imuLabel.textContent = chassis === "tracked_pwm" ? "IMU 数据" : "IMU 标定";
    setText(
      "remote-safety-hint",
      lateralEnabled
        ? "按住移动，松开停止。开启后仅当前页面持有遥控权。后退和横移前请确认周围空间。"
        : "按住移动，松开停止。开启后仅当前页面持有遥控权。后退和转向前请确认周围空间。",
    );
  }
  function log(message, error = false) {
    const area = $("log-area");
    if (area) {
      const row = document.createElement("div");
      const stamp = document.createElement("span");
      stamp.className = "log-time";
      stamp.textContent = `[${new Date().toLocaleTimeString()}] `;
      row.append(stamp, document.createTextNode(message));
      area.append(row);
      while (area.childNodes.length > 120) area.firstChild.remove();
      area.scrollTop = area.scrollHeight;
    }
    if (error) toast(message, true);
  }
  function toast(message, error = false) {
    const box = $("toast");
    box.textContent = message;
    box.classList.toggle("error", error);
    box.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => (box.hidden = true), 4500);
  }
  function confirmAction(message, title = "确认操作") {
    release();
    return new Promise((resolve) => {
      const dialog = $("confirm-dialog");
      setText("confirm-title", title);
      setText("confirm-message", message);
      const done = (value) => {
        dialog.close();
        resolve(value);
      };
      $("confirm-yes").onclick = () => done(true);
      $("confirm-no").onclick = () => done(false);
      dialog.oncancel = (e) => {
        e.preventDefault();
        done(false);
      };
      dialog.showModal();
    });
  }
  async function request(path, options = {}) {
    const response = await fetch(path, {
      cache: "no-store",
      ...options,
      headers: { "X-Kiwi-Request": "1", ...options.headers },
    });
    let value;
    try {
      value = await response.json();
    } catch {
      throw new Error(`请求失败 (${response.status})`);
    }
    if (!response.ok) {
      if (response.status === 401) login();
      throw new Error(value.error || `请求失败 (${response.status})`);
    }
    return value;
  }
  const command = (op, data = {}) =>
    request("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ op, mode_id: state?.mode_id, ...data }),
    });
  function bind(id, fn) {
    if (!$(id)) return;
    $(id).addEventListener("click", async () => {
      const button = $(id);
      button.disabled = true;
      button.dataset.busy = "1";
      try {
        await fn();
      } catch (error) {
        log(error.message, true);
      } finally {
        delete button.dataset.busy;
        button.disabled = false;
        updateButtons();
      }
    });
  }
  function send(value) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return false;
    if (socket.bufferedAmount > 4096) {
      socket.close();
      localRelease();
      return false;
    }
    socket.send(JSON.stringify(value));
    return true;
  }
  function localRelease() {
    enabled = false;
    keys.clear();
    heldPointers.clear();
    document
      .querySelectorAll("[data-move]")
      .forEach((button) => button.classList.remove("pressed"));
    const toggle = $("remote-toggle");
    if (toggle) {
      toggle.classList.remove("on", "active");
      toggle.setAttribute("aria-pressed", "false");
    }
    $("keyboard-hint")?.classList.add("disabled");
    $("map-wrapper")?.classList.remove("remote-active");
    setText("remote-display", "关闭");
  }
  function release() {
    send({ op: "release" });
    localRelease();
  }
  function drive() {
    if (!enabled) return;
    if (
      !connected ||
      !challenge ||
      performance.now() - lastState > 350 ||
      document.hidden
    ) {
      release();
      return;
    }
    let x = (keys.has("w") ? 1 : 0) - (keys.has("s") ? 1 : 0),
      y = lateralEnabled ? (keys.has("a") ? 1 : 0) - (keys.has("d") ? 1 : 0) : 0,
      a = (keys.has("q") ? 1 : 0) - (keys.has("e") ? 1 : 0);
    const norm = Math.max(1, Math.hypot(x, y));
    x /= norm;
    y /= norm;
    send({
      op: "drive",
      seq: sequence++,
      challenge,
      velocity: [
        x * speed * limits[0],
        y * speed * limits[1],
        a * speed * limits[2],
      ],
    });
  }
  function stopMotion() {
    keys.clear();
    heldPointers.clear();
    document
      .querySelectorAll("[data-move]")
      .forEach((b) => b.classList.remove("pressed"));
    drive();
  }
  async function emergency() {
    const sent = send({ op: "estop" });
    localRelease();
    if (!sent) await command("estop");
    log("软件急停：运动输出已锁定");
  }
  async function resetEstop() {
    if (
      await confirmAction(
        "解除后不会自动恢复之前的运动。请重新开启遥控或重新启动导航任务。",
        "解除软件急停",
      )
    ) {
      await command("reset_estop");
      log("软件急停已解除");
    }
  }
  function setSpeed(value) {
    speed = Math.max(0.1, Math.min(1, Number(value)));
    if ($("speed-slider")) $("speed-slider").value = String(speed);
    setText("speed-label", `${Math.round(speed * 100)}%`);
  }
  bind("remote-toggle", () => {
    if (enabled) {
      release();
      return;
    }
    if (!state?.control.manual_ready)
      throw new Error("请先启动对应模式，等待底盘标定与状态就绪");
    if (!send({ op: "acquire" })) throw new Error("设备连接已断开");
  });
  $("speed-slider")?.addEventListener("input", (e) => setSpeed(e.target.value));
  document.querySelectorAll("[data-move]").forEach((button) => {
    button.addEventListener("pointerdown", (e) => {
      e.preventDefault();
      if (button.disabled) return;
      if (!enabled) {
        toast("请先开启遥控模式", true);
        return;
      }
      button.setPointerCapture(e.pointerId);
      heldPointers.set(e.pointerId, button.dataset.move);
      keys.add(button.dataset.move);
      button.classList.add("pressed");
      drive();
    });
    const up = (e) => {
      const key = heldPointers.get(e.pointerId);
      heldPointers.delete(e.pointerId);
      if (![...heldPointers.values()].includes(key)) keys.delete(key);
      button.classList.remove("pressed");
      drive();
    };
    button.addEventListener("pointerup", up);
    button.addEventListener("pointercancel", up);
    button.addEventListener("lostpointercapture", up);
  });
  document.addEventListener("keydown", (e) => {
    if (
      e.target.matches("input,textarea,select,[contenteditable]") ||
      document.querySelector("dialog[open]")
    )
      return;
    const key = e.key.toLowerCase();
    if (["t", " ", "escape"].includes(key)) {
      e.preventDefault();
      if (!e.repeat) emergency().catch((err) => log(err.message, true));
      return;
    }
    if (!enabled) return;
    if ((lateralEnabled ? "wasdqe" : "wsqe").includes(key) && key.length === 1) {
      e.preventDefault();
      keys.add(key);
      drive();
    } else if (key === "x") {
      e.preventDefault();
      stopMotion();
    } else if (key === "+" || key === "=") {
      e.preventDefault();
      setSpeed(speed + 0.05);
    } else if (key === "-") {
      e.preventDefault();
      setSpeed(speed - 0.05);
    }
  });
  document.addEventListener("keyup", (e) => {
    keys.delete(e.key.toLowerCase());
    drive();
  });
  window.addEventListener("blur", release);
  window.addEventListener("pagehide", release);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) release();
  });
  window.addEventListener("beforeunload", (e) => {
    release();
    if (operations.length || routeDirty) {
      e.preventDefault();
      e.returnValue = "";
    }
  });
  document.querySelectorAll(".page-nav a").forEach((link) =>
    link.addEventListener("click", async (e) => {
      if (operations.length || routeDirty) {
        e.preventDefault();
        if (
          await confirmAction(
            "当前修改尚未保存，离开页面将丢失这些修改。确定离开？",
          )
        ) {
          operations = [];
          routeDirty = false;
          location.href = link.href;
        }
      }
    }),
  );
  setInterval(drive, 50);
  setInterval(() => {
    if (connected) send({ op: "ping", sent: performance.now() });
  }, 1000);
  bind("global-stop", emergency);
  bind("btn-emergency", emergency);
  bind("btn-stop", stopMotion);
  bind("banner-reset", resetEstop);
  bind("btn-reset-estop", resetEstop);
  bind("btn-zoom-in", () => canvas.zoomBy(0.15));
  bind("btn-zoom-out", () => canvas.zoomBy(-0.15));
  bind("btn-reset-view", () => canvas.resetView());

  function readiness(robot) {
    if (state.transitioning) return "正在切换运行模式，请稍候…";
    if (state.error) return state.error;
    if (robot.motion_fault) return robot.motion_fault;
    if (robot.conflicts.length) return "速度源冲突：请停止其他遥控或导航程序";
    if (state.mode === "idle") return "选择运行模式后启动设备";
    if (!robot.feedback) return "等待底盘反馈，请检查底盘供电和串口";
    if (!robot.imu_ready)
      return chassis === "tracked_pwm"
        ? "等待 IMU 数据，请检查底盘连接"
        : "IMU 标定中，请保持底盘静止";
    if (!robot.odom) return "等待里程计与 EKF";
    if (!robot.lidar) return "等待激光雷达，请检查雷达供电和串口";
    if (state.mode === "mapping" && !robot.mapping_ready)
      return "等待稳定扫描，保持底盘静止";
    if (state.mode === "navigation" && !robot.pose)
      return "请在地图中设置初始位置与朝向";
    if (state.mode === "navigation" && !robot.action_ready)
      return "Nav2 初始化中";
    if (!state.motion_enabled)
      return state.mode === "mapping"
        ? "建图数据就绪，底盘运动尚未解锁"
        : "底盘运动尚未解锁，请检查标定配置";
    return "设备就绪";
  }
  function updateState(value) {
    state = value;
    if (value.chassis && value.chassis !== chassis) applyChassis(value.chassis);
    lastState = performance.now();
    challenge = value.challenge || challenge;
    const r = state.robot,
      n = state.navigation;
    if (lastModeId !== state.mode_id) {
      const previousModeId = lastModeId;
      lastModeId = state.mode_id;
      if (previousModeId && page === "navigation") {
        target = null;
        routePoints = [];
        routeDirty = false;
        routes = {};
        $("route-name").value = "";
        $("route-selector").replaceChildren(new Option("选择路径", ""));
        renderRoute();
        canvas.clearGoalPose();
        canvas.mapInfo = null;
        canvas.mapData = null;
        liveVersion = -1;
        mapGeneration++;
      }
    }
    if (
      page === "navigation" &&
      !state.transitioning &&
      restoreNavigation &&
      n.total &&
      !routePoints.length &&
      !routeDirty
    ) {
      routePoints = structuredClone(n.points);
      renderRoute();
    }
    if (page === "navigation" && !state.transitioning)
      restoreNavigation = false;
    updateButtons();
    if (enabled && state.control.owner !== clientId) localRelease();
    $("estop-banner").hidden = !state.control.estop;
    setText("mode-display", modeNames[state.mode] || state.mode);
    setText("map-display", r.has_map ? "已更新" : "等待中");
    setText("gate-display", r.mapping_ready ? "已就绪" : "等待中");
    setText("feedback-display", r.feedback ? "在线" : "未连接");
    setText("lidar-display", r.lidar ? "在线" : "未连接");
    setText(
      "imu-display",
      chassis === "tracked_pwm"
        ? r.imu_ready ? "在线" : "未连接"
        : r.imu_ready ? "已完成" : "未完成",
    );
    setText("readiness", readiness(r));
    setText(
      "localization-display",
      r.pose?.frame === "map" ? "已定位" : "未定位",
    );
    setText("nav-status", navNames[n.state] || n.state);
    setText("nav-progress", `${Math.min(n.index + 1, n.total)} / ${n.total}`);
    setText(
      "nav-feedback",
      n.message ||
        (n.state === "running"
          ? `当前第 ${n.index + 1} 点 · 剩余 ${n.distance ?? "--"} m · 预计 ${n.remaining ?? "--"} s`
          : "等待导航任务"),
    );
    if (n.message && n.message !== lastNavMessage) {
      log(n.message, n.state === "failed");
      lastNavMessage = n.message;
    }
    setText(
      "pose-display",
      r.pose
        ? `X ${r.pose.x.toFixed(2)} · Y ${r.pose.y.toFixed(2)} · ${((r.pose.yaw * 180) / Math.PI).toFixed(0)}°`
        : "位置 --",
    );
    setText("host-display", location.host);
    setText("latency-display", `${Math.round(latency)} ms`);
    setText(
      "limits-display",
      lateralEnabled
        ? `遥控上限：前后 ${limits[0]} m/s，横移 ${limits[1]} m/s，转向 ${limits[2]} rad/s。`
        : `遥控上限：前后 ${limits[0]} m/s，转向 ${limits[2]} rad/s；不支持横移。`,
    );
    for (const [id, key] of [
      ["cpu", "cpu"],
      ["memory", "memory"],
      ["disk", "disk"],
    ])
      setText(`${id}-display`, `${state.system[key] ?? "--"}%`);
    setText("clients-display", state.system.clients ?? "--");
    setText("process-logs", state.logs.join("\n"));
    if ($("diagnostics")) {
      $("diagnostics").replaceChildren(
        ...r.diagnostics.map((d) => {
          const line = document.createElement("p");
          line.textContent = `${d.name}：${d.message}`;
          line.className = d.level ? "health-bad" : "health-good";
          return line;
        }),
      );
      if (!r.diagnostics.length) setText("diagnostics", readiness(r));
    }
    if (canvas && page !== "map_editor") {
      if (r.pose) canvas.setRobotPose(r.pose.x, r.pose.y, r.pose.yaw);
      else canvas.clearRobotPose();
      canvas.scanPoints = r.scan;
      canvas.setGlobalPath(r.path);
      if (
        page === "navigation" &&
        n.total &&
        !routeDirty &&
        ["running", "paused"].includes(n.state)
      ) {
        showingExecution = true;
        canvas.setRoutePoints(n.points, n.index, n.state);
      } else if (page === "navigation" && showingExecution) {
        showingExecution = false;
        canvas.setRoutePoints(routePoints);
      }
      if (page === "remote_control") remoteView(r.pose);
      else if (
        ["mapping", "navigation"].includes(state.mode) &&
        r.has_map &&
        liveVersion !== r.map_version &&
        !mapBusy
      )
        loadLive().catch((error) => log(error.message));
    }
    if (lastMode !== state.mode) {
      lastMode = state.mode;
      log(`运行模式：${modeNames[state.mode]}`);
      if (
        (!r.has_map || state.mode === "idle") &&
        page !== "map_editor" &&
        page !== "remote_control"
      ) {
        liveVersion = -1;
        mapGeneration++;
        if (canvas) {
          canvas.mapInfo = null;
          canvas.mapData = null;
          canvas.clearGoalPose();
          canvas.clearRoutePoints();
        }
        setText("map-detail", "地图 --");
      }
      if (page === "map_editor" && editingName) {
        if (state.mode !== "idle") {
          canvas.setEditorInteraction(null);
          setText(
            "map-overlay",
            "设备已进入运行模式，请停止模式后重新打开地图进行编辑。",
          );
        } else setupEditor();
      }
    }
    if (page === "navigation") {
      document.querySelectorAll(".route-list li").forEach((li, i) => {
        li.classList.toggle("current", n.state === "running" && i === n.index);
        li.classList.toggle("completed", n.state === "running" && i < n.index);
      });
    }
  }
  function updateButtons() {
    if (!state) return;
    const disabled = (id, value) => {
      const button = $(id);
      if (button && !button.dataset.busy) button.disabled = value;
    };
    const r = state.robot,
      n = state.navigation;
    disabled(
      "btn-save-map",
      !connected ||
        state.transitioning ||
        state.mode !== "mapping" ||
        !r.has_map ||
        !r.mapping_ready,
    );
    disabled(
      "remote-toggle",
      !connected ||
        (!enabled && (!state.control.manual_ready || state.control.estop)),
    );
    disabled(
      "btn-start-goal",
      !connected ||
        !state.control.nav_ready ||
        !r.action_ready ||
        state.control.estop ||
        state.mode !== "navigation",
    );
    disabled(
      "btn-start-route",
      !connected ||
        !state.control.nav_ready ||
        !r.action_ready ||
        state.control.estop ||
        state.mode !== "navigation",
    );
    disabled("btn-pause-route", n.state !== "running");
    disabled("btn-resume-route", n.state !== "paused" || state.control.estop);
    disabled("btn-skip-route", !["running", "paused"].includes(n.state));
    if (page === "map_editor") editorUi();
  }
  function remoteView(robotPose) {
    if (
      !canvas.mapInfo ||
      (robotPose && !canvas.mapToGrid(robotPose.x, robotPose.y))
    ) {
      const x = robotPose?.x || 0,
        y = robotPose?.y || 0;
      canvas.setMap(
        {
          width: 400,
          height: 400,
          resolution: 0.05,
          origin: {
            position: { x: x - 10, y: y - 10 },
            orientation: { z: 0, w: 1 },
          },
        },
        new Int8Array(160000).fill(-1),
      );
      setText(
        "map-overlay",
        "里程计视图 · 激光点为实时障碍观测，背景不表示可通行区域",
      );
    }
  }
  async function imageGrid(meta, path) {
    const response = await fetch(path, { cache: "no-store" });
    if (!response.ok) throw new Error("地图更新中，正在重新读取");
    const bitmap = await createImageBitmap(await response.blob());
    if (bitmap.width !== meta.width || bitmap.height !== meta.height) {
      bitmap.close();
      throw new Error("地图图像尺寸不一致");
    }
    const raster = document.createElement("canvas");
    raster.width = meta.width;
    raster.height = meta.height;
    const ctx = raster.getContext("2d", { willReadFrequently: true });
    ctx.drawImage(bitmap, 0, 0);
    bitmap.close();
    const pixels = ctx.getImageData(0, 0, meta.width, meta.height).data,
      data = new Int8Array(meta.width * meta.height);
    for (let y = 0; y < meta.height; y++)
      for (let x = 0; x < meta.width; x++) {
        const value = pixels[(y * meta.width + x) * 4];
        data[(meta.height - 1 - y) * meta.width + x] =
          value === 205 ? -1 : Math.round(((254 - value) * 100) / 254);
      }
    const [x, y, a] = meta.origin;
    return {
      info: {
        width: meta.width,
        height: meta.height,
        resolution: meta.resolution,
        origin: {
          position: { x, y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(a / 2), w: Math.cos(a / 2) },
        },
      },
      data,
    };
  }
  async function loadLive() {
    mapBusy = true;
    const generation = mapGeneration;
    try {
      const meta = await request("/api/live/map");
      const result = await imageGrid(
        meta,
        `/api/live/image?version=${meta.version}`,
      );
      if (generation !== mapGeneration) return;
      metadata = meta;
      canvas.setMap(result.info, result.data);
      liveVersion = meta.version;
      setText(
        "map-detail",
        `${meta.width} × ${meta.height} · ${Number(meta.resolution.toFixed(4))} m/格`,
      );
      setText(
        "map-overlay",
        page === "navigation"
          ? "在地图上按下并拖拽设置位置和朝向；右键平移，滚轮缩放。"
          : "二维实时建图 · 红色为激光点，箭头为底盘位置。",
      );
    } finally {
      mapBusy = false;
    }
  }
  async function loadSaved(name) {
    const meta = await request(`/api/maps/${encodeURIComponent(name)}`);
    const result = await imageGrid(
      meta,
      `/api/maps/${encodeURIComponent(name)}/image?revision=${meta.revision}`,
    );
    metadata = meta;
    canvas.setMap(result.info, result.data, { reset: true });
    setText(
      "map-detail",
      `${meta.width} × ${meta.height} · ${Number(meta.resolution.toFixed(4))} m/格`,
    );
    return meta;
  }
  async function refreshMaps(preferred) {
    if (!$("map-selector")) return;
    const selected = preferred || $("map-selector").value || state?.current_map;
    const maps = await request("/api/maps");
    $("map-selector").replaceChildren(new Option("请选择已保存地图", ""));
    for (const map of maps)
      $("map-selector").add(new Option(map.name, map.name));
    if (maps.some((m) => m.name === selected))
      $("map-selector").value = selected;
  }
  function selectedMap() {
    const name = $("map-selector")?.value;
    if (!name) throw new Error("请先选择地图");
    return name;
  }
  async function mode(next, name = "") {
    if (
      state?.mode === "mapping" &&
      next !== "mapping" &&
      !(await confirmAction(
        "切换模式会结束当前建图。请确认需要的地图已经保存。",
        "切换运行模式",
      ))
    )
      return false;
    release();
    liveVersion = -1;
    mapGeneration++;
    await command("mode", { mode: next, map: name });
    log(`已请求${modeNames[next]}`);
    return true;
  }
  bind("btn-refresh-maps", () => refreshMaps());
  bind("btn-start-mapping", () => mode("mapping"));
  bind("btn-start-remote", () => mode("remote"));
  bind("btn-stop-mode", () => mode("idle"));
  bind("btn-save-map", async () => {
    const name = $("map-name").value.trim();
    await command("save_map", { name });
    log(`地图 ${name} 已保存`);
    toast("地图保存成功");
  });
  bind("btn-load-map", async () => {
    const name = selectedMap();
    if (
      routeDirty &&
      !(await confirmAction("切换地图将丢弃未保存的路径修改，是否继续？"))
    )
      return;
    if (await mode("navigation", name)) {
      target = null;
      await refreshRoutes(name);
      setInteraction("initial");
    }
  });
  bind("btn-export-map", () => {
    const name = editingName || selectedMap();
    const link = document.createElement("a");
    link.href = `/api/maps/${encodeURIComponent(name)}/export`;
    link.download = name + ".zip";
    link.click();
  });
  bind("btn-delete-map", async () => {
    const name = selectedMap();
    if (
      await confirmAction(`删除地图 ${name}？设备会保留历史文件。`, "删除地图")
    ) {
      await command("delete_map", { name });
      await refreshMaps();
      log(`地图 ${name} 已删除`);
    }
  });
  bind("btn-import-map", async () => {
    const yaml = $("import-yaml").files[0],
      image = $("import-image").files[0];
    if (!yaml || !image) throw new Error("请选择 YAML 和地图图像");
    const form = new FormData();
    form.append("name", $("import-name").value.trim());
    form.append("yaml", yaml);
    form.append("image", image);
    const result = await request("/api/import", { method: "POST", body: form });
    await refreshMaps(result.name);
    toast("地图导入成功");
  });

  function editorUi() {
    setText("editing-map-display", editingName || "未打开");
    setText(
      "edit-status-display",
      operations.length ? "未保存" : editingName ? "已保存" : "等待",
    );
    if ($("btn-undo"))
      $("btn-undo").disabled = !operations.length || editorSaving;
    if ($("btn-redo")) $("btn-redo").disabled = !redo.length || editorSaving;
    if ($("btn-save-edits"))
      $("btn-save-edits").disabled =
        !operations.length ||
        editorSaving ||
        !connected ||
        state?.mode !== "idle";
    for (const key of ["obstacle", "free", "unknown"])
      $("btn-tool-" + key)?.classList.toggle("tool-active", key === activeTool);
    for (const key of ["brush", "rectangle"])
      $("btn-shape-" + key)?.classList.toggle(
        "choice-active",
        key === activeShape,
      );
    setText(
      "map-overlay",
      editingName
        ? "左键绘制，右键平移，滚轮缩放。修改后点击覆盖保存。"
        : "请先在右侧选择并打开一张已保存地图。",
    );
  }
  function setupEditor() {
    if (!editingName || editorSaving) {
      canvas.setEditorInteraction(null);
      editorUi();
      return;
    }
    const value = { obstacle: 100, free: 0, unknown: -1 }[activeTool];
    canvas.setEditorInteraction({
      shape: activeShape,
      brushRadius: Number($("brush-radius").value),
      onPreview: (cells) => {
        for (const cell of cells)
          if (!previewOld.has(cell.dataIndex))
            previewOld.set(cell.dataIndex, canvas.mapData[cell.dataIndex]);
        canvas.applyOccupancyCells(cells, value);
      },
      onCommit: (cells) => {
        operations.push({
          value: { obstacle: 0, free: 254, unknown: 205 }[activeTool],
          occupancy: value,
          indices: cells.map((c) => c.serviceIndex),
          cells,
          old: [...previewOld],
        });
        previewOld.clear();
        redo = [];
        editorUi();
      },
    });
    editorUi();
  }
  bind("btn-open-map", async () => {
    if (
      operations.length &&
      !(await confirmAction("当前修改尚未保存，是否放弃并打开所选地图？"))
    )
      return;
    const name = selectedMap();
    if (!(await mode("idle"))) return;
    await loadSaved(name);
    editingName = name;
    operations = [];
    redo = [];
    previewOld.clear();
    setupEditor();
    log(`已打开地图 ${name}`);
  });
  for (const key of ["obstacle", "free", "unknown"])
    bind("btn-tool-" + key, () => {
      activeTool = key;
      setupEditor();
    });
  for (const key of ["brush", "rectangle"])
    bind("btn-shape-" + key, () => {
      activeShape = key;
      setupEditor();
    });
  $("brush-radius")?.addEventListener("input", () => {
    setText("brush-radius-value", $("brush-radius").value);
    setupEditor();
  });
  bind("btn-undo", () => {
    const op = operations.pop();
    if (!op) return;
    for (const [i, value] of op.old) canvas.mapData[i] = value;
    canvas.rebuildRaster();
    redo.push(op);
    editorUi();
  });
  bind("btn-redo", () => {
    const op = redo.pop();
    if (!op) return;
    canvas.applyOccupancyCells(op.cells, op.occupancy);
    operations.push(op);
    editorUi();
  });
  bind("btn-save-edits", async () => {
    if (!operations.length) return;
    editorSaving = true;
    setupEditor();
    try {
      const result = await command("edit_map", {
        name: editingName,
        revision: metadata.revision,
        operations: operations.map(({ value, indices }) => ({
          value,
          indices,
        })),
      });
      metadata = result.result;
      operations = [];
      redo = [];
      log("地图修改已保存");
      toast("地图修改已保存");
    } finally {
      editorSaving = false;
      setupEditor();
    }
  });
  if (page === "map_editor") editorUi();

  function currentMap() {
    const name = state?.current_map;
    if (state?.mode !== "navigation" || !name)
      throw new Error("请先加载导航地图");
    return name;
  }
  function setInteraction(value) {
    interaction = value;
    if (!canvas) return;
    $("btn-initial-mode")?.classList.toggle(
      "choice-active",
      value === "initial",
    );
    $("btn-goal-mode")?.classList.toggle("choice-active", value === "goal");
    $("btn-route-point-mode")?.classList.toggle(
      "choice-active",
      value === "route",
    );
    setText(
      "interaction-hint",
      value === "initial"
        ? "按下确定初始位置，拖拽确定朝向，松开发布定位。"
        : value === "goal"
          ? "按下并拖拽选择目标；确认后点击开始导航。"
          : "按下并拖拽添加路径点，可在列表调整坐标与顺序。",
    );
    canvas.setPoseInteraction(true, (x, y, yaw) => {
      if (state?.mode !== "navigation") {
        toast("请先加载导航地图", true);
        return;
      }
      const point = { x, y, yaw };
      if (interaction === "initial")
        command("initial_pose", { pose: point })
          .then(() => log("初始位姿已发布，请观察激光与地图是否重合"))
          .catch((e) => log(e.message, true));
      else if (interaction === "goal") {
        target = point;
        canvas.setGoalPose(x, y, yaw);
        log(
          `目标：(${x.toFixed(2)}, ${y.toFixed(2)}) · ${((yaw * 180) / Math.PI).toFixed(0)}°`,
        );
      } else {
        if (routePoints.length >= 200) {
          toast("每条路径最多 200 个点", true);
          return;
        }
        routePoints.push(point);
        routeDirty = true;
        renderRoute();
      }
    });
  }
  bind("btn-initial-mode", () => setInteraction("initial"));
  bind("btn-goal-mode", () => setInteraction("goal"));
  bind("btn-route-point-mode", () => setInteraction("route"));
  function setMulti(value) {
    multi = value;
    $("route-section").hidden = !multi;
    $("btn-single-navigation").classList.toggle("choice-active", !multi);
    $("btn-multi-navigation").classList.toggle("choice-active", multi);
    setInteraction(multi ? "route" : "goal");
  }
  bind("btn-single-navigation", () => setMulti(false));
  bind("btn-multi-navigation", () => setMulti(true));
  bind("btn-start-goal", async () => {
    if (!target) throw new Error("请先在地图上设置目标点");
    await command("navigate", { points: [target] });
    log("单点导航已开始");
  });
  bind("btn-cancel-goal", () => command("cancel"));
  bind("btn-clear-costmaps", async () => {
    await command("clear_costmaps");
    log("临时障碍已清除");
  });
  async function refreshRoutes(mapName) {
    routes = await request(
      `/api/maps/${encodeURIComponent(mapName || currentMap())}/routes`,
    );
    const selected = $("route-selector").value;
    $("route-selector").replaceChildren(new Option("选择路径", ""));
    for (const name of Object.keys(routes))
      $("route-selector").add(new Option(name, name));
    if (routes[selected]) $("route-selector").value = selected;
  }
  function renderRoute() {
    const list = $("route-list");
    if (!list) return;
    list.replaceChildren();
    routePoints.forEach((point, index) => {
      const li = document.createElement("li"),
        number = document.createElement("span");
      number.className = "point-number";
      number.textContent = String(index + 1);
      li.append(number);
      for (const field of ["x", "y", "yaw"]) {
        const input = document.createElement("input");
        input.type = "number";
        input.step = field === "yaw" ? "1" : ".05";
        input.value = (
          field === "yaw" ? (point.yaw * 180) / Math.PI : point[field]
        ).toFixed(2);
        input.setAttribute("aria-label", `目标 ${index + 1} ${field}`);
        input.addEventListener("change", () => {
          const v = Number(input.value);
          if (!Number.isFinite(v)) {
            input.value = "0";
            return;
          }
          point[field] = field === "yaw" ? (v * Math.PI) / 180 : v;
          routeDirty = true;
          canvas.setRoutePoints(routePoints);
        });
        li.append(input);
      }
      for (const [label, action] of [
        [
          "↑",
          () => {
            if (index > 0)
              [routePoints[index - 1], routePoints[index]] = [
                routePoints[index],
                routePoints[index - 1],
              ];
          },
        ],
        [
          "↓",
          () => {
            if (index < routePoints.length - 1)
              [routePoints[index + 1], routePoints[index]] = [
                routePoints[index],
                routePoints[index + 1],
              ];
          },
        ],
        ["×", () => routePoints.splice(index, 1)],
      ]) {
        const button = document.createElement("button");
        button.textContent = label;
        button.title =
          label === "×" ? "移除目标" : label === "↑" ? "上移目标" : "下移目标";
        button.onclick = () => {
          action();
          routeDirty = true;
          renderRoute();
        };
        li.append(button);
      }
      list.append(li);
    });
    canvas.setRoutePoints(routePoints);
  }
  bind("btn-refresh-routes", () => refreshRoutes());
  bind("btn-new-route", async () => {
    if (routeDirty && !(await confirmAction("放弃尚未保存的路径修改？")))
      return;
    routePoints = [];
    routeDirty = false;
    $("route-name").value = "";
    renderRoute();
    setInteraction("route");
  });
  bind("btn-load-route", async () => {
    const name = $("route-selector").value;
    if (!routes[name]) throw new Error("请选择路径");
    if (routeDirty && !(await confirmAction("放弃尚未保存的路径修改？")))
      return;
    routePoints = structuredClone(routes[name]);
    routeDirty = false;
    $("route-name").value = name;
    renderRoute();
    setMulti(true);
  });
  bind("btn-save-route", async () => {
    const name = $("route-name").value.trim();
    if (routes[name] && !(await confirmAction(`覆盖路径 ${name}？`))) return;
    await command("save_route", {
      map: currentMap(),
      name,
      points: routePoints,
    });
    routeDirty = false;
    await refreshRoutes();
    $("route-selector").value = name;
    toast("路径已保存");
  });
  bind("btn-delete-route", async () => {
    const name = $("route-selector").value;
    if (!name) throw new Error("请选择路径");
    if (await confirmAction(`删除路径 ${name}？`)) {
      await command("delete_route", { map: currentMap(), name });
      await refreshRoutes();
    }
  });
  bind("btn-start-route", () => command("navigate", { points: routePoints }));
  for (const [id, op] of [
    ["btn-pause-route", "pause"],
    ["btn-resume-route", "resume"],
    ["btn-skip-route", "skip"],
    ["btn-stop-route", "cancel"],
  ])
    bind(id, () => command(op));
  if (page === "navigation") setInteraction("initial");

  function connect() {
    clearTimeout(reconnectTimer);
    socket = new WebSocket(
      `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`,
    );
    socket.onopen = () => {
      connected = true;
      setText("connection-status", "● 已连接");
      $("connection-status").className = "page-status connected";
      log("设备连接已建立");
      refreshMaps().catch((e) => log(e.message));
    };
    socket.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === "hello") clientId = data.client_id;
      else if (data.type === "state") updateState(data);
      else if (data.type === "pong") {
        latency = performance.now() - data.sent;
        setText("connection-status", `● 已连接 · ${Math.round(latency)} ms`);
      } else if (data.type === "acquired") {
        enabled = true;
        sequence = 0;
        $("remote-toggle")?.classList.add("on", "active");
        $("remote-toggle")?.setAttribute("aria-pressed", "true");
        $("keyboard-hint")?.classList.remove("disabled");
        $("map-wrapper")?.classList.add("remote-active");
        setText("remote-display", "已开启");
        drive();
      } else if (data.type === "error") {
        localRelease();
        log(data.message, true);
      }
    };
    socket.onclose = () => {
      connected = false;
      challenge = "";
      localRelease();
      updateButtons();
      setText("connection-status", "○ 已断开，正在重连…");
      $("connection-status").className = "page-status error";
      reconnectTimer = setTimeout(async () => {
        try {
          const config = await request("/api/config");
          if (config.auth_required) {
            login();
            return;
          }
          connect();
        } catch {
          reconnectTimer = setTimeout(connect, 2000);
        }
      }, 1500);
    };
    socket.onerror = () => socket.close();
  }
  function login() {
    release();
    if (!$("login-dialog").open) $("login-dialog").showModal();
  }
  $("login-dialog").addEventListener("cancel", (e) => e.preventDefault());
  $("login-form").addEventListener("submit", async (e) => {
    e.preventDefault();
    try {
      await request("/api/login", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ token: $("access-token").value }),
      });
      $("access-token").value = "";
      $("login-dialog").close();
      connect();
    } catch (error) {
      setText("login-error", error.message);
    }
  });
  request("/api/config")
    .then((config) => {
      limits = config.limits;
      applyChassis(config.chassis);
      if (config.auth_required) login();
      else connect();
    })
    .catch((error) => {
      log(error.message, true);
      reconnectTimer = setTimeout(connect, 2000);
    });
})();
