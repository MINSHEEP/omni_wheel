/* Map canvas adapted from the supplied MSHEEP reference. */
(function () {
  "use strict";
  class MapCanvas {
    constructor(canvas, container) {
      this.canvas = canvas;
      this.container = container;
      this.ctx = canvas.getContext("2d");
      this.offscreen = document.createElement("canvas");
      this.offCtx = this.offscreen.getContext("2d");
      this.mapInfo = null;
      this.mapData = null;
      this.mapSignature = "";
      this.width = 1;
      this.height = 1;
      this.pixelRatio = 1;
      this.baseZoom = 1;
      this.zoom = 1;
      this.offsetX = 0;
      this.offsetY = 0;
      this.minZoomRatio = 0.02;
      this.maxZoomRatio = 24;
      this.robotPose = null;
      this.goalPose = null;
      this.globalPath = [];
      this.globalPathWarning = false;
      this.routePoints = [];
      this.activeRoutePointIndex = -1;
      this.routeExecutionState = "idle";
      this.poseInteraction = { enabled: false, callback: null };
      this.editorInteraction = null;
      this.pan = null;
      this.gesture = null;
      this.dragPose = null;
      this.editRect = null;
      this.onMapChanged = null;
      this.resize();
      this.bindEvents();
      this.renderLoop();
    }

    resize() {
      const rect = this.container.getBoundingClientRect();
      this.width = Math.max(1, rect.width);
      this.height = Math.max(1, rect.height);
      this.pixelRatio = Math.max(1, window.devicePixelRatio || 1);
      this.canvas.width = Math.round(this.width * this.pixelRatio);
      this.canvas.height = Math.round(this.height * this.pixelRatio);
      this.ctx.setTransform(this.pixelRatio, 0, 0, this.pixelRatio, 0, 0);
      if (this.mapInfo) this.resetView();
    }

    bindEvents() {
      this.canvas.addEventListener("contextmenu", (event) =>
        event.preventDefault(),
      );
      this.canvas.addEventListener(
        "wheel",
        (event) => this.handleWheel(event),
        { passive: false },
      );
      this.canvas.addEventListener("mousedown", (event) =>
        this.handleMouseDown(event),
      );
      window.addEventListener("mousemove", (event) =>
        this.handleMouseMove(event),
      );
      window.addEventListener("mouseup", (event) => this.handleMouseUp(event));
      window.addEventListener("resize", () => this.resize());
      if (typeof window.ResizeObserver === "function") {
        this.resizeObserver = new window.ResizeObserver(() => this.resize());
        this.resizeObserver.observe(this.container);
      }
    }

    handleWheel(event) {
      event.preventDefault();
      if (!this.mapInfo) return;
      const point = this.eventPoint(event);
      const worldX = (point.x - this.offsetX) / this.zoom;
      const worldY = (point.y - this.offsetY) / this.zoom;
      const delta = event.deltaY > 0 ? -0.1 : 0.1;
      const ratio = Math.max(
        this.minZoomRatio,
        Math.min(this.maxZoomRatio, this.zoom / this.baseZoom + delta),
      );
      this.zoom = ratio * this.baseZoom;
      this.offsetX = point.x - worldX * this.zoom;
      this.offsetY = point.y - worldY * this.zoom;
    }

    handleMouseDown(event) {
      if (!this.mapInfo) return;
      if (event.button === 2) {
        this.pan = {
          x: event.clientX,
          y: event.clientY,
          offsetX: this.offsetX,
          offsetY: this.offsetY,
        };
        this.canvas.style.cursor = "grabbing";
        return;
      }
      if (event.button !== 0) return;
      const point = this.eventPoint(event);
      const mapPoint = this.canvasToMap(point.x, point.y);
      if (!mapPoint) return;
      if (this.editorInteraction && this.editorInteraction.enabled) {
        this.beginEdit(mapPoint);
      } else if (this.poseInteraction.enabled) {
        this.dragPose = {
          start: mapPoint,
          startScreen: point,
          endScreen: point,
        };
      }
    }

    handleMouseMove(event) {
      if (this.pan) {
        this.offsetX = this.pan.offsetX + event.clientX - this.pan.x;
        this.offsetY = this.pan.offsetY + event.clientY - this.pan.y;
        return;
      }
      if (!this.gesture && !this.dragPose) return;
      const point = this.eventPoint(event);
      if (this.dragPose) {
        this.dragPose.endScreen = point;
        return;
      }
      const mapPoint = this.canvasToMap(point.x, point.y, true);
      if (mapPoint) this.updateEdit(mapPoint);
    }

    handleMouseUp(event) {
      if (event.button === 2 && this.pan) {
        this.pan = null;
        this.canvas.style.cursor = "crosshair";
        return;
      }
      if (event.button !== 0) return;
      if (this.dragPose) {
        const drag = this.dragPose;
        this.dragPose = null;
        const dx = drag.endScreen.x - drag.startScreen.x;
        const dy = drag.endScreen.y - drag.startScreen.y;
        // Screen Y grows downward, whereas map Y grows upward.
        const yaw = Math.hypot(dx, dy) > 5 ? Math.atan2(-dy, dx) : 0;
        if (this.poseInteraction.callback)
          this.poseInteraction.callback(drag.start.x, drag.start.y, yaw);
        return;
      }
      if (this.gesture) this.commitEdit();
    }

    beginEdit(mapPoint) {
      const cfg = this.editorInteraction;
      this.gesture = { start: mapPoint, last: mapPoint, cells: new Map() };
      if (cfg.shape === "rectangle") {
        this.editRect = { start: mapPoint, end: mapPoint };
      } else {
        this.addBrushCells(mapPoint, this.gesture.cells);
        this.previewCells(this.gesture.cells);
      }
    }

    updateEdit(mapPoint) {
      const cfg = this.editorInteraction;
      if (cfg.shape === "rectangle") {
        this.gesture.last = mapPoint;
        this.editRect = { start: this.gesture.start, end: mapPoint };
        return;
      }
      this.addBrushLine(this.gesture.last, mapPoint, this.gesture.cells);
      this.gesture.last = mapPoint;
      this.previewCells(this.gesture.cells);
    }

    commitEdit() {
      const cfg = this.editorInteraction;
      const gesture = this.gesture;
      this.gesture = null;
      if (cfg.shape === "rectangle") {
        this.addRectangleCells(gesture.start, gesture.last, gesture.cells);
        this.editRect = null;
        this.previewCells(gesture.cells);
      }
      const cells = [...gesture.cells.values()];
      if (cells.length && cfg.onCommit) cfg.onCommit(cells);
    }

    previewCells(cells) {
      const cfg = this.editorInteraction;
      if (!cfg || !cfg.onPreview) return;
      cfg.onPreview([...cells.values()]);
    }

    addBrushLine(from, to, target) {
      const distance = Math.hypot(to.x - from.x, to.y - from.y);
      const interval = Math.max(this.mapInfo.resolution * 0.45, 0.001);
      const steps = Math.max(1, Math.ceil(distance / interval));
      for (let i = 0; i <= steps; i += 1) {
        this.addBrushCells(
          {
            x: from.x + ((to.x - from.x) * i) / steps,
            y: from.y + ((to.y - from.y) * i) / steps,
          },
          target,
        );
      }
    }

    addBrushCells(point, target) {
      const center = this.mapToGrid(point.x, point.y);
      if (!center) return;
      const radius = Math.max(
        0,
        Math.floor(this.editorInteraction.brushRadius || 0),
      );
      for (let dy = -radius; dy <= radius; dy += 1) {
        for (let dx = -radius; dx <= radius; dx += 1) {
          if (dx * dx + dy * dy > radius * radius) continue;
          this.addCell(center.x + dx, center.y + dy, target);
        }
      }
    }

    addRectangleCells(start, end, target) {
      const a = this.mapToGrid(start.x, start.y);
      const b = this.mapToGrid(end.x, end.y);
      if (!a || !b) return;
      const minX = Math.min(a.x, b.x),
        maxX = Math.max(a.x, b.x);
      const minY = Math.min(a.y, b.y),
        maxY = Math.max(a.y, b.y);
      for (let y = minY; y <= maxY; y += 1) {
        for (let x = minX; x <= maxX; x += 1) this.addCell(x, y, target);
      }
    }

    addCell(x, y, target) {
      const info = this.mapInfo;
      if (x < 0 || y < 0 || x >= info.width || y >= info.height) return;
      const pgmRow = info.height - 1 - y;
      const serviceIndex = pgmRow * info.width + x;
      if (!target.has(serviceIndex))
        target.set(serviceIndex, {
          serviceIndex,
          dataIndex: y * info.width + x,
        });
    }

    eventPoint(event) {
      const rect = this.canvas.getBoundingClientRect();
      return { x: event.clientX - rect.left, y: event.clientY - rect.top };
    }

    canvasToMap(screenX, screenY, clamp) {
      if (!this.mapInfo) return null;
      const info = this.mapInfo;
      let x = info.origin.position.x + (screenX - this.offsetX) / this.zoom;
      let y =
        info.origin.position.y +
        info.height * info.resolution -
        (screenY - this.offsetY) / this.zoom;
      const maxX = info.origin.position.x + info.width * info.resolution;
      const maxY = info.origin.position.y + info.height * info.resolution;
      if (clamp) {
        x = Math.max(info.origin.position.x, Math.min(maxX - 1e-9, x));
        y = Math.max(info.origin.position.y, Math.min(maxY - 1e-9, y));
      }
      if (
        x < info.origin.position.x ||
        y < info.origin.position.y ||
        x >= maxX ||
        y >= maxY
      )
        return null;
      return { x, y };
    }

    mapToGrid(x, y) {
      if (!this.mapInfo) return null;
      const info = this.mapInfo;
      const gx = Math.floor((x - info.origin.position.x) / info.resolution);
      const gy = Math.floor((y - info.origin.position.y) / info.resolution);
      if (gx < 0 || gy < 0 || gx >= info.width || gy >= info.height)
        return null;
      return { x: gx, y: gy };
    }

    mapToCanvas(x, y) {
      if (!this.mapInfo) return null;
      const info = this.mapInfo;
      return {
        x: this.offsetX + (x - info.origin.position.x) * this.zoom,
        y:
          this.offsetY +
          (info.height * info.resolution - (y - info.origin.position.y)) *
            this.zoom,
      };
    }

    setMap(info, data, options) {
      if (!info || !data) return;
      const signature = `${info.width}|${info.height}|${info.resolution}|${info.origin && info.origin.position ? info.origin.position.x : 0}|${info.origin && info.origin.position ? info.origin.position.y : 0}`;
      const reset =
        !this.mapInfo ||
        this.mapSignature !== signature ||
        (options && options.reset);
      this.mapInfo = info;
      // Occupancy grids arrive as very large JSON arrays. A compact
      // typed array avoids retaining hundreds of thousands of boxed
      // JavaScript numbers after every map update. Int16 also preserves
      // the 0..255 values used by the map editor.
      this.mapData = Int16Array.from(data);
      this.mapSignature = signature;
      this.rebuildRaster();
      if (reset) this.resetView();
      if (this.onMapChanged) this.onMapChanged(info, this.mapData);
    }

    applyOccupancyCells(cells, value) {
      if (!this.mapData) return;
      cells.forEach((cell) => {
        this.mapData[cell.dataIndex] = value;
      });
      this.rebuildRaster();
    }

    rebuildRaster() {
      if (!this.mapInfo || !this.mapData) return;
      const info = this.mapInfo;
      this.offscreen.width = info.width;
      this.offscreen.height = info.height;
      const image = this.offCtx.createImageData(info.width, info.height);
      for (let gridY = 0; gridY < info.height; gridY += 1) {
        for (let x = 0; x < info.width; x += 1) {
          const value = Number(this.mapData[gridY * info.width + x]);
          const pixel = ((info.height - 1 - gridY) * info.width + x) * 4;
          let red = 76;
          let green = 82;
          let blue = 96;
          if (Number.isFinite(value) && value >= 0) {
            const shade =
              value === 0
                ? 250
                : value >= 100
                  ? 26
                  : Math.max(44, Math.round(250 - value * 1.8));
            red = shade;
            green = shade;
            blue = shade;
          }
          image.data[pixel] = red;
          image.data[pixel + 1] = green;
          image.data[pixel + 2] = blue;
          image.data[pixel + 3] = 255;
        }
      }
      this.offCtx.putImageData(image, 0, 0);
    }

    resetView() {
      if (!this.mapInfo) return;
      const mapWidth = this.mapInfo.width * this.mapInfo.resolution;
      const mapHeight = this.mapInfo.height * this.mapInfo.resolution;
      this.baseZoom = Math.min(
        (this.width * 0.84) / mapWidth,
        (this.height * 0.84) / mapHeight,
      );
      this.baseZoom =
        Number.isFinite(this.baseZoom) && this.baseZoom > 0 ? this.baseZoom : 1;
      this.zoom = this.baseZoom;
      this.offsetX = (this.width - mapWidth * this.zoom) / 2;
      this.offsetY = (this.height - mapHeight * this.zoom) / 2;
    }

    zoomBy(amount) {
      if (!this.mapInfo) return;
      const point = { x: this.width / 2, y: this.height / 2 };
      const localX = (point.x - this.offsetX) / this.zoom;
      const localY = (point.y - this.offsetY) / this.zoom;
      const ratio = Math.max(
        this.minZoomRatio,
        Math.min(this.maxZoomRatio, this.zoom / this.baseZoom + amount),
      );
      this.zoom = ratio * this.baseZoom;
      this.offsetX = point.x - localX * this.zoom;
      this.offsetY = point.y - localY * this.zoom;
    }

    setRobotPose(x, y, yaw) {
      this.robotPose = { x, y, yaw };
    }
    clearRobotPose() {
      this.robotPose = null;
    }
    setGoalPose(x, y, yaw) {
      this.goalPose = { x, y, yaw };
    }
    clearGoalPose() {
      this.goalPose = null;
    }
    setGlobalPath(points) {
      const options = arguments[1] || {};
      this.globalPath = (points || [])
        .map((point) => ({
          x: Number(point && point.x),
          y: Number(point && point.y),
          z: Number(point && point.z),
        }))
        .filter(
          (point) =>
            Number.isFinite(point.x) &&
            Number.isFinite(point.y) &&
            Number.isFinite(point.z),
        );
      this.globalPathWarning = Boolean(options.warning);
    }
    clearGlobalPath() {
      this.globalPath = [];
      this.globalPathWarning = false;
    }
    setRoutePoints(points, activeIndex, executionState) {
      this.routePoints = (points || [])
        .map((point) => ({
          x: Number(point.x),
          y: Number(point.y),
          yaw: Number(point.yaw),
          name: String(point.name || ""),
          id: Number(point.id) || 0,
          sequence: Number(point.sequence) || 0,
          inspection: Boolean(point.inspection),
          inspectionStatus: String(point.inspectionStatus || ""),
          alerted: Boolean(point.alerted),
        }))
        .filter(
          (point) =>
            Number.isFinite(point.x) &&
            Number.isFinite(point.y) &&
            Number.isFinite(point.yaw),
        );
      this.activeRoutePointIndex = Number.isInteger(activeIndex)
        ? activeIndex
        : -1;
      this.routeExecutionState = executionState || "idle";
    }
    clearRoutePoints() {
      this.routePoints = [];
      this.activeRoutePointIndex = -1;
      this.routeExecutionState = "idle";
    }

    setPoseInteraction(enabled, callback) {
      this.poseInteraction = { enabled: Boolean(enabled), callback };
      if (enabled) this.editorInteraction = null;
    }

    setEditorInteraction(config) {
      this.editorInteraction = config
        ? {
            enabled: true,
            shape: config.shape || "brush",
            brushRadius: config.brushRadius || 0,
            onPreview: config.onPreview,
            onCommit: config.onCommit,
          }
        : null;
      if (config) this.poseInteraction.enabled = false;
    }

    renderLoop() {
      this.render();
      window.requestAnimationFrame(() => this.renderLoop());
    }

    render() {
      const ctx = this.ctx;
      ctx.clearRect(0, 0, this.width, this.height);
      ctx.fillStyle = "#0d1428";
      ctx.fillRect(0, 0, this.width, this.height);
      if (!this.mapInfo || !this.mapData) {
        ctx.fillStyle = "#a9badb";
        ctx.font = "16px sans-serif";
        ctx.textAlign = "center";
        ctx.fillText("等待地图数据…", this.width / 2, this.height / 2);
        return;
      }
      const info = this.mapInfo;
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(
        this.offscreen,
        this.offsetX,
        this.offsetY,
        info.width * info.resolution * this.zoom,
        info.height * info.resolution * this.zoom,
      );
      this.drawGlobalPath(ctx);
      this.drawPoseDrag(ctx);
      this.drawEditRectangle(ctx);
      this.drawRoute(ctx);
      this.drawRobot(ctx);
      this.drawGoal(ctx);
      ctx.fillStyle = "rgba(6, 10, 21, .68)";
      ctx.fillRect(8, 8, 184, 25);
      ctx.fillStyle = "#d9e6fb";
      ctx.font = "12px sans-serif";
      ctx.textAlign = "left";
      ctx.fillText(
        `缩放 ${Math.round((this.zoom / this.baseZoom) * 100)}%  ·  ${info.width} × ${info.height}`,
        15,
        25,
      );
    }

    drawPoseDrag(ctx) {
      if (!this.dragPose) return;
      const drag = this.dragPose;
      ctx.save();
      ctx.strokeStyle = "#ffca62";
      ctx.lineWidth = 2;
      ctx.setLineDash([7, 5]);
      ctx.beginPath();
      ctx.moveTo(drag.startScreen.x, drag.startScreen.y);
      ctx.lineTo(drag.endScreen.x, drag.endScreen.y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#ffca62";
      ctx.beginPath();
      ctx.arc(drag.startScreen.x, drag.startScreen.y, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }

    drawEditRectangle(ctx) {
      if (!this.editRect) return;
      const a = this.mapToCanvas(this.editRect.start.x, this.editRect.start.y);
      const b = this.mapToCanvas(this.editRect.end.x, this.editRect.end.y);
      if (!a || !b) return;
      ctx.save();
      ctx.setLineDash([7, 4]);
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#ffca62";
      ctx.fillStyle = "rgba(255, 202, 98, .12)";
      const x = Math.min(a.x, b.x),
        y = Math.min(a.y, b.y);
      const w = Math.abs(a.x - b.x),
        h = Math.abs(a.y - b.y);
      ctx.fillRect(x, y, w, h);
      ctx.strokeRect(x, y, w, h);
      ctx.restore();
    }

    drawRobot(ctx) {
      if (!this.robotPose) return;
      const p = this.mapToCanvas(this.robotPose.x, this.robotPose.y);
      if (!p) return;
      const size = Math.max(
        7,
        Math.min(15, 8 * Math.sqrt(this.zoom / this.baseZoom)),
      );
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(-this.robotPose.yaw + this.originYaw());
      ctx.fillStyle = "#e75c73";
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(size * 1.25, 0);
      ctx.lineTo(-size * 0.7, -size * 0.65);
      ctx.lineTo(-size * 0.7, size * 0.65);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    drawGlobalPath(ctx) {
      if (this.globalPath.length < 2) return;
      const color = this.globalPathWarning ? "#ffb454" : "#45e39b";
      const drawSegments = (strokeStyle, lineWidth) => {
        let segmentOpen = false;
        ctx.strokeStyle = strokeStyle;
        ctx.lineWidth = lineWidth;
        ctx.beginPath();
        this.globalPath.forEach((point) => {
          // Do not bridge across an invalid/out-of-map section. A
          // straight chord there would falsely look like a valid
          // planner shortcut through unknown space.
          if (!this.mapToGrid(point.x, point.y)) {
            if (segmentOpen) ctx.stroke();
            ctx.beginPath();
            segmentOpen = false;
            return;
          }
          const screen = this.mapToCanvas(point.x, point.y);
          if (!screen) return;
          if (!segmentOpen) {
            ctx.moveTo(screen.x, screen.y);
            segmentOpen = true;
          } else {
            ctx.lineTo(screen.x, screen.y);
          }
        });
        if (segmentOpen) ctx.stroke();
      };

      ctx.save();
      ctx.lineCap = "round";
      ctx.lineJoin = "round";
      ctx.setLineDash([]);
      // A dark underlay keeps the actual Nav2 plan readable over both
      // white free-space cells and dark occupied/unknown cells.
      ctx.strokeStyle = "rgba(4, 14, 22, .86)";
      ctx.lineWidth = 6.4;
      drawSegments(ctx.strokeStyle, ctx.lineWidth);
      ctx.strokeStyle = color;
      ctx.lineWidth = 3.4;
      drawSegments(ctx.strokeStyle, ctx.lineWidth);
      ctx.restore();
    }

    drawRoute(ctx) {
      if (!this.routePoints.length) return;
      const screenPoints = this.routePoints
        .map((point) => ({
          point,
          screen: this.mapToCanvas(point.x, point.y),
        }))
        .filter((entry) => entry.screen);
      if (!screenPoints.length) return;

      ctx.save();
      ctx.lineWidth = 2.4;
      ctx.strokeStyle = "rgba(84, 205, 255, .76)";
      ctx.setLineDash([9, 5]);
      ctx.beginPath();
      screenPoints.forEach((entry, index) => {
        if (index === 0) ctx.moveTo(entry.screen.x, entry.screen.y);
        else ctx.lineTo(entry.screen.x, entry.screen.y);
      });
      ctx.stroke();
      ctx.setLineDash([]);

      screenPoints.forEach((entry, screenIndex) => {
        const routeIndex = this.routePoints.indexOf(entry.point);
        const active = routeIndex === this.activeRoutePointIndex;
        const radius = active ? 15 : 13;
        const status = entry.point.inspectionStatus;
        const color = active
          ? "#ffca62"
          : entry.point.alerted
            ? "#ff9f43"
            : status === "completed"
              ? "#45e39b"
              : status === "failed" || status === "skipped"
                ? "#e85b73"
                : entry.point.inspection
                  ? "#b98cff"
                  : "#4dc5ff";
        ctx.save();
        ctx.translate(entry.screen.x, entry.screen.y);
        ctx.rotate(-entry.point.yaw + this.originYaw());
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.lineWidth = active ? 3.2 : 2.2;
        ctx.beginPath();
        ctx.moveTo(radius + 13, 0);
        ctx.lineTo(radius + 3, -5);
        ctx.lineTo(radius + 3, 5);
        ctx.closePath();
        ctx.fill();
        ctx.beginPath();
        ctx.moveTo(radius - 1, 0);
        ctx.lineTo(radius + 13, 0);
        ctx.stroke();
        ctx.rotate(entry.point.yaw - this.originYaw());
        ctx.fillStyle = active
          ? "rgba(10, 25, 34, .96)"
          : "rgba(12, 31, 50, .94)";
        ctx.beginPath();
        if (entry.point.inspection) {
          ctx.moveTo(0, -radius);
          ctx.lineTo(radius, 0);
          ctx.lineTo(0, radius);
          ctx.lineTo(-radius, 0);
          ctx.closePath();
        } else {
          ctx.arc(0, 0, radius, 0, Math.PI * 2);
        }
        ctx.fill();
        ctx.strokeStyle = color;
        ctx.stroke();
        ctx.fillStyle = "#ffffff";
        ctx.font = `700 ${routeIndex + 1 >= 100 ? 10 : 12}px sans-serif`;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(routeIndex + 1), 0, 0.5);
        ctx.restore();
      });
      ctx.restore();
    }

    drawGoal(ctx) {
      if (!this.goalPose) return;
      const p = this.mapToCanvas(this.goalPose.x, this.goalPose.y);
      if (!p) return;
      ctx.save();
      ctx.strokeStyle = "#4dc5ff";
      ctx.fillStyle = "#4dc5ff";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 11, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    }
  }

  window.MapCanvas = MapCanvas;
})();
