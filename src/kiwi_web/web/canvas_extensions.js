/* ROS origin yaw, laser overlay and touch support for the reference canvas. */
(() => {
  "use strict";
  class KiwiMapCanvas extends window.MapCanvas {
    originYaw() {
      const q = this.mapInfo?.origin.orientation;
      return q
        ? Math.atan2(
            2 * ((q.w ?? 1) * (q.z ?? 0) + (q.x ?? 0) * (q.y ?? 0)),
            1 - 2 * ((q.y ?? 0) ** 2 + (q.z ?? 0) ** 2),
          )
        : 0;
    }
    localToWorld(x, y) {
      const a = this.originYaw(),
        p = this.mapInfo.origin.position;
      return {
        x: p.x + x * Math.cos(a) - y * Math.sin(a),
        y: p.y + x * Math.sin(a) + y * Math.cos(a),
      };
    }
    worldToLocal(x, y) {
      const a = this.originYaw(),
        p = this.mapInfo.origin.position,
        dx = x - p.x,
        dy = y - p.y;
      return {
        x: dx * Math.cos(a) + dy * Math.sin(a),
        y: -dx * Math.sin(a) + dy * Math.cos(a),
      };
    }
    canvasToMap(sx, sy, clamp) {
      if (!this.mapInfo) return null;
      const i = this.mapInfo,
        w = i.width * i.resolution,
        h = i.height * i.resolution;
      let x = (sx - this.offsetX) / this.zoom,
        y = h - (sy - this.offsetY) / this.zoom;
      if (clamp) {
        x = Math.max(0, Math.min(w - 1e-9, x));
        y = Math.max(0, Math.min(h - 1e-9, y));
      }
      if (x < 0 || y < 0 || x >= w || y >= h) return null;
      return this.localToWorld(x, y);
    }
    mapToGrid(x, y) {
      if (!this.mapInfo) return null;
      const p = this.worldToLocal(x, y),
        i = this.mapInfo;
      const gx = Math.floor(p.x / i.resolution + 1e-10),
        gy = Math.floor(p.y / i.resolution + 1e-10);
      return gx >= 0 && gy >= 0 && gx < i.width && gy < i.height
        ? { x: gx, y: gy }
        : null;
    }
    mapToCanvas(x, y) {
      if (!this.mapInfo) return null;
      const p = this.worldToLocal(x, y);
      return {
        x: this.offsetX + p.x * this.zoom,
        y:
          this.offsetY +
          (this.mapInfo.height * this.mapInfo.resolution - p.y) * this.zoom,
      };
    }
    setPoseInteraction(enabled, callback) {
      super.setPoseInteraction(enabled, (x, y, a) =>
        callback(x, y, a + this.originYaw()),
      );
    }
    setGlobalPath(points) {
      super.setGlobalPath(points.map((p) => ({ ...p, z: 0 })));
    }
    bindEvents() {
      this.pointers = new Map();
      this.canvas.addEventListener("contextmenu", (e) => e.preventDefault());
      this.canvas.addEventListener("wheel", (e) => this.handleWheel(e), {
        passive: false,
      });
      this.canvas.addEventListener("pointerdown", (e) => {
        if (!this.mapInfo) return;
        this.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        this.canvas.setPointerCapture(e.pointerId);
        if (this.pointers.size === 2) {
          const [a, b] = [...this.pointers.values()];
          this.pinch = {
            distance: Math.hypot(a.x - b.x, a.y - b.y),
            zoom: this.zoom,
          };
          this.pan = null;
          this.dragPose = null;
          if (this.gesture) this.commitEdit();
          return;
        }
        if (
          e.button === 0 &&
          !this.poseInteraction.enabled &&
          !this.editorInteraction?.enabled
        ) {
          this.pan = {
            x: e.clientX,
            y: e.clientY,
            offsetX: this.offsetX,
            offsetY: this.offsetY,
          };
        } else this.handleMouseDown(e);
      });
      this.canvas.addEventListener("pointermove", (e) => {
        if (this.pointers.has(e.pointerId))
          this.pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        if (this.pinch && this.pointers.size === 2) {
          const [a, b] = [...this.pointers.values()],
            rect = this.canvas.getBoundingClientRect();
          const x = (a.x + b.x) / 2 - rect.left,
            y = (a.y + b.y) / 2 - rect.top;
          const px = (x - this.offsetX) / this.zoom,
            py = (y - this.offsetY) / this.zoom;
          this.zoom = Math.max(
            this.baseZoom * 0.02,
            Math.min(
              this.baseZoom * 24,
              (this.pinch.zoom * Math.hypot(a.x - b.x, a.y - b.y)) /
                Math.max(1, this.pinch.distance),
            ),
          );
          this.offsetX = x - px * this.zoom;
          this.offsetY = y - py * this.zoom;
          return;
        }
        this.handleMouseMove(e);
      });
      const end = (e) => {
        this.pointers.delete(e.pointerId);
        if (this.pinch) {
          this.pinch = null;
          this.pan = null;
          return;
        }
        this.handleMouseUp(e);
        this.pan = null;
      };
      this.canvas.addEventListener("pointerup", end);
      this.canvas.addEventListener("pointercancel", (e) => {
        this.pointers.delete(e.pointerId);
        this.pinch = null;
        this.pan = null;
        this.dragPose = null;
        if (this.gesture) this.commitEdit();
      });
      this.resizeObserver = new ResizeObserver(() => this.resize());
      this.resizeObserver.observe(this.container);
    }
    renderLoop() {
      if (
        !document.hidden &&
        (!this.lastRender || performance.now() - this.lastRender > 32)
      ) {
        this.render();
        this.lastRender = performance.now();
      }
      requestAnimationFrame(() => this.renderLoop());
    }
    render() {
      super.render();
      if (!this.mapInfo || !this.scanPoints) return;
      const ctx = this.ctx;
      ctx.fillStyle = "#ff765b";
      for (const [x, y] of this.scanPoints) {
        const p = this.mapToCanvas(x, y);
        if (p && p.x >= 0 && p.y >= 0 && p.x < this.width && p.y < this.height)
          ctx.fillRect(p.x - 1, p.y - 1, 2, 2);
      }
    }
  }
  window.MapCanvas = KiwiMapCanvas;
})();
