/* Run against browser_fixture.py. Uses installed Chrome and a temporary Playwright install. */
const { chromium } = require("playwright");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");

async function main() {
  const browser = await chromium.launch({
    executablePath: process.env.CHROME || "/usr/bin/google-chrome",
    headless: true,
    args: ["--no-sandbox"],
  });
  const context = await browser.newContext({
    viewport: { width: 1440, height: 960 },
  });
  const page = await context.newPage();
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  const base = "http://127.0.0.1:8081";
  const output = process.env.KIWI_SCREENSHOTS || "/tmp/kiwi-web-screenshots";
  fs.mkdirSync(output, { recursive: true });
  const status = async () =>
    (await context.request.get(base + "/api/status")).json();
  async function until(predicate) {
    const deadline = Date.now() + 10000;
    while (Date.now() < deadline) {
      if (await predicate()) return;
      await new Promise((resolve) => setTimeout(resolve, 50));
    }
    throw new Error("Browser condition timed out");
  }
  async function open(name) {
    await page.goto(`${base}/${name}.html`);
    await page.waitForFunction(() =>
      document
        .getElementById("connection-status")
        .textContent.includes("已连接"),
    );
  }
  async function confirm() {
    await page.locator("#confirm-dialog[open]").waitFor();
    await page.click("#confirm-yes");
  }
  async function screenshot(name) {
    await page.screenshot({
      path: path.join(output, name + ".png"),
      fullPage: true,
    });
  }

  try {
    await open("mapping");
    await page.click("#btn-start-mapping");
    await until(async () => {
      const value = await status();
      return (
        value.mode === "mapping" &&
        value.control.manual_ready &&
        value.robot.has_map
      );
    });
    await page.waitForFunction(() =>
      document.getElementById("map-detail").textContent.includes("160"),
    );
    await screenshot("mapping");
    const coordinates = await page.evaluate(() => {
      const container = document.createElement("div");
      container.style.cssText =
        "position:fixed;width:400px;height:300px;visibility:hidden";
      const element = document.createElement("canvas");
      container.append(element);
      document.body.append(container);
      const map = new window.MapCanvas(element, container);
      map.setMap(
        {
          width: 10,
          height: 20,
          resolution: 0.1,
          origin: {
            position: { x: 2, y: 3 },
            orientation: { z: Math.sin(Math.PI / 4), w: Math.cos(Math.PI / 4) },
          },
        },
        new Int8Array(200),
      );
      const world = map.localToWorld(0.25, 0.35),
        screen = map.mapToCanvas(world.x, world.y);
      const back = map.canvasToMap(screen.x, screen.y),
        grid = map.mapToGrid(world.x, world.y);
      const cells = new Map();
      map.addCell(grid.x, grid.y, cells);
      container.remove();
      return { world, back, grid, index: [...cells.values()][0].serviceIndex };
    });
    assert.ok(
      Math.abs(coordinates.world.x - 1.65) < 1e-8 &&
        Math.abs(coordinates.world.y - 3.25) < 1e-8,
    );
    assert.deepEqual(coordinates.grid, { x: 2, y: 3 });
    assert.ok(Math.abs(coordinates.world.x - coordinates.back.x) < 1e-8);
    assert.equal(coordinates.index, 162);
    await page.fill("#map-name", "browser_saved");
    await page.click("#btn-save-map");
    await until(async () =>
      (await context.request.get(base + "/api/maps/browser_saved")).ok(),
    );
    await page.click("#remote-toggle");
    await page.waitForFunction(
      () =>
        document
          .getElementById("remote-toggle")
          .getAttribute("aria-pressed") === "true",
    );
    await page.keyboard.down("w");
    await new Promise((resolve) => setTimeout(resolve, 100));
    assert.ok((await status()).control.owner);
    await page.keyboard.up("w");
    await page.evaluate(() => window.dispatchEvent(new Event("blur")));
    await until(async () => !(await status()).control.owner);
    // Shared stop button preserves the reference page label with one common id.
    await page.click("#btn-stop-mode");
    await confirm();
    await until(async () => (await status()).mode === "idle");

    await open("map_editor");
    await page.selectOption("#map-selector", "browser_saved");
    await page.click("#btn-open-map");
    await page.waitForFunction(
      () =>
        document.getElementById("editing-map-display").textContent ===
        "browser_saved",
    );
    const before = await (
      await context.request.get(base + "/api/maps/browser_saved")
    ).json();
    await page.mouse.move(320, 380);
    await page.mouse.down();
    await page.mouse.move(350, 410, { steps: 8 });
    await page.mouse.up();
    await page.waitForFunction(
      () =>
        document.getElementById("edit-status-display").textContent === "未保存",
    );
    await page.click("#btn-undo");
    assert.equal(
      await page.locator("#edit-status-display").textContent(),
      "已保存",
    );
    await page.click("#btn-redo");
    await page.click("#btn-shape-rectangle");
    await page.mouse.move(380, 420);
    await page.mouse.down();
    await page.mouse.move(420, 460);
    await page.mouse.up();
    await screenshot("map_editor");
    await page.click("#btn-save-edits");
    await until(
      async () =>
        (
          await (
            await context.request.get(base + "/api/maps/browser_saved")
          ).json()
        ).revision !== before.revision,
    );
    const after = await (
      await context.request.get(base + "/api/maps/browser_saved")
    ).json();
    assert.deepEqual(after.origin, before.origin);
    assert.equal(after.resolution, before.resolution);

    await open("navigation");
    await page.selectOption("#map-selector", "browser_saved");
    await page.click("#btn-load-map");
    await until(
      async () =>
        (await status()).control.nav_ready &&
        (await status()).current_map === "browser_saved",
    );
    await page.waitForFunction(() =>
      document.getElementById("map-detail").textContent.includes("160"),
    );
    await page.mouse.move(330, 390);
    await page.mouse.down();
    await page.mouse.move(370, 390);
    await page.mouse.up();
    await until(async () =>
      (await page.locator("#log-area").textContent()).includes(
        "初始位姿已发布",
      ),
    );
    await page.click("#btn-goal-mode");
    await page.mouse.move(430, 490);
    await page.mouse.down();
    await page.mouse.move(470, 490);
    await page.mouse.up();
    await page.click("#btn-start-goal");
    await until(async () => (await status()).navigation.state === "completed");
    await page.click("#btn-multi-navigation");
    for (const [x, y] of [
      [350, 350],
      [420, 450],
    ]) {
      await page.mouse.move(x, y);
      await page.mouse.down();
      await page.mouse.move(x + 30, y);
      await page.mouse.up();
    }
    await page.fill("#route-name", "browser_route");
    await page.click("#btn-save-route");
    await until(
      async () =>
        (
          await (
            await context.request.get(base + "/api/maps/browser_saved/routes")
          ).json()
        ).browser_route?.length === 2,
    );
    await page.click("#btn-start-route");
    await until(async () => (await status()).navigation.state === "running");
    await page.click("#btn-pause-route");
    await until(async () => (await status()).navigation.state === "paused");
    await page.click("#btn-resume-route");
    await until(async () => (await status()).navigation.state === "completed");
    await screenshot("navigation");

    await open("remote_control");
    await page.click("#btn-start-remote");
    await until(
      async () =>
        (await status()).mode === "remote" &&
        (await status()).control.manual_ready,
    );
    await page.click("#remote-toggle");
    await page.waitForFunction(
      () =>
        document
          .getElementById("remote-toggle")
          .getAttribute("aria-pressed") === "true",
    );
    const forward = await page.locator("[data-move=w]").boundingBox();
    await page.mouse.move(forward.x + 30, forward.y + 20);
    await page.mouse.down();
    await new Promise((resolve) => setTimeout(resolve, 100));
    await page.mouse.up();
    await page.click("#global-stop");
    await until(async () => (await status()).control.estop);
    await screenshot("remote_control");
    await page.click("#banner-reset");
    await confirm();
    await until(async () => !(await status()).control.estop);
    assert.equal((await status()).control.owner, null);
    await open("settings");
    const png = await (
      await context.request.get(base + "/api/maps/browser_saved/image")
    ).body();
    await page.fill("#import-name", "browser_imported");
    await page.setInputFiles("#import-image", {
      name: "import.png",
      mimeType: "image/png",
      buffer: png,
    });
    await page.setInputFiles("#import-yaml", {
      name: "import.yaml",
      mimeType: "text/yaml",
      buffer: Buffer.from(
        "image: import.png\nresolution: 0.1\norigin: [0, 0, 0]\nnegate: 0\nfree_thresh: 0.196\noccupied_thresh: 0.65\n",
      ),
    });
    await page.click("#btn-import-map");
    await until(async () =>
      (await context.request.get(base + "/api/maps/browser_imported")).ok(),
    );
    await page.selectOption("#map-selector", "browser_imported");
    const downloadPromise = page.waitForEvent("download");
    await page.click("#btn-export-map");
    assert.equal(
      (await downloadPromise).suggestedFilename(),
      "browser_imported.zip",
    );
    await page.click("#btn-delete-map");
    await confirm();
    await until(
      async () =>
        (
          await context.request.get(base + "/api/maps/browser_imported")
        ).status() === 404,
    );
    await screenshot("settings");
    for (const name of [
      "mapping",
      "map_editor",
      "navigation",
      "remote_control",
      "settings",
    ]) {
      await page.setViewportSize({ width: 390, height: 844 });
      await open(name);
      assert.ok(
        await page.evaluate(() => document.body.scrollWidth <= innerWidth),
        `${name} mobile overflow`,
      );
      await screenshot(name + "_mobile");
    }
    assert.deepEqual(errors, []);
    console.log(
      "Browser E2E passed: mapping/save, manual ownership/blur, brush/rectangle/undo/redo/save, initial pose, single goal, route/save/pause/resume, emergency/reset, five mobile pages.",
    );
    console.log("Screenshots: " + output);
  } catch (error) {
    console.error("Page log:", await page.locator("#log-area").textContent());
    console.error("Browser errors:", errors);
    await screenshot("failure");
    throw error;
  } finally {
    await browser.close();
  }
}
main().catch((error) => {
  console.error(error);
  process.exit(1);
});
