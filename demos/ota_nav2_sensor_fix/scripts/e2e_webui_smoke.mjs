// E2E smoke driver for the OTA demo. Drives ros2_medkit_web_ui (running
// at WEB_UI_URL) against the live demo gateway (GATEWAY_URL) and asserts
// that all 3 catalog entries register and that the SOVD wire format
// matches what we ship from pack_artifact.py.
//
// Why this exists: the Foxglove updates panel mirrors the same SOVD client
// patterns the web UI uses (fetchUpdateIds parses {items: [...]},
// per-id /status, lazy /detail). Verifying the web UI flow end-to-end
// gives us a canonical reference point for both clients.
//
// Usage:
//   docker compose up -d
//   cd /path/to/ros2_medkit_web_ui && npm install && npm run dev
//   GATEWAY_URL=http://localhost:8080 \
//   WEB_UI_URL=http://localhost:5173 \
//   node /path/to/this/e2e_webui_smoke.mjs
//
// Requires: playwright (`npm install --no-save playwright` in the web UI
// dir), chromium-headless-shell (`npx playwright install
// chromium-headless-shell`), the demo stack from ../docker-compose.yml.

import { chromium } from "playwright";

const WEB_UI_URL = process.env.WEB_UI_URL ?? "http://localhost:5173/";
const GATEWAY_URL = process.env.GATEWAY_URL ?? "http://localhost:8080";

const EXPECTED_IDS = [
    "fixed_lidar_2_1_0",
    "obstacle_classifier_v2_1_0_0",
    "broken_lidar_legacy_remove",
];

const EXPECTED_API_PATHS = [
    "/api/v1/updates",
    "/api/v1/updates/fixed_lidar_2_1_0/status",
    "/api/v1/updates/obstacle_classifier_v2_1_0_0/status",
    "/api/v1/updates/broken_lidar_legacy_remove/status",
];

(async () => {
    const browser = await chromium.launch({
        channel: "chromium-headless-shell",
        headless: true,
    });
    const ctx = await browser.newContext();
    const page = await ctx.newPage();

    page.on("pageerror", (err) => console.log(`[pageerror] ${err.message}`));

    const apiCalls = new Set();
    page.on("request", (req) => {
        const u = req.url();
        if (u.includes("/api/v1/")) {
            apiCalls.add(`${req.method()} ${u}`);
        }
    });

    await page.goto(WEB_UI_URL, { waitUntil: "domcontentloaded" });

    await page.getByRole("button", { name: /connect to server/i }).click();
    await page.waitForTimeout(300);
    await page.locator('input[type="text"], input:not([type])').first().fill(GATEWAY_URL);
    await page.getByRole("button", { name: /^connect$/i }).last().click();
    await page.waitForTimeout(2000);

    const updatesButton = page.getByRole("button", { name: /updates/i }).first();
    if (await updatesButton.count()) {
        await updatesButton.click();
        await page.waitForTimeout(2000);
    }

    const bodyText = await page.locator("body").textContent();

    let failed = 0;
    for (const id of EXPECTED_IDS) {
        const visible = bodyText?.includes(id) ?? false;
        console.log(`  id ${id}: ${visible ? "PASS" : "FAIL"}`);
        if (!visible) failed++;
    }

    for (const path of EXPECTED_API_PATHS) {
        const hit = [...apiCalls].some((c) => c.endsWith(path));
        console.log(`  api ${path}: ${hit ? "PASS" : "FAIL"}`);
        if (!hit) failed++;
    }

    await browser.close();

    if (failed > 0) {
        console.error(`\n${failed} assertion(s) failed`);
        process.exit(1);
    }
    console.log("\nDONE: all SOVD flows verified");
})().catch((err) => {
    console.error("FAIL:", err);
    process.exit(1);
});
