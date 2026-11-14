// Renders Informe.html to "../Murmur - Informe Final.pdf" with headless Chrome
// (via the DevTools protocol), filling the page numbers of the table of
// contents in a second pass. Requires Node.js >= 22, Google Chrome and
// pdftotext (poppler).
//
// Usage: node doc/report/build.mjs [path/to/chrome]

import { spawn, execFileSync } from "node:child_process";
import { mkdtempSync, readFileSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath, pathToFileURL } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const source = join(here, "Informe.html");
const output = resolve(here, "..", "Murmur - Informe Final.pdf");
const chrome = process.argv[2] || "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";
const work = mkdtempSync(join(tmpdir(), "murmur-report-"));
const port = 9300 + Math.floor(Math.random() * 500);

const browser = spawn(chrome, ["--headless=new", `--remote-debugging-port=${port}`, `--user-data-dir=${join(work, "profile")}`, "--no-first-run", "about:blank"], { stdio: "ignore" });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function target() {
	for (let k = 0; k < 50; ++k) {
		try {
			const list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
			const page = list.find((t) => t.type === "page");
			if (page) return page.webSocketDebuggerUrl;
		} catch {}
		await sleep(200);
	}
	throw new Error("Chrome did not start.");
}

const ws = new WebSocket(await target());
await new Promise((r) => ws.addEventListener("open", r, { once: true }));
let nextId = 0;
const pending = new Map();
const events = [];
ws.addEventListener("message", (m) => {
	const msg = JSON.parse(m.data);
	if (msg.id && pending.has(msg.id)) { pending.get(msg.id)(msg); pending.delete(msg.id); }
	else events.push(msg);
});
const send = (method, params = {}) => new Promise((r) => { const id = ++nextId; pending.set(id, r); ws.send(JSON.stringify({ id, method, params })); });

async function print(htmlPath, pdfPath) {
	await send("Page.enable");
	events.length = 0;
	await send("Page.navigate", { url: pathToFileURL(htmlPath).href });
	while (!events.some((e) => e.method === "Page.loadEventFired")) await sleep(50);
	await sleep(300);
	const result = await send("Page.printToPDF", {
		preferCSSPageSize: true,
		printBackground: true,
		displayHeaderFooter: true,
		headerTemplate: "<span></span>",
		footerTemplate: `<div style="width:100%;font:8pt Helvetica,Arial,sans-serif;color:#5b6475;padding:0 20mm;display:flex;justify-content:space-between"><span>Murmur · Informe final</span><span>⟨<span class="pageNumber"></span>⟩</span></div>`,
	});
	writeFileSync(pdfPath, Buffer.from(result.result.data, "base64"));
}

// First pass: locate every heading.
const html = readFileSync(source, "utf8");
const first = join(work, "first.pdf");
await print(source, first);
const pages = execFileSync("pdftotext", ["-layout", first, "-"], { encoding: "utf8" }).split("\f");
const tocPage = pages.findIndex((p) => p.includes("Tabla de contenidos"));
const numbered = html.replace(/<span class="pg" data-find="([^"]+)">\?<\/span>/g, (all, title) => {
	const page = pages.findIndex((p, k) => k > tocPage && p.split("\n").some((line) => line.trim().startsWith(title)));
	if (page < 0) throw new Error(`Heading not found: ${title}`);
	return `<span class="pg">${page + 1}</span>`;
});
const second = join(here, ".Informe.numbered.html");
writeFileSync(second, numbered);
await print(second, output);
execFileSync("rm", ["-f", second]);
ws.close();
browser.kill();
console.log(`Generated: ${output}`);
