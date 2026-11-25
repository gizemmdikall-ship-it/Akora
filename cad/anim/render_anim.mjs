// node anim/render_anim.mjs <threeDir> <dataDir> <framesDir> [workers] [w] [h] [from] [to] [step]
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const { chromium } = require('playwright');

const [threeDir, dataDir, framesDir, workersS = '3', wS = '1920', hS = '1080', fromS = '0', toS = '1800', stepS = '1'] = process.argv.slice(2);
const here = path.dirname(new URL(import.meta.url).pathname);
const FONT = '/usr/share/fonts/truetype/dejavu';
const types = { '.html': 'text/html', '.js': 'text/javascript', '.glb': 'model/gltf-binary', '.json': 'application/json', '.ttf': 'font/ttf' };
const server = http.createServer((req, res) => {
  const u = decodeURIComponent(req.url.split('?')[0]);
  let f;
  if (u === '/anim.html') f = path.join(here, 'anim.html');
  else if (u.startsWith('/three/')) f = path.join(threeDir, u.slice(7));
  else if (u.startsWith('/font/')) f = path.join(FONT, u.slice(6));
  else f = path.join(dataDir, u);
  fs.readFile(f, (e, d) => { if (e) { res.writeHead(404); res.end(); return; }
    res.writeHead(200, { 'Content-Type': types[path.extname(f)] || 'application/octet-stream' }); res.end(d); });
}).listen(0);
const port = server.address().port;
fs.mkdirSync(framesDir, { recursive: true });
const W = +wS, H = +hS, from = +fromS, to = +toS, step = +stepS, NW = +workersS;
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
const t0 = Date.now();
await Promise.all([...Array(NW).keys()].map(async (w) => {
  const page = await browser.newPage({ viewport: { width: W, height: H } });
  page.on('console', m => { if (m.type() === 'error') console.log('console:', m.text()); });
  page.on('pageerror', e => console.log('pageerror:', e.message));
  await page.goto(`http://localhost:${port}/anim.html?w=${W}&h=${H}`);
  await page.waitForFunction(() => window.__ready, null, { timeout: 600000 });
  let n = 0;
  for (let i = from + w * step; i < to; i += NW * step) {
    const out = path.join(framesDir, `f_${String(i).padStart(5, '0')}.jpg`);
    if (fs.existsSync(out)) continue;
    await page.evaluate((k) => window.renderFrame(k), i);
    await page.screenshot({ path: out, type: 'jpeg', quality: 93 });
    if (++n % 50 === 0) console.log(`worker ${w}: ${n} frames, ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  }
}));
await browser.close(); server.close();
console.log('done', ((Date.now() - t0) / 1000).toFixed(0), 's');
