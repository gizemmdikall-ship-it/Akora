// Headless renderer: node render.mjs jobs.json <three_dir> <root_dir>
// Each job: {glb (path relative to root), out, cam:[x,y,z], target:[x,y,z], fov, w, h, lines}
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const { chromium } = require('playwright');

const [jobsFile, threeDir, rootDir] = process.argv.slice(2);
const jobs = JSON.parse(fs.readFileSync(jobsFile, 'utf8'));
const here = path.dirname(new URL(import.meta.url).pathname);
const types = { '.html': 'text/html', '.js': 'text/javascript', '.glb': 'model/gltf-binary' };

const server = http.createServer((req, res) => {
  const u = decodeURIComponent(req.url.split('?')[0].split('#')[0]);
  let f;
  if (u === '/render.html') f = path.join(here, 'render.html');
  else if (u.startsWith('/three/')) f = path.join(threeDir, u.slice(7));
  else f = path.join(rootDir, u);
  fs.readFile(f, (err, data) => {
    if (err) { res.writeHead(404); res.end(); return; }
    res.writeHead(200, { 'Content-Type': types[path.extname(f)] || 'application/octet-stream' });
    res.end(data);
  });
}).listen(0);
const port = server.address().port;

const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
for (const job of jobs) {
  const page = await browser.newPage({ viewport: { width: job.w || 1600, height: job.h || 1000 } });
  page.on('console', m => { if (m.type() === 'error') console.log('console:', m.text()); });
  await page.goto(`http://localhost:${port}/render.html#${encodeURIComponent(JSON.stringify(job))}`);
  await page.waitForFunction(() => window.__done || window.__err, null, { timeout: 600000 });
  const err = await page.evaluate(() => window.__err);
  if (err) { console.log('ERR', job.out, err); await page.close(); continue; }
  await page.locator('canvas').screenshot({ path: job.out });
  console.log('rendered', job.out);
  await page.close();
}
await browser.close();
server.close();
