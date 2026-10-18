#!/usr/bin/env python3
"""Serve the pole review pages for remote review (e.g. over Tailscale) and save decisions on this machine.

GET  /                      index with links to the tracks
GET  /<Track>.html          review page (decisions are loaded from / saved to the server)
GET  /api/<Track>           current decisions
POST /api/<Track>           {"dec": {id: "keep"|"reject"}} -> work/pole_review_<Track>.json (apply_pole_review format)

usage: pole_review_server.py --host 100.x.y.z --port 8765   (bind to the Tailscale IP only)
"""
import argparse
import json
import os
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'work')
PAGES = os.path.join(ROOT, 'pole_review')
TRACKS = ['Track1', 'Track2']

SYNC = r'''
<script>
(async () => {
  const TRACK = '__TRACK__';
  try {
    const r = await fetch('/api/' + TRACK); const j = await r.json();
    if (j && j.dec) { for (const k in dec) delete dec[k]; Object.assign(dec, j.dec); render(); }
  } catch (e) {}
  const status = document.createElement('span'); status.className = 'pill'; status.textContent = '已連線：自動存到伺服器';
  document.querySelector('header').appendChild(status);
  let t = null;
  const push = () => { clearTimeout(t); t = setTimeout(async () => {
    try { const r = await fetch('/api/' + TRACK, {method: 'POST', headers: {'Content-Type': 'application/json'},
                                body: JSON.stringify({dec})});
          status.textContent = r.ok ? '已存到伺服器 ' + new Date().toLocaleTimeString() : '儲存失敗';
    } catch (e) { status.textContent = '儲存失敗（連線中斷）'; } }, 300); };
  document.addEventListener('click', push, true); document.addEventListener('keyup', push, true);
})();
</script></body>'''


def decisions_path(track):
    return os.path.join(ROOT, f'pole_review_{track}.json')


def n_poles(track):
    s = open(os.path.join(PAGES, f'{track}.html')).read()
    return json.loads(re.search(r'const DATA = (\[.*?\]); const KEY', s, re.S).group(1))


class H(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype='text/html; charset=utf-8'):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(b)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        p = self.path.split('?')[0]
        if p in ('/', '/index.html'):
            rows = []
            for t in TRACKS:
                d = {}
                if os.path.exists(decisions_path(t)):
                    d = json.load(open(decisions_path(t))).get('dec', {})
                total = len(n_poles(t))
                rows.append(f'<li><a href="/{t}.html">{t}</a> — {total} poles, decided {len(d)}</li>')
            return self._send(200, '<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">'
                                   '<title>Pole review</title><body style="font:16px system-ui;padding:16px">'
                                   '<h2>Pole review</h2><ul>' + ''.join(rows) + '</ul></body>')
        m = re.fullmatch(r'/(Track[12])\.html', p)
        if m:
            t = m.group(1)
            html = open(os.path.join(PAGES, f'{t}.html')).read()
            return self._send(200, html.replace('</body>', SYNC.replace('__TRACK__', t), 1))
        m = re.fullmatch(r'/api/(Track[12])', p)
        if m:
            t = m.group(1)
            data = json.load(open(decisions_path(t))) if os.path.exists(decisions_path(t)) else {'dec': {}}
            return self._send(200, json.dumps(data), 'application/json')
        self._send(404, 'not found')

    def do_POST(self):
        m = re.fullmatch(r'/api/(Track[12])', self.path.split('?')[0])
        if not m:
            return self._send(404, 'not found')
        t = m.group(1)
        n = int(self.headers.get('Content-Length', 0))
        if n > 2_000_000:
            return self._send(413, 'too large')
        try:
            dec = json.loads(self.rfile.read(n))['dec']
            ids = [c['id'] for c in n_poles(t)]
            dec = {str(k): v for k, v in dec.items() if v in ('keep', 'reject') and int(k) in set(ids)}
        except Exception as e:  # noqa: BLE001
            return self._send(400, f'bad request: {e}')
        out = {'track': t, 'dec': dec,
               'keep': [i for i in ids if dec.get(str(i)) == 'keep'],
               'reject': [i for i in ids if dec.get(str(i)) == 'reject'],
               'undecided': [i for i in ids if str(i) not in dec]}
        tmp = decisions_path(t) + '.tmp'
        json.dump(out, open(tmp, 'w'), indent=1)
        os.replace(tmp, decisions_path(t))
        self._send(200, json.dumps({'ok': True, 'decided': len(dec)}), 'application/json')

    def log_message(self, fmt, *args):
        if self.command == 'POST':
            print(self.address_string(), fmt % args, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True, help='bind address, e.g. the Tailscale IP')
    ap.add_argument('--port', type=int, default=8765)
    a = ap.parse_args()
    print(f'serving pole review on http://{a.host}:{a.port}/', flush=True)
    ThreadingHTTPServer((a.host, a.port), H).serve_forever()


if __name__ == '__main__':
    main()
