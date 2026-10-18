#!/usr/bin/env python3
"""Serve the pole review pages for remote review (e.g. over Tailscale) and save decisions on this machine.

GET  /                      index with links to the tracks
GET  /<Track>.html          review page (decisions are loaded from / saved to the server)
GET  /viewer/<Track>        keyframe viewer (make_viewer.py output): map features projected with a trajectory
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


VIEWER_DIR = os.path.join(ROOT, 'viewer')
VIEWER_PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Viewer __TRACK__</title>
<style>
:root{--bg:#f6f7f9;--ink:#1d2330;--mute:#6b7280;--card:#fff}
@media (prefers-color-scheme:dark){:root{--bg:#111418;--ink:#e6e9ee;--mute:#9aa3af;--card:#1b2027}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
header{position:sticky;top:0;background:var(--bg);padding:8px 16px;border-bottom:1px solid #8883;display:flex;gap:10px;flex-wrap:wrap;align-items:center;z-index:2}
input[type=range]{flex:1;min-width:200px}button{font:inherit;padding:4px 10px;border-radius:6px;border:1px solid #8886;background:var(--card);color:var(--ink)}
main{padding:10px 16px}img{max-width:100%;border-radius:6px;display:block}
.row{display:flex;gap:12px;flex-wrap:wrap;align-items:flex-start;margin-top:10px}.bev{width:min(600px,100%)}
.leg span{display:inline-block;margin-right:12px}.sw{display:inline-block;width:12px;height:12px;border-radius:2px;vertical-align:-2px;margin-right:4px}
.mute{color:var(--mute);font-size:12px}
</style></head><body>
<header><b>__TRACK__</b><span class="mute" id="lab"></span>
<button id="prev">◀</button><input type="range" id="sl" min="0" value="0"><button id="next">▶</button>
<span id="fr"></span></header>
<main>
<div class="leg"><span><i class="sw" style="background:#f00"></i>地圖路面標線</span><span><i class="sw" style="background:#ffe600"></i>地圖道路邊界（路緣／分隔島）</span>
<span><i class="sw" style="background:#0ff"></i>地圖路緣高度落差</span><span><i class="sw" style="background:#0f0"></i>篩選後的桿子</span>
<span class="mute">左前｜前｜右前相機，用此軌跡投影；位置對得準＝定位準</span></div>
<img id="kf" alt="cameras">
<div class="row"><img id="bev" class="bev" alt="bev"><div class="mute" style="max-width:420px">俯視圖（車頭朝上，3–30 m）：灰＝影像的標線響應（三台相機各自計算），藍＝影像分割出的道路，
紅＝地圖標線，黃＝地圖道路邊界，青＝地圖路緣。紅點落在灰白線上、黃線貼著藍色區域邊緣，代表定位對準。鍵盤 ← → 切換。</div></div>
</main>
<script>
const T='__TRACK__'; let F=[];
fetch('/viewer/'+T+'/frames.json').then(r=>r.json()).then(j=>{F=j.frames;document.getElementById('lab').textContent=j.label;
  const s=document.getElementById('sl'); s.max=F.length-1; s.oninput=()=>show(+s.value); show(0);});
function show(k){k=Math.max(0,Math.min(F.length-1,k)); const i=String(F[k]).padStart(5,'0');
  document.getElementById('sl').value=k; document.getElementById('fr').textContent='frame '+F[k]+' ('+(k+1)+'/'+F.length+')';
  document.getElementById('kf').src='/viewer/'+T+'/kf_'+i+'.jpg'; document.getElementById('bev').src='/viewer/'+T+'/bev_'+i+'.jpg';}
document.getElementById('prev').onclick=()=>show(+document.getElementById('sl').value-1);
document.getElementById('next').onclick=()=>show(+document.getElementById('sl').value+1);
document.addEventListener('keydown',e=>{if(e.key==='ArrowRight')show(+document.getElementById('sl').value+1);
  if(e.key==='ArrowLeft')show(+document.getElementById('sl').value-1);});
</script></body></html>'''


def compare_page(track):
    """Side-by-side A/B/C comparison of trajectory variants (viewer dirs <Track>_<variant>) on common frames."""
    vs = sorted(d[len(track) + 1:] for d in os.listdir(VIEWER_DIR) if d.startswith(track + '_'))
    frames = None
    for v in vs:
        f = set(json.load(open(os.path.join(VIEWER_DIR, f'{track}_{v}', 'frames.json')))['frames'])
        frames = f if frames is None else frames & f
    frames = sorted(frames or [])
    return COMPARE_PAGE.replace('__TRACK__', track).replace('__VARIANTS__', json.dumps(vs)).replace('__FRAMES__', json.dumps(frames))


COMPARE_PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Compare __TRACK__</title>
<style>
:root{--bg:#f6f7f9;--ink:#1d2330;--mute:#6b7280;--card:#fff;--acc:#2563eb}
@media (prefers-color-scheme:dark){:root{--bg:#111418;--ink:#e6e9ee;--mute:#9aa3af;--card:#1b2027}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
header{position:sticky;top:0;background:var(--bg);padding:8px 16px;border-bottom:1px solid #8883;display:flex;gap:8px;flex-wrap:wrap;align-items:center;z-index:2}
button{font:inherit;padding:4px 10px;border-radius:6px;border:1px solid #8886;background:var(--card);color:var(--ink);cursor:pointer}
button.on{border-color:var(--acc);box-shadow:0 0 0 2px var(--acc)}
input[type=range]{flex:1;min-width:180px}main{padding:10px 16px}img{max-width:100%;display:block;border-radius:6px}
.mute{color:var(--mute);font-size:12px}
</style></head><body>
<header><b>__TRACK__ 對比</b><span id="vbtn"></span>
<button id="prev">◀</button><input type="range" id="sl" min="0" value="0"><button id="next">▶</button><span id="fr"></span></header>
<main><div class="mute">數字鍵 1/2/3 切換版本（同一幀快速切換最容易看出差別），← → 換幀。看紅點（地圖標線）是否落在白色標線上、綠線（桿子）是否貼著實物、黃線是否沿著路緣。</div>
<img id="kf" alt="cameras"><img id="bev" style="width:min(600px,100%);margin-top:8px" alt="bev"></main>
<script>
const T='__TRACK__', V=__VARIANTS__, F=__FRAMES__; let v=0, k=0;
const vb=document.getElementById('vbtn');
V.forEach((n,i)=>{const b=document.createElement('button'); b.textContent=(i+1)+'. '+n; b.onclick=()=>{v=i;show();}; vb.appendChild(b);});
const sl=document.getElementById('sl'); sl.max=F.length-1; sl.oninput=()=>{k=+sl.value;show();};
function show(){k=Math.max(0,Math.min(F.length-1,k)); sl.value=k; const i=String(F[k]).padStart(5,'0'), d='/viewer/'+T+'_'+V[v]+'/';
  document.getElementById('kf').src=d+'kf_'+i+'.jpg'; document.getElementById('bev').src=d+'bev_'+i+'.jpg';
  document.getElementById('fr').textContent='frame '+F[k]; [...vb.children].forEach((b,j)=>b.className=j===v?'on':'');}
document.getElementById('prev').onclick=()=>{k--;show();}; document.getElementById('next').onclick=()=>{k++;show();};
document.addEventListener('keydown',e=>{if(e.key==='ArrowRight'){k++;show();} if(e.key==='ArrowLeft'){k--;show();}
  const n=parseInt(e.key); if(n>=1&&n<=V.length){v=n-1;show();}});
show();
</script></body></html>'''


DATA_ROOT = os.environ.get('SDC_DATA', os.path.join(ROOT, '..', '..', '..', '..', 'data'))
HOOD_PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Ego mask editor</title>
<style>
:root{--bg:#f6f7f9;--ink:#1d2330;--mute:#6b7280;--card:#fff;--acc:#2563eb}
@media (prefers-color-scheme:dark){:root{--bg:#111418;--ink:#e6e9ee;--mute:#9aa3af;--card:#1b2027}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
header{position:sticky;top:0;background:var(--bg);padding:8px 16px;border-bottom:1px solid #8883;display:flex;gap:8px;flex-wrap:wrap;align-items:center;z-index:2}
button,select,input{font:inherit;padding:4px 10px;border-radius:6px;border:1px solid #8886;background:var(--card);color:var(--ink)}
button.on{border-color:var(--acc);box-shadow:0 0 0 2px var(--acc)}main{padding:10px 16px}
canvas{max-width:100%;border-radius:6px;cursor:crosshair;display:block}.mute{color:var(--mute);font-size:12px}
</style></head><body>
<header><b>自車遮罩編輯</b>
<select id="track"><option>Track1</option><option>Track2</option></select>
<span id="cams"></span>
<button id="fprev">◀ 幀</button><input id="frame" type="number" value="300" min="0" style="width:80px"><button id="fnext">幀 ▶</button>
<label><input type="checkbox" id="orig" checked> 顯示原本遮罩</label>
<button id="undo">復原點</button><button id="clear">清除</button><button id="save">儲存這台相機</button><span id="st" class="mute"></span>
</header><main>
<div class="mute">在影像上依序點出「要捨去的自車車體」外框（引擎蓋、A 柱、後照鏡…），多邊形會自動閉合；可以畫多個多邊形：畫完一個按 <b>Enter</b> 開始下一個。
紅色＝你畫的遮罩，藍色＝題目原本的遮罩（僅前相機）。換幀檢查遮罩在不同畫面都貼合。三台相機各自儲存；兩個 track 是同一台車，共用同一份遮罩。</div>
<canvas id="cv"></canvas></main>
<script>
let cam='f', polys=[[]], img=new Image(), orig=new Image(), haveOrig=false;
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
const cams=document.getElementById('cams');
['fl','f','fr'].forEach(c=>{const b=document.createElement('button');b.textContent=c;b.onclick=()=>{cam=c;loadMask();};cams.appendChild(b);});
function T(){return document.getElementById('track').value} function F(){return +document.getElementById('frame').value}
function loadImg(){img.onload=draw; img.src='/hoodimg/'+T()+'/'+cam+'/'+F()+'.jpg';
  haveOrig=false; if(cam==='f'){orig.onload=()=>{haveOrig=true;draw();}; orig.src='/hoodorig/'+T()+'.png';}
  [...cams.children].forEach(b=>b.className=b.textContent===cam?'on':'');}
async function loadMask(){try{const r=await fetch('/api/hood/'+cam); const j=await r.json(); polys=(j.polys&&j.polys.length)?j.polys.concat([[]]):[[]];}catch(e){polys=[[]];} loadImg();}
function draw(){if(!img.width)return; cv.width=img.width; cv.height=img.height; ctx.drawImage(img,0,0);
  if(document.getElementById('orig').checked&&haveOrig&&cam==='f'){ctx.globalAlpha=0.35; ctx.drawImage(orig,0,0,img.width,img.height); ctx.globalAlpha=1;}
  polys.forEach((p,k)=>{if(!p.length)return; ctx.beginPath(); p.forEach((q,i)=>i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1])); ctx.closePath();
    ctx.fillStyle='rgba(255,0,0,0.30)'; ctx.fill(); ctx.strokeStyle='#ff2020'; ctx.lineWidth=3; ctx.stroke();
    p.forEach(q=>{ctx.fillStyle='#ffff00'; ctx.fillRect(q[0]-4,q[1]-4,8,8);});});}
cv.onclick=e=>{const r=cv.getBoundingClientRect(); const x=(e.clientX-r.left)*cv.width/r.width, y=(e.clientY-r.top)*cv.height/r.height;
  polys[polys.length-1].push([Math.round(x),Math.round(y)]); draw();};
document.addEventListener('keydown',e=>{if(e.key==='Enter'&&polys[polys.length-1].length>2){polys.push([]);draw();}});
document.getElementById('undo').onclick=()=>{let p=polys[polys.length-1]; if(!p.length&&polys.length>1){polys.pop();p=polys[polys.length-1];} p.pop(); draw();};
document.getElementById('clear').onclick=()=>{polys=[[]];draw();};
document.getElementById('save').onclick=async()=>{const P=polys.filter(p=>p.length>2);
  const r=await fetch('/api/hood/'+cam,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({polys:P,w:cv.width,h:cv.height})});
  document.getElementById('st').textContent=r.ok?('已儲存 '+cam+'（'+P.length+' 個多邊形） '+new Date().toLocaleTimeString()):'儲存失敗';};
document.getElementById('track').onchange=loadImg; document.getElementById('frame').onchange=loadImg; document.getElementById('orig').onchange=draw;
document.getElementById('fprev').onclick=()=>{document.getElementById('frame').value=Math.max(0,F()-100);loadImg();};
document.getElementById('fnext').onclick=()=>{document.getElementById('frame').value=F()+100;loadImg();};
loadMask();
</script></body></html>'''


def hood_path(cam):
    return os.path.join(ROOT, f'ego_mask_{cam}.json')


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
                rows.append(f'<li><a href="/{t}.html">{t} 桿子審核</a> — {total} poles, decided {len(d)} ｜ '
                            f'<a href="/viewer/{t}">{t} 定位檢視</a> ｜ <a href="/compare/{t}">{t} 版本對比</a></li>')
            rows.append('<li><a href="/hood">自車遮罩編輯（引擎蓋／車體）</a></li>')
            return self._send(200, '<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">'
                                   '<title>Pole review</title><body style="font:16px system-ui;padding:16px">'
                                   '<h2>Pole review</h2><ul>' + ''.join(rows) + '</ul></body>')
        if p in ('/hood', '/hood/'):
            return self._send(200, HOOD_PAGE)
        m = re.fullmatch(r'/hoodimg/(Track[12])/(f|fl|fr)/(\d{1,5})\.jpg', p)
        if m:
            t, c, i = m.group(1), m.group(2), int(m.group(3))
            d = os.path.join(DATA_ROOT, t, f'{t}_images', f'{t}_{c}_images')
            names = sorted(os.listdir(d))
            return self._send(200, open(os.path.join(d, names[min(i, len(names) - 1)]), 'rb').read(), 'image/jpeg')
        m = re.fullmatch(r'/hoodorig/(Track[12])\.png', p)
        if m:
            t = m.group(1)
            f = os.path.join(DATA_ROOT, t, f'{t}_images', 'camera_info', 'gige_100_f_hdr_mask.png')
            return self._send(200, open(f, 'rb').read(), 'image/png')
        m = re.fullmatch(r'/api/hood/(f|fl|fr)', p)
        if m:
            f = hood_path(m.group(1))
            return self._send(200, open(f).read() if os.path.exists(f) else '{}', 'application/json')
        m = re.fullmatch(r'/compare/(Track[12])/?', p)
        if m:
            return self._send(200, compare_page(m.group(1)))
        m = re.fullmatch(r'/viewer/(Track[12](?:_[A-Za-z0-9._]+)?)/?', p)
        if m:
            return self._send(200, VIEWER_PAGE.replace('__TRACK__', m.group(1)))
        m = re.fullmatch(r'/viewer/(Track[12](?:_[A-Za-z0-9._]+)?)/((?:kf|bev)_\d{5}\.jpg|frames\.json)', p)
        if m:
            f = os.path.join(VIEWER_DIR, m.group(1), m.group(2))
            if not os.path.exists(f):
                return self._send(404, 'not found')
            ctype = 'application/json' if f.endswith('.json') else 'image/jpeg'
            return self._send(200, open(f, 'rb').read(), ctype)
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
        mh = re.fullmatch(r'/api/hood/(f|fl|fr)', self.path.split('?')[0])
        if mh:
            n = int(self.headers.get('Content-Length', 0))
            if n > 1_000_000:
                return self._send(413, 'too large')
            try:
                j = json.loads(self.rfile.read(n))
                polys = [[[int(a), int(b)] for a, b in p] for p in j['polys'] if len(p) >= 3]
                out = {'cam': mh.group(1), 'w': int(j['w']), 'h': int(j['h']), 'polys': polys}
            except Exception as e:  # noqa: BLE001
                return self._send(400, f'bad request: {e}')
            tmp = hood_path(mh.group(1)) + '.tmp'
            json.dump(out, open(tmp, 'w'))
            os.replace(tmp, hood_path(mh.group(1)))
            return self._send(200, json.dumps({'ok': True, 'polys': len(polys)}), 'application/json')
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
