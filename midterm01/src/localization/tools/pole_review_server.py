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
  const s=document.getElementById('sl'); s.max=F.length-1; s.oninput=()=>show(+s.value);
  const h=parseInt(location.hash.slice(1)); let k0=0; if(h>=0){let b=1e9; F.forEach((f,k)=>{if(Math.abs(f-h)<b){b=Math.abs(f-h);k0=k;}});} show(k0);});
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


CALIB_PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Camera calibration pairs</title>
<style>
:root{--bg:#f6f7f9;--ink:#1d2330;--mute:#6b7280;--card:#fff;--acc:#2563eb}
@media (prefers-color-scheme:dark){:root{--bg:#111418;--ink:#e6e9ee;--mute:#9aa3af;--card:#1b2027}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
header{position:sticky;top:0;background:var(--bg);padding:8px 16px;border-bottom:1px solid #8883;display:flex;gap:8px;flex-wrap:wrap;align-items:center;z-index:2}
button,select{font:inherit;padding:4px 10px;border-radius:6px;border:1px solid #8886;background:var(--card);color:var(--ink);cursor:pointer}
button.on{border-color:var(--acc);box-shadow:0 0 0 2px var(--acc)}main{padding:10px 16px}canvas{max-width:100%;display:block;border-radius:6px;cursor:crosshair}
.mute{color:var(--mute);font-size:12px}
</style></head><body>
<header><b>相機校正：地圖車道線 ↔ 影像</b><select id="fr"></select><span id="cams"></span>
<label><input type="checkbox" id="showpts" checked> 顯示地圖點</label>
<button id="undo">刪除最後一組</button><span id="st" class="mute"></span></header>
<main><div class="mute">步驟：① 點一個<b style="color:#ff3030">紅點</b>（地圖車道線投影的位置，會選到最近的點並變黃）；② 再點影像中<b>同一段車道線實際的位置</b>（白線上對應的點）。
綠線＝已完成的對應（從地圖投影位置指向實際位置）。每組都會自動存檔。建議每台相機在不同幀、畫面不同位置（中央、邊緣、近、遠）各標 10–20 組，邊緣越多越能校正魚眼失真。</div>
<canvas id="cv"></canvas></main>
<script>
const FR=__FRAMES__; let cam='f', fi=0, pts=[], pairs=[], sel=null, img=new Image();
const cv=document.getElementById('cv'), ctx=cv.getContext('2d'), frs=document.getElementById('fr'), cams=document.getElementById('cams');
FR.forEach((f,k)=>{const o=document.createElement('option'); o.value=k; o.textContent=f[0]+' frame '+f[1]; frs.appendChild(o);});
['fl','f','fr'].forEach(c=>{const b=document.createElement('button'); b.textContent=c; b.onclick=()=>{cam=c; load();}; cams.appendChild(b);});
frs.onchange=()=>{fi=+frs.value; load();}; document.getElementById('showpts').onchange=draw;
async function load(){const [t,f]=FR[fi]; sel=null;
  [...cams.children].forEach(b=>b.className=b.textContent===cam?'on':'');
  pts=(await (await fetch('/api/calibpts/'+t+'/'+f+'/'+cam)).json()).points||[];
  pairs=(await (await fetch('/api/calibpairs')).json()).pairs||[];
  img.onload=draw; img.src='/hoodimg/'+t+'/'+cam+'/'+f+'.jpg';}
function mine(){const [t,f]=FR[fi]; return pairs.filter(p=>p.track===t&&p.frame===f&&p.cam===cam);}
function draw(){if(!img.width)return; cv.width=img.width; cv.height=img.height; ctx.drawImage(img,0,0);
  if(document.getElementById('showpts').checked){ctx.fillStyle='rgba(255,40,40,0.9)'; pts.forEach(p=>ctx.fillRect(p.uv[0]-2,p.uv[1]-2,4,4));}
  mine().forEach(p=>{ctx.strokeStyle='#00e000'; ctx.lineWidth=3; ctx.beginPath(); ctx.moveTo(p.uv_map[0],p.uv_map[1]); ctx.lineTo(p.uv_img[0],p.uv_img[1]); ctx.stroke();
    ctx.fillStyle='#00e000'; ctx.beginPath(); ctx.arc(p.uv_img[0],p.uv_img[1],5,0,7); ctx.fill();});
  if(sel){ctx.fillStyle='#ffff00'; ctx.beginPath(); ctx.arc(sel.uv[0],sel.uv[1],7,0,7); ctx.fill();}
  document.getElementById('st').textContent=`這台相機這一幀 ${mine().length} 組，全部 ${pairs.length} 組`;}
cv.onclick=async e=>{const r=cv.getBoundingClientRect(); const x=(e.clientX-r.left)*cv.width/r.width, y=(e.clientY-r.top)*cv.height/r.height;
  if(!sel){let best=null,bd=1e9; pts.forEach(p=>{const d=(p.uv[0]-x)**2+(p.uv[1]-y)**2; if(d<bd){bd=d;best=p;}}); if(best&&bd<900){sel=best; draw();} return;}
  const [t,f]=FR[fi]; const pr={track:t,frame:f,cam:cam,xyz:sel.xyz,uv_map:sel.uv,uv_img:[Math.round(x*10)/10,Math.round(y*10)/10]};
  sel=null; pairs.push(pr); draw();
  await fetch('/api/calibpairs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({add:pr})});};
document.getElementById('undo').onclick=async()=>{const m=mine(); if(!m.length)return; const last=m[m.length-1];
  pairs=pairs.filter(p=>p!==last); draw(); await fetch('/api/calibpairs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({remove:last})});};
load();
</script></body></html>'''


def calib_pairs_path():
    return os.path.join(ROOT, 'calib_pairs.json')


def hood_path(cam):
    return os.path.join(ROOT, f'ego_mask_{cam}.json')



ISSUES_PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>__TRACK__ 待修正位置</title><style>
body{font:15px system-ui;margin:0;padding:12px 16px;background:#f4f6f5;color:#18201c}
h2{margin:4px 0 6px} p{margin:4px 0 10px;max-width:80ch;color:#4c5a53}
.row{display:grid;grid-template-columns:110px 70px 1fr;gap:10px;padding:8px 10px;background:#fff;border:1px solid #d6ddd9;border-radius:6px;margin-bottom:6px}
.sev{font-family:ui-monospace,monospace;font-weight:600} .s3{color:#a83a32} .s2{color:#a5741a} .s1{color:#2a7d3f}
.fl{font-family:ui-monospace,monospace;font-size:13px} .hint{font-size:13px;color:#4c5a53;margin-top:3px}
.tag{display:inline-block;font-size:12px;padding:1px 7px;border-radius:9px;border:1px solid #9aa8a1;margin-right:4px}
table{border-collapse:collapse;font:12px ui-monospace,monospace;margin-top:4px} td{padding:1px 8px 1px 0}
@media (max-width:600px){.row{grid-template-columns:1fr}}
</style></head><body>
<h2>__TRACK__ 地圖／相機還可以修正的位置</h2>
<p>軌跡：目前最佳提交。每 60 幀統計車道線、路緣、桿子定位相對軌跡的殘差（前進 fwd／側向 lat，m；車道線另有 yaw，度）。
點幀號打開定位檢視（紅＝地圖標線、黃＝路緣、綠＝桿子投影到影像），看是地圖錯、相機偏，還是軌跡偏。</p>
<p><label>排序 <select id="ord"><option value="sev">嚴重度</option><option value="lo">幀號</option></select></label>
 <label><input type="checkbox" id="pole" checked> 包含只有桿子的項目</label> <span id="cnt"></span></p>
<div id="list"></div>
<script>
const T='__TRACK__', VT='__VIEWER__', D=__DATA__;
function draw(){const ord=document.getElementById('ord').value, pole=document.getElementById('pole').checked;
 let it=D.issues.filter(x=>pole||x.flags.some(f=>!f.startsWith('pole')));
 it=it.slice().sort((a,b)=>ord==='sev'?b.sev-a.sev:a.lo-b.lo);
 document.getElementById('cnt').textContent=it.length+' 個區段';
 document.getElementById('list').innerHTML=it.map(x=>{const c=x.sev>1?'s3':x.sev>0.4?'s2':'s1';
  const st=Object.entries(x.stats).map(([k,v])=>'<tr><td>'+k+'</td><td>n '+v.n+'</td>'+Object.entries(v).filter(([a])=>a!=='n').map(([a,b])=>'<td>'+a+' '+(b>=0?'+':'')+b.toFixed(2)+'</td>').join('')+'</tr>').join('');
  return '<div class="row"><div><a href="/viewer/'+VT+'#'+x.frame+'" target="_blank">幀 '+x.lo+'–'+x.hi+'</a></div><div class="sev '+c+'">'+x.sev.toFixed(2)+'</div><div>'+
   x.flags.map(f=>'<span class="tag">'+f+'</span>').join('')+'<div class="hint">'+x.hint+'</div><table>'+st+'</table></div></div>';}).join('');}
document.getElementById('ord').onchange=draw; document.getElementById('pole').onchange=draw; draw();
</script></body></html>'''


AUDIT_PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>__TRACK__ 地圖檢查</title><style>
body{font:15px system-ui;margin:0;padding:12px 16px;background:#f4f6f5;color:#18201c}
h2{margin:4px 0 6px} h3{margin:16px 0 6px} p{margin:4px 0 10px;max-width:90ch;color:#4c5a53}
.row{display:grid;grid-template-columns:130px 60px 1fr;gap:10px;padding:8px 10px;background:#fff;border:1px solid #d6ddd9;border-radius:6px;margin-bottom:6px}
.sev{font-family:ui-monospace,monospace;font-weight:600} .s3{color:#a83a32} .s2{color:#a5741a} .s1{color:#2a7d3f}
.tag{display:inline-block;font-size:12px;padding:1px 8px;border-radius:9px;border:1px solid #9aa8a1;margin-right:6px;font-weight:600}
.hint{font-size:13px;color:#4c5a53;margin-top:3px}
.chart{background:#fff;border:1px solid #d6ddd9;border-radius:6px;padding:8px;margin-bottom:8px}
svg text{font:11px ui-monospace,monospace;fill:#4c5a53}
@media (max-width:600px){.row{grid-template-columns:1fr}}
</style></head><body>
<h2>__TRACK__ 地圖檢查（沿實際行駛路線，每 __STEP__ m 一站）</h2>
<p id="roof"></p>
<p>每列是一個異常區段：幀號（點開定位檢視）、嚴重度（峰值 / 門檻）、類型與意義。門檻 = max(固定下限, 全段中位數 + 4×MAD)。</p>
<div id="list"></div>
<h3>沿路線的剖面</h3><div id="charts"></div>
<script>
const D = __DATA__, VT = '__VIEWER__';
document.getElementById('roof').innerHTML = '整段共通：行駛車道內 1.3–2.2 m 高處有一層薄點（建圖車自己的車頂被掃進地圖），中位數佔車道內點的 <b>' + Math.round(D.roof_frac * 100) + '%</b>。目前比對只用最低的地面點，不受影響；但任何把「路面以上的點」當障礙物的邏輯都會在我們車道上看到假障礙。';
const names = {'double ground layer': '雙層地面', 'vertical distortion': '路面高度扭曲（地圖 vs GNSS 高度）', 'objects in the driven lane': '車道內有物體點',
               'coverage holes': '路面點缺洞', 'intensity jump': '路面亮度跳變'};
document.getElementById('list').innerHTML = D.segments.map(x => {const c = x.sev > 3 ? 's3' : x.sev > 1.5 ? 's2' : 's1';
  return '<div class="row"><div><a href="/viewer/' + VT + '#' + x.frame + '" target="_blank">幀 ' + x.lo + '–' + x.hi + '</a><br><small>' + x.s_lo + '–' + x.s_hi + ' m</small></div>' +
    '<div class="sev ' + c + '">' + x.sev.toFixed(1) + '</div><div><span class="tag">' + (names[x.type] || x.type) + '</span>峰值 ' + x.peak + '（門檻 ' + x.thr + '）<div class="hint">' + x.hint + '</div></div></div>';}).join('');
const st = D.stations, n = st.s.length, W = 900, H = 120, L = 46, R = 10;
const X = s => L + (s / st.s[n - 1]) * (W - L - R);
[['layer', '雙層地面：地面點高度離散 (m)'], ['vert', '路面高度扭曲：地圖 - GNSS 高度，高通 (m)'], ['objects', '車道內 0.3–1.2 m 物體點數'], ['holes', '路面缺洞比例'], ['inten', '路面背景亮度']].forEach(([k, t]) => {
  const v = st[k].map(q => q === null ? NaN : (k === 'vert' ? Math.abs(q) : q)); const vv = v.filter(isFinite).sort((a, b) => a - b);
  const top = vv.length ? vv[Math.floor(vv.length * 0.995)] || vv[vv.length - 1] : 1, Y = q => H - 18 - Math.min(q / (top || 1), 1) * (H - 30);
  let d = ''; v.forEach((q, i) => { if (isFinite(q)) d += (d ? 'L' : 'M') + X(st.s[i]).toFixed(1) + ',' + Y(q).toFixed(1); });
  const segs = D.segments.filter(x => x.metric === k || (k === 'vert' && x.metric === 'vert')).map(x => '<rect x="' + X(x.s_lo) + '" y="10" width="' + Math.max(2, X(x.s_hi) - X(x.s_lo)) + '" height="' + (H - 28) + '" fill="#f2c6c0" fill-opacity="0.6"/>').join('');
  document.getElementById('charts').insertAdjacentHTML('beforeend', '<div class="chart"><b style="font-size:13px">' + t + '</b><svg viewBox="0 0 ' + W + ' ' + H + '" width="100%">' + segs +
    '<path d="' + d + '" fill="none" stroke="#0f6b5a" stroke-width="1.3"/><text x="' + L + '" y="' + (H - 4) + '">0 m</text><text x="' + (W - R) + '" y="' + (H - 4) + '" text-anchor="end">' + Math.round(st.s[n - 1]) + ' m</text>' +
    '<text x="' + (L - 4) + '" y="16" text-anchor="end">' + (+top.toPrecision(2)) + '</text></svg></div>');});
</script></body></html>'''

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
                            f'<a href="/viewer/{t}">{t} 定位檢視</a> ｜ <a href="/compare/{t}">{t} 版本對比</a> ｜ <b><a href="/issues/{t}">{t} 待修正位置（最佳軌跡）</a></b> ｜ <b><a href="/audit/{t}">{t} 地圖檢查</a></b></li>')
            rows.append('<li><a href="/hood">自車遮罩編輯（引擎蓋／車體）</a></li>')
            rows.append('<li><a href="/calib">相機校正：標地圖車道線 ↔ 影像對應點</a></li>')
            return self._send(200, '<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">'
                                   '<title>Pole review</title><body style="font:16px system-ui;padding:16px">'
                                   '<h2>Pole review</h2><ul>' + ''.join(rows) + '</ul></body>')
        if p in ('/calib', '/calib/'):
            d = os.path.join(ROOT, 'calib_pts')
            fr = sorted({(n.split('_')[0], int(n.split('_')[1])) for n in os.listdir(d)}) if os.path.isdir(d) else []
            return self._send(200, CALIB_PAGE.replace('__FRAMES__', json.dumps([list(x) for x in fr])))
        m = re.fullmatch(r'/api/calibpts/(Track[12])/(\d{1,5})/(f|fl|fr)', p)
        if m:
            f = os.path.join(ROOT, 'calib_pts', f'{m.group(1)}_{int(m.group(2))}_{m.group(3)}.json')
            return self._send(200, open(f).read() if os.path.exists(f) else '{}', 'application/json')
        if p == '/api/calibpairs':
            f = calib_pairs_path()
            return self._send(200, open(f).read() if os.path.exists(f) else '{"pairs": []}', 'application/json')
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
        m = re.fullmatch(r'/audit/(Track[12])/?', p)
        if m:
            t = m.group(1)
            f = os.path.join(ROOT, f'audit_{t}.json')
            if not os.path.exists(f):
                return self._send(404, 'run tools/map_audit.py first')
            d = json.load(open(f))
            vt = max([q for q in os.listdir(VIEWER_DIR) if q.startswith(t + '_best')] or [t])
            return self._send(200, AUDIT_PAGE.replace('__TRACK__', t).replace('__VIEWER__', vt)
                              .replace('__STEP__', str(d.get('step', 2))).replace('__DATA__', json.dumps(d)))
        m = re.fullmatch(r'/issues/(Track[12])/?', p)
        if m:
            t = m.group(1)
            f = os.path.join(ROOT, f'issues_{t}.json')
            if not os.path.exists(f):
                return self._send(404, 'run tools/fix_issues.py first')
            vt = max([d for d in os.listdir(VIEWER_DIR) if d.startswith(t + '_best')] or [t])
            return self._send(200, ISSUES_PAGE.replace('__TRACK__', t).replace('__VIEWER__', vt).replace('__DATA__', open(f).read()))
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
        m = re.fullmatch(r'/pcd/([A-Za-z0-9_.-]+\.(?:ply|json|zip))', p)   # map slices for CloudCompare (pcd_slice.py)
        if m:
            f = os.path.join(VIEWER_DIR, 'pcd', m.group(1))
            if not os.path.exists(f):
                return self._send(404, 'not found')
            b = open(f, 'rb').read()
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Disposition', f'attachment; filename="{m.group(1)}"')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            try:
                self.wfile.write(b)
            except BrokenPipeError:
                pass
            return
        m = re.fullmatch(r'/video/([A-Za-z0-9_.-]+\.mp4)', p)   # comparison videos (render_topdown.py)
        if m:
            f = os.path.join(VIEWER_DIR, '..', 'video', m.group(1))
            if not os.path.exists(f):
                return self._send(404, 'not found')
            b = open(f, 'rb').read()
            self.send_response(200)
            self.send_header('Content-Type', 'video/mp4')
            self.send_header('Content-Disposition', f'attachment; filename="{m.group(1)}"')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            try:
                self.wfile.write(b)
            except BrokenPipeError:
                pass
            return
        if p in ('/video', '/video/'):
            d = os.path.join(VIEWER_DIR, '..', 'video')
            rows = ''.join(f'<li><a href="/video/{n}" download>{n}</a> ({os.path.getsize(os.path.join(d, n)) / 1e6:.0f} MB)</li>'
                           for n in sorted(os.listdir(d)) if n.endswith('.mp4')) if os.path.isdir(d) else ''
            return self._send(200, f'<h3>comparison videos</h3><ul>{rows}</ul>')
        if p in ('/pcd', '/pcd/'):
            d = os.path.join(VIEWER_DIR, 'pcd')
            rows = ''.join(f'<li><a href="/pcd/{n}" download>{n}</a> ({os.path.getsize(os.path.join(d, n)) / 1e6:.1f} MB)</li>'
                           for n in sorted(os.listdir(d)) if n.endswith(('.ply', '.json', '.zip')))
            return self._send(200, f'<h3>map slices (pcd_slice.py)</h3><ul>{rows}</ul>')
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
        if self.path.split('?')[0] == '/api/calibpairs':
            n = int(self.headers.get('Content-Length', 0))
            if n > 100_000:
                return self._send(413, 'too large')
            try:
                j = json.loads(self.rfile.read(n))
                f = calib_pairs_path()
                data = json.load(open(f)) if os.path.exists(f) else {'pairs': []}
                if 'add' in j:
                    q = j['add']
                    rec = {'track': str(q['track']), 'frame': int(q['frame']), 'cam': str(q['cam']),
                           'xyz': [float(v) for v in q['xyz']], 'uv_map': [float(v) for v in q['uv_map']],
                           'uv_img': [float(v) for v in q['uv_img']]}
                    if rec['cam'] not in ('f', 'fl', 'fr') or rec['track'] not in TRACKS:
                        raise ValueError('bad cam/track')
                    data['pairs'].append(rec)
                if 'remove' in j:
                    q = j['remove']
                    for k in range(len(data['pairs']) - 1, -1, -1):
                        r_ = data['pairs'][k]
                        if r_['track'] == q['track'] and r_['frame'] == q['frame'] and r_['cam'] == q['cam'] and r_['uv_img'] == q['uv_img']:
                            data['pairs'].pop(k)
                            break
            except Exception as e:  # noqa: BLE001
                return self._send(400, f'bad request: {e}')
            tmp = calib_pairs_path() + '.tmp'
            json.dump(data, open(tmp, 'w'))
            os.replace(tmp, calib_pairs_path())
            return self._send(200, json.dumps({'ok': True, 'n': len(data['pairs'])}), 'application/json')
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
