#!/usr/bin/env python3
"""Build an offline HTML page to review the auto-detected pole landmarks by hand.

For every pole: a camera crop from the frame where it is best seen (8-20 m, unmasked, front camera preferred)
with tick marks at its projected base/top, and a LiDAR side view of the map points around it. In the page you
mark each pole Keep / Reject (keyboard: arrows, K, R, U) and export the decisions as JSON; then run
apply_pole_review.py to write poles_curated.npy, used by pole_match.py --poles poles_curated.npy.
"""
import argparse
import base64
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.dirname(__file__))
from bev import GroundRaster  # noqa: E402
from camera_model import Cameras  # noqa: E402
from common import TiledMap  # noqa: E402


def b64jpg(img, q=80):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return 'data:image/jpeg;base64,' + base64.b64encode(buf).decode()


def side_view(M, px, py, zb, h, size=(140, 240)):
    pts = M.query(px, py, 2.0)
    W, Hh = size
    img = np.full((Hh, W, 3), 245, np.uint8)
    if len(pts):
        d = np.hypot(pts[:, 0] - px, pts[:, 1] - py)
        ang = np.arctan2(pts[:, 1] - py, pts[:, 0] - px)
        xs = d * np.cos(ang)  # project onto an arbitrary vertical plane through the pole
        zr = pts[:, 2] - zb
        m = (zr > -0.5) & (zr < 9.5)
        u = ((xs[m] + 2.0) / 4.0 * (W - 1)).astype(int).clip(0, W - 1)
        v = ((9.5 - zr[m]) / 10.0 * (Hh - 1)).astype(int).clip(0, Hh - 1)
        img[v, u] = (60, 60, 60)
    g = int((9.5 - 0) / 10 * (Hh - 1))
    cv2.line(img, (0, g), (W - 1, g), (0, 160, 0), 1)
    for z in range(1, 10):
        y = int((9.5 - z) / 10 * (Hh - 1))
        cv2.line(img, (0, y), (4, y), (150, 150, 150), 1)
    cv2.putText(img, '4 m wide', (4, Hh - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('track_dir')
    ap.add_argument('traj_csv', help='best trajectory (x, y, yaw per frame)')
    ap.add_argument('out_html')
    ap.add_argument('--poles', default='poles.npy')
    ap.add_argument('--cam_height', type=float, default=1.5)
    ap.add_argument('--yaw_mount', type=float, default=-1.5)
    a = ap.parse_args()
    track = os.path.basename(os.path.normpath(a.track_dir))
    P = np.load(os.path.join(a.track_dir, a.poles))
    tr = pd.read_csv(a.traj_csv)
    M = TiledMap(os.path.join(a.track_dir, 'map.npy'))
    G = GroundRaster(M.pts)
    C = Cameras(os.path.join(a.track_dir, f'{track}_images', 'camera_info'))
    cams = ['f', 'fl', 'fr']
    imgdir = {c: os.path.join(a.track_dir, f'{track}_images', f'{track}_{c}_images') for c in cams}
    names = {c: sorted(os.listdir(imgdir[c])) for c in cams}
    masks = {c: np.load(os.path.join(a.track_dir, 'masks', f'{c}.npz'))['masks'] for c in cams}
    Rm = Rotation.from_euler('ZYX', [a.yaw_mount, 0, 0], degrees=True).as_matrix()
    X, Y, YAW = tr.x.values, tr.y.values, tr.yaw.values
    zv = G.height(X, Y) + a.cam_height
    frames = np.arange(0, len(tr), 3)
    cards = []
    for k, (px, py, zb, zt, h) in enumerate(P):
        d = np.stack([px - X[frames], py - Y[frames]], 1)
        c, s = np.cos(YAW[frames]), np.sin(YAW[frames])
        u = c * d[:, 0] + s * d[:, 1]
        v = -s * d[:, 0] + c * d[:, 1]
        r = np.hypot(u, v)
        n_seen = int(((r > 4) & (r < 35)).sum())
        best = None
        for j in np.argsort(np.abs(r - 12.0)):
            if not (8 <= r[j] <= 20):
                continue
            i = frames[j]
            hs = np.array([0.0, min(h, 6.0)])
            pts = np.stack([np.full(2, u[j]), np.full(2, v[j]), zb + hs - zv[i]], 1) @ Rm
            for cam in cams:
                uv, ok = C.project(cam, pts)
                if not ok.all():
                    continue
                mk = np.unpackbits(masks[cam][i], axis=1)
                mx, my = (uv[:, 0] / 2).astype(int), (uv[:, 1] / 2).astype(int)
                if mk[np.clip(my, 0, mk.shape[0] - 1), np.clip(mx, 0, mk.shape[1] - 1)].any():
                    continue
                best = (i, cam, uv, r[j])
                break
            if best:
                break
        crop_b64 = ''
        if best:
            i, cam, uv, rr = best
            img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]))
            x0 = int(uv[:, 0].mean())
            top, bot = int(uv[1, 1]), int(uv[0, 1])
            hpx = max(bot - top, 40)
            y0, y1 = max(top - hpx // 3, 0), min(bot + hpx // 4, img.shape[0])
            half = max(int(0.45 * (y1 - y0)), 60)
            xa, xb = max(x0 - half, 0), min(x0 + half, img.shape[1])
            crop = img[y0:y1, xa:xb].copy()
            for (pu, pv) in uv:
                pu, pv = int(pu) - xa, int(pv) - y0
                cv2.line(crop, (pu - 18, pv), (pu - 7, pv), (0, 0, 255), 2)
                cv2.line(crop, (pu + 7, pv), (pu + 18, pv), (0, 0, 255), 2)
            sc = 240.0 / max(crop.shape[0], 1)
            crop = cv2.resize(crop, (max(int(crop.shape[1] * sc), 1), 240))
            crop_b64 = b64jpg(crop)
            src = f'frame {i} cam {cam} {rr:.0f} m'
        else:
            src = 'no clear view (8-20 m, unmasked)'
        cards.append({'id': k, 'x': round(float(px), 2), 'y': round(float(py), 2), 'h': round(float(h), 1),
                      'seen': n_seen, 'src': src, 'img': crop_b64, 'side': b64jpg(side_view(M, px, py, zb, h))})
        if k % 25 == 0:
            print(f'{k}/{len(P)}', flush=True)
    html = PAGE.replace('__TRACK__', track).replace('__DATA__', json.dumps(cards))
    open(a.out_html, 'w').write(html)
    print('wrote', a.out_html, len(cards), 'poles')


PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Pole review __TRACK__</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--ink:#1d2330;--mute:#6b7280;--keep:#1f9d55;--rej:#d64545;--unk:#9aa3af;--focus:#2563eb}
@media (prefers-color-scheme:dark){:root{--bg:#111418;--card:#1b2027;--ink:#e6e9ee;--mute:#9aa3af}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
header{position:sticky;top:0;z-index:2;background:var(--bg);padding:10px 16px;border-bottom:1px solid #8883;display:flex;gap:12px;flex-wrap:wrap;align-items:center}
header b{font-size:16px}.pill{padding:2px 8px;border-radius:10px;background:#8882}
button,select{font:inherit;padding:5px 10px;border-radius:6px;border:1px solid #8886;background:var(--card);color:var(--ink);cursor:pointer}
main{display:grid;grid-template-columns:repeat(auto-fill,minmax(330px,1fr));gap:12px;padding:12px 16px}
.card{background:var(--card);border-radius:10px;padding:8px;border:3px solid var(--unk);outline:none}
.card.keep{border-color:var(--keep)}.card.reject{border-color:var(--rej);opacity:.55}.card:focus{box-shadow:0 0 0 3px var(--focus)}
.imgs{display:flex;gap:6px;align-items:flex-start;height:240px}.imgs img{height:240px;border-radius:6px;max-width:190px;object-fit:cover}
.none{width:180px;height:240px;display:flex;align-items:center;justify-content:center;color:var(--mute);border:1px dashed #8886;border-radius:6px;text-align:center;padding:6px}
.meta{color:var(--mute);font-size:12px;margin:6px 0}.row{display:flex;gap:6px}
.row button.k{border-color:var(--keep)}.row button.r{border-color:var(--rej)}
.help{color:var(--mute);font-size:12px}
</style></head><body>
<header><b>Pole review — __TRACK__</b>
<span class="pill" id="cnt"></span>
<select id="flt"><option value="all">全部</option><option value="unk">未決定</option><option value="keep">保留</option><option value="reject">剔除</option><option value="noimg">沒有截圖</option></select>
<button id="allkeep">未決定的全部保留</button>
<button id="exp">匯出 JSON</button>
<span class="help">鍵盤：← → 移動，K 保留，R 剔除，U 取消。左：相機截圖（紅色刻度＝桿子底和頂的投影位置）；右：LiDAR 側視圖（綠線＝地面，刻度每 1 m）。</span>
</header><main id="grid"></main>
<script>
const DATA = __DATA__; const KEY = 'pole_review___TRACK__';
let dec = {}; try { dec = JSON.parse(localStorage.getItem(KEY) || '{}'); } catch (e) { dec = {}; }
const save = () => { try { localStorage.setItem(KEY, JSON.stringify(dec)); } catch (e) {} };
const grid = document.getElementById('grid');
function render(){
  const f = document.getElementById('flt').value; grid.innerHTML = '';
  for (const p of DATA){
    const d = dec[p.id] || 'unk';
    if (f === 'unk' && d !== 'unk') continue; if (f === 'keep' && d !== 'keep') continue;
    if (f === 'reject' && d !== 'reject') continue; if (f === 'noimg' && p.img) continue;
    const el = document.createElement('div'); el.className = 'card ' + (d === 'unk' ? '' : d); el.tabIndex = 0; el.dataset.id = p.id;
    el.innerHTML = `<div class="imgs">${p.img ? `<img src="${p.img}" alt="camera">` : `<div class="none">${p.src}</div>`}<img src="${p.side}" alt="lidar side view"></div>
      <div class="meta">#${p.id} · 高 ${p.h} m · 可見 ${p.seen} 次 · (${p.x}, ${p.y}) · ${p.src}</div>
      <div class="row"><button class="k">保留 K</button><button class="r">剔除 R</button><button class="u">取消 U</button></div>`;
    el.querySelector('.k').onclick = () => set(p.id, 'keep'); el.querySelector('.r').onclick = () => set(p.id, 'reject');
    el.querySelector('.u').onclick = () => set(p.id, null);
    grid.appendChild(el);
  }
  count();
}
function count(){ const v = Object.values(dec); document.getElementById('cnt').textContent =
  `共 ${DATA.length}｜保留 ${v.filter(x=>x==='keep').length}｜剔除 ${v.filter(x=>x==='reject').length}｜未決定 ${DATA.length - v.length}`; }
function set(id, v){ if (v) dec[id] = v; else delete dec[id]; save();
  const el = grid.querySelector(`[data-id="${id}"]`); if (el){ el.className = 'card ' + (v || ''); } count(); }
document.getElementById('flt').onchange = render;
document.getElementById('allkeep').onclick = () => { for (const p of DATA) if (!dec[p.id]) dec[p.id] = 'keep'; save(); render(); };
document.getElementById('exp').onclick = () => {
  const out = {track: '__TRACK__', keep: DATA.filter(p => dec[p.id] === 'keep').map(p => p.id),
               reject: DATA.filter(p => dec[p.id] === 'reject').map(p => p.id),
               undecided: DATA.filter(p => !dec[p.id]).map(p => p.id)};
  const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([JSON.stringify(out, null, 1)], {type: 'application/json'}));
  a.download = 'pole_review___TRACK__.json'; a.click(); };
document.addEventListener('keydown', e => {
  const cards = [...grid.querySelectorAll('.card')]; const i = cards.indexOf(document.activeElement);
  if (e.key === 'ArrowRight' || e.key === 'ArrowDown'){ (cards[i + 1] || cards[0])?.focus(); e.preventDefault(); }
  else if (e.key === 'ArrowLeft' || e.key === 'ArrowUp'){ (cards[i - 1] || cards[cards.length - 1])?.focus(); e.preventDefault(); }
  else if (i >= 0 && 'kru'.includes(e.key.toLowerCase())){ const id = +cards[i].dataset.id;
    set(id, {k: 'keep', r: 'reject', u: null}[e.key.toLowerCase()]); (cards[i + 1] || cards[i]).focus(); }
});
render(); grid.querySelector('.card')?.focus();
</script></body></html>'''

if __name__ == '__main__':
    main()
