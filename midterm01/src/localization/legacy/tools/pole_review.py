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


NS = 16


def find_views(k, px, py, zb, h, u, v, r, frames, zv, C, cams, masks, Rm, n_views=2):
    """Up to n_views (frame, cam) views at different ranges: >= half of the pole visible, least YOLO-masked,
    range close to 12 m preferred; falls back to the nearest frame (<= 60 m) if nothing better exists."""
    hs = np.linspace(0.0, min(h, 6.0), NS)
    cand = []
    order = np.argsort(np.abs(r - 12.0))
    for j in order:
        if r[j] > 60 or r[j] < 3:
            continue
        i = frames[j]
        pts = np.stack([np.full(NS, u[j]), np.full(NS, v[j]), zb + hs - zv[i]], 1) @ Rm
        for cam in cams:
            uv, ok = C.project(cam, pts)
            if ok.mean() < 0.5:
                continue
            mk = np.unpackbits(masks[cam][i], axis=1)
            mx = np.clip((uv[ok, 0] / 2).astype(int), 0, mk.shape[1] - 1)
            my = np.clip((uv[ok, 1] / 2).astype(int), 0, mk.shape[0] - 1)
            mfrac = float(mk[my, mx].mean())
            cost = abs(r[j] - 12.0) / 4.0 + 6.0 * mfrac + (0.0 if 4 <= r[j] <= 40 else 5.0) + (0.0 if cam == 'f' else 0.3)
            cand.append((cost, i, cam, uv, ok, float(r[j]), mfrac))
        if len(cand) > 60:
            break
    cand.sort(key=lambda c: c[0])
    views = []
    for c_ in cand:
        if all(abs(c_[5] - w[4]) > 5.0 for w in views):
            views.append(c_[1:])
        if len(views) == n_views:
            break
    return views


def snap_shift(img, uv, ok, search=80):
    """Horizontal pixel shift (+-search) of the projected pole that maximises the image edge energy across it."""
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32), (0, 0), 1.0)
    gx = np.abs(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3))
    gx = cv2.GaussianBlur(gx, (0, 0), 2.0)
    p = uv[ok]
    best, bs = -1, 0
    for s_ in range(-search, search + 1, 2):
        xi = np.clip((p[:, 0] + s_).astype(int), 0, img.shape[1] - 1)
        yi = np.clip(p[:, 1].astype(int), 0, img.shape[0] - 1)
        e = gx[yi, xi].mean()
        if e > best:
            best, bs = e, s_
    return bs


def crop_view(img, uv, ok):
    shift = snap_shift(img, uv, ok)
    p = uv[ok]
    x0 = int(np.median(p[:, 0]))
    top, bot = int(p[:, 1].min()), int(p[:, 1].max())
    hpx = max(bot - top, 60)
    y0, y1 = max(top - hpx // 3, 0), min(bot + hpx // 3, img.shape[0])
    half = max(int(0.45 * (y1 - y0)), 80 + abs(shift))
    xa, xb = max(x0 - half, 0), min(x0 + half, img.shape[1])
    crop = img[y0:y1, xa:xb].copy()
    for (pu, pv) in [p[np.argmax(p[:, 1])], p[np.argmin(p[:, 1])]]:
        pu, pv = int(pu) - xa, int(pv) - y0
        cv2.line(crop, (pu - 20, pv), (pu - 8, pv), (0, 0, 255), 2)
        cv2.line(crop, (pu + 8, pv), (pu + 20, pv), (0, 0, 255), 2)
        qu = pu + shift
        cv2.line(crop, (qu - 14, pv + 6), (qu - 6, pv + 6), (0, 200, 0), 2)
        cv2.line(crop, (qu + 6, pv + 6), (qu + 14, pv + 6), (0, 200, 0), 2)
    sc = 240.0 / max(crop.shape[0], 1)
    return cv2.resize(crop, (max(int(crop.shape[1] * sc), 1), 240)), shift


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
        if n_seen == 0:  # never within the matching range of the route: not used by pole_match, skip
            continue
        views = find_views(k, px, py, zb, h, u, v, r, frames, zv, C, cams, masks, Rm)
        imgs_b64, srcs = [], []
        for (i, cam, uv, ok, rr, mfrac) in views:
            img = cv2.imread(os.path.join(imgdir[cam], names[cam][i]))
            crop, shift = crop_view(img, uv, ok)
            imgs_b64.append(b64jpg(crop))
            srcs.append(f'frame {i} {cam} {rr:.0f} m, snap {shift:+d} px' + (f' ({100 * mfrac:.0f}% masked)' if mfrac > 0 else ''))
        crop_b64 = imgs_b64
        src = ' | '.join(srcs) if srcs else 'never within 60 m of the route'
        cards.append({'id': k, 'x': round(float(px), 2), 'y': round(float(py), 2), 'h': round(float(h), 1),
                      'seen': n_seen, 'src': src, 'imgs': crop_b64, 'side': b64jpg(side_view(M, px, py, zb, h))})
        if len(cards) % 25 == 0:
            print(f'{len(cards)} cards ({k}/{len(P)} poles scanned)', flush=True)
    html = PAGE.replace('__TRACK__', track).replace('__DATA__', json.dumps(cards))
    open(a.out_html, 'w').write(html)
    print('wrote', a.out_html, len(cards), 'poles near the route (of', len(P), 'in the map)')


PAGE = r'''<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Pole review __TRACK__</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--ink:#1d2330;--mute:#6b7280;--keep:#1f9d55;--rej:#d64545;--unk:#9aa3af;--focus:#2563eb}
@media (prefers-color-scheme:dark){:root{--bg:#111418;--card:#1b2027;--ink:#e6e9ee;--mute:#9aa3af}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.4 system-ui,sans-serif}
header{position:sticky;top:0;z-index:2;background:var(--bg);padding:10px 16px;border-bottom:1px solid #8883;display:flex;gap:12px;flex-wrap:wrap;align-items:center}
header b{font-size:16px}.pill{padding:2px 8px;border-radius:10px;background:#8882}
button,select{font:inherit;padding:5px 10px;border-radius:6px;border:1px solid #8886;background:var(--card);color:var(--ink);cursor:pointer}
main{display:grid;grid-template-columns:repeat(auto-fill,minmax(520px,1fr));gap:12px;padding:12px 16px}
.card{background:var(--card);border-radius:10px;padding:8px;border:3px solid var(--unk);outline:none}
.card.keep{border-color:var(--keep)}.card.reject{border-color:var(--rej);opacity:.55}.card:focus{box-shadow:0 0 0 3px var(--focus)}
.imgs{display:flex;gap:6px;align-items:flex-start;height:240px}.imgs{overflow-x:auto}.imgs img{height:240px;border-radius:6px;max-width:220px;object-fit:cover}
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
<span class="help">鍵盤：← → 移動，K 保留，R 剔除，U 取消。左：相機截圖（紅色＝用目前軌跡投影的桿底／桿頂；綠色＝左右 80 px 內影像邊緣最強的位置，紅綠差距反映軌跡誤差）；右：LiDAR 側視圖（綠線＝地面，刻度每 1 m）。</span>
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
    if (f === 'reject' && d !== 'reject') continue; if (f === 'noimg' && p.imgs.length) continue;
    const el = document.createElement('div'); el.className = 'card ' + (d === 'unk' ? '' : d); el.tabIndex = 0; el.dataset.id = p.id;
    el.innerHTML = `<div class="imgs">${p.imgs.length ? p.imgs.map(s => `<img src="${s}" alt="camera">`).join('') : `<div class="none">${p.src}</div>`}<img src="${p.side}" alt="lidar side view"></div>
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
