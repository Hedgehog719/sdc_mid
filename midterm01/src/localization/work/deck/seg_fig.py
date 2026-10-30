# deck figure: camera frame / YOLO road-user masks / SegFormer road probability, three cameras side by side
import os, sys
import cv2, numpy as np
TD, j, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
T = os.path.basename(os.path.normpath(TD))
rows = [[], [], []]
for c in ('fl', 'f', 'fr'):
    names = sorted(os.listdir(os.path.join(TD, f'{T}_images', f'{T}_{c}_images')))
    img = cv2.imread(os.path.join(TD, f'{T}_images', f'{T}_{c}_images', names[j]))
    m = np.unpackbits(np.load(os.path.join(TD, 'masks', f'{c}.npz'))['masks'][j], axis=1)[:, :img.shape[1] // 2]
    m = cv2.resize(m, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    rp = np.load(os.path.join(TD, 'road', f'{c}.npz'))['road'][j].astype(np.float32) / 255.0
    rp = cv2.resize(rp, (img.shape[1], img.shape[0])) > 0.5
    a = img.copy(); a[m] = (0.45 * a[m] + 0.55 * np.array([180, 40, 160])).astype(np.uint8)
    b = img.copy(); b[rp] = (0.5 * b[rp] + 0.5 * np.array([200, 120, 40])).astype(np.uint8)
    for r, im in zip(rows, (img, a, b)):
        r.append(cv2.resize(im, (720, 464)))
lab = ['camera', 'YOLO road users (masked out)', 'SegFormer road (p > 0.5)']
tiles = []
for r, l in zip(rows, lab):
    t = np.hstack(r)
    cv2.rectangle(t, (0, 0), (40 + 26 * len(l), 64), (0, 0, 0), -1)
    cv2.putText(t, l, (16, 46), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 3)
    tiles.append(t)
cv2.imwrite(out, np.vstack(tiles), [cv2.IMWRITE_JPEG_QUALITY, 85])
