# SDC 2026 Midterm01：線上（因果）定位

用三台相機影像、IMU（以及 Track01/02 的 GNSS）對預建的 LiDAR 地圖定位。最終流程是 `tools/run_online.sh`，核心是 `tools/localize_online.py`：一個依時間順序往前跑的 Kalman filter。

10/22 以前的做法（RTS 雙向平滑、Leaderboard 分析工具、舊提交檔）都移到 `legacy/`，只當紀錄保留，最終流程沒有用到；規則變更的時間線見 `legacy/README.md`。

## 1. 規則符合性（10/22 公告）

- **因果**：第 k 幀的位姿只用第 1…k 幀的資料估計，寫出後不再修改。沒有 backward smoothing，沒有非因果平滑，也沒有定位完成後的離線修正或人工調整。
- **前處理可以平行**：道路分割、車輛遮罩、影像標線響應是每張影像各自計算；視覺里程計只用相鄰兩幀 (k−1, k)。這些步驟批次平行計算的結果，和依時間順序逐幀計算完全相同，不會用到未來幀。
- **地圖**：標線地圖只由 LiDAR 地圖（`map.pcd`）產生，不使用任何軌跡（`marking_map.py ... ALL`）。Track02 地圖的 `tl`、`th` 欄位不讀取。
- **校正參數**是感測器常數，不是軌跡資訊：
  - 相機旋轉／平移修正（`calib/cam_fix_*.json`）
  - 前相機安裝 yaw（−1.5°）
  - 車身遮罩（`calib/ego_mask_*.png`）
  - 陀螺儀零偏初值（濾波器會繼續線上估計）
  - 車道比對的航向偏差（+0.32°）
- **沒有用 Leaderboard 分數調參或擬合**。參數用本地驗證決定：和 GNSS 輔助結果比較、量測擾動測試、濾波器 innovation 統計。

## 2. 環境

```bash
cd <repo root>                      # docker-compose.yml 所在處
docker compose up -d --build        # ROS 2 Humble + CUDA，資料夾 data/ 掛在 /root/data
docker exec -it sdc_midterm01 bash
pip3 install transformers           # SegFormer 道路分割（Dockerfile 之後加入的套件）
```

我們使用的版本：Python 3.10、numpy 1.23.5、scipy 1.8、pandas 2.0、opencv 4.11、torch 2.11 (cu128)、transformers 5.20、ultralytics 8.4、rosbags 0.11。

模型權重：
- SegFormer-B2 Cityscapes：第一次執行時會從 Hugging Face 自動下載 `nvidia/segformer-b2-finetuned-cityscapes-1024-1024`。
- YOLO11x-seg：ultralytics 會自動下載 `yolo11x-seg.pt`。也可以用 `YOLO_WEIGHTS=/path/yolo11x-seg.pt` 指定本機檔案。

硬體：需要 GPU 做分割（我們用 RTX 5070 Laptop 8 GB）。記憶體建議 ≥ 20 GB，因為定位時會載入整張地圖。

## 3. 執行

資料夾格式就是 Kaggle 下載的內容，放在 `data/Track1`、`data/Track2`、`data/Track3`：`raw_ply/`、`<name>_images/`、`map.pcd`、`init_guess.txt`，以及 bag（`*.db3` + `metadata.yaml`）。

```bash
cd /root/midterm/src/localization
bash tools/run_online.sh /root/data/Track1 T1    # -> work/runs/T1/T1_online.csv
bash tools/run_online.sh /root/data/Track2 T2    # -> work/runs/T2/T2_online.csv
bash tools/run_online.sh /root/data/Track3 T3    # -> work/runs/T3/T3_online.csv（不用 GNSS）
```

輸出是 Kaggle 格式 `idx,x,y,yaw`（yaw 單位 rad）。每個步驟的輸出已存在時會跳過，所以中斷後可以接著跑。

| 步驟 | 程式 | 輸出 |
|---|---|---|
| 1. 地圖、感測器 | `map_to_npy.py`、`extract_bag.py` | `map.npy`、`map_intensity.npy`、`sensors/imu.csv`（`gnss.csv`） |
| 2. 逐張影像 | `segment_road.py`（SegFormer）、`segment_vehicles.py`（YOLO） | `road/*.npz`、`masks/*.npz` |
| 3. 標線地圖 | `marking_map.py ALL` | `work/runs/<T>/mk_D.npz`、`mk_S.npz` |
| 4. 視覺里程計 | `bev_vo.py` | `work/runs/<T>/vo.npz` |
| 5. 線上定位 | `localize_online.py` | `<T>_online.csv` |

## 4. 方法

每一幀依時間順序處理：

1. **初始位姿（第 0 幀）**：位置取 `init_guess.txt`（Track01/02 改用第 0 幀當下的 GNSS）。航向用第 0 幀的車道比對在 ±25° 內搜尋；結果和 `init_guess` 差超過 5° 才採用（Track02 的 init_guess 航向誤差可達 18°）。
2. **預測**：航向由陀螺儀積分，扣掉線上估計的零偏；速度由 IMU 沿車身方向的加速度積分，扣掉線上估計的加速度零偏。
3. **視覺里程計**：前相機影像投影到路面，相鄰兩幀做樣板比對，量出前進距離；當作速度量測，比例因子也是濾波器的狀態。
4. **GNSS**（只有 Track01/02）：收到的樣本外推到該幀時間，當作位置量測，沿車身與橫向給不同的雜訊。
5. **地圖量測**（每 3 幀一次）：
   - 車道比對：三台相機的標線響應投影成鳥瞰圖，和 LiDAR 標線地圖做相關，得到前後、橫向、航向修正。
   - 近距對齊：0–14 m 內的地圖標線投影回三台相機影像，得到橫向修正。
   - 雜訊由比對峰值的銳利度決定，再加上品質門檻和 4σ innovation 門檻。
6. **聯合 EKF**：狀態 [x, y, 航向, 速度, 加速度零偏, VO 比例, 陀螺儀零偏] 放在同一個濾波器。車子前進時，橫向修正也會透過運動模型修正航向。
7. **靜止偵測**：IMU 一秒內沒有變化、速度也沒有改變時，陀螺儀讀值就是零偏、速度為 0。
8. **Track03 起點（沒有 GNSS）**：
   - init_guess 的航向偏約 17°，第 0 幀的車道比對在好幾度範圍內幾乎一樣好，差幾度起步的濾波器可能鎖不住。
   - 因此以第 0 幀的搜尋結果為中心，用 5 個航向假設（0、±1.5°、±3°）各跑前 600 幀。
   - 每一幀輸出「到該幀為止地圖量測的對數似然最高」的假設（最大似然）；第 600 幀保留最好的假設繼續跑完（`tools/hyp_select.py`）。
   - 每一幀的輸出都只用到該幀以前的資料。
   - Track03 一開始就在行駛（約 8 m/s），初速未知，用最初 5 對 VO 的中位數初始化。
   - 起步時位置還不確定（前後 σ > 0.5 m），前後方向接受較弱的車道比對（峰值 > 0.13），第 6 幀就能完成前後鎖定。
9. **輸出**：該幀的後驗位姿，寫出後不再修改。

可重現性：用 `tools/run_online.sh` 從頭執行，T1、T2、T3 的輸出和我們提交的檔案逐位元相同（`submissions/online/`：`t1_online_v6.csv`、`t2_online_v6.csv`、`t3_online_bank_mle.csv`）。GPU 推論的微小差異可能讓分割結果有極小不同。

量測品質用「擾動測試」檢查：把參考軌跡偏移已知的量（航向 +1°、橫向 +0.3 m、前後 +1 m），在偏移後的位姿做量測，看量測能修回多少。近距對齊的航向只修回約 10%，等於一直把航向拉回先驗，所以最終流程不使用它。車道比對的航向能完整修回。

## 5. 檔案

| 檔案 | 用途 |
|---|---|
| `tools/run_online.sh` | 端到端執行（上表 5 個步驟） |
| `tools/localize_online.py` | 線上 Kalman filter（最終定位） |
| `tools/hyp_select.py` | Track03 起點航向假設的因果選擇 |
| `tools/lane_single.py`、`tools/bev_match.py`、`tools/bev.py` | 單幀鳥瞰車道比對 |
| `tools/marking_map.py` | LiDAR 地圖 → 標線地圖 |
| `tools/bev_vo.py` | 路面視覺里程計 |
| `tools/camera_model.py` | 魚眼相機投影（含 `calib/` 校正） |
| `tools/segment_road.py`、`tools/segment_vehicles.py` | 道路分割、車輛遮罩 |
| `tools/extract_bag.py`、`tools/map_to_npy.py` | 資料轉換 |
| `calib/` | 相機校正、車身遮罩 |
| `docs/PLAN.md` | 完整實驗紀錄 |
| `legacy/` | 規則變更前的程式與提交（不屬於最終流程） |
