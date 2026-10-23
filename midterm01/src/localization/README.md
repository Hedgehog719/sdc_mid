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

輸出是 Kaggle 格式 `idx,x,y,yaw`（yaw 單位 rad，範圍 (−π, π]，和助教 `processing_codes/txt_to_idx_x_y_yaw.py` 的 atan2 相同；濾波器內部的航向是連續的，Track03 在第 2458 幀轉過 180°，輸出時才換回這個範圍）。每個步驟的輸出已存在時會跳過，所以中斷後可以接著跑。

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
2. **預測**：航向由陀螺儀積分，扣掉線上估計的零偏；速度由 IMU 沿車身方向的加速度積分，扣掉線上估計的加速度零偏。IMU 積分只用到該幀時間以前的最後一筆樣本（用該樣本的變化率外推到幀時間，`--imu_causal`）。
3. **視覺里程計**：前相機影像投影到路面，相鄰兩幀做樣板比對，量出前進距離；當作速度量測，比例因子也是濾波器的狀態。
4. **GNSS**（只有 Track01/02）：收到的樣本外推到該幀時間，當作位置量測，沿車身與橫向給不同的雜訊。
5. **地圖量測**（每 3 幀一次）：
   - 車道比對：三台相機的標線響應投影成鳥瞰圖，和 LiDAR 標線地圖做相關，得到前後、橫向、航向修正。
   - 近距對齊：0–14 m 內的地圖標線投影回三台相機影像，得到橫向修正。
   - 雜訊由比對峰值的銳利度決定，再加上品質門檻和 4σ innovation 門檻。
6. **聯合 EKF**：狀態 [x, y, 航向, 速度, 加速度零偏, VO 比例, 陀螺儀零偏] 放在同一個濾波器。車子前進時，橫向修正也會透過運動模型修正航向。
7. **靜止偵測**：IMU 一秒內沒有變化、速度也沒有改變時，陀螺儀讀值就是零偏、速度為 0。
   - Track02 一開始在車陣中靜止約 8 秒（旁邊有遊覽車，標線被擋住），第 0 幀的航向搜尋偏約 1.8°：靜止且航向不確定（σ > 0.5°）時，每 30 幀用當下影像重做一次寬範圍航向搜尋（`--stand_search 30`），後面的幀能看到別的標線。
8. **Track03 起點（沒有 GNSS）**：
   - init_guess 的航向偏約 17°，第 0 幀的車道比對在好幾度範圍內幾乎一樣好，差幾度起步的濾波器可能鎖不住。
   - 因此以第 0 幀的搜尋結果為中心，用 5 個航向假設（0、±1.5°、±3°）各跑前 600 幀。
   - 每一幀輸出「到該幀為止地圖量測的對數似然 + 航向偏移的高斯先驗（σ 2°，以第 0 幀搜尋結果為中心）最高」的假設（最大後驗）；第 600 幀保留最好的假設繼續跑完（`tools/hyp_select.py`）。只用最大似然時，只做過兩次量測的第 6 幀會在幾乎平手的情況下選到 +3° 假設。
   - 每一幀的輸出都只用到該幀以前的資料。
   - Track03 一開始就在行駛（約 7 m/s），初速未知，用最初 5 對 VO 的中位數初始化；在那之前（第 1–4 幀）用每對 VO 的位移推進位置（`--vo_dr_init`）。起點幾幀的時間間隔不規則（11.7 / 48.3 / 23.7 ms，之後約 36 ms），但 VO 位移穩定在每對 0.25–0.28 m，所以不用「速度 × 時間間隔」。
   - 起步時位置還不確定（前後 σ > 0.5 m），前後方向接受較弱的車道比對（峰值 > 0.13），第 6 幀就能完成前後鎖定。
   - 第 0 幀位置：init_guess 約偏 1 m，改用第 0 幀車道比對（在搜尋出的航向下）修正位置（`--init_pos 2.5`）。
   - 紅燈停車時（IMU 靜止偵測）不做地圖量測（`--still_freeze`）：同一張影像被量測上百次，原本會讓位姿慢慢漂移。
9. **Track03 地形投影（`--terrain`）**：
   - 平面鳥瞰圖假設前方路面和車底在同一平面。實際上 Track03 前方 15–20 m 的路面常比車底平面高或低 0.1–0.2 m（縱向曲線），距離 d、高差 dz 的標線會被投影到 d·h/(h−dz)，遠處偏 1–2 m。
   - 改成把影像投影到地圖路面：車底平面取車輛位置 ±4 m × ±2 m 的地圖地面高度最小平方平面（和近距對齊相同），前方路面取車道中央帶（|v| ≤ 2 m）的縱向剖面，穩健三次擬合，鳥瞰圖每一格用「路面相對車底平面的高度」投影回相機。
   - 剖面在 25 m 內偏離車底平面超過 0.45 m 時視為地圖扭曲，退回平面投影。
   - 驗證（不用 ground truth）：在 Track03 軌跡上每 15 幀比較，車道比對峰值平均 0.246 → 0.269（75% 的幀變高），地圖量測對數似然 1687 → 1786。Track01/02 的地圖起點有明顯扭曲（前向坡度可達 12°），峰值沒有提升（T1 0.294 → 0.283、T2 持平），所以只用在 Track03。
10. **Track03 剎車點頭／加速後蹲（`--dyn_att --dyn_roll_scale 0 --pitch_per_accel -0.3`）**：車身俯仰 = 0.3°/(m/s²) × 過去 0.5 s 的 IMU 縱向加速度（一般轎車的數值，不是擬合出來的），用在鳥瞰投影和近距對齊。疊在地形投影上時，剎車段（780–885 幀，−1.0 至 −1.8 m/s²）的地圖量測似然明顯提高；第 0 幀車子正以 +0.9 m/s² 加速（後蹲），考慮後第 0 幀航向搜尋從 165.4° 變成 163.4°（事後由陀螺儀回推約 162.3°）。T1 持平、T2 略差（沒有地形投影時），所以只用在 Track03。
11. **Track03 路面不平的快速俯仰（`--att_imu_map 1.5 --att_resid --att_pitch_only`）**：路面不平時單邊車輪壓過凸起、懸吊阻尼不足，車身會快速點頭或翹頭（陀螺儀：第 617 幀 0.6 s 內 0.9°），遠處鳥瞰圖會偏好幾公尺。每幀計算「IMU 俯仰 − 先驗位姿下的地圖路面平面俯仰 − 點頭模型」，減去它過去 1.5 s 的指數平均（因果高通），加到點頭模型上。慢的部分不用：IMU 姿態解算的俯仰在剎車後會飄 0.5°、靜止時才慢慢回來，直接用會把停車位置往前推 0.6 m（靜態車道比對證實是錯的）。驗證（不用 ground truth；不同設定收進的量測數不同，地圖似然不能直接比，改比較所有版本都有量測的幀上的預測誤差）：近距橫向 0.082 → 0.066 m、車道航向 0.77° → 0.46°、車道橫向 0.121 → 0.110 m，停車位置不變。IMU 側傾、0.8 s 或 3 s 的平均都沒有更好。
12. **Track03 車身相對路面的固定側傾（`--roll_imu_map 120`）**：Track03 全程「IMU 側傾 − 車底地圖路面平面的側傾」穩定在 +0.37°（每 300 幀 +0.18～+0.61，停車時也一樣），也就是車身（連同相機）相對路面向右傾、左側較高；相機校正是用 Track01/02 做的，沒有這個側傾。線上估計：每幀把這個差值做累積平均（只用到當下為止的幀；第 0 幀 0、第 100 幀 +0.32°、之後 +0.36～+0.42°），加到相機的側傾。驗證（同上，公平比較）：車道橫向 0.106 → 0.099 m、車道前後 0.096 → 0.085 m、近距橫向 0.066 → 0.064 m、車道航向 0.43° → 0.37°，通過品質門檻的量測變多；反向（−0.37°）則每一項都變差。
13. **輸出**：該幀的後驗位姿，寫出後不再修改。

可重現性：用 `tools/run_online.sh` 從頭執行，T1、T2、T3 的輸出和 `submissions/online/` 的候選檔逐位元相同（目前版本：`t1_online_v6c.csv`、`t2_online_v17.csv`、`t3_online_tpr.csv`；`t3_online_tpa.csv` 是加固定側傾（第 12 點）之前、`t3_online_tpd.csv` 是加快速俯仰（第 11 點）之前的版本，已提交的 `t3_online_tp.csv` 再早一版（沒有 `--vo_dr_init`）；較早提交的 `t1_online_v6`、`t2_online_v6`、`t3_online_b6` 是之前的流程設定）。GPU 推論的微小差異可能讓分割結果有極小不同。

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
