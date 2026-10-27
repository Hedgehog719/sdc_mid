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
- **不從 Leaderboard 反推逐幀真值或擬合軌跡修正**。候選由感測器幾何、量測擾動、因果 GNSS 留出及 innovation 提出；Kaggle 用於比較完整、事先定義的模型，允許採用實際較好的候選，但不依分數設計逐段平移／旋轉、解隱藏誤差或擬合輸出權重。這與競賽說明允許一般模型比較、禁止 leaderboard-based trajectory fitting 的界線一致。

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

可選的 `SDC_CAM_SYNC=1` 會用當前幀的因果速度估計，補償側相機相對前相機的拍攝時間差；改跑前請先清除或更換 `work/runs/<T>/` 中的舊中間產物，避免跳過重算。

搭配 `SDC_CAM_SYNC_YAW=1` 可實驗完整平面運動補償：用當前幀之前的陀螺儀積分估計角速度，補償時間差內的旋轉與轉彎弧線位移。預設關閉，與只有平移的同步分開驗證。
`localize_online.py --near_sync` 則把近距對齊的地圖點轉到各相機拍攝時刻，與鳥瞰圖使用同一套運動模型；同樣需要 `SDC_CAM_SYNC=1`。
同步回歸測試：`python3 -m unittest discover -s tests -p 'test_camera_sync.py' -v`（容器內執行）。

前向量測的實驗規則不再寫死在共用濾波器：`--relaxed_sharp_f 0.012 --relaxed_sharp_from 600` 可啟用後段較弱的匹配，`--fwd_jump_gate 0.7 0.25` 可啟用鎖定時的大位移弱峰值拒絕；兩者預設關閉，便於分別比較。

可重現的獨立實驗（地圖／VO 快取仍共用 `work/runs/T3/`，輸出與參數紀錄不覆寫舊實驗）：

```bash
SDC_CAM_SYNC=1 SDC_CAM_SYNC_YAW=1 SDC_LOC_EXTRA="--near_sync" \
   SDC_RUN_DIR=work/runs/T3_near_sync \
   bash tools/run_online.sh /root/data/Track3 T3
```

執行設定會保存到實驗目錄的 `args.txt`、`environment.txt`；`SDC_LOC_EXTRA` 可傳入其他濾波器實驗參數。輸出目錄在計算前建立。

`SDC_HYP_ESTIMATOR=mean` 可實驗起點航向假設的逐幀後驗平均（位置加權平均、航向圓周平均），取代硬式 MAP 輸出。權重只用累積到當幀的量測似然與原本的 2° 航向先驗；第 600 幀仍保留 MAP 假設繼續定位。預設 `map` 不變；測試用 `python3 -m unittest discover -s tests -v`。

輸出是 Kaggle 格式 `idx,x,y,yaw`（yaw 單位 rad，範圍 (−π, π]，和助教 `processing_codes/txt_to_idx_x_y_yaw.py` 的 atan2 相同；濾波器內部的航向是連續的，Track03 在第 2458 幀轉過 180°，輸出時才換回這個範圍）。每個步驟的輸出已存在時會跳過，所以中斷後可以接著跑。

| 步驟 | 程式 | 輸出 |
| --- | --- | --- |
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
13. **Track03 路面的橫向起伏（`--terrain_lat`）**：第 9 點的地形投影只取車道中央帶的縱向剖面，橫向一律沿用車底平面的坡度。但靠路邊停車的那一側常比車底平面低（排水坡、路拱）：第 705–735 幀左側停車格旁的車道線，地圖比影像偏外 0.4 m，右側的線卻對得上（影像左右兩半分開和地圖比對，`work/online/side_shift.py`）。改成鳥瞰圖每一格都用地圖路面高度（1 m × 0.5 m 網格，相對「車底平面 + 縱向剖面」的差，±0.3 m 截斷，沿道路方向中位數濾波，雙線性內插）後，左右偏差都在 0.1 m 內。整段 Track03（公平比較）：車道橫向 0.096 → 0.026 m、近距橫向 0.064 → 0.042 m、車道航向 0.40° → 0.24°；直線段「航向 − 行進方向」從 −0.26° 變成 −0.01°（之前找不到原因的航向偏差，就是左右路面高度不同讓鳥瞰圖歪斜）。
14. **Track02 只用相機看得到的路面（`--seg_gate 0.5 --seg_gate_track`）**：追蹤時的車道比對，影像響應只保留 SegFormer 道路機率 > 0.5 的區域（投影到鳥瞰圖、外擴 0.35 m）：停在路邊的車、路緣、建築物、旁邊遊覽車的反光都不參與比對。地圖那一側不遮（初始航向錯的時候，用影像的路面去遮地圖，會剛好把要找的標線遮掉）。第 0 幀和靜止時的航向搜尋不套用（套用後起點航向反而更偏）。和參考軌跡比：0.238 → 0.223（第 400 幀以後 0.197 → 0.180、橫向 0.096 → 0.070 m）；Track01 套用後變差（0.411 → 0.429）。Track03 也套用，但從第 300 幀才開始（`--seg_gate_from 300`）：從起點就套用時，起步段（第 105–177 幀）正確的前後修正因峰值降低被拒、反而接受一個錯的，位置落後約 0.9 m；第 300 幀以後套用則起點不變，之後車道前後 0.089 → 0.067 m、車道航向 0.227° → 0.221°。
   另外 Track02 的車道航向雜訊放大到 2.5°（`--yaw_sigma 2.5`，三軌共用預設 0.7°）：Track02 的車道航向在某些路段有偏差（第 426–570 幀被採用的修正一致是 −0.5～−0.8°，航向被拉偏約 1°），有 GNSS 時航向也能從 GNSS 與橫向修正經運動模型得到：和參考軌跡比 0.223 → 0.214（第 400 幀以後 0.180 → 0.163）。Track01 放大後變差（0.411 → 0.430–0.441），Track03 沒有 GNSS，都維持 0.7°。
15. （試過、未採用）**Track01 多濾波器逐幀平均（`tools/ensemble.py`）**：Track01 的濾波器在標線稀疏處很敏感（設定只差一點分數就差 ±0.02），試過 9 個濾波器（初始航向不確定度 × 車道航向雜訊）逐幀平均：對參考軌跡 0.408 → 0.404（航向 0.56° → 0.48°），但起點位置變差，Kaggle public 0.46 → 0.47，所以正式流程仍用單一濾波器。
16. **輸出**：該幀的後驗位姿，寫出後不再修改。

**2026-10-27 正式預設更新**：T3 的 `run_online.sh` 現預設 `SDC_CAM_SYNC=1`、`SDC_CAM_SYNC_YAW=1` 與 `--yaw_soft_peak 0.2`，對應已確認 public 0.22 的 `t3_batch_soft_heading.csv`；T1／T2 預設不變。可顯式設同步環境變數為 0 停用，或用 `SDC_LOC_EXTRA="--yaw_soft_peak 0"` 重現舊 T3 門檻。下段 `t3_online_tpg.csv` 與早期版本清單保留作歷史對照，不是最新 T3 推薦版本。

可重現性：用 `tools/run_online.sh` 從頭執行，T1、T2、T3 的輸出和 `submissions/online/` 的候選檔逐位元相同（目前版本：`t1_online_v7.csv`、`t2_online_v19.csv`、`t3_online_tpg.csv`；`t1_online_v6c.csv` 是 T1 加剎車點頭模型之前、`t3_online_tpl.csv` 是 T3 加路面遮罩之前的版本；`t2_online_v18.csv` 是 T2 放大車道航向雜訊之前、`t2_online_v17.csv` 是加第 14 點之前、`t3_online_tpr.csv` 是加第 13 點之前的版本；`t3_online_tpa.csv` 是加固定側傾（第 12 點）之前、`t3_online_tpd.csv` 是加快速俯仰（第 11 點）之前的版本，已提交的 `t3_online_tp.csv` 再早一版（沒有 `--vo_dr_init`）；較早提交的 `t1_online_v6`、`t2_online_v6`、`t3_online_b6` 是之前的流程設定）。GPU 推論的微小差異可能讓分割結果有極小不同。

量測品質用「擾動測試」檢查：把參考軌跡偏移已知的量（航向 +1°、橫向 +0.3 m、前後 +1 m），在偏移後的位姿做量測，看量測能修回多少。近距對齊的航向只修回約 10%，等於一直把航向拉回先驗，所以最終流程不使用它。車道比對的航向能完整修回。

## 5. 檔案

| 檔案 | 用途 |
| --- | --- |
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

## 6. T3 同步實驗（2026-10-26）

**最新確認最佳：Kaggle public 0.22，達成 `<0.23`，提交 ID 57675838。** 檔案 `submissions/online/t3_batch_soft_heading.csv`，完整設定／核心雜湊／SHA-256／來源 revision 見同名 JSON。模型為原 SE(2) 因果流程加 `--yaw_soft_peak 0.2`：峰值 0.20–0.25 的航向採用較大的量測噪聲，不是硬丟棄，也不放寬所有位置修正。完整五假設與全長 live matching、600 幀前綴一致、3142 幀 CSV、截止及額度防護皆通過；以下較早「尚未達標」敘述是當時實驗進展，非目前最佳狀態。private 分數未知。

**本輪收尾狀態（2026-10-27 05:39，Asia/Taipei）**：全部工作完成，沒有本輪定位 Python／Kaggle 提交程序或後續排程。最後 VO timing public 0.31（57679196），不採用；正式 T1／T2／T3 為 0.46／0.26／0.22。本日 UTC 提交核對：T1 2 次、T2 0 次、T3 15 次（其中本輪新增 12 次），沒有超過每日 15 次，也沒有借用午夜後額度。08:00 後不再提交；目前已提前停止本輪自動工作。正式 CSV 的列數、索引、有限值、wrapped yaw 與採用來源逐位元一致性均通過。

先依感測器時間戳與本地幾何／innovation 驗證決定修改，再提交檢查；不對提交軌跡做離線修正。

| 版本 | 修改 | Kaggle public | 提交 ID |
| --- | --- | --- | --- |
| `T3_sync.csv`（既有） | 側相機平移同步 | 0.23 | 57636864 |
| `t3_online_se2.csv` | 因果平移＋旋轉同步，恢復原前向門檻，完整 MAP 航向假設 | 0.23 | 57672334 |
| `t3_online_mean.csv` | 同組 SE(2) 假設，前 600 幀改因果後驗平均 | 0.23 | 57672914 |
| `t3_online_near_sync.csv` | SE(2) 加近距投影拍攝時刻補償，未採用 | 0.27 | 57673028 |
| `t3_online_stop_budget.csv` | 修正 VO 抖動重置停車額度，最多三次停車量測 | 0.23 | 57674442 |

旋轉同步的單一 0° 假設對照：轉彎區段（2260–2699）前向 innovation RMS 0.230 → 0.175 m，近距航向 innovation RMS 0.351 → 0.327°，但完整假設選擇後 Kaggle 仍為 0.23，尚未達到 `<0.23`。近距影像投影的時間差補償另由 `--near_sync` 控制，需獨立驗證。

### 6.1 分析與進展（2026-10-27 01:20，台北時間）

- 已讀取現有分支、最近 T3 commits 與過去的 terrain、IMU 姿態、road gate、假設選擇實驗。開始時分支為 `copilot/research-kaggle-competition`，最佳已提交 public 分數是 0.23，不是最新 v4 的 0.24。
- **硬編碼門檻缺少對照**：舊版把第 600 幀後放寬 sharpness、大位移弱峰值拒絕寫死在共用濾波器，可能連 T1／T2 一起改變。現改為獨立 opt-in 參數，預設回到歷史基準。
- **相機同步缺少旋轉**：T3 右相機相對前相機拍攝時間差中位數 −14.122 ms，最早 −34.364 ms；左相機中位數 +0.640 ms。原本只平移，轉彎時會留下旋轉殘差。新增 SE(2) 補償，但不宣稱本地 innovation 變小等於 ground-truth 誤差變小。
- **近距與鳥瞰使用不同時間基準**：新增可選近距投影補償，單元幾何與 600 幀因果前綴測試通過，但 public 退步到 0.27，保留為實驗，預設關閉，不推薦採用。
- **硬式 MAP 切換會跳動**：逐幀後驗平均把前 600 幀最大單步位置變化從 1.127 降到 0.927 m，600 幀後逐值不變；Kaggle 仍 0.23，預設 MAP 不變。
- **已排除的假設**：三個 Track 的前相機與 `raw_ply` 時間戳逐幀完全相同，不能再加一個臆測的全體相機時間平移。
- **停車額度重置抖動**：`--still_meas 3` 原本在長停車用了六次量測。第 1003／1093／1107／1115／1121 幀，速度僅 0.006–0.015 m/s，VO ±4–6 mm 雜訊使靜止旗標中斷並重置額度。改成只有速度回到既有靜止界線 0.3 m/s 才清除停車狀態；此修正需完整重跑再評分，不能用舊實驗檔充當修正後結果。
- **弱峰值航向**：固定量測 replay 的 `--yaw_soft_peak 0.2` 讓 lane-heading innovation RMS 0.778 → 0.767°、近距橫向 0.0771 → 0.0765 m，但其他指標混合；replay 只用於篩查，正式候選必須重新做 live matching。
- **起點位置／航向耦合**：原先先在 0° offset 航向下修正位置，再對所有假設直接加航向 offset，使五個假設共用同一個位置。新增 `--init_pos_per_hyp`，各假設在自己的航向下求第 0 幀位置。Docker 單幀實測：0° 開關前後逐值相同，−1.5° 的位置改變 0.533 m、航向不變；是否更準需完整因果假設流程檢查，不能只看單幀峰值。
- **世界 z 角速度不等於航向導數**：舊版使用 `R(q)·ω` 的 z 分量，純側傾在俯仰車身上也會產生偽航向。新增 `--gyro_heading`，計算世界前進方向水平投影的角度導數；純側傾、傾斜車身的世界 yaw 及四元數有限差分測試均通過。三軌實際兩定義積分差約 4.8–5.5°，但其中包含零偏定義改變，不能把此數字當作定位改善。
- **新模型零偏轉換**：只用獨立 Track2 起點 5 秒的 500 筆靜止 IMU 做感測器校正（加速度模長標準差 0.0291 m/s²）。body gyro 中位數為 [−0.441294, −0.184566, −0.291105]°/s；在各軌第 0 幀當時的姿態轉換舊校正，T1 −0.220 → −0.263836007、T2 −0.247 → −0.293533891、T3 −0.230 → −0.266065785°/s。這不是用 T3 未來軌跡或分數擬合；新模型仍需 live matching／獨立評分，預設關閉。
- **執行中腳本更新風險**：停車候選已完成全長與因果合併，原 Bash 腳本尾端卻因執行中檔案更新出現 `select.log: command not found`。已另驗其 3142 幀 CSV 與 600 幀前綴完整後才提交（0.23）。批次現改為執行 `runner_snapshot.sh` 凍結副本，核心 Python 模組前後 SHA-256 不同則拒絕混合版本提交；運算時不要改核心數值程式。
- 目前未達 `<0.23`；所有分數以上表 Kaggle 回報為準，不把相對既有軌跡的差異當成定位誤差。

### 6.2 改動檔案與驗證

| 檔案 | 修改 |
| --- | --- |
| `tools/localize_online.py` | 門檻實驗參數、因果速度／角速度傳遞、可選近距同步、停車額度保留 |
| `tools/bev.py` | SE(2) 影像 warp、相機拍攝座標逆轉換，共用平面運動模型 |
| `tools/lane_single.py` | 傳遞當幀角速度，保留原本兩參數呼叫相容性 |
| `tools/hyp_select.py` | 可選逐幀 posterior mean、圓周航向平均、穩定 softmax |
| `tools/run_online.sh` | 隔離實驗輸出、重用前處理快取、保存參數與環境、可選假設估計方式 |
| `tools/submit_guarded.py` | 驗證 CSV、明確時區截止與上傳前二次時間檢查 |
| `tools/overnight_experiments.py` | 固定有限候選、全長 Docker／前綴驗證、保留額度、逐組歸檔與 commit/push |
| `tools/validate_cross_track.py` | T1／T2 固定 GNSS 留出、腳本快照、逐組保存摘要；沒有上傳功能 |
| `tools/camera_timing.py` | 抽取 H.265 影像 header 時間、按檔名到達時間精確整數對齊，不做缺失內插 |
| `tools/gnss_heading.py` | 尚待整合的弱航向先驗：到達時間、過期、一次使用、重複與相關間隔控制 |
| `tools/ensemble.py` | 原逐幀等權／圓周平均，增加相同連續 idx、欄位／長度及有限數值驗證 |
| `tests/test_camera_sync.py` | 平移相容、旋轉方向、遮罩、拍攝座標逆運算、當前幀限制 |
| `tests/test_hyp_select.py` | 權重、±π 航向、數值穩定、因果前綴 |
| `tests/test_submit_guarded.py` | 時區、07:58／08:00 拒絕、無效 CSV |
| `tests/test_localize_options.py` | 停車額度跨偵測抖動保留、GNSS 子集隔離、起點位置依航向求解 |
| `tests/test_overnight_experiments.py` | 批次的因果前綴一致、拒絕被改寫前綴、拒絕不完整假設 |
| `tests/test_cross_track_validation.py` | 留出評估只外推前一幀、度／弧度及橫向誤差、無重疊拒絕 |
| `tests/test_camera_timing.py` | 精確 metadata 配對、缺失拒絕、拍攝／到達時間順序 |
| `tests/test_gnss_heading.py` | 未來訊息不得改 frame0、過期／無效拒絕、一次額度、重複及 ±π 等價 |
| `tests/test_ensemble.py` | 固定等權、±π、因果前綴、索引錯位與無效輸入拒絕 |
| `submissions/online/t3_online_*.csv` | 保留實際提交的產物，不用手工修正軌跡 |

Docker：`sdc_midterm01`，工作目錄 `/root/midterm/src/localization`。執行 `python3 -m unittest discover -s tests -v`；同步候選完整 3142 幀，檢查欄位、連續索引、有限數值及 yaw ∈ [−π, π]。近距候選前 600 幀與完整執行前綴一致（1e−12 容忍度）。編輯器 diagnostics 與 Pylance 語法檢查亦已檢查；這些不是隱藏真值的正確性保證。

### 6.3 自主實驗的提交界線

使用者授權截止為 **2026-10-27 08:00 Asia/Taipei（UTC+08:00）**。預留 120 秒緩衝，07:58 起不得啟動新提交；08:00 後不提交，也不自動延到隔天。截止前只提交已完成本地驗證的候選，quota 用完或沒有合理候選就停止，不用重複提交耗盡額度。

提交額度按競賽獨立管理，T1／T2 的額度不代表 T3 也有相同剩餘次數。2026-10-27 01:20 台北時間，T3 CLI 最近回報還有 9 次；每次上傳前查當日提交與剩餘額度，保留最後確認用額度。T3 的同步／品質模型可先在 T1／T2 Docker 本地套用，比較共同幀及因果 GNSS 留出表現，不為替 T3 探參數而耗用兩軌提交次數。

已確認 T1／T2 本日尚無提交，各保留 15 次額度；正確競賽 ID 為 T1 `sdc-2026-midterm-01-track-01`、T2 `sdc-2026-localization-track-02`。T1 不是 `sdc-2026-localization-track-01`，後者查詢會得到 403。

`submit_guarded.py` 會先查該競賽 CSV 歷史，以 **UTC 日期** 計算每日已用次數（不能因台北午夜而假設額度重置），包含 pending 提交。預設上限 15 次；`--reserve 2` 保留兩次確認額度，查詢錯誤或無法解析時拒絕上傳。2026-10-27 01:40 台北時間，實際只讀檢查結果為 T3=9、T1=15、T2=15。

單次上傳另設最長 90 秒 timeout，且不超過硬截止前一秒；即使網路卡住，也不讓 CLI 無限制拖過 08:00。逾時代表結果未知，必須先只讀確認提交歷史，不能自動重試而重複消耗額度。此邊界亦有 Docker 回歸測試。

VO 因果性補查：T1／T2／T3 共 3010／3298／3141 個相鄰影像對，速度預測插值使用的右側 IMU 樣本晚於配對終點的次數均為 0。插值可以使用 `(k−1,k)` 內已收到的 IMU，但不能越過 k；本資料檢查通過，不做無依據修改。

```bash
python3 midterm01/src/localization/tools/submit_guarded.py \
   midterm01/src/localization/work/runs/T3_ab/submission_still3.csv \
   --deadline 2026-10-27T08:00:00+08:00 \
   -m "commit hash, method, local checks"
```

上例僅展示命令格式，不表示舊的 `submission_still3.csv` 已含停車額度修正。每次修改先 focused validation，再 commit／push 到目前分支；提交與實際 public score 記錄到本節。失敗實驗保留紀錄，但不替換最佳候選。

固定批次（在主機 repository root 執行，不在容器內）：

```bash
python3 midterm01/src/localization/tools/overnight_experiments.py \
   --deadline 2026-10-27T08:00:00+08:00 --reserve 2
```

五個預先宣告的模型實驗為 `conditioned_init`、`soft_heading`、`conditioned_soft`、`stop_soft`、`gyro_heading`（最後一組使用上述獨立校正轉換後的零偏）。每組以 live matching 跑完整五假設與全長軌跡，不用 replay 當提交檔；逐組檢查 3142 幀 CSV，以及勝出假設「只跑 600 幀」和全長前 600 幀相同。結果的 CSV、SHA-256、核心程式 SHA-256、參數／revision、提交 ref／public score 及 README 表格會 commit／push。開始時工作樹必須乾淨，已存在的候選檔名拒絕重複提交；不足保留額度、截止將近或驗證失敗即停止。07:30 起不再啟動新重型實驗，07:58 起不啟動上傳。查詢剛提交尚未評分時記 pending，後續再只讀補齊，不用重複上傳取分數。

`SDC_HYP_WORKERS=1|2|3` 可限制起點假設同時載入地圖的數量（預設 3，批次採 1）。與 T1／T2 留出工作並行時維持總記憶體低於 Docker 20 GiB；不並行啟動多個批次。

### 6.4 後續 T3 改進順序

1. 修正停車量測額度後重跑，使用最多有限次獨立、間隔足夠的量測；檢查停車位姿、起步後共同幀 innovation 與因果前綴。
2. 弱峰值航向以品質相關的較大噪聲吸收，不直接放寬所有 lane fix。先固定量測篩查，再 live matching，觀察是否造成初始錯鎖。
3. 針對共用 gate、VO scale 與道路姿態模型用 T1／T2 的 GNSS 留出資料做可轉移驗證；避免只追 T3 public 子集。
4. 改善相機投影與近距誤差模型的失配：目前近距同步退步，應先查動態 pitch／roll、路面傾斜座標及不同相機的殘差，不再盲加位姿偏移。
5. 有限且因果的多假設估計，不能在完整軌跡算完後選一條、再倒回改寫早期幀；亦不能用 public score 回推逐幀真值。

### 6.5 T1／T2 合法提升方向

**共同限制**：第 k 幀僅用第 1…k 幀可得的相機／IMU／允許的 GNSS，以及賽方提供的預建地圖；逐幀寫出後不修改。允許每張影像獨立分割、相鄰幀 VO、感測器常數校正與因果 EKF／多假設；不使用 RTS／雙向平滑、未來影像、全軌跡擬合、隱藏真值、從排行榜反推標籤，或使用 Track2 的 `tl`／`th` 時間欄位挑選／清理地圖。舊 legacy 方法不可拿回正式流程。

**T1**：目前歷史候選為 v7（記錄 public 0.46）。優先驗證 GNSS 的時間相關偏差模型（現有 `--gnss_bias_sd`／`--gnss_bias_tau`），以過去樣本估偏，避免把相鄰 GNSS 當獨立量測導致過度自信。用分離的連續路段或因果留出 GNSS 做 validation，校正集與驗證集分開；GNSS 是 noisy reference，不是真值。可重做感測器常數校正、陰影下標線響應與遮擋品質估計。T1 地圖坡度有扭曲，不能直接搬用 T3 terrain；過去 9-filter 平均 public 0.46 → 0.47，不能因平均通常穩就預設採用。

**T2**：目前歷史候選為 v19（記錄 public 0.26）。優先改善起點靜止車陣中的航向可觀測性：保留靜止時的因果寬範圍重搜、品質加權及動態車輛遮罩，不用後續行駛方向倒推起點。保留已驗證有效的 road gate 與較寬 lane-heading noise（2.5°），用共同幀與 GNSS 留出檢查是否需要品質相關噪聲；不要把 T3 的 0.7° 直接套回。相機時間同步可獨立比較，但先核對左右相機偏差、近距一致性；以 T3 失敗實驗為警惕，物理模型更完整不代表目前校正下必然更準。

T1／T2 以上是有既有程式與歷史依據的待驗證方向，**本輪尚未完成兩軌完整驗證或重新提交，不能宣稱它們分數已改善**。最終可用設定以各軌獨立 Docker 重跑、因果檢查和實際提交回報為準。

留出測試設定：`SDC_LOC_EXTRA="--gnss_stride 2 --gnss_phase 0 --frames 600"` 只把偶數編號 GNSS 輸入濾波器，奇數編號保留在檔案供評估；反相驗證則用 `--gnss_phase 1`。預設 stride=1／phase=0 維持全部 GNSS。已用 Track1 的 440 筆樣本驗證預設逐值不變、兩子集無交集、無效參數拒絕。部分幀留出輸出不是正式提交檔，不能以其分數冒充 Kaggle 成績；GNSS 時間相關誤差仍使此指標不等於 ground truth。

完整可重現的本地留出批次（容器內執行）：

```bash
python3 tools/validate_cross_track.py --frames 600 --phase 0 \
   --deadline 2026-10-27T08:00:00+08:00
```

對 T1／T2 依序跑 baseline、translation、SE(2)、gyro-heading 四組（共八組），每組保存獨立輸出、快照與 `holdout.json`，累積摘要寫入 `work/runs/cross_track_validation_p0.json`。評估樣本與濾波器 GNSS 子集互斥；每個 GNSS 時刻只用其之前最後一幀的位姿／速度做短時間外推，不用後一幀內插。相鄰 GNSS 仍可能有時間相關偏差，所以改善只作本地模型篩選，不能當作隱藏真值準確度。腳本沒有 Kaggle 指令，不消耗提交額度。先前臨時長命令未產生完整摘要，不列為完成成果。

額外本地模型可用 `--variants gnss_bias gnss_bias_heading`：前者只啟用已存在的因果 GNSS 偏差狀態（σ=0.25 m、τ=3 s），後者再加 SE(2) 與新航向模型。兩組是不同因子的對照，不將 GNSS 偏差狀態套到沒有 GNSS 的 T3。續跑會保留摘要裡既有的其他模型結果，不因只重跑一個子集而清掉舊紀錄。

**T1 第一階段實測（前 600 幀、phase 0，43 筆留出，僅本地參考）**：

| 模型 | 位置 RMSE m | 前向 RMSE m | 橫向 RMSE m |
| --- | ---: | ---: | ---: |
| baseline | 0.697875 | 0.560897 | 0.415239 |
| translation | 0.703717 | 0.569808 | 0.412961 |
| SE(2) | 0.698762 | 0.565543 | 0.410402 |
| gyro-heading + SE(2) | 0.689995 | 0.554036 | 0.411264 |

新航向模型在 25/43 個共同時刻的距離較小，四個連續 GNSS 子區段 RMSE 均略降；平均成對距離差 −0.00613 m。幅度很小，受 GNSS 時間相關偏差影響，不能當作統計顯著或 Kaggle 改善。T2 對照、反相留出及全長驗證尚待完成，尚未消耗兩軌提交額度。

**GNSS 航向的待驗證缺口**：提供且允許使用的 `/localizer_node/gnss_pose` 已有有效、會變動的 yaw 四元數，但目前濾波器只讀 x／y。T1／T2 相對既有因果輸出航向的差值分別約 mean −0.359／+0.904°、std 1.859／1.461°；不是精確真值，也不是單純逐對 GPS 位移角度，不能直接覆蓋濾波器航向。競賽說明允許 T1／T2 GNSS pose sensor fusion，T3 不提供 GNSS，不能套用此資料。

T2 起點前 5 秒的提供航向固定 −15.2055°，現有 frame0 為 −20.6901°；但第一筆 GNSS 在 frame0 之後才到，**不能讀第一筆之後的訊息倒回改 frame0**。可考慮在第一筆真正到達時做一次弱航向先驗，或明確建模重複／時間相關量測，不能把停車時同一個持續不變航向反覆當獨立樣本。T1 frame0 前最近 GNSS 約 14.4 ms，起點 5 秒 GNSS yaw std 1.77°；兩軌需各自驗證。此項尚未實作，待核心凍結批次結束後處理。

弱先驗的獨立支援模組 `gnss_heading.py` 已準備，但尚未接入 EKF：無效／零四元數變成 invalid，不當成 0°；樣本必須已到達、未過期，一次模式只消耗第一筆有效樣本，連續模式至少間隔 2 秒且拒絕完全相同的持續航向。6 項 Docker 回歸測試通過，包含第一次到達晚於 frame0、±π 等價與過期／無效樣本不消耗額度；尚不能宣稱任何 T1／T2 分數改善。

**0.22 方法的量測對照**：原 SE(2) 與 soft-heading 完整方法的勝出 offset 分別 −1.5°／0°，不能當作純單濾波器消融。前 600 幀共同 199 次量測的 lane-heading innovation RMS 0.8071 → 0.6333°、near-heading 0.4773 → 0.3456°、near-lateral 0.05238 → 0.04815 m；全長航向分別 0.8132 → 0.7474°、0.4167 → 0.3668°。部分前向／橫向指標未改善，起步最大單步變化仍 1.127 m；實際採用理由以已確認的 Kaggle public 0.22 與完整因果模型驗證為準，不宣稱 private 已知或所有殘差全面更好。

**已否定的起點側傾捷徑**：T3 frame0 IMU roll 減地圖道路 roll，在錯誤粗航向下為 0.637°，在匹配後航向下只剩 0.065°。不能因初值看起來有側傾就直接初始化；這會把錯誤航向造成的地圖平面旋轉誤認成車身姿態。本輪不採用。

**預先定義的 T3 ensemble 對照**：待所有成員完成，使用既有 `tools/ensemble.py` 對原 SE(2)、soft-heading、gyro-heading 三個因果模型逐幀等權平均（各 1/3，yaw 用圓周平均）。不按分數擬合權重、不依路段挑成員、沒有未來幀參與某一幀輸出；這與三模型在線平行跑再即時平均等價。此計畫在後兩個成員評分前宣告，仍需 CSV／因果一致性驗證與額度防護，不能保證更好。過去 T1 ensemble 曾退步，不能把平均當作必然改善。

工具原本只核對相同長度，現增加相同連續 `idx`、欄位與有限數值檢查，避免錯時刻的列被平均。4 項 Docker 測試及兩份實際 3142 幀資料對照通過，等權數值與原實作逐值相同；這是輸入保護，不是新增擬合權重。尚未提交 ensemble，等預定成員完成後才評估。

**固定 ensemble 實際更新**：`t3_fixed_ensemble.csv` 已通過 3142 幀格式、相同 idx 與 600 幀前綴逐值一致；預先指定的三個成員各 1/3、yaw 圓周平均。Kaggle public **0.23**（57678540），不優於 soft-heading 0.22，**不採用、不再依分數調權重或挑段**。成員來源提交 ID、SHA-256、權重及驗證見同名 JSON。此一次預留確認後，T3 剩最後 1 次；只保留給已記錄的 VO 曝光時間分母單因子完整驗證，不做盲目掃描。

**已驗證的影像時間語義缺口**：現有 `camera_time_offsets()` 用 JPG 檔名的 bag receive time 作為相機拍攝偏移，但 bag 的 H.265 topic 內另有真正的 `header.stamp`。T3 三相機各 3142 幀都能精確配對，不缺失、不以未來資料插值。前相機 receive−header 中位數 7.912 ms、最大 31.651 ms；真實 fl 相對 f 曝光時間中位數 +1.292 ms（範圍 +0.835…+1.851 ms），fr −13.429 ms（−13.888…−12.552 ms），比 receive time 差值穩定。用既有因果速度換算，誤用 receive offset 的行駛中 warp 位移誤差 RMS：fl 0.01222 m、fr 0.01250 m；最大 0.0650／0.0563 m。這不是 ground-truth 定位誤差，而是兩種時間定義造成的可量測投影差。

```bash
python3 tools/camera_timing.py /root/data/Track3
```

工具會建立可重算的 `sensors/camera_timing.csv` 並逐檔精確檢查。3 項 Docker 回歸測試已通過。定位核心尚未接入 header offset，避免污染目前凍結批次；完成後只比較相機相對曝光同步，不擅自改變 Kaggle frame 的輸出時間定義。

同一時間語義問題也影響 VO 的速度分母：三軌 receive pair interval 減 capture pair interval 的 **RMS** 分別約 T1 0.980／T2 0.477／T3 1.041 ms（MAE 0.785／0.315／0.684 ms，兩者分開計算）。T3 最初接收間隔 11.7／48.3 ms 會把同一影像位移換成約 23.9／5.18 m/s；以各自曝光間隔診斷時約 7.81／6.89 m/s。這是速度量測時間定義，不是已知真實速度，不能用後續幀回填 frame0。

已建立尚未提交的單因子診斷輸入 `work/runs/T3/vo_capture_dt.npz`：3141 個 `(k−1,k)` 對的 j／k／fwd／lat／resp 與原快取逐值一致，只把 dt 換為該兩幀 header 差並重算 speed；不改影像匹配位移、輸出時鐘或軌跡。原快取 SHA-256 `55a718754c67fb95a75e8d328549e34ba7eaea86476941389edfdb14153a1743`，診斷快取 `a3708b00e370d045a1066a62ff9130b2728092de967504dcebcac8cb988260dc`，JSON 保存轉換依據。此候選在 header 實驗最終評分前記錄，若測試，只用同幀對已可得曝光時間、保持其他模型固定，需全長／前綴／額度／截止檢查，不視為已採用改善。

**05:10 正式化的最後單因子對照**：`camera_timing.retime_vo_pairs()`／CLI 已加入相鄰整數索引、有限位移／品質、單調且正時間間隔檢查；保留來源陣列不變，不修改原 `vo.npz`。時間工具相關 13 項 Docker 測試通過，包含更改未來曝光時間不能改先前 pair 的 dt／speed。重現輸入：

```bash
python3 tools/camera_timing.py /root/data/Track3 \
   --vo work/runs/T3/vo.npz --vo-out work/runs/T3/vo_capture_dt.npz
```

單一全流程入口（主機執行，使用本日最後一次 T3 額度，尚未提交前先通過完整驗證）：

```bash
python3 midterm01/src/localization/tools/overnight_experiments.py \
   --profiles vo_capture_dt --deadline 2026-10-27T08:00:00+08:00 --reserve 0
```

該 profile 明確 `SDC_CAM_CAPTURE=0`，保留既有最佳的側相機 receive-offset 模型，只覆寫 `--vo` 為新時間分母快取，避免兩個因素混用。執行前自動重建／保存 JSON，記錄實際 VO SHA-256；核心或輸入快取在運算期間改變都拒絕提交。仍需五假設、3142 幀、600 幀因果前綴、CSV 與八點硬截止／90 秒 upload timeout。即使退步也不覆蓋最佳 0.22；此後不借用午夜後額度、不再開新 T3 參數掃描。

**最後 VO timing 的實際結果**：全長、五假設、600 幀前綴、輸入／核心雜湊、CSV 及截止檢查全部通過，但 Kaggle public **0.31**（57679196），不優於 0.22，**不採用，不改原 `vo.npz` 或正式 runner 的 VO 預設**。真實曝光時間修正可能與現有 process time、既有校正、量測品質及起點假設耦合；此分數不提供逐幀誤差真值，不能據此對輸出平移或回填速度。本輪在既有預算內完成此事先記錄的單因子後停止，不再新增模型。

跨軌抽取進一步發現 **檔名慣例混用**：T1／T2 前相機檔名是 receive time，兩個側相機則是 header capture time；T3 三台皆是 receive time。T1 的 3011 個側相機檔名全部不匹配 receive，但全部精確匹配 header，不能當成缺幀，也不能用最近訊息猜測。工具現按每相機「全量精確匹配」識別兩種慣例，模糊、空或部分匹配拒絕；共 6 項時間對齊測試通過。T1／T2 分別 3011／3299 幀全部配對成功。

| Track | fl−f capture 中位 ms | fr−f capture 中位 ms |
| --- | ---: | ---: |
| T1 | +6.952182 | +21.189413 |
| T2 | −2.395117 | +20.704074 |
| T3 | +1.292269 | −13.429226 |

這解釋為何直接用檔名差值的同步不能無條件跨軌複製。metadata 工具修正本身沒有改定位結果；正式 capture-time 同步仍待獨立 live matching 驗證。

`camera_timing.capture_offsets()` 已提供相對前相機 header 的精確秒偏移；即使只請求側相機也保留正確前相機參照，幀數不一致則拒絕。相關測試增加到 8 項，T1 3011／T2 3299／T3 3142 幀的有限值與前相機零偏移檢查通過；定位入口尚未啟用此函式。

**T2 第一階段實測（前 600 幀、phase 0，43 筆留出，僅本地參考）**：

| 模型 | 位置 RMSE m | 前向 RMSE m | 橫向 RMSE m |
| --- | ---: | ---: | ---: |
| baseline | 0.189316 | 0.096643 | 0.162790 |
| translation | 0.191289 | 0.102690 | 0.161389 |
| SE(2) | 0.191710 | 0.102478 | 0.162022 |
| gyro-heading + SE(2) | 0.205834 | 0.105437 | 0.176779 |

T2 這一階段不支持直接採用上述修改；T1 小幅改善也不能外推到 T2。八組本地對照已全部完成，未使用任何 T1／T2 提交額度。下一階段先做 GNSS bias 控制與 baseline／gyro-heading 的反相留出，不只保留有利拆分。

**投影診斷與反證**：已在 `work/runs/T3_projection_review/` 生成 0／105／630／2400／2448／2700 幀的三相機標線投影。投影落在車身可能只是地面標線被遮擋，不等於地圖保存了高車身點。`MarkingMap.points()` 的 z 本來就來自 `MarkingMap.height()`；630 幀半徑 24 m 內 25,734 格差值全為 0，所以「再用自身高度圖過濾高物體」是無效檢查，本輪不採用。影像遮擋仍應由車輛遮罩及量測可見性處理，不手工移動軌跡。

**額外校正假設的量級檢查**：同一個獨立 T2 靜止三軸 bias 投影到各軌新 heading-rate 定義時，隨姿態變動、相對 frame0 常數校正的最大額外積分差約 T1 0.062°、T2 0.226°、T3 0.081°。這是校正敏感度估計，不是已觀測定位誤差；量級不足以支持本輪再加三軸偏差參數，優先驗證已確認的拍攝時間語義問題。

**地圖來源邊界核對**：目前 Docker 的 `SDC_MAP_TAG` 為空，`bev_match.map_files()` 明確讀 `map.npy`／`map_intensity.npy`，不因資料夾存在許多 `map_clean*.npy` 就自動挑舊清圖。`map_to_npy.py` 只輸出 x／y／z 與 intensity，不以 Track2 的 `tl`／`th` 選點；正式 runner 以 `marking_map.py ... ALL` 建標線快取。此檢查確認目前程式路徑，不能單憑它聲稱所有歷史快取都重新審核過；重建時仍須照正式流程。

**已否定的初速過度自信假設**：T1 前 300 幀 VO accepted=0，乍看像初速先驗太窄，但實際前十對 VO 的位移與品質皆為 0，沒有可靠量測可融合。frame0 前兩秒有 7 筆 GNSS，逐對速度僅約 0.002–0.222 m/s，符合近乎靜止；T2 frame0 前無 GNSS、前十對 VO 品質也為 0。因此不盲目放寬 `v0_sd` 或回填未來速度，問題應分類為影像速度不可觀測／遮擋，不是已證實的 gate bug。

**GNSS 留出不是獨立真值**：T1 phase0 的 43 個留出樣本中位時間間隔約 0.500 s；baseline 前向／橫向殘差 lag-1 correlation 約 0.964／0.904，GNSS bias 模型仍約 0.943／0.890。不能把 43 筆當成 43 次獨立準確度證據，也不能以小幅平均差宣稱統計顯著；後續保留反相、分段、全長與實際 Kaggle 驗證。

**第二階段跨軌本地結果（均前 600 幀，不是 Kaggle 分數）**：

| Track／模型 | phase0 位置 RMSE m | phase1 位置 RMSE m | 結論 |
| --- | ---: | ---: | --- |
| T1 baseline | 0.697875 | 0.730889 | 原設定控制 |
| T1 gyro-heading + SE(2) | 0.689995 | 0.726159 | 兩拆分均小幅改善，仍需全長 |
| T1 GNSS bias only | 0.634929 | 0.663155 | 兩拆分均約 9% 改善，進入全長驗證 |
| T1 GNSS bias + gyro-heading | 0.663796 | 未測 | 不優於 bias-only，暫不擴展 |
| T2 baseline | 0.189316 | 0.226255 | 原設定控制 |
| T2 gyro-heading + SE(2) | 0.205834 | 0.224127 | 不同拆分方向不一致，不直接採用 |
| T2 GNSS bias only | 0.263233 | 未測 | 第一拆分明顯退步，不採用 |
| T2 GNSS bias + gyro-heading | 0.273266 | 未測 | 第一拆分明顯退步，不採用 |

phase0 每組 43 個留出，phase1 T1 43／T2 44 個。總計已完成八組初始對照、四組 bias 控制、四組 baseline／gyro 反相及一組 T1 bias 反相，共 17 組。T1 bias-only 的橫向平均殘差改變符號（phase0 −0.263 → +0.284 m），總誤差下降不代表每個偏差消失；全長仍須檢查。正在以全部合法 GNSS 跑 T1 600 幀／3011 幀兩次，確認前綴一致後才考慮單次 Kaggle 評分。T1／T2 本輪尚未提交，額度各保留 15 次。

**T1 全長與實際提交更新**：`t1_online_bias_causal.csv` 已完成全部 GNSS 的 3011 幀 live matching，與獨立 600 幀輸出前綴一致，CSV／核心版本檢查通過；實際 Kaggle public **0.59**（ref 57677630），明顯差於原 v7 的 0.46，**不採用、不替換 `submissions/final/result_1.csv`**。完整參數／環境／SHA-256 見同名 JSON。本地兩拆分較好並不能保證真實定位更好：相鄰 GNSS 高度相關、map／camera 偏差可能被當成 GNSS bias 吸收，起步局部指標也無法代表整段 public 評估。此失敗不作隱藏真值回推，不按分數擬合位置偏移；後續需更獨立的幾何／可觀測性驗證。T1 本日已用 1 次、剩 14 次；T2 仍未提交、保留 15 次。

**04:30 整合更新**：五個固定 T3 模型已完成，gyro-heading public 0.24（57677706），stop-soft 0.22（57677040），仍採用較簡單的 soft-heading 0.22；T3 剩 3 次額度。核心凍結釋放後，以下先前「待整合」項目已完成，但尚未有新全長評分：

- `SDC_CAM_CAPTURE=1`：在 `camera_time_offsets()` 使用精確 header 曝光偏移；runner 先抽取／驗證 metadata，並保存此環境設定。預設 0，因此不改已有 0.22 輸出；新增 `capture_header` 單一全流程對照，仍必須通過完整五假設、3142 幀與前綴檢查才提交。沒有改變 Kaggle 輸出幀的時間定義。
- `--gnss_yaw_sigma 3 --gnss_yaw_once`：預設關閉的 T1／T2 弱航向先驗；與位置使用同一留出子集，過期／未到達／無效／重複資料拒絕。只在 sample time ≤ frame time 時融合，使用當時可得陀螺儀傳播到當幀；不回改已輸出幀。真實 T2 30 幀測試：frame0 逐值不變，第一筆先驗於 frame1 才生效。
- 預設相容性：真實 T3 原最佳設定與 T2 原設定的前 30 幀，整合前後容忍 1e−10 一致；runner 命令行替身測試確認 T3 新預設、T2 未被同步設定污染。這是短程相容性證據，不冒充全長重跑。
- 本地 `validate_cross_track.py --variants gnss_yaw_once capture_header` 可比較兩個獨立因子，無上傳功能。核心雜湊加入 `camera_timing.py`、`gnss_heading.py` 兩個新增數值依賴；舊 JSON 雜湊使用當時的六個核心檔，不直接跨定義比較。

重現新單一 T3 header 實驗（主機執行，會使用一個額度但保留至少兩次，若截止／額度／驗證不通過就不提交）：

```bash
python3 midterm01/src/localization/tools/overnight_experiments.py \
   --profiles capture_header --deadline 2026-10-27T08:00:00+08:00 --reserve 2
```

同時保留舊 `conditioned_init`／`gyro_heading` 的定義：批次參數明確傳入 `--yaw_soft_peak 0`，不因正式 runner 新預設而暗中改變歷史實驗。

**新整合實測更新（05:00）**：T3 `capture_header` 全長／前綴通過，實際 Kaggle public **0.25**（57678479），差於 soft-heading 0.22，**不採用，`SDC_CAM_CAPTURE` 預設仍為 0**。正確 header 語義不保證在既有外參、地圖匹配與輸出時鐘下更準；完整方法勝出假設也由 offset 0° 變為 −1.5°，不能把 0.03 分差直接解讀成時間修正本身的公尺誤差。保留為可重現研究對照，不修改目前正式檔。T3 剩 2 次。

新本地前 600 幀 phase0（非 Kaggle）：T1 一次弱航向 0.698479 m，對 baseline 0.697875 幾乎無改善；T1 header 同步 0.663700 m，值得反相檢查但不僅憑它採用。T2 一次弱航向 0.237969 m，差於 baseline 0.189316，暫不做全長提交；停車時 orientation 可能是持續舊方向，訊息到達新不代表方向資訊更新。T2 header 尚在計算；本輪 T2 未消耗提交額度，原 v19／正式 0.26 檔保持不變。

四組新本地對照於 05:04 前全部完成：T2 header 位置 RMSE 0.192695 m（前向 0.105421、橫向 0.161300），仍不優於 baseline 0.189316；沒有將 T2 的任何新模型拿去消耗額度。T1 header 的局部改善也未宣稱為實際提升，先保留原正式模型。T1 本日 14 次、T2 15 次、T3 1 次可用；沒有自動重設或借用不同競賽額度。

T1 header 額外反相留出已完成：phase1 位置 RMSE 0.683824 m（baseline 0.730889），前向 0.524406、橫向 0.438878；phase0 0.663700（baseline 0.697875），兩拆分方向一致。現在只對此模型做全部 GNSS 的 600／3011 幀全長前綴驗證，未提交前不替換原 0.46 正式檔；不再把 T2 負結果模型拿去消耗額度。這一額外反相使本地短程對照合計 22 組。

**T1 header 最終驗證**：`t1_capture_header.csv` 已完成 3011 幀 live matching、獨立 600 幀前綴及核心版本一致性檢查，Kaggle public **0.46**（57679143），與原 v7 相同。沒有證據顯示較低公開分數，因此保留較簡單的原 T1 預設及 `result_1.csv`，不因相同四捨五入分數推測 private 更好。實際參數、環境、影像 header metadata SHA-256、CSV SHA-256 與驗證在同名 JSON。T1 本輪提交 2 次（0.59 不採用、0.46 同分不替換），剩 13 次；T2 未提交、仍 15 次。接下來不再新增兩軌模型。

失敗後只作模型診斷：相對原 v7（不是 ground truth），bias 候選前 0–299／300–599 幀的橫向差 RMS 約 0.403／0.766 m，600 幀後兩段約 0.070／0.068 m；前 300 幀 lane-yaw／VO accepted 都為 0。增加 GNSS bias 狀態可能讓初始航向與側向偏差在低可觀測區間互相吸收，即使相鄰留出 GNSS 較接近，也可能沿著錯誤模型解漂移。此分析不產生逐段修正量；後續若處理，應以因果的可觀測性與感測器品質控制模型自由度，不按 public 分數決定某幀後手動切換。

### 6.6 Autonomous Batch Results

Fixed model hypotheses; live Docker matching, causal-prefix and CSV checks before guarded upload. Scores below are Kaggle public scores, not local residuals.

<!-- overnight-results:begin -->
| Profile | Extra Options | Public | Ref | Status | File |
| --- | --- | --- | --- | --- | --- |
| conditioned_soft | `--init_pos_per_hyp --yaw_soft_peak 0.2` | 0.42 | 57676447 | SubmissionStatus.COMPLETE | `t3_batch_conditioned_soft.csv` |
| soft_heading | `--yaw_soft_peak 0.2` | 0.22 | 57675838 | SubmissionStatus.COMPLETE | `t3_batch_soft_heading.csv` |
| conditioned_init | `--init_pos_per_hyp` | 0.24 | 57675166 | SubmissionStatus.COMPLETE | `t3_batch_conditioned_init.csv` |
| stop_soft | `--still_meas 3 --yaw_soft_peak 0.2` | 0.22 | 57677040 | SubmissionStatus.COMPLETE | `t3_batch_stop_soft.csv` |
| gyro_heading | `--gyro_heading --gyro_bias -0.266065785` | 0.24 | 57677706 | SubmissionStatus.COMPLETE | `t3_batch_gyro_heading.csv` |
| capture_header | `--yaw_soft_peak 0.2` | 0.25 | 57678479 | SubmissionStatus.COMPLETE | `t3_batch_capture_header.csv` |
| vo_capture_dt | `--vo work/runs/T3/vo_capture_dt.npz --yaw_soft_peak 0.2` | 0.31 | 57679196 | SubmissionStatus.COMPLETE | `t3_batch_vo_capture_dt.csv` |
<!-- overnight-results:end -->

### 6.7 最終採用與後續

| Track | 正式檔 | 採用來源 | 已確認 public | 本輪結論 |
| --- | --- | --- | ---: | --- |
| T1 | `submissions/final/result_1.csv` | `t1_online_v9_ens5.csv` | **0.45** | 5 個因果成員的逐幀平均（v7 設定、`--fwd_batch 8/10/12`、`--gnss_huber`；6.13），離線 0.427 → 0.404 |
| T2 | `submissions/final/result_2.csv` | `t2_online_v22.csv` | 0.26 | v19 + `--slow_sharp_f 0.035 2` + 停車航向搜尋每 15 幀／σ5.6（6.8、6.12），離線 0.214 → 0.201 |
| T3 | `submissions/final/result_3.csv` | `t3_online_vofill.csv` | **0.22** | SE(2)+soft-heading+VO 缺口補值（6.8），runner 預設 |

最新版已通過全部 58 項 Docker unittest、Bash 語法及編輯器 diagnostics；完整候選另外通過 live matching、CSV 與 causal prefix 驗證。三份正式檔共 3011／3299／3142 筆，SHA-256 與採用來源相同，清單見 `submissions/final/README.md`。所有本輪程式、測試、實際 CSV／JSON 與分析已 commit／push 到 `copilot/research-kaggle-competition`；中間模型與失敗結果保留供重現，不能誤當正式最佳。

後續合法研究重點不是再以排行榜掃逐段位移：先處理 clock convention／measurement age 與既有外參校正的一致性；用獨立幾何擾動與跨路段可觀測性驗證，不把高度相關 GNSS 留出當 ground truth；對 lane peak／sharpness／遮擋建立更可靠的量測噪聲模型；在訊息實際到達時因果處理，已輸出幀不回寫。T1／T2 的舊低分 RTS／leaderboard probing 結果不恢復使用。private 分數未知，不能宣稱已達到未公開評估的準確度。Kaggle 網站最終選擇未由本輪自動更改，需與正式候選一致；本輪沒有留下會在 08:00 後提交的工作。

### 6.8 VO 缺口修正與 10/27 檢查（2026-10-27，台北時間）

**檢查結果**：10/26–27 的改動（SE(2) 側相機同步、`--yaw_soft_peak 0.2`、選用的 capture timing／GNSS yaw／gyro_heading／hyp mean、`submit_guarded.py`、`overnight_experiments.py`、tests）均為選用或已寫入 T3 預設；runner 以目前程式重跑 T3，與 0.22 正式檔逐值相同（max dpos 0.0），58 項 unittest 通過。

**發現的問題：路面 VO 長段缺失**。`bev_vo.py` 在模板區塊有效像素 < 90%（前車遮罩）時整對放棄，改寫 IMU 預測位移且 `resp=0`，濾波器忽略 `resp<=0.5` 的對，所以跟車時完全沒有 VO：T1 第 1–351 幀（車速 0.5–6 m/s）、T2 第 1–408 幀、T3 第 1147–2160 與 2854–3141 幀。

- `bev_vo.py --mask_tol 0.3`：有效比例介於 0.3 與 0.9 的區塊以有效遮罩做 `TM_CCOEFF_NORMED`；預設 0.9 時結果逐值不變。
- `tools/vo_fill.py`：只補原本 `resp<=0` 的對，其餘逐值保留（全面改用遮罩匹配會讓 T1 的逐對峰值追蹤發散）。補上的對數為 T1 312、T2 371、T3 1073。
- T3 結果：前 900 幀完全相同，接受的 VO 從 1740 增為 2813；共同幀 innovation 中 lane fwd 0.080 → 0.078 m、near yaw 0.314 → 0.308°，其餘差異在 ±0.001 以內。
  - 位置差：900–1257 幀平均 0.016 m，1257–2200 幀 0.056 m，2900–3142 幀 0.094 m。
  - 已接入 `run_online.sh`，僅 T3：步驟 4 產生 `vo_m30.npz` 與 `vo_fill.npz`。
- T1／T2 不採用：T1 對參考，同環境基準 0.427 → 0.437（混沌，約 ±0.02 屬雜訊範圍）；T2 0.214 → 0.215（持平）。

**未採用：`--fwd_batch N`**。把單次太弱的 lane 前向偏移累積 N 次，取中位數作一次量測；預設關閉。
- T1：N=10 得 0.414、N=6 得 0.426（同環境基準 0.427），都在混沌範圍內；各批中位數本身也不一致（+0.44、−1.1、+0.39、−0.58）。
- T3：結尾段的批次值為 −0.39、−0.89 等。這類弱匹配本身已知有 −0.2～−0.5 m 偏差，而共同幀 innovation 在 2400 幀後有好有壞（lane lat 0.101 → 0.107、near lat 0.076 → 0.069），N=6 還會改動起步段。

**T3 提交**：runner 輸出 `submissions/online/t3_online_vofill.csv`（Kaggle 57688125）public **0.22**，與 soft_heading 同分；第 157–1257 幀平均只差 0.005 m，差異主要落在後段（註：T3 public 實際是軌跡前段約 10–30%，從第 0 幀開始，見 6.11）。runner 重跑逐位元相同；`vo_m30.npz`／`vo_fill.npz` 重建後逐陣列相同。

**發現的問題：T1／T2 正式檔無法以目前 Docker 逐位元重現**。容器在 10/25 15:26 重建（映像仍是 10/16 的 `sdc_ros2_dev`），v7／v19 則是 10/24 在舊容器產生；舊容器裡手動裝過的套件已不存在。用 v7 當時的 commit（6f19f83）在目前容器跑 T1，前 60 幀就差 2.4e-5 m，再被混沌的 T1 濾波器放大，所以不是程式改動造成。目前環境本身是確定的：重跑三次結果逐位元相同。

| Track | 舊容器檔 | 目前映像重跑（runner） | 對參考離線 | public |
| --- | --- | --- | --- | --- |
| T1 | t1_online_v7（0.46） | `t1_online_v7_img.csv` | 0.408 → 0.427 | 0.47（57689088） |
| T2 | t2_online_v19（0.26） | `t2_online_v19_img.csv` | 0.214 → 0.214 | 0.26（57689091） |

為符合「runner 可重現」的規定，`submissions/final/result_1..3` 改為目前映像可逐位元重現的版本。舊的 v7（0.46）在目前環境無法重現；若要選它，需自行承擔 TA 重跑不一致的風險。

T1 變體也要對同環境基準 0.427 比較，不能對舊檔 0.408：fwd_batch 10 得 0.414、6 得 0.426，vofill 0.437，masked VO 全面使用 0.468。這些都在 T1 約 ±0.02 的混沌範圍內，fb10 的各批中位數也互相矛盾，所以不採用。

本地篩選腳本放在 `work/online/`（未納入版控）：`t3_full.sh`（runner 的 T3 完整定位階段，假設 0°，可加選項）、`vo_valid.py`（逐列帶的模板有效比例）。

**T2：起步車陣的弱前向匹配（v20，`--slow_sharp_f 0.035 2`）**。T2 前 300 幀對參考一直落後 0.23–0.31 m。當時被接受的 lane 前向修正，sharpness 只有 0.020–0.034、peak 0.20–0.25，都剛過門檻，而且一致指向 −0.3～−0.5 m；同一時間 GNSS 前向 innovation 都是正的（+0.04～+0.15 m），方向相反。這與既有記錄「低 sharpness 前向匹配偏差 −0.2～−0.5 m」一致。
- 新選項 `--slow_sharp_f SHARP SPEED`：估計車速低於 SPEED 時，前向修正的 sharpness 要高於 SHARP。
- T2 結果：第 0–400 幀的 GNSS 前向 innovation rms 0.116 → 0.083 m（全程 0.100 → 0.096）；對參考 0.214 → 0.206，400 幀之後不變。
- 對參數不敏感：0.035 和 0.05、2 和 4 m/s 的結果完全相同。全程改用 0.035 則穩態變差（0.163 → 0.171），所以只在低速時套用。
- T1 輸出逐位元不變。T3 只在第 2979 幀之後改變，共同幀 innovation 持平略差，不採用。
- runner 重跑逐位元相同。Kaggle 57690754 public 0.26，兩位小數看不出 0.008 的離線差異。

**同日其他已測、未採用**（皆以同環境基準比較）：
- **T2 GNSS 時間相關偏差狀態**（`--gnss_bias_sd 0.25 --gnss_bias_tau 3`、`0.2 / 10`）：lane 共同幀 innovation 大降（lat 0.112 → 0.034 m），但獨立的 near lat 不變（0.100）；對參考則變差，0.206 → 0.261／0.230。T1 同模型的 public 為 0.59（原 0.46），方向一致。
- **T2 放寬 GNSS 前向 σ 到 0.25**：0.207，持平。
- **T2 起步段**：剩下的 −0.2 m 前向差在 GNSS 和參考之間（濾波器的 GNSS innovation 已約為 0），車陣中沒有地圖量測可依據，不調整。
- **T1 `--fwd_batch 8/10/12`**：對參考 0.427 → 0.408／0.414／0.407，方向一致。以 N=10 提交為 t1_online_v8（Kaggle 57691455），public 0.47，與 v7_img 同分，保留較簡單的 v7_img。
- **T2 v21（`--stand_search_sigma 4`，原本 2.5）**：靜止時的五次航向搜尋看的是同一個車陣場景，誤差彼此相關，相對起步後 lane 航向推定的位置都約 +1.5°。若每次都當成 σ=2.5° 的獨立量測，五次合起來會把航向轉到 +0.77°。
  - 共同幀 innovation：lane 航向 0.581 → 0.518°，lane lat 0.113 → 0.100 m；起步後 300–600 幀的 lane 航向 innovation −0.36 → −0.31°。
  - 對參考 0.206 → 0.205。σ=5.6 時前 200 幀反向偏太多（0.215），每 60 幀搜一次則為 0.212。
  - runner 重跑逐位元相同；Kaggle 57692689 public 0.26，已採用為正式檔。

### 6.9 GNSS-denied 預演與 T3 VO 噪聲（2026-10-27 晚，台北時間）

T3 沒有 ground truth，所以前向模型的參數改用 **GNSS-denied 預演**驗證：在 T1／T2 上用 T3 的濾波設定，但去掉 GNSS 和依賴 T3 地形的選項，起點取參考軌跡第 0 列（僅驗證用），再和參考軌跡比較（腳本 `work/online/gd.sh`，未納入版控）。

| 變體 | T1 預演（全程／400 幀後） | T2 預演（全程／400 幀後） |
| --- | ---: | ---: |
| T3 設定，原始 VO | 81.1／86.9（發散） | 0.635／0.597 |
| + VO 缺口補值（6.8） | 2.666／1.668 | 0.584／0.569 |
| + 前向 sd 下限 0.1 | 2.653／1.642 | 0.610／0.599 |
| + `--vo_noise 0.05`（原 0.08） | **0.925／0.635** | **0.514／0.496** |
| + `--vo_noise 0.04` | 0.899／0.769 | — |
| + `--vo_noise 0.03` | — | 0.529／0.502 |
| `--vo_noise 0.05` + `--acc_noise 0.2` | — | 0.545／0.519 |

- VO 缺口補值在沒有 GNSS 時是必要的：T1 跟車段（第 1–351 幀）完全沒有 VO 時，前向會發散。
- 在稀疏的 lane 前向修正之間，速度更貼近 VO 時誤差最小，兩條路線都以 0.05 最好。
- **T3 本身**：runner 加上 `--vo_noise 0.05` 後（`work/runs/T3_vn05`），heading 假設庫在第 600 幀選 −1.5°（分數 906 對 0° 的 896）。
  - 共同幀 innovation：lane fwd 0.077 → 0.068 m、lane 航向 0.239 → 0.229°；lane／near lat 持平，near 航向 0.306 → 0.325°（near 航向會回到先驗，不可靠）。
  - 第 157–1257 幀的位姿平均只移動 0.018 m／0.014°，因為那裡前向修正密集；0–156 幀差 0.076 m，1258 幀以後差 0.033 m。
  - 提交為 `submissions/online/t3_online_vn05.csv`（Kaggle 57699242），public **0.26**（vofill 0.22），**不採用**，runner 維持 0.08。
  - 差異集中在起步：第 16–48 幀前向差到 −0.58 m、航向 +0.53°，因為假設庫改選 −1.5°，前 600 幀的逐幀 MAP 切換了 3 次。差異 rms：0–60 幀 0.25 m／0.20°，0–314 幀（前 10%）0.112 m／0.090°，0–1257 幀 0.061 m／0.047°。
  - 分數是 ATE 的 RMSE；規則寫明 T3 public 是軌跡前段約 10–30%（從第 0 幀開始），起步段的差異完全落在 public 內，與 public 跳 0.04 一致。（原本以「157–1257 幀」推論，那個區段是錯的，見 6.11。）
  - 因此這次 public 反映的是起步段，不代表 private（1257 幀以後）；但依規則不能看排行榜後只在某段換參數，所以整個變體不採用。
  - 教訓：GNSS-denied 預演適合檢查前向漂移，但 T1／T2 前向修正稀疏，T3 修正密集，結論不能直接套用；而且 T3 起步段對任何改動都很敏感。

**未採用：停車偵測改用 VO**。T3 約在第 920 幀就已停下（VO 速度約 0），但濾波器速度到 960 幀前仍有 0.2–0.35 m/s，因為 IMU 靜止判斷要等煞車後的車身晃動消失；停車期間有幾幀被單一 VO pair 的 4–6 mm 雜訊誤判為移動，於是在第 1107／1152 幀各收了一次 lane 量測，停車位姿移動了 −0.13／+0.07 m。
- 試過兩個選項：`--vo_zupt N TOL`（最近 N 對 VO 平均速度 < TOL 時把速度歸零）和 `--vo_move_n N`（用 N 對 VO 的平均判斷是否在移動）。
- T3 共同幀 innovation：lane fwd 0.078 → 0.072／0.074 m；2400–2700 幀的 lateral 有好有壞。
- T2 預演明顯變差（0.584 → 0.708），所以撤回程式碼，未採用。

### 6.10 10/27 晚間工作紀錄：問題、分析流程、成功與失敗（台北時間）

**目標**：在不看排行榜調參的前提下降低 T3 分數，T1／T2 跟著降。以下依時間順序記錄每個問題的發現方式、驗證方法和結論。

#### 1. 建立 T3 的離線驗證方法：GNSS-denied 預演
- **問題**：T3 沒有 ground truth；fair innovation 只能比較量測一致性，不能直接代表誤差。
- **做法**：`work/online/gd.sh`，在 T1／T2 套用 T3 濾波設定（去掉地形選項），關掉 GNSS，起點用參考軌跡第 0 列，再和參考軌跡比較。
- **發現（成功）**：T1 使用原始 VO 時前向發散到 81 m；換成 VO 缺口補值後是 2.67 m，T2 也是 0.635 → 0.584。這直接證明 6.8 的 VO 補值在沒有 GNSS 時是必要的。
- **後來的限制（失敗）**：預演選出的 `--vo_noise 0.05` 在 T3 public 變差（見第 3 點）。T1／T2 的 lane 前向修正稀疏，T3 修正密集，結論不能直接套用。

#### 2. T3 停車位姿分析：VO 停車偵測（未採用）
- **發現的問題**：
  - T3 約在第 920 幀停下（VO 速度約 0），但濾波器速度到 960 幀前仍有 0.2–0.35 m/s：IMU 靜止判斷要等煞車後的車身晃動消失。
  - 長時間停車中，單一 VO pair 的 4–6 mm 雜訊（門檻 4 mm）讓靜止旗標在第 977／1003／1093／1107／1115／1121／1152 幀斷掉。斷掉的幀如果剛好是量測幀（1107、1152），就會收一次 lane 量測，停車位姿因此移動 −0.13／+0.07 m、航向 −0.08°。
- **分析流程**：
  1. 逐幀檢查 debug 的 `still` 旗標和量測採用情況。
  2. 用 `work/online/still_diag.py` 確認 IMU 靜止條件其實都成立，因此斷點不是 IMU 造成的。
  3. 再對照 VO pair，確認是 `vo_moving` 誤觸發。
- **嘗試**：`--vo_zupt N TOL`（VO 連續 N 對平均速度 < TOL 時速度歸零）和 `--vo_move_n N`（用 N 對 VO 平均判斷是否在移動）。
- **結果（失敗）**：
  - T3 共同幀 lane fwd innovation 0.078 → 0.072，但 2400–2700 幀的 lateral 有好有壞。
  - 停車位姿變化的方向無法判斷：前向較接近 lane 量測的平均，航向則離停車前的量測較遠。
  - T2 預演明顯變差（0.584 → 0.708），程式碼已撤回。

#### 3. `--vo_noise 0.05`：離線驗證成功、public 失敗
- **離線**：預演 T1 2.67 → 0.93、T2 0.584 → 0.514（0.03／0.04 都比 0.05 差）；T3 共同幀 lane fwd 0.077 → 0.068、lane 航向 0.239 → 0.229。
- **public**：0.26（vofill 0.22），Kaggle 57699242。
- **失敗原因分析**：
  1. 先計算兩版差異：第 157–1257 幀只有 rms 0.028 m／0.018°。（當時誤以為 public 是 157–1257 幀，用三角不等式推論「解釋不了 +0.04」；這個推論本身也只是勉強成立：上限約 0.032，而顯示值的最小可能差距是 0.030。正確的 public 是前段 10–30%，見 6.11。）
  2. 差異其實集中在第 16–48 幀：前向最多 −0.58 m、航向 +0.53°。原因是 VO 噪聲改變了起步段的似然，假設庫第 600 幀改選 −1.5°（906 對 896），逐幀 MAP 切換 3 次。
  3. 這與既有紀錄「T3 public 主要由前 60 幀決定」一致：public 對起步段的任何改動都極敏感。
- **結論**：不採用，runner 維持 0.08。依規則不能看到 public 後只在起步段換回舊參數。
- **自己犯的錯**：我在 T3 runner 重現跑到一半時，把 `run_online.sh` 改回 0.08；bash 會邊跑邊讀腳本，該次重現因此作廢並已停掉。之後要先等 runner 結束再改腳本。

#### 4. T1／T2（有 GNSS）的 VO 噪聲
- T2 原為 0.15，改成 0.08 得 0.207、0.05 得 0.214（原本 0.205）；T1 原為 0.02，改成 0.05 得 0.450（原本 0.427）。
- **結論**：兩軌目前的值都是局部最佳。有 GNSS 時，VO 太緊會把 VO 的尺度和偏差帶進位置；沒有 GNSS 時，VO 是修正之間唯一的速度來源，所以預演偏好較緊的值。

#### 4b. T1／T2 改為全段評分後的誤差組成（使用者告知 T1／T2 public 是全段）
- **T1**：第 0–300 幀佔全段平方航向誤差的 52%（航向 rms 1.26°），佔平方位置誤差的 19%；第 1800–2400 幀佔平方位置誤差的 35%（前向）。
- **T2**：第 0–300 幀（原地排隊）佔平方航向誤差的 59%（0.82°），佔平方位置誤差的 29%。
- **T1 起步**：這段沒有任何 lane 航向量測（地圖變形，峰值 < 0.25），航向完全由 GNSS 帶動。第 105–180 幀 GNSS 側向 innovation 連續約 −0.2 m，車速約 2 m/s，航向因此從 +1.5° 擺到 −1.2°。
- **短窗軌跡匹配原型**（文獻 Chen 2023：GNSS 序列對 IMU 推算軌跡做 2D 旋轉擬合，`work/online/traj_yaw_proto.py`）：窗長 5／10／15 秒在第 150–300 幀都偏 −1.3～−1.9°，和濾波器一樣。這代表 GNSS 軌跡本身的方向就和參考差 1.3°，換估計方法也無法修正。
- **T1 `--yaw_soft_peak`**（第 200–300 幀峰值 0.15–0.20 的 lane 航向，相對參考中位數 −0.16°，濾波器為 −1.53°）：
  - 0.15：第 150–300 幀航向 1.33° → 1.04°，全段 0.427 → 0.423。
  - 0.2：第 300–600 幀變差（0.64° → 1.18°），全段 0.447。
  - 結果不單調，屬 T1 的混沌範圍，不採用。
- **T1 第 1800–2400 幀**：第 2040 幀附近一個 sharpness 0.024 的弱 lane 前向修正（df −0.63）把位置往回拉；速度尺度 kvo 先前也被一串偏負的弱修正拉低到 0.95，因此落後參考 0.6–1.0 m，到第 2190 幀才恢復。這段 GNSS 前向偏 +0.5～+1.4 m，σ 1.0 拉不動。`--fwd_batch` 正是針對這個問題（離線 0.407–0.414，public 仍 0.47）。

#### 5. 外部建議逐項查證
| 建議 | 查證方式 | 結論 |
| --- | --- | --- |
| 側相機因果時間同步 | 讀 `bev.py`／`run_online.sh` 與過去的留出驗證 | T3 已預設啟用（SE(2)，使用當下的速度與角速度）；T1／T2 的 GNSS 留出驗證 0.698 → 0.699、0.189 → 0.192，T2 全程 0.205 → 0.205，持平 |
| IMU／GNSS 的 bag `t` 與 header `stamp` | 三軌實測 | IMU 中位差 0.2–0.3 ms、p95 0.5 ms；GNSS 約 1 ms；只有 T3 最開頭 6 筆 IMU 成批寫入，延遲 7–26 ms，影響第 0–3 幀。整體可忽略 |
| T3 航向偏差 | 過去的 ablation | `lane_yaw_off 0` 讓航向指標變好、位置變差，已記錄（PLAN） |
| T1 起步航向 | 過去的掃描 | yaw0_sd 沒有趨勢；ensemble 的 public 0.46 → 0.47 |
| 假設庫 600 幀後固定 | 過去的提交 | 後驗平均 0.23，沒有改善 |
| 中間產物快取 | 流程檢查 | 每個實驗用獨立的 `SDC_RUN_DIR`；VO 和標線圖不受所測參數影響 |

#### 6. 前相機曝光延遲假設（否定）
- **實測**：影像在檔名時間（接收時間）之前 7–9 ms 拍攝（`camera_timing.csv`，三軌三台相機都是 6–9 ms）。濾波器把影像當作檔名時刻的量測，理論上 lane 前向修正應落後「車速 × 8 ms」，10 m/s 時約 8 cm。
- **檢驗**：lane 前向 innovation（sharpness > 0.05）按車速分組，T2 在 5–11 m/s 為 +0.08～+0.095、T1 為 +0.01～+0.035、T3 為 +0.01～+0.02。
- **結論**：都不是隨車速增大的負值，所以假設不成立；補償反而會讓 T2 更偏。

#### 7. 新工具：T3 起步重啟測試（進行中）
- **動機**：T3 public 由起步段主導，但起步段沒有任何離線驗證方法。
- **做法**：
  - `localize_online.py` 新增 `--first K0`（從第 K0 幀開始，當作第 0 幀）和 `--guess x,y,yaw`（取代 init_guess.txt 的位姿，第 0 幀的航向搜尋與位置比對照常執行）。兩者都只用於驗證，預設不變；`--first 0` 跑 60 幀與原本的 hyp_0 逐值相同。
  - `work/online/restart.sh`：在已收斂的路段，以擾動過的 init_guess（航向約 17°、位置約 1 m，與實際起步相同）重新跑假設庫和逐幀 MAP，再以收斂軌跡為代理真值，比較前 150 幀的誤差。
- **用途**：先用 vn05 檢驗方法是否有預測力（public 已知 vn05 起步較差），再比較起步設定。
- **結果**（5 個重啟點，K0=600 的寬範圍航向搜尋鎖錯 25°，另計為失敗；其餘 4 點的 pooled／中位數）：

  | 設定 | 前 60 幀 | 前 150 幀 |
  | --- | --- | --- |
  | 基準 | 0.261／0.227 | 0.193／0.171 |
  | `--vo_noise 0.05` | 0.267／0.243 | 0.191／0.169 |
  | `--yaw0_sd 0.75` | 0.264／0.235 | 0.195／0.173 |
  | `--yaw0_sd 1.0` | 0.263／0.234 | 0.195／0.174 |

  - 假設庫的組合方式（MAP 先驗 sd 1／2／4°、後驗平均）也只差 ±0.006。
  - 起步誤差主要是所有假設共同的收斂過程，這些參數都改變不了。vn05 在前 60 幀較差，方向與 public 一致，但差距太小，還不能證明這個方法有預測力。
  - K0=600 的失敗顯示：init_guess 偏 17° 時，±25° 的寬範圍搜尋可能抓到錯的峰；實際 T3 起步沒有發生，但這是起步流程的脆弱點。

#### 教訓
- T3 public 是 RMSE，而且由起步段主導：public 的變化主要反映起步段，不代表 private。
- 離線驗證要對準被評分的段落。預演適合前向漂移；起步段要用重啟測試。
- 共同幀 innovation 變好不等於誤差變小（vn05 就是例子）。
- runner 執行中不要改 `run_online.sh`。

### 6.11 自我檢討：換個角度重新檢查做法（2026-10-27 晚）

逐項列出原本的做法或假設、漏洞、驗證結果，以及影響與修正。

**1. T3 public 區段用錯（已確認是錯誤）**
- **原本**：從前一輪紀錄沿用「public ≈ 第 157–1257 幀」。
- **查證**：規則整理（`docs/midterm01_rules_2026-10-21.md`）寫的是「Public 只算軌跡前段約 10–30%」，PLAN 引用 PDF 則是前 5–40%；兩者都從第 0 幀開始，所以 public 大約是第 0 幀到第 314–943 幀。
- **影響**：
  - (a) vn05 提交前，我只檢查了 157–1257 幀的差異（0.018 m），沒看第 0–60 幀的差異（0.25 m／0.20°），因此沒發現這其實是在賭起步段。
  - (b) 停車段（第 960 幀以後）全部屬於 private；之前估計的「對 public 的影響」不成立。這段對最終排名仍然重要。
  - (c) 6.9／6.10 的推論已更正；memory 也已修正。

**2. 用參考軌跡當真值有循環論證的風險**
- **原本**：T1／T2 的離線分數都以 `t1_pipe_v14m`／`t2_pipe_v16` 為準。
- **漏洞**：這兩條是舊版非因果流程的輸出，用的是同一套標線地圖匹配、相機校正和 GNSS，誤差可能和我們的相關，所以離線「變好」可能只是共同偏差。今晚提出的「地圖變形路段的真值跟著地圖走」，證據是「lane 航向和參考一致」，但參考本身就是用 lane 匹配做出來的，這是循環論證，目前只是假說，沒有據此修改任何東西。
- **歷史對照**：

  | Track | 離線（對參考） | public | 一致嗎 |
  | --- | --- | --- | --- |
  | T2 | 0.223 → 0.214 → 0.205 | 0.27 → 0.26 → 0.26 | 方向一致 |
  | T1 | ensemble 0.404（優於 v7 的 0.408） | 0.47（v7 為 0.46） | 失準一次 |

- **結論**：T1 小於約 0.02 的離線差異不可靠；T2 的參考大致可用。

**3. 共同幀 innovation 變好，不等於誤差變小**
- innovation 衡量的是預測和量測的一致性。如果濾波器更緊地跟著有偏差的量測，innovation 也會變小，但不代表離真值更近；vn05 就是 innovation 變好、public 變差（雖然主因是起步段）。
- 以前也有只靠 innovation 就採用的改動（seg_gate_from、roll_imu_map、terrain_lat），當時有 public 同分或改善作旁證。之後 innovation 只當輔助證據。

**4. GNSS-denied 預演的外推限制**
- 預演的起點取自參考軌跡第 0 列（完美起步），也去掉了地形選項；T1／T2 的前向修正又比 T3 稀疏。它能回答「沒有修正時的漂移」，但測不到起步段，而 T3 public 偏偏由起步段主導。
- 我根據預演採用 vn05 之前，應該先檢查起步段的變化。

**5. 相機延遲檢驗有混淆因素，結論說得太滿**
- 原本把 lane 前向 innovation 按車速分組，對照以 GNSS 為主的狀態，就寫「假設不成立」。
- 但 GNSS 自身的延遲、lane 前向的尺度偏差和狀態本身的偏差都會混在一起，所以應改為「這個檢驗不支持延遲假說，結論不確定」。6.10 第 6 點以此為準。

**6. 重啟測試還不能當決策依據**
- 代理真值是我們自己收斂後的軌跡，擾動只有一個方向，有效重啟點只有 4 個；vn05 和基準只差 0.006，預測力尚未建立。
- 要先增加重啟點和擾動方向，確認能穩定分辨好壞，才拿來做決策。

**7. 三角不等式的推論說得太肯定**
- 原本說「157–1257 幀的差異不可能造成 +0.04」，但上限約 0.032，而顯示值的最小可能差距是 0.030，只是勉強成立（而且區段本身也用錯了）。

**8. 流程錯誤：執行中改腳本**
- 今天兩次在背景工作執行期間修改它正在跑的 `.sh`，一次 runner 重現作廢，一次 DONE 標記沒寫出。已加入 memory 規則：要改就先等工作結束，或另存新檔。

**9. 低資訊量的提交**
- 離線改善只有 0.01–0.02 時，2 位小數幾乎看不出來（T2 v20／v21、T1 v8 都同分）。沒有違規，額度也夠，但資訊價值很低；之後只提交預期變化至少 0.01、或能回答特定問題的版本。

**經得起檢查的部分**
- VO 缺口補值：預演中 T1 從發散 81 m 降到 2.67 m，T2 0.635 → 0.584；T3 public 維持 0.22。證據來自不同路線，結果一致。
- 正式檔可逐位元重現（runner 重跑），全部都是因果輸出。
- `--first`／`--guess` 只用於驗證，預設行為逐值不變。

### 6.12 使用者肉眼比對與後續查證（2026-10-27 晚）

使用者在 `/compare/Track1`、`/compare/Track2`（版本 1 = 正式檔，2 = 參考軌跡）上逐幀比對地圖標線投影和影像，以下是觀察與查證結果。

- **T1 起步段的參考軌跡不可靠**：第 162、180 幀是正式檔較準，第 342、414–444 幀則是參考較準。所以 6.10 中「T1 起步航向佔 52%」有一部分其實是參考軌跡本身的誤差。
- **T1 第 2124 幀**：正式檔的紅點太前面（位置在真值後方），參考太後面（在真值前方），真值介於兩者之間。因此「落後參考 0.6–1.0 m」大約只有一半是真的。
- **檢視器假象**：版本 1／2 的紅點是把原始強度做門檻，再投影到平坦地面上。遠處、左前方和有坡度的路段會因此偏掉；T2 起點（建圖車停過的地方）還會出現大量假紅點。已另外產生版本 3／4（`make_viewer --marks mk_D`：定位實際使用的標線地圖，畫在地圖的 3D 高度上）。
- **T2 起點的標線地圖雜訊（測試後不採用）**：起點的 `mk_D` 佈滿斑點。新增 `tools/marking_clean.py`（只用地圖：保留長 ≥ 1.5 m 且長寬比 ≥ 4、或面積 ≥ 1 m² 的區塊，標線格數剩 66–70%）後，T2 0.205 → 0.214（lane 側向 innovation 0.100 → 0.113），T1 0.427 → 0.440。這些斑點是影像中也看得到的真實結構（地磚、路面紋理），拿掉反而傷害匹配。
- **T2 停車時的「跳動」**：第 0→1 幀跳 0.635 m，是因為第 0 幀時第一筆 GNSS 還沒到，只能用 init_guess（偏 0.94 m），只影響這一幀。之後每 30 幀的航向搜尋讓航向從 −1.89° 逐步收斂到第 150 幀的 0.01°。
- **T2 何時開始移動**：VO 補洞版（影像匹配，不依賴 IMU）的速度在第 170 幀為 0.03 m/s、180 幀 0.10、200 幀 0.30、240 幀 1.0，濾波器和參考也一致。0.1–0.3 m/s 的蠕動肉眼很難察覺，所以速度估計是對的，不修改。
- **側相機航向（使用者觀察「右前準、左前歪」）**：用 `work/online/percam.py` 在正式位姿上每 10 幀分別只用單一相機做 lane 匹配。T1 同幀配對 36 幀：左前比右前低 0.42°（MAD 0.70）；相對融合結果，左前 −0.09°、右前 +0.06°。T2 左前的未配對中位數比融合低約 0.6°。雜訊比差異大，兩軌的幅度也不一致，加上以前查過改校正會讓 lane 變差，因此不修改校正。
- **影像空間的渲染比對（使用者建議，Wolcott & Eustice 2014 的做法）**：`work/online/rc_yaw.py` 把地圖地面點連同反射強度投影到三台相機（粗網格 z-buffer 處理遮擋），掃描航向 ±3°，以 normalized mutual information 評分。
  - 在濾波器已經準的幀（T2 第 900／1500／2400 幀），最佳航向偏 +0.3～+1.4°，峰值對比只有 0.0001–0.03，幾乎是平的。
  - 在起步幀反而指向錯誤的方向。
  - 路面反射強度和相機灰階之間的訊號太弱，要可用還需要合成影像渲染、車輛遮罩、局部 NMI 等工程，不在剩餘時間內進行。
- **T2 停車航向搜尋的頻率**：v21 是每 30 幀一次、σ 4°。

  | 變體 | 對參考 | 第 0–300 幀航向 rms | 備註 |
  | --- | --- | --- | --- |
  | v21（30 幀／σ4） | 0.205 | 0.82° | |
  | 15 幀／σ4 | 0.201 | 0.72° | 第 150–200 幀過衝到 +0.5°，lane 側向 innovation 0.100 → 0.107 |
  | 10 幀／σ4 | 0.207 | 0.83° | 過衝 +0.82° |
  | 15 幀／σ5.6 | 0.201 | 0.76° | 總資訊量和 v21 相同、分散得更平均；innovation 全部持平或略好（lane 側向 0.099、near 側向 0.098、lane 航向 0.515） |

  「15 幀／σ5.6」原理上說得通，但只改善 0.004，2 位小數看不出來，依 6.11 第 9 點先記錄、不提交。
- **T2「15 幀／σ5.6」加側相機同步**：0.202（不加同步 0.201），沒有疊加效果；lane 前向 innovation 0.260 → 0.266，側向和航向略好。不提交。

**10/27 晚間結論**：今晚驗證的方向都沒有超過基準，或是改善小於 2 位小數看得出的範圍。正式檔維持 T1 `t1_online_v7_img`（0.47）、T2 `t2_online_v21`（0.26）、T3 `t3_online_vofill`（0.22）。
- **T3 起步的事後推算指標（診斷用，不採用）**：拿收斂後第 K 幀的位姿，用 VO 位移和陀螺儀航向往回推，得到「事後的第 0–K 幀路徑」，再拿各版本相比（`work/online/hindsight_prof.py`、`hindsight_rank.py`）。
  - vofill 在第 0 幀偏右 0.53 m，約 30 幀後收斂；前向會隨錨點改變（K=90 時 −1.4 m，K=120 時 −2.7 m），不可靠。
  - 拿 10 個已知 public 的 T3 提交比較，側向加航向指標和 public 是**負相關**（Spearman −0.72）；去掉 VO 側向後仍為 −0.59～+0.18。
  - 結論：事後路徑本身有偏差（錨點航向、VO 尺度），無法當作 T3 public 的代理指標。
- **T2 v22 提交**（每 15 幀／σ5.6，Kaggle 57703980）：public 0.26（與 v21 同分）。runner 預設已更新，重跑結果和提交檔逐位元相同；因為離線分數和 innovation 都比 v21 好，正式檔 `result_2.csv` 改為 v22。

### 6.13 T1 v9：混沌濾波器的成員平均（2026-10-27 23:20，public 0.47 → 0.45）

**問題**：T1 的濾波器在標線稀疏處是混沌的。今晚新增的 `--gnss_huber`（GNSS 更新也套用 Huber 權重；T1 的 GNSS innovation 可達 1–2 m，σ 卻只設 0.2 m）單獨使用時，對參考 0.427 → 0.409；和 `--fwd_batch` 組合反而是 0.417–0.438。可見單一變體 ±0.02 的差異大多是混沌造成的，挑「運氣好」的單一變體並不可靠。

**做法**：逐幀平均 5 個因果 runner 輸出（`tools/ensemble.py`：位置取平均、航向取圓形平均）。第 k 幀只平均各成員的第 k 幀，每個成員都只用第 k 幀以前的資料，所以平均後仍是因果的。
- 成員：v7_img 設定、+ `--fwd_batch 8`、`10`、`12`、+ `--gnss_huber`。
- 各成員對參考：0.427／0.408／0.414／0.407／0.409；平均後 **0.404**。11 個變體全部平均是 0.406，任何組合都在約 0.405，表示這一類設定的真實水準大約就在這裡。
- 分段：第 0–300 幀 0.437 → 0.373 m（這段參考本身不可靠，見 6.12），第 1800–2400 幀 0.420 → 0.361 m（`fwd_batch` 處理掉第 2040 幀那個弱前向修正），其餘路段相同。成員間的位置 sd：起步 0.16 m，其他路段 0.01–0.05 m。

**結果**：`submissions/online/t1_online_v9_ens5.csv`（Kaggle 57705538）public **0.45**，正式版為 0.47，是今天第一次真正降低分數。
- `run_online.sh` 的 T1 改為成員平均模式（5 個成員，最多 3 個同時跑，順序和提交檔相同，確保浮點加總逐位元一致），runner 重跑（`work/runs/T1_v9`）和提交檔逐位元相同，5 個成員也各自和原本的 run 相同；正式檔 `result_1.csv` 已改為 v9。
- 對照：T2 的變體平均只有 0.199–0.201（T2 不混沌）；T3 先前的 3 模型平均是 0.23（`t3_fixed_ensemble`），所以這個做法只用在 T1。
- **MapAnything 點雲用校正後的相機高度重試（使用者建議）**：10/23 只試過把尺度正規化到 1.45 m 和 1.65 m。相機校正（`calib/cam_fix_v10t*.json`：各相機的旋轉與高度修正，PLAN 記錄高度 1.52 m）的結果其實介於兩者之間，所以這次用 1.52 m 重跑 4 自由度 ICP（`work/online/ma_icp_batch.py ... -1.52`）。
  - 已知很準的移動段：第 600 幀 ICP 航向 +3.46°（濾波器 −0.38°），第 900 幀 −2.21°（濾波器 −0.17°），第 1500 幀 −0.04°；位置誤差 0.1–1.8 m；每幀的尺度在 0.66–0.80 之間變動。
  - 起步段：航向 −0.48 ～ −1.19°（第 240 幀 +2.99°），側向偏 0.75–1.0 m。
  - 結論：連位姿已知的幀都做不準，問題出在模型點雲本身（助教說明：尺度每幀約 4% 雜訊、沿路漂移 ±9%，每幀 1–1.6° 扭曲），換相機高度也無法改善，不採用。
