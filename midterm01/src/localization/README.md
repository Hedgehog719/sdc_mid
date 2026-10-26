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
13. **Track03 路面的橫向起伏（`--terrain_lat`）**：第 9 點的地形投影只取車道中央帶的縱向剖面，橫向一律沿用車底平面的坡度。但靠路邊停車的那一側常比車底平面低（排水坡、路拱）：第 705–735 幀左側停車格旁的車道線，地圖比影像偏外 0.4 m，右側的線卻對得上（影像左右兩半分開和地圖比對，`work/online/side_shift.py`）。改成鳥瞰圖每一格都用地圖路面高度（1 m × 0.5 m 網格，相對「車底平面 + 縱向剖面」的差，±0.3 m 截斷，沿道路方向中位數濾波，雙線性內插）後，左右偏差都在 0.1 m 內。整段 Track03（公平比較）：車道橫向 0.096 → 0.026 m、近距橫向 0.064 → 0.042 m、車道航向 0.40° → 0.24°；直線段「航向 − 行進方向」從 −0.26° 變成 −0.01°（之前找不到原因的航向偏差，就是左右路面高度不同讓鳥瞰圖歪斜）。
14. **Track02 只用相機看得到的路面（`--seg_gate 0.5 --seg_gate_track`）**：追蹤時的車道比對，影像響應只保留 SegFormer 道路機率 > 0.5 的區域（投影到鳥瞰圖、外擴 0.35 m）：停在路邊的車、路緣、建築物、旁邊遊覽車的反光都不參與比對。地圖那一側不遮（初始航向錯的時候，用影像的路面去遮地圖，會剛好把要找的標線遮掉）。第 0 幀和靜止時的航向搜尋不套用（套用後起點航向反而更偏）。和參考軌跡比：0.238 → 0.223（第 400 幀以後 0.197 → 0.180、橫向 0.096 → 0.070 m）；Track01 套用後變差（0.411 → 0.429）。Track03 也套用，但從第 300 幀才開始（`--seg_gate_from 300`）：從起點就套用時，起步段（第 105–177 幀）正確的前後修正因峰值降低被拒、反而接受一個錯的，位置落後約 0.9 m；第 300 幀以後套用則起點不變，之後車道前後 0.089 → 0.067 m、車道航向 0.227° → 0.221°。
   另外 Track02 的車道航向雜訊放大到 2.5°（`--yaw_sigma 2.5`，三軌共用預設 0.7°）：Track02 的車道航向在某些路段有偏差（第 426–570 幀被採用的修正一致是 −0.5～−0.8°，航向被拉偏約 1°），有 GNSS 時航向也能從 GNSS 與橫向修正經運動模型得到：和參考軌跡比 0.223 → 0.214（第 400 幀以後 0.180 → 0.163）。Track01 放大後變差（0.411 → 0.430–0.441），Track03 沒有 GNSS，都維持 0.7°。
15. （試過、未採用）**Track01 多濾波器逐幀平均（`tools/ensemble.py`）**：Track01 的濾波器在標線稀疏處很敏感（設定只差一點分數就差 ±0.02），試過 9 個濾波器（初始航向不確定度 × 車道航向雜訊）逐幀平均：對參考軌跡 0.408 → 0.404（航向 0.56° → 0.48°），但起點位置變差，Kaggle public 0.46 → 0.47，所以正式流程仍用單一濾波器。
16. **輸出**：該幀的後驗位姿，寫出後不再修改。

可重現性：用 `tools/run_online.sh` 從頭執行，T1、T2、T3 的輸出和 `submissions/online/` 的候選檔逐位元相同（目前版本：`t1_online_v7.csv`、`t2_online_v19.csv`、`t3_online_tpg.csv`；`t1_online_v6c.csv` 是 T1 加剎車點頭模型之前、`t3_online_tpl.csv` 是 T3 加路面遮罩之前的版本；`t2_online_v18.csv` 是 T2 放大車道航向雜訊之前、`t2_online_v17.csv` 是加第 14 點之前、`t3_online_tpr.csv` 是加第 13 點之前的版本；`t3_online_tpa.csv` 是加固定側傾（第 12 點）之前、`t3_online_tpd.csv` 是加快速俯仰（第 11 點）之前的版本，已提交的 `t3_online_tp.csv` 再早一版（沒有 `--vo_dr_init`）；較早提交的 `t1_online_v6`、`t2_online_v6`、`t3_online_b6` 是之前的流程設定）。GPU 推論的微小差異可能讓分割結果有極小不同。

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

## 6. T3 同步實驗（2026-10-26）

**最新確認最佳：Kaggle public 0.22，達成 `<0.23`，提交 ID 57675838。** 檔案 `submissions/online/t3_batch_soft_heading.csv`，完整設定／核心雜湊／SHA-256／來源 revision 見同名 JSON。模型為原 SE(2) 因果流程加 `--yaw_soft_peak 0.2`：峰值 0.20–0.25 的航向採用較大的量測噪聲，不是硬丟棄，也不放寬所有位置修正。完整五假設與全長 live matching、600 幀前綴一致、3142 幀 CSV、截止及額度防護皆通過；以下較早「尚未達標」敘述是當時實驗進展，非目前最佳狀態。private 分數未知。

先依感測器時間戳與本地幾何／innovation 驗證決定修改，再提交檢查；不對提交軌跡做離線修正。

| 版本 | 修改 | Kaggle public | 提交 ID |
|---|---|---|---|
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
|---|---|
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
|---|---:|---:|---:|
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

**已驗證的影像時間語義缺口**：現有 `camera_time_offsets()` 用 JPG 檔名的 bag receive time 作為相機拍攝偏移，但 bag 的 H.265 topic 內另有真正的 `header.stamp`。T3 三相機各 3142 幀都能精確配對，不缺失、不以未來資料插值。前相機 receive−header 中位數 7.912 ms、最大 31.651 ms；真實 fl 相對 f 曝光時間中位數 +1.292 ms（範圍 +0.835…+1.851 ms），fr −13.429 ms（−13.888…−12.552 ms），比 receive time 差值穩定。用既有因果速度換算，誤用 receive offset 的行駛中 warp 位移誤差 RMS：fl 0.01222 m、fr 0.01250 m；最大 0.0650／0.0563 m。這不是 ground-truth 定位誤差，而是兩種時間定義造成的可量測投影差。

```bash
python3 tools/camera_timing.py /root/data/Track3
```

工具會建立可重算的 `sensors/camera_timing.csv` 並逐檔精確檢查。3 項 Docker 回歸測試已通過。定位核心尚未接入 header offset，避免污染目前凍結批次；完成後只比較相機相對曝光同步，不擅自改變 Kaggle frame 的輸出時間定義。

跨軌抽取進一步發現 **檔名慣例混用**：T1／T2 前相機檔名是 receive time，兩個側相機則是 header capture time；T3 三台皆是 receive time。T1 的 3011 個側相機檔名全部不匹配 receive，但全部精確匹配 header，不能當成缺幀，也不能用最近訊息猜測。工具現按每相機「全量精確匹配」識別兩種慣例，模糊、空或部分匹配拒絕；共 6 項時間對齊測試通過。T1／T2 分別 3011／3299 幀全部配對成功。

| Track | fl−f capture 中位 ms | fr−f capture 中位 ms |
|---|---:|---:|
| T1 | +6.952182 | +21.189413 |
| T2 | −2.395117 | +20.704074 |
| T3 | +1.292269 | −13.429226 |

這解釋為何直接用檔名差值的同步不能無條件跨軌複製。metadata 工具修正本身沒有改定位結果；正式 capture-time 同步仍待獨立 live matching 驗證。

`camera_timing.capture_offsets()` 已提供相對前相機 header 的精確秒偏移；即使只請求側相機也保留正確前相機參照，幀數不一致則拒絕。相關測試增加到 8 項，T1 3011／T2 3299／T3 3142 幀的有限值與前相機零偏移檢查通過；定位入口尚未啟用此函式。

**T2 第一階段實測（前 600 幀、phase 0，43 筆留出，僅本地參考）**：

| 模型 | 位置 RMSE m | 前向 RMSE m | 橫向 RMSE m |
|---|---:|---:|---:|
| baseline | 0.189316 | 0.096643 | 0.162790 |
| translation | 0.191289 | 0.102690 | 0.161389 |
| SE(2) | 0.191710 | 0.102478 | 0.162022 |
| gyro-heading + SE(2) | 0.205834 | 0.105437 | 0.176779 |

T2 這一階段不支持直接採用上述修改；T1 小幅改善也不能外推到 T2。八組本地對照已全部完成，未使用任何 T1／T2 提交額度。下一階段先做 GNSS bias 控制與 baseline／gyro-heading 的反相留出，不只保留有利拆分。

**投影診斷與反證**：已在 `work/runs/T3_projection_review/` 生成 0／105／630／2400／2448／2700 幀的三相機標線投影。投影落在車身可能只是地面標線被遮擋，不等於地圖保存了高車身點。`MarkingMap.points()` 的 z 本來就來自 `MarkingMap.height()`；630 幀半徑 24 m 內 25,734 格差值全為 0，所以「再用自身高度圖過濾高物體」是無效檢查，本輪不採用。影像遮擋仍應由車輛遮罩及量測可見性處理，不手工移動軌跡。

**額外校正假設的量級檢查**：同一個獨立 T2 靜止三軸 bias 投影到各軌新 heading-rate 定義時，隨姿態變動、相對 frame0 常數校正的最大額外積分差約 T1 0.062°、T2 0.226°、T3 0.081°。這是校正敏感度估計，不是已觀測定位誤差；量級不足以支持本輪再加三軸偏差參數，優先驗證已確認的拍攝時間語義問題。

**GNSS 留出不是獨立真值**：T1 phase0 的 43 個留出樣本中位時間間隔約 0.500 s；baseline 前向／橫向殘差 lag-1 correlation 約 0.964／0.904，GNSS bias 模型仍約 0.943／0.890。不能把 43 筆當成 43 次獨立準確度證據，也不能以小幅平均差宣稱統計顯著；後續保留反相、分段、全長與實際 Kaggle 驗證。

### 6.6 Autonomous Batch Results

Fixed model hypotheses; live Docker matching, causal-prefix and CSV checks before guarded upload. Scores below are Kaggle public scores, not local residuals.

<!-- overnight-results:begin -->
| Profile | Extra Options | Public | Ref | Status | File |
| --- | --- | --- | --- | --- | --- |
| conditioned_init | `--init_pos_per_hyp` | 0.24 | 57675166 | SubmissionStatus.COMPLETE | `t3_batch_conditioned_init.csv` |
| soft_heading | `--yaw_soft_peak 0.2` | 0.22 | 57675838 | SubmissionStatus.COMPLETE | `t3_batch_soft_heading.csv` |
| conditioned_soft | `--init_pos_per_hyp --yaw_soft_peak 0.2` | 0.42 | 57676447 | SubmissionStatus.COMPLETE | `t3_batch_conditioned_soft.csv` |
| stop_soft | `--still_meas 3 --yaw_soft_peak 0.2` |  |  | validated | `t3_batch_stop_soft.csv` |
<!-- overnight-results:end -->
