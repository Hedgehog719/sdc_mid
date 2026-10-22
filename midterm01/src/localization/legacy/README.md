# legacy/：規則變更前的做法（保留作紀錄，不屬於最終流程）

這個資料夾保存 10/22 規則更新前的程式、實驗腳本和提交檔。**最終提交的定位流程完全不使用這裡的任何程式**（見 `../README.md` 與 `tools/localize_online.py`）。

## 規則變更時間線

| 日期 | 公告 | 內容 | 對我們的影響 |
|---|---|---|---|
| 10/15 | 競賽開始 | Track01/02 可使用影像、IMU、GNSS；沒有限制離線平滑或後處理 | 用 RTS 雙向平滑融合 GNSS、IMU、車道與近處 fix |
| 10/20 | 論壇回覆＋「Track03 Updates & Rule Adjustments」 | 利用 Leaderboard 分數反推誤差、擬合軌跡的結果不得作為正式評分；Track03 不提供 GNSS | 停用以 Leaderboard 擬合的檔案（0.17429 / 0.41123），改為純流程輸出（T2 0.17、T1 0.40） |
| 10/22 | 「Mid01 Updates Rules」 | 要求 Causal / Online Localization：估計第 k 幀只能用第 1…k 幀；禁止 Backward Smoothing、非因果平滑、完成定位後的離線修正；助教會重跑程式 | 本資料夾內的流程全部不再合規，重寫為 `tools/localize_online.py`（只往前走的卡爾曼濾波） |

## 這裡保存了什麼

- `tools/`
  - **雙向平滑融合**：`fuse.py`、`rts.py`、`yaw_rts.py`、`cv_fuse.py`（RTS 平滑器：用未來資料回頭修正，10/22 起禁止）
  - **依賴完整軌跡的 fix 與篩選**：`near_fix.py`、`fix_consistency.py`（前後 ±45 幀窗口）、`fix_anchor.py`、`fix_calib.py`、`near_accel_corr.py`、桿子比對（`pole_*.py`、`poles.py`）、`init_search.py`（起點用前 30 幀）、`track_seq.py`（第一版追蹤器，中間幀用前後內插）
  - **用軌跡修補或篩選地圖**：`clear_roofs.py`、`splice_level.py`、`column_splice.py`、`level_start.py`、`clean_map.py`、`marking_road_vote.py`、`marking_paint_vote.py`
  - **Leaderboard 分析工具**（10/20 前尚未禁止時使用）：`board_opt.py`、`kaggle_predict.py`、`probe.py`
  - 其他實驗與舊版校正工具
- `work_mb/`：實驗、網格搜尋與評估腳本（包括以 Leaderboard 分數建立的參考估計 `est_score.py`、`est_lib.py`）
- `submissions/`：所有早期提交檔，包括以 Leaderboard 擬合的檔案（檔名帶分數者）

完整的實驗紀錄見 `../docs/PLAN.md`。
