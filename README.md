# SDC 2026 Midterm01：影像定位（Track01／02／03）

最終流程與完整執行說明在 [`midterm01/src/localization/README.md`](midterm01/src/localization/README.md)。

- 方法：線上（因果）定位。每一幀只用到當下以前的資料，依時間順序跑一個 Kalman filter（10/22 規則）。
- 執行：在容器內跑 `bash /root/midterm/src/localization/tools/run_online.sh /root/data/Track1 T1`（T2、T3 同理）。
- 10/22 以前的做法（RTS 雙向平滑、Leaderboard 分析工具、舊提交檔）只保留作紀錄，放在 `midterm01/src/localization/legacy/`，規則變更時間線見該資料夾的 README。

資料不在版本庫內（見 `.gitignore`）。

## T3 改善進度（2026-10-27）

**已達目標：T3 Kaggle public 0.22，提交 ID `57675838`。** 最佳檔為 [`t3_batch_soft_heading.csv`](midterm01/src/localization/submissions/online/t3_batch_soft_heading.csv)，對應 [`JSON 紀錄`](midterm01/src/localization/submissions/online/t3_batch_soft_heading.json)。這是完整因果五假設／EKF 的輸出：SE(2) 同步加 `--yaw_soft_peak 0.2`，弱峰值航向用較大噪聲融合，沒有離線軌跡修正；尚無 private 分數可宣稱。

正式交付檔 [`result_3.csv`](midterm01/src/localization/submissions/final/result_3.csv) 已同步為該 0.22 檔，格式與 SHA-256 完全一致；重現參數與其他兩軌正式候選見 [final README](midterm01/src/localization/submissions/final/README.md)。

本輪已測試前向門檻隔離、因果 SE(2) 同步、近距同步、後驗平均、停車額度與起點位置／航向耦合。近距同步 0.27、起點耦合 0.24，不取代最佳檔；固定批次仍在完成其餘預先宣告的模型對照，所有實際分數逐組記錄。T1／T2 已完成八組本地 GNSS 留出對照，尚未消耗兩軌提交額度。

分析依據、逐檔改動、驗證方法、實際提交 ID、失敗實驗、後續 T3 改進與 **T1／T2 合法提升方向** 均記錄在 [定位 README 第 6 節](midterm01/src/localization/README.md#6-t3-同步實驗2026-10-26)。所有更新提交到目前分支 `copilot/research-kaggle-competition`。

本次自主提交截止：**2026-10-27 08:00（Asia/Taipei）**。提交工具預留 120 秒緩衝，07:58 起拒絕啟動新上傳，08:00 後不提交；不使用未來幀、離線軌跡修正或從排行榜推導逐幀真值。

```text
Dockerfile, docker-compose.yml, ros_entrypoint.sh   ROS 2 Humble 開發容器（GPU）
midterm01/src/localization/                         colcon package（助教範本 + 我們的 Python 流程）
  tools/                                            定位流程
  calib/                                            相機校正、車身遮罩
  docs/PLAN.md                                      實驗紀錄
  legacy/                                           規則變更前的程式與提交（不屬於最終流程）
data/Track{1,2,3}/                                  資料集（容器內掛在 /root/data）
```
