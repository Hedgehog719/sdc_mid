# SDC 2026 Midterm01：影像定位（Track01／02／03）

最終流程與完整執行說明在 [`midterm01/src/localization/README.md`](midterm01/src/localization/README.md)。

- 方法：線上（因果）定位。每一幀只用到當下以前的資料，依時間順序跑一個 Kalman filter（10/22 規則）。
- 執行：在容器內跑 `bash /root/midterm/src/localization/tools/run_online.sh /root/data/Track1 T1`（T2、T3 同理）。
- 10/22 以前的做法（RTS 雙向平滑、Leaderboard 分析工具、舊提交檔）只保留作紀錄，放在 `midterm01/src/localization/legacy/`，規則變更時間線見該資料夾的 README。

資料不在版本庫內（見 `.gitignore`）。

## T3 改善進度（2026-10-27）

本輪從 commit 歷史與既有 Kaggle 提交建立對照，在 Docker 內測試：前向門檻改為 opt-in、因果 SE(2) 相機同步、近距拍攝時刻投影、起點假設後驗平均，以及停車量測額度重置修正。截至 01:50 已確認的新提交分數為 0.23／0.23／0.27／0.23，尚未達 `<0.23`；0.27 的近距同步不採用，預設關閉。後續固定批次會獨立測試起點位置／航向耦合、弱峰值航向與正確的陀螺儀航向導數，逐組保存實際 Kaggle 回報。

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
