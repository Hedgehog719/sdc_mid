# SDC 2026 Midterm01：影像定位（Track01／02／03）

最終流程與完整執行說明在 [`midterm01/src/localization/README.md`](midterm01/src/localization/README.md)。

- 方法：線上（因果）定位。每一幀只用到當下以前的資料，依時間順序跑一個 Kalman filter（10/22 規則）。
- 執行：在容器內跑 `bash /root/midterm/src/localization/tools/run_online.sh /root/data/Track1 T1`（T2、T3 同理）。
- 10/22 以前的做法（RTS 雙向平滑、Leaderboard 分析工具、舊提交檔）只保留作紀錄，放在 `midterm01/src/localization/legacy/`，規則變更時間線見該資料夾的 README。

資料不在版本庫內（見 `.gitignore`）。

```text
Dockerfile, docker-compose.yml, ros_entrypoint.sh   ROS 2 Humble 開發容器（GPU）
midterm01/src/localization/                         colcon package（助教範本 + 我們的 Python 流程）
  tools/                                            定位流程
  calib/                                            相機校正、車身遮罩
  docs/PLAN.md                                      實驗紀錄
  legacy/                                           規則變更前的程式與提交（不屬於最終流程）
data/Track{1,2,3}/                                  資料集（容器內掛在 /root/data）
```
