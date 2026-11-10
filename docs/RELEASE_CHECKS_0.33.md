# 0.33.0 发布验证记录（2026-11-10）

发布分支 `1110-deng`，提交标题 `平滑360`，上传前基线 `ca9d664`。本次只包含主机与构建证据，尚未烧录或实车整圈验收。

## 全量入口与基线对比

当前 `python tools/run_host_checks.py --full` 执行 **23 项，13 通过、10 失败**。另从 `ca9d664` 的 Git 归档提取干净源码，用基线自带脚本执行 22 项，结果为 **12 通过、10 失败**。两侧失败入口集合完全一致，新增的普通白路续接入口通过。

| 入口 | 0.33.0 | 基线 ca9d664 | 结果说明 |
| --- | --- | --- | --- |
| `test_ground_preprocess.ps1` | 失败 | 失败 | img_process.c:450 的 shadow 与 track_vision.c:59/62/63 的 signedness 告警被 -Werror 拒绝 |
| `test_img_process.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_imu660rb.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_imu_navigation.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_line_following.ps1` | 失败 | 失败 | track_vision.c:59/62/63 signedness 告警被 -Werror 拒绝 |
| `test_motor_switch.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_pdf_ring.ps1` | 失败 | 失败 | test_pdf_ring.c:24 仍期待旧 S2 / direction 阶段断言 |
| `test_pid.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_ring_control.ps1` | 失败 | 失败 | 旧 ZORE 字段 zore360_match 和旧 RING_360_CONFIRM_HITS 等宏已不属于当前接口 |
| `test_ring_revision2.ps1` | 失败 | 失败 | test_ring_revision2.c:25 的旧阶段 expected 断言 |
| `test_ring_revision3.ps1` | 失败 | 失败 | test_ring_revision3.c:30 的旧 S3 / selected_path.valid 断言 |
| `test_ring_revision4.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_ring_road_continuation.ps1` | 通过 | 新增入口 | 本次实际运行通过 |
| `test_ring_routes.ps1` | 失败 | 失败 | remembered_line、RING_NEAR_CONFIRM_FRAMES、RING_LINE_MAX_TILT 等旧接口不匹配 |
| `test_ring_third_repair.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_ring_visual_exit.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_track_control.ps1` | 通过 | 通过 | 本次实际运行通过 |
| `test_track_preprocess.ps1` | 失败 | 失败 | track_vision.c:59/62/63 signedness 告警被 -Werror 拒绝 |
| `test_track_vision.ps1` | 失败 | 失败 | track_vision.c:59/62/63 signedness 告警被 -Werror 拒绝 |
| `test_vision_options.ps1` | 失败 | 失败 | track_vision.c signedness 告警及严格告警策略阻止构建 |
| `test_stripe_brake.py` | 通过 | 通过 | 本次实际运行通过 |
| `test_stop_diagnostics.py` | 通过 | 通过 | 本次实际运行通过 |
| `test_steering_export.py` | 通过 | 通过 | 本次实际运行通过 |

原始结果保存在本地 `demo/mdk/Objects/release-checks/results.json`，各入口日志位于同目录。基线归档测试结果保存在发布工作区的 `baseline-ca9d664-docs-audit/demo/mdk/Objects/release-checks`。生成物按 Git 忽略规则保留本地，本表保留可审查的入口、结果和失败原因。

## 普通白路续接专项

- 实际调用普通视觉、环岛视觉和控制选择入口，回放 `outer_v.bin` 的 120 帧 ±1 像素平移。
- 检查原蓝线逐值一致，所有原有中线采样点保留；新增远端来自当前普通行中线，并沿同一白路连接。
- 确认新增的连接位实际存在，保留左支，拒绝右侧白条；黑色隔离带阻断续接。
- 普通检测只丢失两行、像素仍白时，检查补齐后的采样点与连接位连续，普通输出本身保留原漏行标志。
- 用 `latest_p3.bin` 验证宽白区上方的错误直行支不会替代左转参考。
- 连续帧保持 S4/G15，控制实际 E/H 等于续接后的 guide，回放未触发保持或路径回退。

典型原图 `outer_v.bin` 的左 S4 从 4 个采样点增至 11 个。有效支持上限从 y=114 延伸到约 y=42，E/H 随完整路线重算。夹具来源和本机重建帧均属于离线图像，不能据此判定实车卡顿已消除。

## 其他相关检查

第三线专项覆盖近角优先、600 帧稳定与 720 帧平移、左右角点固定、原近端与蓝线、白路内移、同角恢复及输入不变性。revision4 完整回放覆盖普通道路、入口交接、第三线、分隔带、第四线和 270°。条纹检测 / 状态机与实际 ISR 模拟、停车诊断、日志导出均在本轮全量入口中通过。

## Keil 完整构建

- 目标 `nor_sdram_zf_dtcm`，Arm Compiler 6.19，`NXP.MIMXRT1064_DFP.25.09.00`。
- 使用 `python tools/build_keil_local.py --uv4 C:/Keil_v5/UV4/UV4.exe`，原工程通过隔离输出目录构建。
- 结果 **0 Error(s), 0 Warning(s)**。
- 程序大小 Code=94392，RO-data=9760，RW-data=244，ZI-data=1454636 字节。
- 日志 `demo/mdk/Objects/local-build-20261110-195555-339126/rebuild.log`。
- AXF `demo/mdk/Objects/local-build-20261110-195555-339126/rt1064.axf`。
- AXF SHA-256 `8fcf34651cc89a44eba1d109921a6ab192994cf643072a74729420f78379017b`。
- main.c 的 `#CAM`、`#SPD`、`#LOG` 均为 `FW=0.33.0`，version.yml 为 `0.33.0 / 1110-deng / 平滑360`。

## 工程与文档完整性

按工程 Git 忽略规则核对当前源快照，共 704 个工程文件，包含 510 个库 / SDK 文件和 18 份 Markdown。发布检出与这些源文件逐字节同步；Git 对文本换行按仓库设置规范化。原 CHANGELOG 的 69 个历史版本标题全部保留，加入 0.33.0 后共 70 个版本条目。Markdown 的本地链接、代码围栏和三个固件版本字符串已核对。

生成的 Objects、Listings、测试可执行文件、本地 firmware 和个人工作区缓存按现有规则保留本地。仓库已有的项目与设备配置随工程同步。

## 复现与待验收

```powershell
python tools/run_host_checks.py --full
powershell -NoProfile -ExecutionPolicy Bypass -File tools/test_ring_road_continuation.ps1 -Compiler gcc
python tools/build_keil_local.py --uv4 C:/Keil_v5/UV4/UV4.exe
```

当前历史测试失败仍须处理，PR 以本表说明检查状态。实车平滑性、右环对应实现、连续左右环、条纹停车距离 / 保持温升、长时间日志与 IAR 均见 [待测试](../待测试.md)。本次未执行合并或烧录。
