# 0.28.0 本次主机验收记录（2026-11-07）

## 结果边界

完整入口执行19个PS1和3个Python检查，共22项：12通过、10失败。10项失败已在未合并的C当前快照复现，不为迎合旧断言/旧接口修改本轮算法，不能宣称全仓门禁全部通过。PS1统一显式指定 `-Compiler gcc`；断言使用临时CPATH安全头文件，失败输出终端并正常退出，避免Windows CRT弹窗。未烧录、无实车结论。

| 检查 | 本次结果 | 说明 |
| --- | --- | --- |
| test_ground_preprocess.ps1 | 失败；C基线同样失败 | img_process.c:450变量shadow及track_vision.c:59/62/63 signedness告警被-Werror拒绝 |
| test_img_process.ps1 | 通过 | 本次运行通过 |
| test_imu660rb.ps1 | 通过 | 本次运行通过 |
| test_imu_navigation.ps1 | 通过 | 本次运行通过 |
| test_line_following.ps1 | 失败；C基线同样失败 | 显式-Compiler gcc后仍有track_vision.c:59/62/63 signedness告警；旧默认编译器路径另有缺失 |
| test_motor_switch.ps1 | 通过 | 本次运行通过 |
| test_pdf_ring.ps1 | 失败；C基线同样失败 | test_pdf_ring.c:30旧corner_stage==6断言 |
| test_pid.ps1 | 通过 | 本次运行通过 |
| test_ring_control.ps1 | 失败；C基线同样失败 | 旧ZORE字段/宏已删除，与现行视觉环岛接口不匹配 |
| test_ring_revision2.ps1 | 失败；C基线同样失败 | test_ring_revision2.c:39旧corner_stage==6断言 |
| test_ring_revision3.ps1 | 失败；C基线同样失败 | test_ring_revision3.c:30旧corner_stage==3及selected_path.valid断言 |
| test_ring_revision4.ps1 | 通过 | 本次运行通过 |
| test_ring_routes.ps1 | 失败；C基线同样失败 | 旧remembered_line/revisit_armed/RING_LINE_MAX_TILT等接口与当前实现不匹配 |
| test_ring_third_repair.ps1 | 通过 | 本次运行通过 |
| test_ring_visual_exit.ps1 | 通过 | 本次运行通过 |
| test_track_control.ps1 | 通过 | 本次运行通过 |
| test_track_preprocess.ps1 | 失败；C基线同样失败 | 显式-Compiler gcc后仍有track_vision.c:59/62/63 signedness告警；旧默认路径另有缺失 |
| test_track_vision.ps1 | 失败；C基线同样失败 | track_vision.c:59/62/63 signedness告警被-Werror拒绝 |
| test_vision_options.ps1 | 失败；C基线同样失败 | signedness及img_process.c:450 shadow告警被-Werror拒绝 |
| test_stripe_brake.py | 通过 | 本次运行通过 |
| test_stop_diagnostics.py | 通过 | 本次运行通过 |
| test_steering_export.py | 通过 | 本次运行通过 |

## 专项证据

- 当前第三状态专项：600静态＋720微移帧、左右镜像、角点固定、左倾旋转、平行中线、误点拒绝、当前像素恢复、舍弃侧边界扰动和图像不变性通过。
- E刹车检测/状态/保持单测和抽取当前实际ISR分支模拟通过：3帧准备、单帧到位、大小裁切记忆、失帧保持、C13禁止解除、待机触发LAST、正反PWM和舵机保持、复位清除。模块字节及参数与E一致，不把物理零位移/实际停车距离视作已验收。
- `verify_merge_sources.py`通过：530个C视觉/补线/库文件原字节一致；9个E刹车与接入文件一致；正常控制代码只扩展合法停车原因到9，保留90/70和0.06。C整个补线屏幕overlay保留；main.c仅接E刹车显示/检测/发布和三处FW版本更新。
- 25组revision4夹具×左右镜像×全部7阶段，共350组：每一补线候选、普通逐行边界/中线、控制所选路径和阶段与C一致。这是离线比较，不是整圈实车闭环。
- 第一个E快照：revision4、visual_exit、track_control、motor_switch、stripe_brake、stop_diagnostics、steering_export共7入口全部通过；不能据此宣称E历史所有测试通过。

## 复现

`python tools/run_host_checks.py` 运行9个当前核心入口，失败不会弹窗；加 `--full` 执行上述22个检查并返回非零如实报告旧失败。日志在本地 `demo/mdk/Objects/release-checks/`，不入Git。
`python tools/verify_merge_sources.py --c-base <C当前基线> --e-base <E刹车基线>` 核对来源；基线需保存当时的磁盘快照。
源码使用Keil正式目标 `nor_sdram_zf_dtcm`，临时指定本机ARMCLANG6.21和DFP25.09.00构建，没有修改源工程编译器选择或烧录设备。
