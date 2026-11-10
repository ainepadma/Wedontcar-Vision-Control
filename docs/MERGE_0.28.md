> 文档定位更新于 2026-11-10。本文保留原版本参数、测试与交付记录；当前 0.33.0 / `1110-deng` 见 [README](../README.md)、[环岛诊断](RING_DIAGNOSTICS.md) 和 [当前验证](RELEASE_CHECKS_0.33.md)。本文中的“当前 / 本轮 / 优先”均限定为原记录时点。

# 0.28.0 合并来源、提交与交付

## 一、用户确认的边界

以2026-11-07 C盘工程当前磁盘代码为主要来源，保留用户新增直道90、弯道70、第三状态左倾0.06及Keil现有设置；从E盘0.27.0只接入七条纹刹车。刹车触发前算法与C一致，触发后的制动/锁存/舵机保持与E一致。两份输入工程不覆盖，输出在E盘独立的 `Wedontcar-Vision-Control-0.27.0-merged` 目录。

## 二、两次提交

| 顺序 | 用户指定标题 | 版本 | 内容 |
| --- | --- | --- | --- |
| 1 | 最终补线 | 0.27.0 | E当前代码快照，速度60/45，原第三次补线，**已经含E刹车**；补齐现行文档和本次真实验证 |
| 2 | 最终刹车和补线 | 0.28.0 | C当前补线/运动算法＋E完整刹车，速度90/70、左倾0.06；统一版本和文档 |

所有实现和验证结束后才按此顺序commit，然后快进推送 `1106-deng`，不修改main、不强推、不创建额外算法提交。版本和文档用提交标题标识自身，SHA以 `git log --format="%h %s" origin/1106-deng` 为准。

## 三、具体选择

- 原样保留C：ring_corner.h/.c、ring_control.h/.c、track_vision、track_near_route.inc、ring_360_zore.inc、img_process、pid、IMU及库；正常转向、差速、斜坡、C15和相机配置。
- 原样取E：track_stripe_brake.h/.c、ISR刹车接入、main.h接口、Keil新增源条目、两份刹车测试。track_control.c/停车诊断仅扩展停车原因上限到9。
- track_control.h从C只追加TRACK_STOP_STRIPE；main.c按块接入刹车显示和独立检测/发布，保留C补线叠加；三处版本输出更新0.28.0。
- 远端旧track_stripe.h/.c、STRIPE_STOP和旧固定60°/定时直行/条纹测试均不在两份当前输入中且未接入，第一快照提交清理这些遗留文件，避免把旧可解除条纹方案当成当前E主动锁停。
- 按协作规范不再跟踪远端历史firmware二进制/构建日志，加入ignore；最新固件和日志仍在本地firmware目录及完整工程包中，默认Download的Objects/rt1064.axf同步相同哈希。未新增跟踪个人IDE状态或Objects/主机exe。

## 四、本次验证

完整结果见[验收记录](RELEASE_CHECKS_0.28.md)。来源530文件＋9文件一致、350组路径比较一致、1320帧第三状态专项及E刹车单测/ISR模拟通过。全22入口为12通过/10个已复现C基线失败，未达到全仓历史门禁，不声称全部通过。
E快照Keil：0 Error(s), 0 Warning(s)；Code=87856 RO-data=9460 RW-data=236 ZI-data=1453008；AXF SHA256 `aa1c6c4a928385383e23566bbcfa298191c4551d0bf441939e2f0f54cdfb5a21`；日志 `demo/mdk/Objects/local-build-20261107-135821-565236/rebuild.log`。
最终Keil：0 Error(s), 0 Warning(s)；Code=90320 RO-data=9200 RW-data=240 ZI-data=1453076；AXF SHA256 `26b8ae12cbb9ea50531098ce941771a5b35b013ac65dc80930f32e8cf37534f3`；日志 `demo/mdk/Objects/local-build-20261107-135859-377183/rebuild.log`。

## 五、尚未完成

未连接车辆、未烧录、未启动电机、未做实车整圈或长期保持温升验证。环内乱补线/中线消失与右环岛独立流程共参建议已写[待测试](../待测试.md)，本轮未擅自实现。IAR未接入新刹车源且未验证，仅交付Keil构建结果。
