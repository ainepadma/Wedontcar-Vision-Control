# 0.3.3-beta 验证记录

日期：2026-10-21。源码分支：`10-21-medium`，发布标题：`passmedium`。

## 主机测试

gcc 16.1.0。以独立 PowerShell 进程逐项执行仓库内 13 个 `tools/test_*.ps1`，均传 `-Compiler gcc`。10 个入口通过，3 个失败；失败明细见 [CHANGELOG](../CHANGELOG.md)。这是入口脚本计数，`test_vision_options` 内含两个测试程序。

默认配置 `RING_FEATURE_ENABLED=0`。单独验证 `test_track_vision` 开启配置的复现命令如下（先执行任一主机脚本以建立输出目录）：

```powershell
gcc -std=c99 -Wall -Wextra -Werror -Wshadow -pedantic -O2 `
  -DRING_FEATURE_ENABLED=1 -I demo/user/inc `
  tools/test_track_vision.c demo/user/src/track_vision.c -lm `
  -o demo/mdk/Objects/host-tests/test_track_vision_enabled.exe
.\demo\mdk\Objects\host-tests\test_track_vision_enabled.exe
```

结果：188×120、376×240、752×480 全部通过。该检查不改变发布固件开关，也不证明专用环岛运动控制通过。

## Keil 完整构建

Keil MDK，Arm Compiler 6.19，`NXP.MIMXRT1064_DFP.25.09.00`；目标 `nor_sdram_zf_dtcm`；对更新诊断版本后的最终源码执行 `UV4.exe -r` 完整重编译。

```text
Keil exit code: 0
Program Size: Code=59208 RO-data=7088 RW-data=240 ZI-data=437692
".\Objects\rt1064.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed: 00:00:12
```

生成文件 `demo/mdk/Objects/rt1064.axf` SHA-256：

```text
826EA1CC75F55A7052061450AE860B5EBC41008D68B327D85EA6CB08B449D338
```

构建产物与原始日志没有加入源码提交，以上保留本轮结果和哈希供追溯。其他机器的构建时间、路径和二进制哈希可能不同。

## 同步完整性

按原 `.gitignore` 排除生成文件后，逐个对照源目录的 587 个工程、库和测试文件（比较时统一换行）：内容一致，唯一有意差异是 `main.c` 中固件诊断版本号。源目录算法未因本次整理而修改。新增文档均检查 UTF-8 和本地链接；参考 main 的 v0.3.1 记录及远端完整 CHANGELOG 历史均保留。

`git diff --check` 提示源目录原有测试夹具中的尾随空白、末尾空行，以及历史 Markdown 的双空格换行。本次保留参考夹具原文，未作为算法或编译错误处理。

## 未验证项目

未烧录、未启动电机、未进行新一轮实车测试；未验证 IAR；未运行全部额外的专用环岛 C 测试。实车清单见 [待测试](../待测试.md)。
