# Validation / 验证记录

Date / 日期：2026-09-29。

## Environment / 环境

- Ubuntu 24.04.4 LTS, x86_64, Python 3.12.3.
- New isolated `.venv` in a temporary copy; no `--system-site-packages` and no reuse of the deployed environment's installed Python packages. / 临时副本中创建独立 `.venv`，未启用系统包共享，也未复用部署环境已安装的 Python 包。
- Installed CPU PyTorch 2.8.0, `requirements.txt`, the included editable dex-retargeting, and OmniHand 1.1.8 wheel using the README sequence. Pip could reuse its download cache. / 按 README 顺序安装依赖；允许复用 pip 下载缓存。
- `pip check`: `No broken requirements found.`
- Existing Ubuntu system packages were used. This was a fresh Python environment, not a fresh OS installation. / 使用主机已有系统包；验证的是全新 Python 环境，不是重装操作系统。

## Checks / 检查

| Check / 项目 | Result / 结果 |
|---|---|
| Left and right O10 retargeter construction, coupling, finite targets, joint limits / 左右手重定向、耦合与限位 | Passed / 通过 |
| Offline HTS, handedness, driver, device identity, recording suite / 离线协议、左右手、驱动、设备编号、录屏测试 | 36 tests passed / 36 项通过 |
| English / Chinese command parity / 中英文命令一致性 | 18 identical Bash blocks / 18 个 Bash 代码块相同 |
| Documented Bash syntax / 文档 Bash 命令语法 | Passed `bash -n` / 通过 |
| Adapter Python syntax / 适配代码 Python 语法 | 26 files parsed / 26 个文件通过 |
| Original ARX source snapshot / ARX 原始源码快照 | All 570 file hashes still match / 570 个文件校验值仍一致 |

The device-identity tests reject an unconfigured hand and a mismatched port before hardware initialization. / 设备编号测试确认，未配置的手或不匹配的端口会在硬件初始化前被拒绝。

## Scope / 范围

The tests use synthetic HTS input, temporary HTTP services on ephemeral ports, and fake hand/recording backends. This documentation update did not start the physical hand, reconnect Quest, install the APK, modify D405 settings, or start/restart an arm controller. / 测试使用合成 HTS 输入、临时端口和模拟手部/录屏后端。本次没有启动实体手、重新连接 Quest、安装 APK、修改 D405 或启停机械臂控制器。

USB authorization, headset settings, physical follow behavior, and native video recording must be checked on the intended setup using the preview and hardware steps in the README. No new end-to-end physical latency result is claimed. / USB 授权、头显设置、实机跟随与原生录屏需要在目标设备上按 README 检查；没有宣称新的实机端到端延迟结果。

## Changes for portability / 可移植性修改

The O10 retargeting and HTS streaming algorithms were copied from the current server snapshot. Deployment changes resolve the project relative to its own directory, select USB identifiers through `config/devices.env`, and provide explicit check/preview/start/stop commands. Empty or mismatched identifiers fail before starting hand control. / O10 重定向和 HTS 流程来自服务器当前快照。部署修改包括相对项目路径、独立设备配置和明确的检查/预览/实机/停止入口；空编号或不匹配编号会阻止手部控制启动。

Original snapshots remain under `sources/`; machine-specific verification logs are retained locally under `.local/`. / 原始快照保留在 `sources/`，机器相关验证日志保留在本地 `.local/`。
