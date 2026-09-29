# Third-party notices / 第三方说明

| Component / 组件 | Source / 来源 | Terms and modifications / 条款与修改 |
|---|---|---|
| dex-retargeting | [dexsuite/dex-retargeting](https://github.com/dexsuite/dex-retargeting) | MIT; license retained at `third_party/dex-retargeting/LICENSE`. O10 configuration and polynomial coupling changes are included. / 保留 MIT 许可证，包含 O10 配置和多项式耦合修改。 |
| HTS | [wengmister/hand-tracking-streamer](https://github.com/wengmister/hand-tracking-streamer) | Upstream Apache-2.0. APK pinned to snapshot `5ff7c1cfea0ead1bb8a0e233bc7770d94d31feb5`; the network-gate source patch is not built into the APK. / 上游 Apache-2.0；APK 不包含未构建的网络检查补丁。 |
| OmniHand SDK | [AgibotTech/agillink_omnihand_sdk](https://github.com/AgibotTech/agillink_omnihand_sdk) | Upstream declares Mulan PSL v2. SDK 1.1.8 is installed as a separate dependency. / 上游声明 Mulan PSL v2，SDK 1.1.8 作为单独依赖安装。 |
| O10 robot model | Supplied manufacturer model package / 厂家模型包 | Preserve the supplied package and its terms; no model redistribution license is inferred from the adapter code. / 保留模型原包和条款，不由适配代码推定模型再分发许可。 |

The vendored dex-retargeting adds O10 left/right vector and DexPilot configurations, a polynomial joint-coupling adapter, and its integration into retargeting configuration and sequential retargeting. / 内置 dex-retargeting 增加 O10 左右手 vector/DexPilot 配置、多项式关节耦合适配器，以及对应的配置与顺序重定向接入。

For retargeting citations, see the upstream AnyTeleop/DexPilot references in the [dex-retargeting README](https://github.com/dexsuite/dex-retargeting). / 重定向方法引用见上游 README 中的 AnyTeleop/DexPilot 文献。

Documentation structure was informed by [Open-TeleVision](https://github.com/OpenTeleVision/TeleVision), [dex-retargeting](https://github.com/dexsuite/dex-retargeting), and [Unitree XR Teleoperate](https://github.com/unitreerobotics/xr_teleoperate). Their installation commands were not copied into this project's setup. / 文档结构参考上述项目，安装命令按本项目实际依赖编写。

The adapter is distributed under the root MIT license. The SDK, APK, and manufacturer model package are not redistributed in this repository; obtain them separately under their upstream terms. / 适配代码采用根目录 MIT 许可证；本仓库不再分发 SDK、APK 和厂家模型包，请按上游条款单独获取。
