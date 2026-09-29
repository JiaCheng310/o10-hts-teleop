# Asset setup / 依赖文件准备

[English guide](../README.md) · [中文教程](../README_zh-CN.md)

Run from the project root on Ubuntu. Download/copy the separately supplied files into the paths below. / 在 Ubuntu 项目根目录执行，将单独提供的依赖文件下载或复制到以下位置。

## 1. HTS APK

This pins the upstream snapshot used by this integration, rather than an unverified newer protocol. / 固定为本项目使用的上游版本，不自动替换成未经验证的新协议版本。

```bash
mkdir -p dependencies/hts
curl -fL --retry 3 \
  https://raw.githubusercontent.com/wengmister/hand-tracking-streamer/5ff7c1cfea0ead1bb8a0e233bc7770d94d31feb5/hand_tracking_streamer.apk \
  -o dependencies/hts/hand_tracking_streamer.apk
echo 'e0b41ab36e0bcbbc1a1d616bd659bad6d93c02efcb332c1d87c2bee1c6e04879  dependencies/hts/hand_tracking_streamer.apk' | sha256sum -c -
```

Expect `OK`. Do not install a file that fails verification. Install/USB authorization commands are in the main README. / 应显示 `OK`；校验失败时不要安装。USB 授权与安装命令见主教程。

## 2. OmniHand SDK wheel

Obtain **1.1.8, CPython 3.12, Linux x86_64** from the [official SDK repository](https://github.com/AgibotTech/agillink_omnihand_sdk) or your supplied SDK package. Place the wheel in `~/Downloads`, then run: / 从[官方 SDK 仓库](https://github.com/AgibotTech/agillink_omnihand_sdk)或已提供的 SDK 包中取得 **1.1.8、CPython 3.12、Linux x86_64** wheel，放到 `~/Downloads` 后执行：

```bash
mkdir -p dependencies/omnihand
cp "$HOME/Downloads/omnihand-1.1.8-cp312-cp312-linux_x86_64.whl" dependencies/omnihand/
echo 'cae7a0d5bce7e7c9d72cc90a0dd15e152f11e2a1ecf04cb6761e8171399ff170  dependencies/omnihand/omnihand-1.1.8-cp312-cp312-linux_x86_64.whl' | sha256sum -c -
```

Expect `OK`. The current upstream default branch may offer a newer SDK. This guide does not treat a newer API version, ARM wheel, or Windows installer as a drop-in replacement. / 应显示 `OK`。上游默认分支可能提供新 SDK；本教程没有验证新 API 版本、ARM wheel 或 Windows 安装包可以直接替换。

## 3. O10 model package

Use the supplied `omnihand_description-omnihandT2_1` package containing both URDFs, MJCFs, and meshes. General manufacturer downloads are listed on the [OmniHand documentation page](https://www.agibot.com.cn/DOCS/OS/Omnihand-O10). A newer URDF alone is insufficient for this coupling implementation. / 使用已提供的 `omnihand_description-omnihandT2_1` 完整模型包，包含左右手 URDF、MJCF 与 meshes。厂家资料见 [OmniHand 文档页](https://www.agibot.com.cn/DOCS/OS/Omnihand-O10)。仅下载新版 URDF 不足以复现当前耦合实现。

After extracting this directory into `~/Downloads`: / 将该目录解压到 `~/Downloads` 后：

```bash
mkdir -p vendor
cp -a "$HOME/Downloads/omnihand_description-omnihandT2_1" vendor/
ls vendor/omnihand_description-omnihandT2_1/assets/urdf/omnihand_{left,right}.urdf
ls vendor/omnihand_description-omnihandT2_1/assets/MJCF/omnihand_{left,right}.xml
ls vendor/omnihand_description-omnihandT2_1/assets/meshes
```

Then run `python scripts/check_install.py` in the activated environment. / 最后在已激活环境中运行 `python scripts/check_install.py`。

The SDK, APK, and model files retain their upstream terms. They are kept separate from the adapter code. / SDK、APK 和模型保留各自上游条款，与适配代码单独管理。
