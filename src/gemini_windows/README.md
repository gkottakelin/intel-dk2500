# Gemini Windows 第二阶段代码

本目录用于 Windows 上位机阶段读取 Gemini Pro Plus 的 RGB-D 数据。此阶段不依赖 ROS2，目标是先验证相机 SDK、彩色帧、深度帧和内参读取。

当前本地 Windows 资料显示，Gemini Pro Plus 更适合优先走 `OpenNI v2.3.0.85 + SensorDriver` 链路；`pyorbbecsdk2` 能导入但枚举不到当前设备时，不作为首选读取方案。

## 本地 OpenNI 示例

已找到读取相机的 OpenNI 示例源码和 exe：

```text
gemini深度相机windows资料/Windows/Windows/OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release/
```

便捷运行：

```powershell
python project/src/gemini_windows/run_openni_sample.py depth
python project/src/gemini_windows/run_openni_sample.py color-uvc
python project/src/gemini_windows/run_openni_sample.py viewer
python project/src/gemini_windows/run_openni_sample.py pointcloud
```

注意：`ColorReaderPoll.exe` 使用 OpenNI 的 `SENSOR_COLOR`，当前 Gemini Pro Plus 在 Windows 下通常把彩色头暴露为 UVC 设备，所以它可能启动失败并打印 `Couldn't start the depth stream`。这是示例源码里的提示文字复用错误，本质是 OpenNI 彩色流启动失败。当前首选：

```powershell
python project/src/gemini_windows/run_openni_sample.py color-uvc
```

也可以用 OpenCV 直接读取 UVC 彩色头：

```powershell
python project/src/gemini_windows/opencv_uvc_color_test.py
```

如果自动选择的不是 Gemini 彩色画面，指定索引：

```powershell
python project/src/gemini_windows/opencv_uvc_color_test.py --index 1
```

对应源码和说明见：

```text
project/docs/GeminiProPlus_Windows_OpenNI资料定位.md
```

## 环境准备

官方 Python 包名是 `pyorbbecsdk2`，代码中导入的模块名仍是 `pyorbbecsdk`。如果你正在使用 Python `3.14` 且安装失败，建议安装 Python `3.11`、`3.12` 或 `3.13` 后创建虚拟环境。

```powershell
cd D:\jetarm
py -3.12 -m venv .venv-gemini
.\.venv-gemini\Scripts\activate
python -m pip install --upgrade pip
pip install pyorbbecsdk2 opencv-python numpy
```

如果安装时报 `ProxyError('Cannot connect to proxy')`，先清理当前 PowerShell 会话和 pip 配置里的代理：

```powershell
Remove-Item Env:HTTP_PROXY -ErrorAction SilentlyContinue
Remove-Item Env:HTTPS_PROXY -ErrorAction SilentlyContinue
Remove-Item Env:ALL_PROXY -ErrorAction SilentlyContinue
python -m pip config unset global.proxy
python -m pip config unset user.proxy
python -m pip config unset site.proxy
```

然后强制从官方 PyPI 安装：

```powershell
python -m pip install --upgrade pip
python -m pip install -i https://pypi.org/simple --trusted-host pypi.org --trusted-host files.pythonhosted.org pyorbbecsdk2 opencv-python numpy
```

如果仍提示 `No matching distribution found for pyorbbecsdk2`，先检查 Python 版本：

```powershell
python --version
python -m pip --version
```

若显示 Python `3.14`，请换成 Python `3.11`、`3.12` 或 `3.13` 的虚拟环境再安装。官方文档要求 Python `3.8+`，但实际 SDK wheel 通常按具体 Python 小版本发布，过新的 Python 版本容易没有匹配包。

## 设备枚举

Gemini Pro Plus 是 USB 深度相机，不是串口设备，不能通过 COM 口直接打开。COM 口只用于总线舵机控制板、USB 转串口模块等串口设备。

如果 SDK 枚举不到设备，先检查 Windows 是否能看到 USB/PnP 设备：

```powershell
python project/src/gemini_windows/windows_usb_check.py
```

它不依赖 `pyorbbecsdk`，只查询 Windows 设备管理器层面的设备列表。若这里也看不到相机，问题在 USB 线、接口、供电、驱动或相机连接；若这里能看到但 SDK 看不到，再排查 SDK 版本、Viewer 占用和 Orbbec 驱动。

如果运行脚本没有任何输出，先跑环境诊断：

```powershell
python project/src/gemini_windows/diagnose_environment.py
```

它会逐步打印 Python 路径、依赖是否能找到、`numpy/cv2/pyorbbecsdk` 是否能实际导入。若某一步之后程序直接退出，通常是该原生库或 DLL 加载失败。

如果诊断显示 `numpy` 和 `cv2` 正常，但 `pyorbbecsdk` 导入失败或直接终止进程，按下面顺序排查：

```powershell
python -m pip show pyorbbecsdk2
python -m pip list
```

1. 确认使用的是 64 位 Python：输出中应包含 `AMD64` 或 `64 bit`。
2. 安装或修复 Microsoft Visual C++ Redistributable 2015-2022 x64。
3. 确认 Orbbec Viewer 已能正常打开 Gemini Pro Plus。
4. 关闭 Orbbec Viewer 后再运行 Python 脚本，避免设备被占用。
5. 重装 SDK wheel：

```powershell
python -m pip uninstall -y pyorbbecsdk2
python -m pip install -i https://pypi.org/simple --no-cache-dir --force-reinstall pyorbbecsdk2
```

若仍失败，优先下载并安装 Orbbec 官方 Windows SDK/Viewer 对应版本，再重新安装 Python wheel。

```powershell
python project/src/gemini_windows/list_devices.py
```

作用：

- 检查 Windows 是否能通过 SDK 发现 Gemini Pro Plus。
- 打印设备名、序列号、PID/VID、固件版本等信息。

## 读取相机内参

```powershell
python project/src/gemini_windows/read_intrinsics.py
```

作用：

- 启动彩色和深度流。
- 读取 color/depth 内参。
- 打印 `fx, fy, cx, cy, width, height`。

## RGB-D 实时显示和点击测深

```powershell
python project/src/gemini_windows/camera_stream_test.py
```

操作：

- 左键点击彩色图：读取点击位置附近深度中位数。
- `s`：保存当前彩色图、深度预览图、原始深度数组。
- `q` 或 `ESC`：退出。

保存样本默认目录：

```text
project/data/rgbd_samples/
```

## 输出数据意义

程序会从相机读取：

- `color_image`：OpenCV BGR 彩色图。
- `depth_mm`：单位为毫米的深度图。
- `intrinsics`：相机内参。

后续阶段会基于这些数据开发：

```text
颜色/目标检测
  -> 目标中心像素
  -> 深度测距
  -> 相机三维坐标
  -> 机械臂基座坐标
```

## 常见问题

- 如果提示找不到 `pyorbbecsdk`：确认已安装 `pyorbbecsdk2`，并使用 Python `3.8..3.13`。
- 如果提示未发现设备：关闭 Orbbec Viewer，检查 USB3.0 线和接口。
- 如果电脑只有 USB2.0：先用 Viewer/设备管理器确认能否枚举；低带宽模式可能可以观察，但高帧率 RGB-D、点云和多流同步不可靠。
- 如果 DK2500 与当前电脑 USB 条件不同：不要复用当前电脑的帧率/分辨率结论，迁移后重新运行 `list_devices.py` 和 RGB-D 出流测试。
- 如果彩色格式不支持：尝试添加 `--prefer-default-color`。
- 如果点击深度无效：目标可能处于黑洞区域，或彩色图和深度图未对齐，后续阶段会专门处理对齐。
