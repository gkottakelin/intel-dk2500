# Gemini Pro Plus Windows OpenNI 资料定位

> 结论：`gemini深度相机windows资料` 中真正适配当前 Gemini Pro Plus 的 Windows 链路是 `OpenNI v2.3.0.85 + SensorDriver`，不是之前尝试的 `pyorbbecsdk2`。Orbbec Viewer 能看到图像，是因为它自带了 `OrbbecSDK.dll`、`ob_usb.dll`、深度引擎 DLL 和驱动；Windows 教程包则提供 OpenNI2 示例程序和源码。

## 资料位置

| 类型 | 路径 | 说明 |
|---|---|---|
| Windows 使用说明 | `gemini深度相机windows资料/Windows/Windows/3.Windows下的配置和使用.pdf` | Windows 下安装、配置、测试流程 |
| OpenNI SDK | `gemini深度相机windows资料/Windows/Windows/OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release/` | OpenNI2 SDK、示例源码、示例 exe |
| 传感器驱动 | `gemini深度相机windows资料/Windows/Windows/SensorDriver_V4.3.0.20/SensorDriver_V4.3.0.20.exe` | 深度传感器驱动安装程序 |
| Viewer | `gemini深度相机windows资料/OrbbecViewer_v1.10.27_202509252154_win_x64_release/OrbbecViewer.exe` | 可视化彩色、深度、红外、点云 |
| Viewer 驱动 | `gemini深度相机windows资料/OrbbecViewer_v1.10.27_202509252154_win_x64_release/driver/SensorDriver_V4.3.0.22.exe` | Viewer 包内附带的较新驱动 |

## 已找到的读取相机示例

OpenNI 示例 exe 位于：

```text
gemini深度相机windows资料/Windows/Windows/OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release/samples/bin/
```

| exe | 作用 |
|---|---|
| `DepthReaderPoll.exe` | 轮询读取深度帧，打印中心像素深度 |
| `DepthReaderEvent.exe` | 事件回调读取深度帧 |
| `ColorReaderPoll.exe` | 轮询读取 OpenNI 彩色帧，打印中心像素 RGB；当前 Gemini Pro Plus 可能不支持此路径 |
| `ColorReaderEvent.exe` | 事件回调读取彩色帧 |
| `ColorReaderUVC.exe` | 通过 UVC 读取彩色 MJPEG/YUV 数据；当前 Gemini Pro Plus 的 Windows 彩色读取首选 |
| `InfraredReaderPoll.exe` | 轮询读取红外帧 |
| `InfraredReaderEvent.exe` | 事件回调读取红外帧 |
| `SimpleViewer.exe` | 图形化显示深度/彩色，并包含 D2C 对齐逻辑 |
| `GeneratePointCloud.exe` | 从深度帧生成点云 `.ply` |
| `MultiDepthViewer.exe` | 多深度视图示例 |
| `ExtendedAPI.exe` | 扩展属性/API 示例 |

## 对应源码文件

源码位于：

```text
gemini深度相机windows资料/Windows/Windows/OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release/samples/samples/
```

| 源码 | 关键逻辑 |
|---|---|
| `DepthReaderPoll/DepthReaderPoll.cpp` | `OpenNI::initialize()`、`device.open(ANY_DEVICE)`、`depth.create(...SENSOR_DEPTH)`、`depth.readFrame()` |
| `ColorReaderPoll/ColorReaderPoll.cpp` | `color.create(...SENSOR_COLOR)`、`color.readFrame()`、读取 `RGB888Pixel` |
| `ColorReaderUVC/ColorReaderUVC.cpp` | 使用 `UVC_Swapper` 打开彩色 UVC MJPEG 流 |
| `InfraredReaderPoll/InfraredReaderPoll.cpp` | 读取红外帧 |
| `GeneratePointCloud/GeneratePointCloud.cpp` | 读取深度、获取内参、按公式生成点云 |
| `SimpleViewer/main.cpp` | OpenNI + UVC + D2C 对齐 + OpenCV 显示 |

## 深度读取核心流程

来自 `DepthReaderPoll.cpp`：

```text
OpenNI::initialize()
  -> Device device
  -> device.open(ANY_DEVICE)
  -> VideoStream depth
  -> depth.create(device, SENSOR_DEPTH)
  -> depth.start()
  -> OpenNI::waitForAnyStream(...)
  -> depth.readFrame(&frame)
  -> DepthPixel* pDepth = (DepthPixel*)frame.getData()
```

深度格式判断：

```text
PIXEL_FORMAT_DEPTH_1_MM
PIXEL_FORMAT_DEPTH_100_UM
```

中心点深度读取：

```text
middleIndex = (height + 1) * width / 2
pDepth[middleIndex]
```

## 彩色读取核心流程

来自 `ColorReaderPoll.cpp`：

```text
OpenNI::initialize()
  -> device.open(ANY_DEVICE)
  -> VideoStream color
  -> color.create(device, SENSOR_COLOR)
  -> color.start()
  -> color.readFrame(&frame)
  -> RGB888Pixel* pColor = (RGB888Pixel*)frame.getData()
```

彩色格式判断：

```text
PIXEL_FORMAT_RGB888
```

如果 OpenNI 彩色流不可用，可参考 `ColorReaderUVC.cpp`，它通过 `UVC_Swapper` 直接打开彩色 MJPEG 流。

当前测试中，`ColorReaderPoll.exe` 启动失败并打印：

```text
Couldn't start the depth stream
```

这个提示来自示例源码中复用的错误文字，实际失败的是 OpenNI color stream。由于 Windows PnP 中能看到 `USB Camera VID_2BC5&PID_0511`，说明彩色头走 UVC 更合理。后续 Windows 阶段采用：

```text
深度：OpenNI DepthReaderPoll / SENSOR_DEPTH
彩色：UVC ColorReaderUVC / OpenCV VideoCapture
```

## 点云生成核心公式

来自 `GeneratePointCloud.cpp`：

```text
world_x = depth * (u - cx) / fx
world_y = depth * (v - cy) / fy
world_z = depth
```

源码中会通过扩展属性读取内参：

```text
Device.getProperty(OBEXTENSION_ID_CAM_PARAMS, ...)
```

这与项目后续“像素坐标 + 深度 -> 相机坐标”的算法一致。

## 当前开发判断

当前设备在 Windows PnP 中能看到：

```text
VID_2BC5&PID_0511  USB Camera / USB Composite Device
VID_2BC5&PID_0614  ORBBEC Depth Sensor
```

但 `pyorbbecsdk2` 枚举为 0。结合 Windows 资料包内容，后续 Windows 阶段应优先走：

```text
OpenNI2 示例/SDK
  -> 验证深度和彩色读取
  -> 再决定是否用 C++ 封装，或寻找 OpenNI2 Python 绑定
```

`pyorbbecsdk2` 可作为新一代 Orbbec SDK 路线保留，但不作为当前 Gemini Pro Plus 的首选 Windows 数据读取方案。

## 建议验证顺序

1. 关闭 Orbbec Viewer。
2. 安装/确认 `SensorDriver_V4.3.0.20.exe` 或 Viewer 附带的 `SensorDriver_V4.3.0.22.exe`。
3. 运行 `DepthReaderPoll.exe`，确认能打印深度值。
4. 运行 `ColorReaderUVC.exe`，确认能收到彩色图像数据。
5. 运行 `SimpleViewer.exe`，确认能图形化显示。
6. 运行 `GeneratePointCloud.exe`，确认能生成点云 `.ply`。

项目中已新增便捷运行脚本：

```powershell
python project/src/gemini_windows/run_openni_sample.py depth
python project/src/gemini_windows/run_openni_sample.py color-uvc
python project/src/gemini_windows/run_openni_sample.py viewer
python project/src/gemini_windows/run_openni_sample.py pointcloud
python project/src/gemini_windows/opencv_uvc_color_test.py
```
