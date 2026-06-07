# Gemini Windows 调试代码

本目录用于 Windows 上位机阶段调试 Gemini Pro Plus 深度相机。当前策略是优先复用厂家资料包里的 OpenNI 示例框架，不重新创造一套深度图渲染逻辑。

| 数据 | 推荐方式 |
|---|---|
| 深度数据 | OpenNI `DepthReaderPoll.exe` / `SimpleViewer.exe` |
| 彩色图像 | UVC / OpenNI `ColorReaderUVC.exe` |
| RGB-D 对齐显示 | OpenNI `SimpleViewer.exe` |
| 点云生成 | OpenNI `GeneratePointCloud.exe` |
| 点云可视化 | `pointcloud_viewer.py` 读取 OpenNI 生成的 `.ply` |

## 环境诊断

```powershell
python project/src/gemini_windows/diagnose_environment.py
python project/src/gemini_windows/windows_usb_check.py
```

`windows_usb_check.py` 只检查 Windows PnP/USB 层面是否识别设备。Gemini Pro Plus 是 USB 设备，不是 COM 口设备。

## 运行 OpenNI 官方示例

```powershell
python project/src/gemini_windows/run_openni_sample.py depth
python project/src/gemini_windows/run_openni_sample.py color-uvc
python project/src/gemini_windows/run_openni_sample.py viewer
python project/src/gemini_windows/run_openni_sample.py depth-viewer
python project/src/gemini_windows/run_openni_sample.py pointcloud
```

| 命令 | 作用 |
|---|---|
| `depth` | 打印深度帧中心点距离 |
| `color-uvc` | 通过 UVC 读取彩色图像 |
| `viewer` | 运行官方 `SimpleViewer.exe`，显示 RGB-D 对齐叠加 |
| `depth-viewer` | `viewer` 的同框架别名，仍然运行官方 `SimpleViewer.exe` |
| `pointcloud` | 运行官方 `GeneratePointCloud.exe`，生成 `.ply` 点云文件 |
| `pointcloud-viewer` | 自动运行并重启点云生成器，连续显示点云窗口 |
| `pointcloud-watch` | 不占用相机，只观察并刷新最新 `.ply` 点云 |

## SimpleViewer 镜像参数

厂家 `SimpleViewer.exe` 的参数格式实际是：

```powershell
SimpleViewer.exe [0:Non UVC/1:UVC] [colorMirror: 0/1]
```

源码里第二个参数的执行逻辑是 `1表示镜像，0表示非镜像`；当 `colorMirror=1` 时，程序会对深度层执行水平翻转。

当前项目的 `viewer` 和 `depth-viewer` 默认传入：

```text
1 1
```

原因是 Gemini Pro Plus 在当前 Windows UVC 路线上，彩色图像和黄色深度叠加层会出现左右反相。第二个参数设为 `1` 后，官方 `SimpleViewer` 会对对齐后的深度层执行水平翻转，使黄色深度层和 RGB 图像对齐。

如果换到另一台电脑或 DK2500 后画面不需要镜像修正，可以手动指定：

```powershell
python project/src/gemini_windows/run_openni_sample.py viewer 1 0
python project/src/gemini_windows/run_openni_sample.py depth-viewer 1 0
```

## 点云可视化

官方 `GeneratePointCloud.exe` 源码中写死了 `MAX_FRAME_COUNT 50`，因此生成 50 帧后会自动停止。项目中的连续点云窗口通过后台自动重启该官方生成器，并读取最新 `.ply` 文件来显示。

```powershell
python project/src/gemini_windows/run_openni_sample.py pointcloud-viewer
```

窗口操作：

| 操作 | 功能 |
|---|---|
| 鼠标左键拖动 | 旋转点云 |
| 鼠标滚轮 | 缩放 |
| `V` | 在前视 Viewer 风格和三维旋转视角之间切换 |
| `+` / `-` | 调整点大小 |
| `R` | 重置视角 |
| `S` | 保存当前窗口截图 |
| `ESC` | 退出 |

## 激光和近距离保护

当前 `viewer/depth-viewer` 直接复用官方 OpenNI `SimpleViewer.exe`，不再使用自写 Python 深度渲染器。激光、LDP/近距离保护等硬件属性如果需要强制写入，下一步应在 OpenNI C++ 示例层修改并重新编译，或者等 Python SDK 能稳定枚举设备后再通过 `pyorbbecsdk` 设置。

在现在的验证阶段，重点先保证：

1. OpenNI 能稳定读到深度。
2. UVC 彩色图像能稳定显示。
3. `SimpleViewer` 的黄色深度叠加层和 RGB 方向一致。
4. `.ply` 点云能生成并可视化。
