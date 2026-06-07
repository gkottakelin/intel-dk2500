# Gemini Windows 调试代码

本目录用于 Windows 上位机阶段调试 Gemini Pro Plus 深度相机。当前路线是：

| 数据 | 推荐方式 |
|---|---|
| 深度图 | OpenNI `DepthReaderPoll.exe` / `GeneratePointCloud.exe` |
| 彩色图 | UVC / OpenCV / `ColorReaderUVC.exe` |
| RGB-D 对齐显示 | OpenNI `SimpleViewer.exe` |
| 点云可视化 | `pointcloud_viewer.py` |

## 1. 环境诊断

```powershell
python project/src/gemini_windows/diagnose_environment.py
python project/src/gemini_windows/windows_usb_check.py
```

`windows_usb_check.py` 不依赖 Orbbec SDK，只检查 Windows PnP/USB 层面是否识别相机。Gemini Pro Plus 是 USB 设备，不是 COM 口设备。

## 2. 运行 OpenNI 官方示例

```powershell
python project/src/gemini_windows/run_openni_sample.py depth
python project/src/gemini_windows/run_openni_sample.py color-uvc
python project/src/gemini_windows/run_openni_sample.py viewer
python project/src/gemini_windows/run_openni_sample.py pointcloud
```

说明：

| 命令 | 作用 |
|---|---|
| `depth` | 打印深度帧中心点距离 |
| `color-uvc` | 通过 UVC 读取彩色图 |
| `viewer` | 运行 `SimpleViewer.exe`，显示 RGB-D 对齐叠加效果 |
| `pointcloud` | 运行 `GeneratePointCloud.exe`，生成 `.ply` 点云文件 |

`viewer` 显示的是彩色图和深度图的二维对齐叠加，不是点云。真正的点云文件由 `pointcloud` 生成。

## 3. 点云可视化

先生成点云：

```powershell
python project/src/gemini_windows/run_openni_sample.py pointcloud
```

它会在 OpenNI 示例目录中生成 `1.ply`、`2.ply` 等文件。然后运行：

```powershell
python project/src/gemini_windows/pointcloud_viewer.py
```

脚本会自动打开最新的 `.ply` 文件。

如果要指定某个点云文件：

```powershell
python project/src/gemini_windows/pointcloud_viewer.py "D:\jetarm\gemini深度相机windows资料\Windows\Windows\OpenNI_v2.3.0.85_20220615_1b09bbfd_windows_x64_x86_release\samples\bin\1.ply"
```

窗口操作：

| 操作 | 功能 |
|---|---|
| 鼠标左键拖动 | 旋转点云 |
| 鼠标滚轮 | 缩放 |
| `+` / `-` | 调整点大小 |
| `R` | 重置视角 |
| `S` | 保存当前窗口截图 |
| `ESC` | 退出 |

如果窗口打开但点云很稀疏，说明 `.ply` 中有效深度点较少。可以重新运行 `pointcloud`，调整相机朝向，让深度图中心和画面主体对准有纹理、非反光、非透明的物体。

## 4. UVC 彩色图测试

```powershell
python project/src/gemini_windows/opencv_uvc_color_test.py
```

如果默认索引不是 Gemini Pro Plus，可以指定索引：

```powershell
python project/src/gemini_windows/opencv_uvc_color_test.py --index 1
```

## 5. Python SDK 路线说明

当前 `pyorbbecsdk2` 可以作为新 Orbbec SDK 路线保留，但本机测试中出现过“能导入但枚举不到设备”的情况。因此 Windows 阶段优先使用：

```text
OpenNI 深度 + UVC 彩色 + 项目脚本可视化
```

后续迁移到 DK2500/ROS2 时，再切换到 Orbbec ROS2 wrapper。

