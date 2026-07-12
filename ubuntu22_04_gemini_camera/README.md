# Gemini 相机 Ubuntu 22.04 独立程序

本目录是不依赖 ROS2 的 Gemini RGB-D 查看程序，可整体复制到 Intel/AMD x86_64 的 Ubuntu 22.04 板卡。代码移植自 `src/gemini_windows/`，设备选择和部署结构与 `ubuntu22_04_operation_terminal/` 一致。

本项目中的 Gemini 是旧 OpenNI 协议设备（Orbbec VID `2bc5`，Gemini PID `0614`、彩色 PID `0511`）。Windows 代码中验证可用的是 OpenNI/UVC 路径，而 `pyorbbecsdk2` 日志没有枚举到设备。因此本程序复用资料中随附的 Linux OrbbecSDK 1.5.7，不使用 Windows DLL，也不要求 ROS2。

## 1. 安装系统依赖

```bash
cd ubuntu22_04_gemini_camera
sudo apt update
sudo apt install -y python3 python3-venv python3-tk python3-numpy python3-opencv libusb-1.0-0 usbutils fonts-noto-cjk
bash setup.sh
```

程序使用 Ubuntu 官方 NumPy/OpenCV 包，创建带 `--system-site-packages` 的虚拟环境，不需要从 PyPI 下载相机 SDK。

## 2. 配置 USB 权限

相机不是串口设备，不应加入 `dialout` 组。这里把机械臂串口模板中的权限步骤替换为 Orbbec 官方 udev 规则：

```bash
sudo bash install_udev_rules.sh
```

执行后拔插相机，再检查：

```bash
lsusb | grep -i 2bc5
bash run.sh --diagnose
```

不要使用 `sudo bash run.sh`。如果普通用户仍然无法访问相机，注销并重新登录后再试。

## 3. 运行

直接运行会打开与机械臂 COM 口设置窗口同样结构的 USB 相机选择窗口。
`run.sh`默认只读取和显示RGB彩色图，不读取、转换或显示深度图：

```bash
bash run.sh
```

也可以跳过窗口：

```bash
bash run.sh --list-devices
bash run.sh --serial 你的相机序列号
bash run.sh --first-device
```

查看内参：

```bash
bash run.sh --serial 你的相机序列号 --read-intrinsics
```

查看器只显示RGB彩色图。鼠标左键点击图像可显示该点的像素坐标，终端也会输出
`X`、`Y`；坐标原点为图像左上角，X向右、Y向下。按 `s` 只保存RGB图像；
按 `q` 或 `Esc` 退出。
如果确实需要单独调试原RGB-D查看器，可不经`run.sh`直接运行：

```bash
.venv/bin/python gemini_camera.py
```

## 4. 配置文件

配置位于 `config/camera.json`：

- `device.serial_number`：留空时启动可视化选择；填写后固定相机。
- `stream.frame_timeout_ms`：等待一组 RGB-D 帧的超时时间。
- `depth.min_mm/max_mm`：深度显示范围。
- `depth.click_window`：点击测距使用的正方形中值窗口，必须为奇数。
- `display.mirror_color/mirror_depth`：彩色和深度镜像。
- `capture.save_dir`：快照目录，相对于本程序目录。

SDK 路径通常不需要修改。默认流参数来自 `sdk/OrbbecSDKConfig.xml` 中的 Gemini 配置：深度 `640x400@30 Y11`，彩色 `640x480@30 MJPG`。

## 5. 测试

```bash
.venv/bin/python -m unittest discover -s tests
```

## 6. 常见问题

- `未发现 Orbbec 相机`：先确认 `lsusb` 能看到 `2bc5:0614` 或 `2bc5:0511`，再安装 udev 规则并拔插相机。
- `error while loading shared libraries`：必须通过 `bash run.sh` 启动，脚本会设置 SDK 动态库路径。
- 只有彩色、没有深度：确认深度 PID `0614` 存在，关闭 Orbbec Viewer/其他占用相机的程序，换 USB 3.0 数据口和数据线。
- 没有图形窗口：Ubuntu Server 需接显示器/桌面环境；SSH 运行需正确配置 X11 转发。
- 多台相机：使用 `--serial`，或把序列号写入 `config/camera.json`，不要依赖 USB 插入顺序。
