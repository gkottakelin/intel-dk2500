# JetArm Ubuntu 22.04 操作终端

这是一个独立目录，不依赖或导入上级项目中的 Python 模块。将整个
`ubuntu22_04_operation_terminal` 文件夹复制到 Ubuntu 22.04 即可部署。

## 1. 系统依赖

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-tk python3-numpy python3-serial fonts-noto-cjk
```

创建独立 Python 环境：

```bash
cd ubuntu22_04_operation_terminal
bash setup.sh
```

`setup.sh` 默认使用 Ubuntu 官方的软件包并创建带 `--system-site-packages` 的虚拟
环境，不需要访问 PyPI，因此在 pip 代理不可用时也可以安装。

## 2. 串口权限

将当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"
```

执行后必须注销并重新登录。可用下面命令确认：

```bash
groups
bash run.sh --list-ports
```

程序会结合 PySerial、稳定路径 `/dev/serial/by-id/*`、`/dev/serial/by-path/*`
以及常见的 `ttyUSB`、`ttyACM`、`ttyAMA`、`ttyTHS` 等设备节点查找串口。
只发现一个设备时会自动选择；发现多个设备时必须使用 `--port` 指定。

如果 USB 设备管理器能看到 CH340，但下拉框为空，先运行：

```bash
bash run.sh --diagnose-ports
```

这通常表示 USB 层识别了设备，但 Linux 没有创建 `/dev/ttyUSB*`，不是下拉框
过滤规则的问题。诊断结果会提示检查 `ch341` 驱动及 Ubuntu 22.04 的
`brltty` 抢占问题。

## 3. 运行

先进行无硬件界面预览：

```bash
bash run.sh --dry-run
```

直接控制舵机：

```bash
bash run.sh
```

未指定 `--port` 时，程序启动后会先打开“COM口设置”窗口。可以从下拉列表选择
已发现的串口、点击“刷新”，或手动输入 `/dev/ttyUSB0`、`/dev/ttyACM0`、
`/dev/serial/by-id/...` 后连接。连接成功后，主界面右上角会显示当前 COM 口。

明确指定串口：

```bash
bash run.sh --port /dev/ttyUSB0
```

推荐使用不会随插拔编号变化的路径：

```bash
bash run.sh --port /dev/serial/by-id/usb-你的设备名称
```

## 4. 控制行为

| 控件 | 行为 |
|---|---|
| 上 / 下 | TCP 以 `5 cm/s` 上升或下降，鼠标松开停止 |
| 水平摇杆 | 前后左右速度随拖动幅度变化，松开自动回中并停止 |
| J5 逆/顺时针 | 速度 `-100 / +100`，松开速度置 `0` |
| J6 松/闭 | 速度 `-100 / +100`，松开速度置 `0` |
| 抓紧 | 红色时 J6 速度为 `300` 并忽略松/闭误触；再按变绿色并停止 J6 |
| Home | 发送 `config/terminal.json` 中的 home 位姿 |
| 全部停止 | TCP 控制清零，J5/J6 速度置 `0` |

## 5. 配置

舵机 ID、关节限位、机械臂尺寸、速度和 home 位姿均在：

```text
config/terminal.json
```

默认 J6 舵机 ID 为 `10`。修改配置前应先断开机械臂电源。

## 6. 测试

```bash
.venv/bin/python -m unittest discover -s tests
```

## 7. 常见问题

- `No module named tkinter`：安装 `python3-tk` 后重新运行 `setup.sh`。
- `pip` 显示 `Cannot connect to proxy`：新版 `setup.sh` 不再调用 pip；安装
  `python3-numpy` 和 `python3-serial` 后重新运行即可。
- `Permission denied: /dev/ttyUSB0`：加入 `dialout` 组并重新登录。
- 未发现串口：运行 `bash run.sh --diagnose-ports`，根据结果检查 USB 转串口
  驱动和内核日志。
- 中文显示方框：安装 `fonts-noto-cjk` 后重新启动程序。
- WSL2 Ubuntu 22.04 需要 WSLg 显示 GUI，并通过 `usbipd` 将 USB 串口附加到 WSL。
