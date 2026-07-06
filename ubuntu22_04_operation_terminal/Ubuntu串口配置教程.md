# Ubuntu 22.04 串口配置教程

本文适用于 JetArm Ubuntu 操作终端，以及常见的 CH340、CH341、CP210x、FTDI 和 USB CDC 串口设备。

程序默认波特率为 `115200`。除特别说明外，命令均以普通用户执行。

## 1. 连接设备并确认系统已识别

插入 USB 转串口设备后执行：

```bash
lsusb
ls -l /dev/serial/by-id/ 2>/dev/null
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

本项目实测设备的稳定路径为：（这个是和机械臂连接的口）

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

`/dev/ttyUSB0` 的编号可能在重新插拔或连接其他串口后改变，建议优先使用 `/dev/serial/by-id/` 下的路径。

如果没有发现设备，在一个终端中执行：

```bash
sudo dmesg -w
```

然后重新插拔 USB。正常情况下会看到 `ttyUSB0` 或 `ttyACM0`。按 `Ctrl+C` 退出日志监视。

对于 VID 为 `1a86` 的 CH340/CH341 设备，可检查驱动：

```bash
lsmod | grep ch341
sudo modprobe ch341
```

如果 `lsusb` 也看不到设备，应检查 USB 数据线、接口、供电和虚拟机的 USB 直通设置。

## 2. 永久解决串口权限（这一步我已经做过了）

Ubuntu 通常将串口设备设置为 `root:dialout`，权限为 `660`。先检查：

```bash
ls -l /dev/ttyUSB0
```

典型输出如下：

```text
crw-rw---- 1 root dialout ... /dev/ttyUSB0
```

将当前用户永久加入 `dialout` 组：

```bash
sudo usermod -aG dialout "$USER"
```

检查是否已经写入组配置：

```bash
getent group dialout
```

输出末尾应包含当前用户名。然后必须**注销 Ubuntu 用户并重新登录**，也可以直接重启：

```bash
sudo reboot
```

重新登录后验证：

```bash
groups
```

输出中必须包含 `dialout`。此后运行程序不需要 `sudo`。

> `sudo chmod 777 /dev/ttyUSB0` 不是永久方案。设备重新插拔后权限会恢复，而且它允许所有用户访问机械臂串口，不建议使用。

### 不重启的临时验证方法

完成 `usermod` 后，可在当前终端临时进入新组：

```bash
newgrp dialout
```

该方法只影响当前 shell。正式使用仍建议注销并重新登录。

## 3. 配置固定串口名称（可选）

`/dev/serial/by-id/` 已经是稳定路径，通常不需要额外配置。如果希望使用更短的 `/dev/jetarm`，可以添加 udev 规则。

先读取设备的厂商 ID 和产品 ID：

```bash
udevadm info --query=property --name=/dev/ttyUSB0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL'
```

截图中的设备厂商 ID 为 `1a86`，但产品 ID 必须以上述命令的实际输出为准。假设输出为：

```text
ID_VENDOR_ID=1a86
ID_MODEL_ID=7523
```

创建规则文件：

```bash
sudo nano /etc/udev/rules.d/99-jetarm-serial.rules
```

写入下面一行；如果产品 ID 不是 `7523`，请替换为实际值：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", GROUP="dialout", MODE="0660", SYMLINK+="jetarm"
```

保存后重新加载规则：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔出并重新插入设备，然后验证：

```bash
ls -l /dev/jetarm
```

如果有多个相同型号的 USB 转串口设备，还应在规则中增加设备序列号条件，避免多个设备同时匹配：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{serial}=="实际序列号", GROUP="dialout", MODE="0660", SYMLINK+="jetarm"
```

## 4. 在 JetArm 程序中使用串口

进入独立 Ubuntu 项目目录：

```bash
cd ~/Desktop/ubuntu22_04_operation_terminal
```

列出程序能够识别的串口：

```bash
bash run.sh --list-ports
```

使用截图中已经识别到的稳定路径启动：

```bash
bash run.sh --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

如果配置了 `/dev/jetarm`，可使用：

```bash
bash run.sh --port /dev/jetarm
```

也可以使用动态设备名：

```bash
bash run.sh --port /dev/ttyUSB0
```

覆盖默认波特率的示例：

```bash
bash run.sh --port /dev/jetarm --baudrate 115200
```

注意：下面的命令是错误的，因为 `--port` 后缺少串口路径：

```bash
bash run.sh --port
```

如果直接运行：

```bash
bash run.sh
```

程序会打开 COM 口设置窗口。若终端看起来“停住不动”，请用 `Alt+Tab` 检查设置窗口是否位于其他窗口后面，或者直接使用带 `--port` 的命令启动。

## 5. 测试串口是否可打开

安装 pyserial 后可查看详细端口信息：

```bash
python3 -m serial.tools.list_ports -v
```

使用项目虚拟环境测试打开串口：

```bash
.venv/bin/python -c "import serial; s=serial.Serial('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',115200,timeout=0.2); print('串口打开成功:',s.name); s.close()"
```

如果使用 `/dev/jetarm`，将命令中的设备路径替换为 `/dev/jetarm`。

## 6. 常见问题排查

### `Permission denied`

依次执行：

```bash
groups
ls -l /dev/ttyUSB0
getent group dialout
```

确认当前用户属于 `dialout`，设备所属组也是 `dialout`。如果刚执行过 `usermod`，必须注销并重新登录。

不要使用 `sudo bash run.sh`。以 root 运行图形程序会产生额外的显示、配置文件所有权和安全问题。

### `No such file or directory`

设备名称可能已经变化，重新查询：

```bash
ls -l /dev/serial/by-id/ 2>/dev/null
bash run.sh --list-ports
```

然后使用最新路径启动。

### `Device or resource busy`

检查哪个进程占用了串口：

```bash
sudo fuser -v /dev/ttyUSB0
sudo lsof /dev/ttyUSB0
```

关闭串口调试工具、其他 JetArm 程序或占用该端口的进程后再启动。

部分 Ubuntu 环境中的 ModemManager 会探测 USB 串口。先检查：

```bash
systemctl status ModemManager
```

如果确认这台电脑的串口只用于机械臂，并且日志显示 ModemManager 正在占用端口，可禁用它：

```bash
sudo systemctl disable --now ModemManager
```

如果电脑还需要使用 4G/5G USB 调制解调器，不应禁用该服务，应改用针对设备的 udev 忽略规则。

### 找不到 `/dev/ttyUSB0`

执行：

```bash
lsusb
sudo dmesg | tail -n 50
lsmod | grep -E 'ch341|cp210x|ftdi_sio|cdc_acm'
```

如果虚拟机中的 Ubuntu 看不到设备，需要先在 VMware 或 VirtualBox 菜单中将 USB 串口连接到 Ubuntu 虚拟机，而不是宿主机。

### 程序启动后终端没有新输出

这是图形界面程序的正常表现。检查任务栏或按 `Alt+Tab` 切换到 JetArm 窗口。也可先验证界面：

```bash
bash run.sh --dry-run
```

## 7. 推荐的最终配置

1. 当前用户加入 `dialout` 组，并注销后重新登录。
2. 优先使用 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
3. 需要简短名称时配置 `/dev/jetarm` udev 规则。
4. 始终以普通用户运行 `bash run.sh`，不要使用 `sudo`。
5. 启动机械臂前保证急停和断电手段可用，首次联调使用低风险姿态。

推荐启动命令：

```bash
cd ~/Desktop/ubuntu22_04_operation_terminal
bash run.sh --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```
