# AI 视觉自然语言机械臂开发文件夹

`project/` 是 JetArm 机械臂项目的开发工作区，用于集中保存当前 RGB 视觉闭环方案、源码、ROS2 机械臂描述、配置、测试和阶段总结。

## 当前唯一主线

```text
自然语言任务
-> 云端大模型解析/目标检测
-> 本地 RGB 彩色图视觉闭环
-> J1-J5 servo 运动规划
-> J6 motor 夹取/释放
-> 任务完成后回 home
```

当前方案只读取 RGB 彩色图。

## AI对话与机械臂工具终端（第一至三阶段）

当前已接入OpenAI-compatible文本API、多轮命令行对话和白名单本地工具调用。
机械臂工具默认关闭；可在`dry-run`中验证AI调用，也可显式启用真实Ubuntu串口。
当前仍不会读取相机，距离控制是运动学估计加舵机位置反馈，不是视觉闭环。

Ubuntu 22.04自带Python 3.10。不要安装根目录的`requirements.txt`，该文件还
包含Windows/Python 3.12相机依赖。为AI终端建立独立环境：

```bash
sudo apt install -y python3-venv
python3 -m venv .venv-ai
source .venv-ai/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-ai.txt
```

默认已按Kimi中国区官方配置使用`https://api.moonshot.cn/v1`和`kimi-k2.6`。
只需在项目根目录创建`.env`保存新申请的Key：

```dotenv
MOONSHOT_API_KEY=你的Kimi_API_Key
JETARM_API_BASE_URL=https://api.moonshot.cn/v1
JETARM_API_MODEL=kimi-k2.6
```

```bash
chmod 600 .env
python3 -m src.jetarm_agent
```

`.env.example`可作为模板。Kimi K2.6同时支持文本、图片、视频和工具调用，适合后续
RGB图像理解与机械臂工具链。程序已按Kimi官方兼容要求禁用思考模式、不发送自定义
`temperature`，工具调用只使用`tool_choice=auto/none`。

### Git与Agent代理隔离

Agent的HTTP客户端固定使用`trust_env=False`，不会读取终端中的`HTTP_PROXY`、
`HTTPS_PROXY`或`ALL_PROXY`。因此Git可以单独使用代理，不需要为运行Agent修改或
清除系统环境变量。

在项目目录中为当前Git仓库配置本地SOCKS代理：

```bash
git config --local http.proxy "socks5h://127.0.0.1:7897"
git config --local http.version HTTP/1.1
git pull
```

若7897是HTTP/混合端口，将代理值改为`http://127.0.0.1:7897`。该设置只写入
当前仓库的`.git/config`，不会传给Agent。取消时执行
`git config --local --unset http.proxy`。

后续重新打开终端时，需要先执行：

```bash
cd ~/Desktop/workspace/intel-dk2500
source .venv-ai/bin/activate
```

也可以只发送一条消息：

```bash
python3 -m src.jetarm_agent --once "你好，请介绍一下你自己"
```

### AI调用本地代码贯通测试

该测试执行以下链路：程序等待3秒并向AI发送`ok`，AI返回结构化函数调用，
本地白名单工具将计数器从0加到1，程序把`{"status":"ok","count":1}`回传AI，
AI再生成最终回复。通常产生两次API请求；如果Kimi首次只回复文字，程序会按官方
兼容建议追加一次工具调用提示。测试不会访问相机或机械臂。

```bash
python3 -m src.jetarm_agent --tool-test
```

也可在交互终端输入`/tool-test`。成功时最后显示：

```text
[tool-test] 通过：工具调用1次，计数器=1
```

自动化测试使用假AI客户端，不访问网络、不等待3秒，也不消耗API额度：

```bash
python3 -m unittest tests.test_ai_agent tests.test_ai_arm_control
```

### AI自然语言控制机械臂

机械臂工具复用`ubuntu22_04_operation_terminal`中的串口协议、J1-J4运动学、限位、
Home、J5和J6控制。先在模拟模式验证：

```bash
python3 -m src.jetarm_agent --arm-mode dry-run
```

如果仍使用不带参数的`python3 -m src.jetarm_agent`，机械臂工具默认关闭。也可以在
`.env`中持久设置模拟模式：

```dotenv
JETARM_ARM_MODE=dry-run
```

进入对话后可以输入：

```text
向前移动5厘米
向左移动2厘米
上升3厘米
J5顺时针旋转0.5秒
打开夹爪0.5秒
抓紧
松开夹爪
回到home
停止机械臂
```

模拟结果会以`[arm-tool]`开头输出，不会打开串口。确认方向、舵机ID和Home位姿均
正确后，再明确启用硬件：

```bash
python3 ubuntu22_04_operation_terminal/jetarm_terminal.py --list-ports

python3 -m src.jetarm_agent \
  --arm-mode hardware \
  --arm-port /dev/serial/by-id/usb-你的设备名称
```

需要以后直接运行时，可在`.env`中改为：

```dotenv
JETARM_ARM_MODE=hardware
JETARM_ARM_PORT=/dev/serial/by-id/usb-你的设备名称
```

若只有一个`ttyUSB/ttyACM`设备，也可以省略`--arm-port`自动选择。硬件模式启动时
读取J1-J4当前位置，不会自动回Home。退出程序时会停止J5/J6并关闭串口。

直接终端命令：

- `/arm-status`：读取J1-J4位置和估算TCP坐标。
- `/arm-stop`：立即停止笛卡尔速度、J5和J6。
- `/arm-home`：发送配置中的六关节Home位姿。

安全限制：

- 基座坐标定义为`+X前、+Y左、+Z上`。
- TCP按约`0.4 cm`小步规划，默认单次最多`10 cm`。
- J5及J6的单次定时动作最多`2 s`，完成后自动停止。
- `grip_lock`会持续抓紧，必须通过“松开/停止”解除。
- 当前没有RGB反馈、碰撞检测和目标物闭环，运行前必须清空工作空间并准备断电。

默认配置位于`config/ai_agent.json`。配置优先级为命令行参数、环境变量、JSON
配置文件。API Key只从`MOONSHOT_API_KEY`读取，不写入JSON或源码。

交互命令：`/help`、`/clear`、`/history`、`/config`、`/tool-test`、
`/arm-status`、`/arm-stop`、`/arm-home`、`/exit`。

home 位置：

```text
J1=500, J2=550, J3=550, J4=900, J5=500, J6=360
```

## 目录结构

| 目录 | 用途 |
|---|---|
| `项目规划/` | 当前项目架构、资料索引和主路线说明 |
| `docs/` | 当前运动规划、舵机接口和文档索引 |
| `src/` | Python/ROS2/算法/应用源码 |
| `ros2_ws/src/jetarm_description/` | ROS2 Humble 机械臂 URDF/Xacro、RViz 显示和 J6 夹爪映射 |
| `tests/` | 单元测试、集成测试、硬件链路测试 |
| `config/` | 相机、舵机限位、AI 模型、视觉闭环参数等配置 |
| `launch/` | 后续 ROS2 启动文件 |
| `scripts/` | 环境配置、设备检查、调试脚本 |
| `data/` | RGB 样本、标定数据、实验数据 |
| `ubuntu22_04_gemini_camera/` | 可独立复制的 Ubuntu Gemini RGB-D 查看器、USB 权限规则和 Linux SDK |

## 当前关键文件

| 文件 | 说明 |
|---|---|
| `docs/README.md` | 文档索引和当前主线说明 |
| `docs/方案1_视觉闭环运动规划开发方案.md` | 方案1：RGB-only 视觉闭环运动规划方案 |
| `docs/方案2_云端多模态大模型机械臂控制方案.md` | 方案2：云端多模态大模型生成结构化任务，本地安全校验和执行 |
| `docs/方案3_固定物块尺寸单目测距机械臂规划方案.md` | 方案3：固定物块尺寸单目测距，并结合实际舵机位置 FK 做抓取规划 |
| `docs/总线舵机Python控制接口.md` | 舵机状态读取、servo 模式、motor 模式接口说明 |
| `项目规划/机械臂开发规划.md` | 下一阶段机械臂运动规划和视觉闭环开发路线 |
| `使用手册.md` | 当前代码、ROS2 显示包、舵机调试入口 |
| `项目进度总结.md` | 当前项目完成情况、未完成任务、下一阶段建议 |
| `项目方法总结.md` | 当前项目采用的方法、解决的问题和实现的功能 |
| `项目规划/AI视觉自然语言机械臂项目架构规划.md` | 当前总体蓝图 |
| `项目规划/资料并入索引.md` | 当前资料入口索引 |
| `ros2_ws/src/jetarm_description/README.md` | ROS2 机械臂描述、J6 夹爪映射和 RViz 显示说明 |
| `src/bus_servo.py` | 基于总线舵机协议的 Python 串口控制接口，默认 `115200bps` |
| `ubuntu22_04_gemini_camera/README.md` | Gemini 在 Ubuntu 22.04 下的安装、设备选择、RGB-D 查看和排障说明 |
| `tests/test_bus_servo.py` | 舵机协议帧、校验和状态解析单元测试 |
| `tests/test_urdf_description.py` | ROS2 机械臂描述、J6 夹爪和 launch 参数测试 |

## 当前技术路线

| 链路 | 当前策略 |
|---|---|
| 舵机控制 | Python 串口协议控制，后续封装为 ROS2 节点 |
| ROS2 显示 | `jetarm_description` 提供 URDF/Xacro、RViz 显示、J6 GUI 原始值映射 |
| J6 夹爪 | 真实位置范围 `0..1000`；`0` 完全张开，`700` 几何闭合，`700..1000` 只增加夹持力 |
| 彩色图 | 只读取 RGB 彩色图，用于目标检测、夹爪定位和视觉闭环 |
| 视觉闭环 | 使用夹爪橙色末端标记和目标像素误差，迭代修正 J1-J5 |
| 云端大模型 | 方案1中负责任务解析；方案2中负责结合 RGB 图像和视觉状态生成结构化任务步骤 |
| 安全策略 | 使用现有关节限位，桌面高度为 `z=0`，每次任务完成后回 home |
