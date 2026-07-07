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

## AI对话终端（第一阶段）

当前已先接入OpenAI-compatible文本API和多轮命令行对话。这个阶段不会读取
相机，也不会控制机械臂。

Ubuntu 22.04自带Python 3.10。不要安装根目录的`requirements.txt`，该文件还
包含Windows/Python 3.12相机依赖。为AI终端建立独立环境：

```bash
sudo apt install -y python3-venv
python3 -m venv .venv-ai
source .venv-ai/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-ai.txt
```

然后配置API并启动：

```bash
export JETARM_API_KEY="你的API Key"
export JETARM_API_BASE_URL="https://你的服务地址/v1"
export JETARM_API_MODEL="支持的模型名"
python3 -m src.jetarm_agent
```

使用Gemini的OpenAI兼容接口时：

```bash
export JETARM_API_KEY="你的Gemini API Key"
export JETARM_API_BASE_URL="https://generativelanguage.googleapis.com/v1beta/openai/"
export JETARM_API_MODEL="gemini-3.5-flash"
python3 -m src.jetarm_agent
```

如果出现`Unknown scheme for proxy URL ... socks://...`，说明代理协议写法不受
`httpx`支持。若该端口是SOCKS代理，将`socks://`改为`socks5://`，并在激活
`.venv-ai`后安装SOCKS支持：

```bash
export ALL_PROXY="socks5://127.0.0.1:7897"
export all_proxy="$ALL_PROXY"
python -m pip install -r requirements-ai.txt
```

若代理软件提供的是HTTP或混合端口，则应使用`http://127.0.0.1:端口`。还需确认
对应端口正在监听。若不需要代理，可在当前终端执行
`unset ALL_PROXY all_proxy HTTP_PROXY HTTPS_PROXY http_proxy https_proxy`后再启动。

后续重新打开终端时，需要先执行：

```bash
cd ~/Desktop/workspace/intel-dk2500
source .venv-ai/bin/activate
```

也可以只发送一条消息：

```bash
python3 -m src.jetarm_agent --once "你好，请介绍一下你自己"
```

默认配置位于`config/ai_agent.json`。配置优先级为命令行参数、环境变量、JSON
配置文件。API Key只从环境变量读取，不写入项目文件。

交互命令：`/help`、`/clear`、`/history`、`/config`、`/exit`。

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
