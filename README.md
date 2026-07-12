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

## JetArm总控终端

Ubuntu 22.04桌面环境下，可以从项目根目录启动轻量总控界面：

```bash
chmod +x run_control_center.sh
bash run_control_center.sh
```

总控提供Git Pull、机械臂控制、相机显示、人工测试模块V2和Agent按钮。
按下按钮后会打开一个独立桌面终端并执行对应的现有入口；总控不会重构、
复制或接管任何机械臂运动、相机采集及抓取工作流。
“打开使用说明”会使用系统默认文本查看器打开项目根目录的`使用教程.txt`。

“配置中心”包含：

- “接口与抓取点”：编辑本机机械臂串口、相机UID和抓取点像素，保存到
  `config/devices.json`。
- “Agent接口”：编辑OpenAI-compatible接口地址、模型、API Key环境变量名和超时。
  API Key本身仍只保存在环境变量或`.env`中。
- “机械臂参数（只读）”：逐项显示操作终端`terminal.json`的Home、关节限位、
  速度、几何和摄像头控制参数，只能刷新、复制或导出副本，不能修改原参数。

机械臂控制、人工测试模块V2和Agent可能争用机械臂串口；相机显示和Agent可能
争用Gemini相机。请在打开另一个占用同一硬件的模块前退出当前模块。

## AI对话与机械臂工具终端（第一至三阶段）

当前已接入OpenAI-compatible多模态API、多轮命令行对话和本地stdio MCP工具调用。
机械臂可在`dry-run`中验证，也可通过设备配置程序启用真实Ubuntu串口。
单路RGB接口通过MCP把JPEG、抓取点坐标和相机姿态一起返回给Agent；移动采用
“最新图像与姿态→单步动作→重新取图与读取姿态”的视觉闭环。
其中厘米位移量仍由机械臂运动学估算，不等同于深度测距或视觉标定后的绝对距离。

Ubuntu 22.04自带Python 3.10。不要安装根目录的`requirements.txt`，该文件还
包含Windows/Python 3.12相机依赖。为AI终端建立独立环境：

```bash
sudo apt install -y python3-venv python3-tk
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

Agent同时支持普通聊天。当本地路由确认本轮明显与JetArm机械臂、相机、
抓取和项目配置无关时，只向Kimi开放内置`$web_search`，由模型在新闻、天气、
价格或用户明确要求查证时按需调用。该模式不会同时暴露本地机械臂工具；
JetArm相关话题则只使用现有本地工具。联网聊天复用现有Kimi URL、模型和API Key，
不需要新增URL或配置项。

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
python3 -m unittest tests.test_ai_agent tests.test_ai_arm_control tests.test_ai_camera tests.test_ai_mcp
```

### AI自然语言控制机械臂

实现参考`IliaLarchenko/robot_MCP`的分层方式：AI终端作为MCP Client，自动启动本地
stdio MCP Server；MCP Server再调用JetArm控制器。控制器继续复用
`ubuntu22_04_operation_terminal`中的串口协议、J1-J4运动学、限位、Home、J5和J6。

激活`.venv-ai`后，先运行启动前设备配置程序：

```bash
source .venv-ai/bin/activate
python3 -m src.jetarm_agent.device_config
```

窗口中配置机械臂模式、串口，并通过Orbbec SDK选择Gemini USB设备/序列号，结果保存在本机专用的
`config/devices.json`，该文件已被Git忽略。无GUI的模拟配置方式：

```bash
python3 -m src.jetarm_agent.device_config --no-gui --arm-mode dry-run
```

配置完成后启动Agent：

```bash
python3 -m src.jetarm_agent
```

Agent会自动启动本地stdio MCP Server，并显示五步工作流。进入对话后可输入：

```text
向前移动5厘米
向左移动2厘米
上升3厘米
J5顺时针旋转0.5秒
打开夹爪0.5秒
抓紧
松开夹爪
和我握个手
回到home
停止机械臂
查看摄像头画面
描述当前RGB图像
```

“握手”使用独立的固定动作，不调用相机：先初始化到Home并将J6置于400，随后J6以
速度100持续收紧；抓取点以5cm/s上移5cm、下移5cm，循环3次；最后停止J6并再次
初始化。任何一段运动失败时也会停止J6，并尝试返回初始化状态。

以“向前5厘米”为例，实际链路为：

```text
向前5厘米
→ MCP调用get_rgb_camera_frame并把最新RGB图像、抓取点坐标和相机姿态传给Agent
→ Agent读取图像及其配套姿态，只下达当前一条移动命令
→ MCP返回status=ok，旧图失效
→ MCP再次取图并传给Agent
→ Agent读取新图后再决定下一条，循环直至完成或停止
```

完整规范位于`workflows/jetarm_mcp_workflow.md`，对话中输入`/workflow`可以显示。

控制约束：

- 未指定速度时固定使用`1.5 cm/s`。
- 用户指定速度必须在`1–5 cm/s`。
- Agent普通MCP移动命令默认必须严格小于`100 cm`，控制器按一个v2目标执行且不自动拆分；目标不可达时前往最近可达点。
- Agent不能预先生成完整移动序列；每次只能根据最新RGB图像决定一条动作。
- 每条移动命令返回`status=ok`后必须重新取图，新图进入Agent后才允许决定下一条。
- 没有新RGB图像时，运行时会拒绝机械臂移动调用。
- `get_jetarm_state`返回关节限位、Home、连杆尺寸、控制速度、坐标系和当前抓取点/相机姿态。
- V2六方向按抓取点XYZ标定：前/后使Y减小/增加，左/右使X减小/增加，上/下使Z增加/减小；运动保持摄像头—抓取点姿态。目标受关节限位不可达时前往本次方向上最近的可达点，并在状态中标明。
- 单次任务的Agent视觉闭环上限为`200`轮，达到上限后不再继续移动。
- 单次用户请求的总距离最多`10 cm`。
- 只有MCP返回`status=ok`后Agent才能报告完成。
- 当前相机配置保存Orbbec USB设备的序列号/UID，不再保存容易混淆的`/dev/video*`节点。
- 取图时生成仅启用Color的SDK配置，不启动或读取Depth流。

RGB图像链路参考`IliaLarchenko/robot_MCP`：Orbbec SDK按序列号采集最新彩色帧，MCP返回JSON文本和
JPEG图像内容块，Agent把JPEG转换为Kimi支持的Base64 `image_url`，模型读取图像后生成
描述或后续工具调用。抓取定位时，Agent对最新原图执行3×3数据层递归分块：每次选择目标
中心所在分块，连续4层后由程序把最终分块边界中心换算为原图坐标，并强制传给V2控制器。
分块图不绘制坐标标签；每次新取图都会清空旧分块路径。程序只保留最近一次MCP图像，
避免对话历史持续累积Base64数据。

相机测试命令：

```text
/camera
```

也可直接输入“查看摄像头画面”。视觉闭环移动会在首条动作前取图，并在每条动作成功后
自动再次调用`get_rgb_camera_frame`。取帧失败时禁止继续移动，但不会改变已完成动作的回执。

也可以用命令行直接覆盖设备配置：

```bash
python3 -m src.jetarm_agent \
  --arm-mode hardware \
  --arm-port /dev/serial/by-id/usb-你的设备名称
```

硬件模式启动时读取J1-J4当前位置，不会自动回Home。退出程序时会停止J5/J6并
关闭串口。

直接终端命令：

- `/arm-status`：读取J1-J4位置和估算TCP坐标。
- `/arm-stop`：立即停止笛卡尔速度、J5和J6。
- `/arm-home`：发送配置中的六关节Home位姿。

只有`get_rgb_camera_frame`或带图像的机械臂工具实际返回JPEG后，Agent才能声称看到
目标。运行真实机械臂前必须清空工作空间并准备断电。

默认配置位于`config/ai_agent.json`。配置优先级为命令行参数、环境变量、JSON
配置文件。API Key只从`MOONSHOT_API_KEY`读取，不写入JSON或源码。

交互命令：`/help`、`/clear`、`/history`、`/config`、`/tool-test`、`/camera`、
`/workflow`、`/arm-status`、`/arm-stop`、`/arm-home`、`/exit`。

home 位置：

```text
J1=500, J2=478, J3=641, J4=890, J5=500
```

J6 保持当前状态，home 时不发 J6 信号。

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
