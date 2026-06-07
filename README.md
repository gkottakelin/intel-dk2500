# AI 视觉自然语言机械臂开发文件夹

`project/` 是本项目的开发工作区，用于集中保存架构规划、资料索引、源代码、配置、测试、阶段总结和后续 ROS2/DK2500 迁移内容。

## 目录结构

| 目录 | 用途 |
|---|---|
| `项目架构规划/` | 项目总体架构规划和资料并入索引 |
| `docs/` | 开发说明、接口说明、相机调试文档 |
| `src/` | Python/ROS2/算法/应用源代码 |
| `src/gemini_windows/` | Windows 阶段 Gemini Pro Plus RGB-D 数据读取和调试代码 |
| `tests/` | 单元测试、集成测试、硬件链路测试 |
| `config/` | 相机、手眼标定、舵机限位、AI 模型等配置 |
| `launch/` | 后续 ROS2 启动文件 |
| `scripts/` | 环境配置、设备检查、调试脚本 |
| `data/` | RGB-D 样本、标定数据、实验数据 |

## 当前关键文件

| 文件 | 说明 |
|---|---|
| `项目进度总结.md` | 当前项目完成情况、未完成任务、下一阶段建议 |
| `项目方法总结.md` | 项目中使用的方法、解决的问题和实现的功能 |
| `GitHub仓库准备说明.md` | GitHub 建仓、提交内容和虚拟环境处理建议 |
| `环境重建说明.md` | 如何重新创建 `.venv-gemini` 开发环境 |
| `requirements.txt` | 项目运行依赖 |
| `requirements-venv-gemini.txt` | 当前 `.venv-gemini` 依赖快照 |
| `项目架构规划/AI视觉自然语言机械臂项目架构规划.md` | 项目总体蓝图 |
| `项目架构规划/资料并入索引.md` | 相关资料在工作区中的位置索引 |
| `src/bus_servo.py` | 基于总线舵机协议的 Python 串口控制接口，默认 `115200bps` |
| `docs/总线舵机Python控制接口.md` | 舵机状态读取、servo 模式、motor 模式接口说明 |
| `docs/GeminiProPlus深度相机Windows使用说明.md` | Windows 上位机观察 Gemini Pro Plus 和 RGB-D 数据处理说明 |
| `docs/Gemini相机分步开发步骤.md` | Gemini Pro Plus 从 Windows 验证到 ROS2/DK2500 迁移的阶段计划 |
| `docs/GeminiProPlus_Windows_OpenNI资料定位.md` | Windows 资料包中 OpenNI 示例、源码和运行方式定位 |
| `src/gemini_windows/README.md` | 第二阶段 Windows SDK/OpenNI/UVC 数据读取代码说明 |
| `src/gemini_windows/run_openni_sample.py` | 运行 Windows OpenNI 示例程序；`viewer/depth-viewer` 默认使用官方 `SimpleViewer.exe 1 1` |
| `src/gemini_windows/opencv_uvc_color_test.py` | 使用 OpenCV 读取 Gemini Pro Plus 的 UVC 彩色画面 |
| `src/gemini_windows/pointcloud_viewer.py` | 可视化 OpenNI 生成的 `.ply` 点云文件 |
| `src/gemini_windows/windows_usb_check.py` | Windows PnP/USB 层面的相机检查脚本 |
| `tests/test_bus_servo.py` | 舵机协议帧、校验和状态解析单元测试 |

## 当前技术路线

| 链路 | 当前策略 |
|---|---|
| 舵机控制 | Python 串口协议控制，后续封装为 ROS2 节点 |
| 彩色图 | Windows 阶段优先使用 UVC/OpenCV |
| 深度图 | Windows 阶段优先使用 OpenNI |
| RGB-D 对齐可视化 | 使用 `run_openni_sample.py viewer` / `depth-viewer` 调用官方 `SimpleViewer.exe` |
| 镜像修正 | 默认传入 `SimpleViewer.exe 1 1`，修正黄色深度层和 RGB 左右反相 |
| 点云 | 参考 OpenNI `GeneratePointCloud` 示例生成 `.ply` |
| 点云可视化 | 使用 `run_openni_sample.py pointcloud-viewer` 连续生成并显示 Viewer 风格点云 |
| RGB-D 融合 | 后续做 ROI 深度统计、坐标反投影和手眼标定 |
| DK2500 迁移 | 在 Ubuntu + ROS2 环境中使用 Orbbec ROS2 wrapper |
| GitHub 管理 | 上传 `project/` 和依赖清单，不直接上传 `.venv-gemini/` |
