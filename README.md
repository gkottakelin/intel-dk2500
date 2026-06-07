# AI 视觉自然语言机械臂开发文件夹

> `project/` 作为后续开发工作区，集中放置项目规划、源代码、配置、启动文件、脚本、测试和阶段总结。

## 目录结构

| 目录 | 用途 |
|---|---|
| `项目架构规划/` | 项目架构规划和资料并入索引 |
| `docs/` | 开发说明、接口说明、相机调试文档 |
| `src/` | Python/ROS2/算法/应用源代码 |
| `src/gemini_windows/` | Windows 阶段 Gemini Pro Plus RGB-D 数据读取和调试代码 |
| `tests/` | 单元测试、集成测试、硬件链路测试 |
| `config/` | 相机、手眼标定、舵机限位、AI 模型等配置 |
| `launch/` | 项目级 ROS2 启动文件 |
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
| `src/gemini_windows/run_openni_sample.py` | 运行 Windows OpenNI 示例程序 |
| `src/gemini_windows/opencv_uvc_color_test.py` | 使用 OpenCV 读取 Gemini Pro Plus 的 UVC 彩色画面 |
| `src/gemini_windows/windows_usb_check.py` | Windows PnP/USB 层面的相机检查脚本 |
| `tests/test_bus_servo.py` | 舵机协议帧、校验和状态解析单元测试 |

## 开发定位

本目录用于承载“Gemini Pro Plus 深度相机 + Intel DK2500 + AI 自然语言任务规划 + 总线舵机机械臂控制”的项目化开发内容。工作区原有教程、硬件资料和源码资料仍保持在原目录中，`project/` 负责组织可迭代开发产物。

## 当前技术路线

| 链路 | 当前策略 |
|---|---|
| 舵机控制 | Python 串口协议控制，后续封装为 ROS2 节点 |
| 彩色图 | Windows 阶段优先使用 UVC/OpenCV |
| 深度图 | Windows 阶段优先使用 OpenNI |
| 点云 | 参考 OpenNI `GeneratePointCloud` 示例 |
| RGB-D 融合 | 后续做 ROI 深度统计、坐标反投影和手眼标定 |
| DK2500 迁移 | 在 Ubuntu + ROS2 环境中使用 Orbbec ROS2 wrapper |
| GitHub 管理 | 上传 `project/` 和依赖清单，不直接上传 `.venv-gemini/` |

