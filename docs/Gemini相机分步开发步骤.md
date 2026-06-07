# Gemini Pro Plus 相机分步开发步骤

> 当前阶段：先在 Windows 上位机完成 RGB-D 数据读取和算法验证，再迁移到 ROS2 / DK2500。

## 阶段 1：相机出流验证

目标：确认 Gemini Pro Plus 硬件和 Windows 上位机链路正常。

工作内容：

- 使用 Orbbec Viewer 打开彩色、深度、红外、点云。
- 记录分辨率、帧率和格式。
- 检查 USB3.0 稳定性。
- 观察深度黑洞、反光、过近失效、遮挡等问题。

验收标准：

- 彩色、深度、红外画面稳定。
- 深度图随物体远近变化。
- 点云能显示桌面和物块轮廓。

## 阶段 2：Windows SDK 数据读取

目标：脱离 Viewer，用 Python 程序读取相机数据。

工作内容：

- 安装 `pyorbbecsdk2`、`opencv-python`、`numpy`。
- 编写设备枚举代码。
- 读取彩色帧、深度帧、相机内参。
- 用 OpenCV 显示 RGB 和 Depth。
- 支持鼠标点击图像，读取点击点附近深度中位数。

开发文件：

```text
project/src/gemini_windows/
  gemini_common.py
  list_devices.py
  camera_stream_test.py
  read_intrinsics.py
```

验收标准：

- Python 程序能显示彩色图和深度图。
- 程序能打印设备信息和相机内参。
- 点击图像后能输出像素坐标和深度值。

## 阶段 3：深度测距实验

目标：确认深度值能用于机械臂抓取。

工作内容：

- 放置标准物块。
- 手动点击物块中心。
- 读取 `5x5` 或 `7x7` 深度窗口。
- 过滤 `0` 等无效深度。
- 取中位数作为距离。
- 与尺子测量值对比。

验收标准：

- 深度输出稳定。
- 中近距离误差可接受。
- 无效深度区域能被识别并提示。

## 阶段 4：物块识别

目标：从彩色图像中找到目标物块。

优先实现：

- HSV 颜色分割。
- 轮廓提取。
- 最大目标筛选。
- 输出目标框、中心点和面积。

后续增强：

- YOLO 检测。
- 多物块识别。
- 自然语言指定目标类别或颜色。

验收标准：

- 能稳定识别红、蓝、绿等实验物块。
- 能输出目标中心像素 `(u, v)`。

## 阶段 5：RGB-D 三维定位

目标：把图像中的物块变成相机坐标系中的三维点。

处理流程：

```text
物块中心像素 (u, v)
  -> 读取邻域深度 Z
  -> 获取内参 fx, fy, cx, cy
  -> 计算相机坐标 X, Y, Z
```

计算公式：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

验收标准：

- 物块左右移动时，`X` 方向变化正确。
- 物块上下移动时，`Y` 方向变化正确。
- 物块远近移动时，`Z` 方向变化正确。

## 阶段 6：相机到机械臂坐标转换

目标：把相机三维点转换到机械臂基座坐标系。

工作内容：

- 确认相机安装方式。
- 建立 `camera_link`、`camera_optical_frame`、`base_link` 坐标关系。
- 完成手眼标定。
- 保存变换矩阵到配置文件。

建议配置：

```text
project/config/hand_eye_calibration.yaml
```

验收标准：

- 输出 `base_link` 坐标系下的目标点。
- 机械臂移动到目标附近时，空间误差可控。

## 阶段 7：抓取点生成

目标：把视觉目标转换成可执行抓取任务。

动作序列：

```text
预抓取点
  -> 下降
  -> 夹取
  -> 抬升
  -> 移动到放置点
  -> 松开
```

验收标准：

- 不接自然语言时，能抓取固定颜色物块。
- 全流程具备低速、限位、急停策略。

## 阶段 8：迁移到 ROS2 / DK2500

目标：把 Windows 阶段验证过的感知逻辑迁移到正式 ROS2 系统。

对应关系：

```text
Windows SDK 彩色帧 -> /camera/color/image_raw
Windows SDK 深度帧 -> /camera/depth/image_raw
Windows SDK 内参 -> /camera/color/camera_info
Windows object_pose -> ROS2 GraspTarget
```

ROS2 节点建议：

```text
perception_node
  输入：RGB、Depth、CameraInfo、TF
  输出：DetectedObject / GraspTarget

arm_controller_node
  输入：GraspTarget
  输出：舵机控制命令
```

最终链路：

```text
Gemini Pro Plus
  -> RGB-D 感知
  -> 物块识别
  -> 深度测距
  -> 三维定位
  -> 手眼坐标转换
  -> 抓取规划
  -> 总线舵机控制
  -> 机械臂执行
```

## 当前推荐推进顺序

1. 完成阶段 2：Windows SDK 数据读取。
2. 完成阶段 3：点击测距和误差记录。
3. 完成阶段 4：颜色物块识别。
4. 完成阶段 5：RGB-D 三维定位。
5. 再进入手眼标定和机械臂抓取闭环。
