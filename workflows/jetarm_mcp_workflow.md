---
name: jetarm-mcp-motion
description: 将自然语言机械臂任务转换为 JetArm MCP 调用，并按真实工具回执总结。
---

# JetArm MCP 工作流规范

## 通用控制规则

1. 只有用户明确要求移动、抓取、回 Home、停止、旋转腕部、控制夹爪或查看画面时，才调用会改变机械臂状态的工具。
2. 每次需要视觉定位时必须调用 `get_rgb_camera_frame`，让 Agent 看到最新 RGB 画面和同一时刻的 `arm_pose`。
3. `move_jetarm` 使用 camera-vector 控制系：上为抓取点到摄像头，下为摄像头到抓取点；前后左右位于垂直于摄像头-抓取点连线的平面。
4. 普通手动移动仍保持单条命令严格小于 `2 cm`，控制程序只执行当前收到的一条命令。
5. 任一工具返回 `status=error` 后立即停止后续动作；存在运动风险时调用 `stop_jetarm`。
6. 最终总结必须以 MCP 工具真实返回值为依据，不得虚构视觉结果、位置或完成状态。

## Agent 与控制程序职责

1. Agent 只负责解析用户命令，并在传输来的 RGB 图像中寻找目标点。
2. Agent 返回目标点像素 `target_x/target_y`，不得自行决定机械臂前后左右、下降距离或速度。
3. 图像中的抓取点像素由控制程序提供在 `camera.grasp_point_pixel` 中；当前默认是图像中心，来源字段为 `image_center_default`。
4. 机械臂控制程序调用 `control_jetarm_to_target_pixel`，根据目标点像素和抓取点像素自行决定前后左右移动或下降。
5. 每次机械臂发生移动后旧图像失效，必须重新获取图像，再由 Agent 重新寻找目标点像素。

## 视觉抓取物块工作流

当用户要求抓取物块、方块、积木、目标物体或 block/cube/object 时，必须使用以下闭环流程。

1. 调用 `set_jetarm_gripper_position(position=370)`，在抓取成功前让 J6 保持松开状态。
2. 调用 `get_rgb_camera_frame` 获取最新画面和 `camera.grasp_point_pixel`。
3. Agent 只在画面中寻找目标点，并返回 `target_x/target_y`。
4. 调用 `control_jetarm_to_target_pixel(target_x=..., target_y=...)`。
5. 控制程序读取各关节当前位置，用 FK 实时解算抓取点位置和高度；该高度解算独立于运动命令。
6. 控制程序按抓取点高度选择像素容差：
   - 高度 `>15 cm`：容差 `18 px`
   - 高度 `>10 cm` 且 `<=15 cm`：容差 `15 px`
   - 高度 `>5 cm` 且 `<=10 cm`：容差 `10 px`
   - 高度 `<=5 cm`：容差 `8 px`
7. 如果目标点和抓取点像素未在当前容差内重合，控制程序自行决定前/后/左/右移动方向和步长；Agent 不参与运动决策。
8. 水平移动距离按主轴像素误差除以 `13.3 px/cm` 计算，并受单次运动上限约束。例如目标点在抓取点左侧 `26.6 px`，则抓取点向左移动 `2 cm`。
9. 如果两个像素点在当前容差内重合，控制程序以 `2 cm/s` 向下运动，并在下降过程中持续基于关节角度/FK 解算抓取点位置。
10. 每下降 `2 cm`，必须重新调用 `get_rgb_camera_frame`，由 Agent 再次寻找目标点像素，然后重新调用 `control_jetarm_to_target_pixel`。
11. 抓取动作使用 `control_jetarm_gripper(action="grip_lock")`。
12. 抓取完成后调用 `move_jetarm_home` 复位。
13. 复位后调用 `get_rgb_camera_frame`，由 Agent 检查物块是否被成功抓起。
14. 如果 Agent 判断抓取失败，必须调用 `set_jetarm_gripper_position(position=370)` 重新松开 J6，再从取图和目标像素查找开始重试。

## 抓取示例

```text
set_jetarm_gripper_position(position=370)
get_rgb_camera_frame()
Agent 只返回 target_x/target_y
control_jetarm_to_target_pixel(target_x=..., target_y=...)

若返回 controller_decision="horizontal_align"：
  重新 get_rgb_camera_frame()
  Agent 重新返回 target_x/target_y
  再调用 control_jetarm_to_target_pixel(...)

若返回 controller_decision="descend_after_alignment"：
  控制程序已下降 2 cm，并返回 FK 高度样本
  重新 get_rgb_camera_frame()
  Agent 重新返回 target_x/target_y
  再调用 control_jetarm_to_target_pixel(...)

到达抓取高度后：
  control_jetarm_gripper(action="grip_lock")
  move_jetarm_home()
  get_rgb_camera_frame()
  Agent 检查抓取是否成功
```
