---
name: jetarm-mcp-motion
description: 将自然语言机械臂指令转换为 JetArm MCP 调用，并按真实工具回执总结。
---

# JetArm MCP 工作流规范

## 通用控制规则

1. 只有用户明确要求移动、抓取、回 Home、停止、旋转腕部、控制夹爪或查看画面时，才调用会改变机械臂状态的工具。
2. 每次笛卡尔移动前必须调用 `get_rgb_camera_frame`，让 Agent 看到最新 RGB 画面和同一时刻的 `arm_pose`。
3. 每张 RGB 图像只允许驱动紧随其后的一条移动命令。机械臂移动后旧图立即失效。
4. `move_jetarm` 的单条移动距离必须严格小于 `2 cm`。控制程序只执行当前收到的一条命令，不替 Agent 自动拆分长距离。
5. 普通移动未指定速度时使用 `1.5 cm/s`；用户指定速度时必须在工具允许范围内。
6. 任一工具返回 `status=error` 后立即停止后续动作；存在运动风险时调用 `stop_jetarm`。
7. 最终总结必须以 MCP 工具真实返回值为依据，不得虚构视觉结果、位置或完成状态。

## 视觉抓取物块工作流

当用户要求抓取物块、方块、积木、目标物体或 block/cube/object 时，必须使用以下闭环流程。

1. 调用 `set_jetarm_gripper_position(position=370)`，在抓取成功前让 J6 保持松开状态。
2. 调用 `get_rgb_camera_frame` 获取最新画面。Agent 必须在画面中分别找出物块中心点像素和抓取点像素。
3. 将这两个像素传给 `move_jetarm_by_pixel_error`：
   - `block_center_x/block_center_y` 为物块中心像素。
   - `grasp_point_x/grasp_point_y` 为抓取点像素。
   - 对齐容差为 `±10 px`。
   - 像素误差控制速度范围固定为 `0.5..1.5 cm/s`。
   - `dx>0` 向右，`dx<0` 向左，`dy>0` 向后，`dy<0` 向前。
4. 如果 `move_jetarm_by_pixel_error` 返回 `aligned=false`，必须等待工具返回 `status=ok` 后重新取图，再由 Agent 用新图重新找像素并继续对准。
5. 如果两个像素点重合在 `±10 px` 内，`move_jetarm_by_pixel_error` 返回 `aligned=true`；此时开始下降阶段。
6. 下降阶段速度固定为 `2 cm/s`。下降时要读取 `get_jetarm_state` 中的抓取点高度 `tcp_cm.up_z`。
7. 每累计下降 `3 cm`，必须再次调用 `get_rgb_camera_frame` 触发一次 AI 校准，然后回到像素对准步骤。
8. 当高度第一次 `<=2 cm` 时，必须再调用一次 `get_rgb_camera_frame` 触发 AI 校准，然后回到像素对准步骤。
9. 完成 `<=2 cm` 的校准后，下降到 `1 cm` 处执行抓取。
10. 抓取动作使用 `control_jetarm_gripper(action="grip_lock")`。
11. 抓取完成后调用 `move_jetarm_home` 复位。
12. 复位后调用 `get_rgb_camera_frame`，由 Agent 检查物块是否被成功抓起。
13. 如果 AI 判断抓取失败，必须调用 `set_jetarm_gripper_position(position=370)` 重新松开 J6，再从取图和像素对准开始重试。

## 抓取示例

```text
get_rgb_camera_frame()
Agent 找到物块中心像素和抓取点像素
set_jetarm_gripper_position(position=370)
move_jetarm_by_pixel_error(block_center_x=..., block_center_y=..., grasp_point_x=..., grasp_point_y=...)
若 aligned=false：重新 get_rgb_camera_frame 后继续像素对准
若 aligned=true：get_jetarm_state() 读取 tcp_cm.up_z
move_jetarm(command="下1.9", speed_cm_s=2.0)
每累计下降 3 cm 或首次高度 <=2 cm：get_rgb_camera_frame() 重新 AI 校准
下降到 1 cm：control_jetarm_gripper(action="grip_lock")
move_jetarm_home()
get_rgb_camera_frame()
Agent 检查抓取是否成功；失败则 set_jetarm_gripper_position(position=370) 后重试
```
