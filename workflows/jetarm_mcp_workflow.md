---
name: jetarm-mcp-motion
description: Agent识别抓取意图和目标中心像素，控制端执行人工测试V2闭环。
---

# JetArm Agent 抓取工作流规范

## 职责边界

1. Agent 根据用户自然语言自行判断是否要求抓取物品。只有抓取任务才能启动以下闭环。
2. 抓取点像素必须预先保存在接口与抓取点配置中，或由用户在调用前临时覆盖；控制端固定使用该值。未配置时禁止猜测或使用图像中心。
3. Agent 只负责在最新 RGB 原图中识别用户指定物品，并通过 `zoom_rgb_target_tile` 完成数据层递归分块。分块图不绘制标签；程序根据每层原图边界计算物品中心 `target_x/target_y`。坐标原点严格为左上角 `(0,0)`，X向右、Y向下，右下角为 `(width-1,height-1)`。
4. Agent 不得决定机械臂方向、距离、速度、容差、下降阶段或关节姿态。
5. 控制端完整复用人工测试模块 V2：动态容差、按高度变化的像素比例、2 cm 分段下降、最终对准、最终下降、夹取、等待 J6 稳定和 Home。
6. Agent 抓取工作流使用 `camera_vector_terminal_v2`，有效进展检测固定关闭；进展异常仍记录。规划拒绝、没有实际下发运动、串口异常和最终抓取高度安全保护不会关闭。

## 抓取点配置与临时覆盖

推荐在接口配置界面中与机械臂串口、Gemini 相机一起保存抓取点像素：

```text
python3 -m src.jetarm_agent.device_config
```

如需在当前运行期间临时重新标定，可在交互终端使用：

```text
/grasp-point 320 147
```

也可在启动时使用 `--agent-grasp-x` 和 `--agent-grasp-y` 覆盖配置。最终生效值会出现在每张图像的 `camera.grasp_point_pixel` 中。

## Agent 视觉抓取闭环

用户要求初始化时，Agent 调用 `initialize_jetarm`。该工具严格按“J1-J5 回 Home → J6 张开到 350”执行，并重置当前抓取闭环。

1. Agent 判断用户自然语言是否为抓取任务；不是抓取任务时不得调用抓取控制工具。
2. 抓取任务先调用 `get_rgb_camera_frame`。
3. Agent 在最新画面中找到用户指定物品，然后把当前画面在数据层视为 3×3：`row=0`为最上方、`column=0`为最左侧，只选择物品中心所在分块并调用一次 `zoom_rgb_target_tile`。
4. 工具返回选中分块的 JPEG 及其原图坐标边界。Agent 必须先查看返回的新图，再选择下一层；每个模型回合只能选择一层，共完成 4 层。
5. 四层完成后，程序以最终原图边界的中心作为目标像素。Agent 调用 `control_jetarm_to_target_pixel`；即使模型填写了其他 `target_x/target_y`，会话也会强制替换为该分块坐标。Agent不发送或决定机械臂运动方向。
6. 控制端首次动作前自动把 J6 设置为松开位置，然后根据抓取点和目标点执行一次 V2 水平对准或下降。
   V2 水平方向固定以实际抓取点 XYZ 坐标为准：`forward` 使 Y 减小，`backward` 使 Y 增加，`left` 使 X 减小，`right` 使 X 增加。
7. 每次运动结束后旧图像及旧分块路径立即失效；会话自动重新调用 `get_rgb_camera_frame`，把新图交给 Agent。
8. Agent 必须在新图中重新识别同一物品，并重新执行四层分块定位，不得复用旧坐标。
9. 控制端按抓取点 FK 高度选择动态容差：`>15 cm: 40 px`、`>10 cm: 25 px`、`>5 cm: 13 px`、`<=5 cm: 8 px`。
10. 未对准时，控制端按当前高度像素比例计算水平运动；水平运动保持抓取点高度及摄像头—抓取点姿态。
11. 对准后，控制端按 2 cm 阶段下降。接近最终高度时先要求一张新图完成最终对准。
12. 最终对准后，控制端自动下降到最终抓取高度，先执行夹取并持续读取 J6 位置；只有 J6 在容差内稳定后才返回 Home。若 J6 超时未稳定，则停止抓紧并禁止回 Home。此时结果是 `grasp_completion_status="awaiting_visual_verification"`，不能直接宣称成功。
13. 会话自动获取 Home 后画面。Agent必须检查目标物品是否确实被抓起，再调用 `confirm_jetarm_grasp_result(success=...)`。
14. 只有确认工具返回 `grasp_completed=true` 才能结束并报告成功。若确认失败，Agent用当前新图重新识别同一物品并重新分块定位，直到确认成功或出现硬错误。

## 每一步记录

每次 `control_jetarm_to_target_pixel` 的 `grasp_step_record` 和终端输出严格使用以下顺序：

1. `target_pixel`：目标点像素坐标，即目标物块中心。
2. `original_grasp_point_xyz_cm`：动作前原抓取点实际坐标。
3. `motion_plan`：V2 运动规划。
4. `expected_grasp_point_xyz_cm`：预计抓取点坐标。
5. `actual_grasp_point_xyz_cm`：动作后实际抓取点坐标。
6. `camera_grasp_vertical_angle_deg`：摄像头—抓取点连线与竖直方向夹角。

任何工具返回 `status=error` 后必须停止后续动作，不得沿用旧图像或声称抓取成功。
