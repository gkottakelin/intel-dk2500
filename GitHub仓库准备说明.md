# GitHub 仓库准备说明

> 建议仓库内容：提交 `project/` 开发文件夹、当前 RGB 视觉闭环方案、ROS2 机械臂描述、源码、测试和依赖说明；不直接提交本地虚拟环境目录。

## 1. 不提交虚拟环境

虚拟环境包含大量平台相关文件、绝对路径和缓存内容，不适合直接提交。

建议提交：

| 文件 | 用途 |
|---|---|
| `project/requirements.txt` | 当前 Python 依赖清单 |
| `project/环境重建说明.md` | 如何重新创建开发环境 |

不建议提交：

| 内容 | 原因 |
|---|---|
| `.venv*/` | 体积大、不可移植 |
| `__pycache__/` | 自动生成缓存 |
| 大量实验图片/视频 | 体积大，建议只提交必要样例 |
| 本地日志 | 与开发机器相关 |

## 2. 推荐仓库内容

| 路径 | 是否提交 | 说明 |
|---|---|---|
| `README.md` | 提交 | 项目入口说明 |
| `docs/` | 提交 | 当前方案和接口说明 |
| `src/` | 提交 | 当前源码 |
| `ros2_ws/src/jetarm_description/` | 提交 | ROS2 机械臂描述包 |
| `tests/` | 提交 | 单元测试 |
| `config/` | 提交 | 配置模板 |
| `requirements.txt` | 提交 | Python 依赖 |
| `.venv*/` | 不提交 | 本地虚拟环境 |
| `data/` 大量样本 | 默认不提交 | 只保留少量必要样例 |

## 3. 推荐 `.gitignore`

```gitignore
.venv*/
__pycache__/
*.pyc
.pytest_cache/
.mypy_cache/
.ruff_cache/

data/raw/
data/videos/
data/tmp/
logs/

build/
install/
log/
```

## 4. 初次提交建议

```powershell
cd D:\jetarm\project
git status
git add README.md docs src tests config ros2_ws 项目架构规划 *.md requirements.txt
git commit -m "Initial JetArm RGB visual servo project"
```

如果仓库根目录是 `D:\jetarm`，则从根目录提交：

```powershell
cd D:\jetarm
git status
git add project
git commit -m "Initial JetArm RGB visual servo project"
```
