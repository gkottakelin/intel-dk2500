# GitHub 仓库准备说明

> 建议仓库名：`jetarm-ai-rgbd-servo`  
> 建议仓库内容：提交 `project/` 开发文件夹和依赖说明，不直接提交 `.venv-gemini/` 虚拟环境目录。

## 1. 为什么不建议上传 `.venv-gemini`

虽然当前项目使用了 `.venv-gemini` 虚拟环境，但不建议把它直接放入 GitHub。

原因如下：

| 原因 | 说明 |
|---|---|
| 机器绑定 | 虚拟环境中包含本机 Python 路径，例如 `C:\Users\ASUS\AppData\Local\Programs\Python\Python312` |
| 体积较大 | OpenCV、pyorbbecsdk2 等包包含大量二进制文件 |
| 平台相关 | Windows 虚拟环境不能直接用于 Linux/DK2500 |
| 易损坏 | GitHub 克隆后，虚拟环境里的解释器路径通常会失效 |
| 不利于协作 | 其他机器更适合通过依赖清单重新安装 |

推荐方式是上传：

| 文件 | 用途 |
|---|---|
| `project/requirements.txt` | 项目运行所需依赖 |
| `project/requirements-venv-gemini.txt` | 当前 `.venv-gemini` 已观察到的包版本快照 |
| `project/环境重建说明.md` | 如何重新创建 `.venv-gemini` |
| `project/` | 项目代码、文档、测试、配置 |

## 2. 推荐仓库内容

建议把 `D:\jetarm\project` 作为 Git 仓库根目录。

推荐提交：

| 内容 | 是否提交 |
|---|---|
| `docs/` | 提交 |
| `src/` | 提交 |
| `tests/` | 提交 |
| `config/` | 提交 |
| `launch/` | 提交 |
| `scripts/` | 提交 |
| `项目架构规划/` | 提交 |
| `requirements.txt` | 提交 |
| `requirements-venv-gemini.txt` | 提交 |
| `环境重建说明.md` | 提交 |
| `.venv-gemini/` | 不提交 |
| `__pycache__/` | 不提交 |
| 大体积相机资料包 | 默认不提交，改为在文档中说明路径 |
| 运行过程中保存的大量 RGB-D 样本 | 默认不提交，只提交少量必要样例 |

## 3. 本机当前工具状态

当前本机已经安装：

```powershell
git version 2.51.1.windows.1
```

当前本机未检测到 GitHub CLI：

```powershell
gh : 无法将“gh”项识别为 cmdlet、函数、脚本文件或可运行程序的名称。
```

因此目前可以先初始化本地 Git 仓库；如果要我直接创建 GitHub 远端仓库，需要先安装并登录 GitHub CLI，或提供一个已经创建好的 GitHub 仓库 URL。

## 4. 创建 GitHub 仓库的推荐命令

### 4.1 方式一：使用 GitHub 网页创建仓库后推送

先在 GitHub 网页创建一个空仓库，例如：

```text
https://github.com/<你的用户名>/jetarm-ai-rgbd-servo.git
```

然后在 PowerShell 中执行：

```powershell
cd D:\jetarm\project
git init
git add .
git commit -m "Initial JetArm AI RGB-D servo project"
git branch -M main
git remote add origin https://github.com/<你的用户名>/jetarm-ai-rgbd-servo.git
git push -u origin main
```

### 4.2 方式二：安装 GitHub CLI 后创建并推送

安装并登录 GitHub CLI 后执行：

```powershell
cd D:\jetarm\project
git init
git add .
git commit -m "Initial JetArm AI RGB-D servo project"
gh auth login
gh repo create jetarm-ai-rgbd-servo --private --source . --remote origin --push
```

如果希望仓库公开，把 `--private` 改成 `--public`。

## 5. 外部资料包处理

当前 Gemini Pro Plus Windows 资料包位于：

```text
D:\jetarm\gemini深度相机windows资料
```

其中包含 Orbbec Viewer、OpenNI SDK、示例程序和驱动安装包。这类资料包通常体积较大，并且可能包含厂商二进制文件，建议不直接放入 GitHub 仓库。

推荐处理方式：

| 资料 | 推荐方式 |
|---|---|
| Orbbec Viewer 安装包 | 文档记录来源和本地路径 |
| OpenNI SDK 示例 | 文档记录路径，必要时只提交自己写的封装代码 |
| 运行生成的 RGB-D 样本 | 只提交少量测试样例，大量数据放外部网盘或本地 |
| PDF 手册 | 如体积不大且允许分发，可以单独建立 `docs/vendor/`；否则只记录路径 |

## 6. 推荐提交前检查

提交前建议执行：

```powershell
cd D:\jetarm\project
git status
python -m pytest tests
python src\gemini_windows\diagnose_environment.py
```

如果当前 Python 环境没有安装 `pytest`，可以先执行：

```powershell
pip install pytest
```

