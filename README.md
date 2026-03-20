# 智能移动技术大作业 - 仿真控制端

本项目是我们小组用于完成智能移动技术大作业的仿真代码仓库。通过与本地 Client 仿真环境进行 UDP 通信，我们将在这里实现机器人的全局路径规划与局部避障控制。

## 环境配置

为了避免依赖冲突，团队统一使用 Conda 管理开发环境。请严格按照以下步骤进行配置：

**1. 克隆代码仓库**
在终端中执行以下命令将代码拉取到本地：
```bash
git clone https://github.com/Wunicheng233/Client-planner.git
cd Client-planner
```

**2. 创建并激活 Conda 环境**
[cite_start]我们的程序要求 Python 版本 >= 3。推荐使用 Python 3.8：
```bash
conda create -n ssl_env python=3.8 -y
conda activate ssl_env
```

**3. 安装核心依赖**
[cite_start]本项目依赖 `protobuf` 进行数据序列化。版本过高会导致运行时直接报错退出。我这里指定安装 `3.20.0` 版本：
```bash
pip install protobuf==3.20.0
```

## 运行与测试

在确认环境配置无误后，可以通过以下步骤跑通基础 Demo：

1. **启动仿真器：** 打开桌面的 `Client` 软件。
2. **连接图像：** 点击右侧的“连接图像”按钮（带有插头图标），并选择测试场景（SIMPLE 或 HARD）
3. **运行主程序：** 在激活了 `ssl_env` 环境的终端中，运行以下命令：
   ```bash
   python main.py
   ```
如果终端成功打印出坐标信息，且 Client 中的蓝车 0 号开始画圈（Debug 测试图形），说明你的环境已完美配置完毕！

## 协作开发说明

我们的核心优化工作主要集中在 `main.py` 中。

* **获取状态：** 通过 `vision.my_robot.x` 等成员变量获取自身与障碍物的实时坐标。
* **核心算法区：** 在 `main.py` 的 `while True` 循环内，找到 `#1. path planning & velocity planning` 注释处 [cite: 261, 272]。这里是我们大展拳脚的地方，你需要在这里实现 A*、RRT 或 DWA 等算法。
* **发送指令：** 计算出目标线速度 `vx` 和角速度 `vw` 后，通过 `action.sendCommand()` 下发指令控制小车移动。

## 常见错误排查 (Troubleshooting)

* **错误 1：Client 运行白屏**
  * **解决：** 在系统 `C:\Windows\System32` 路径下找到 `opengl32.dll`，将其复制并覆盖到 Client 的安装路径下。
* **错误 2：`TypeError: Descriptors cannot not be created directly`**
  * **解决：** 这是 protobuf 版本过高导致的。请执行 `pip install protobuf==3.20.0` 进行降级。
* **错误 3：`OSError: [WinError 10048] 通常每个套接字地址只允许使用一次`**
  * **解决：** 端口被占用。请确保后台没有运行多个 Python 实例，并在 Client 中点击“快速关闭程序”按钮彻底关闭软件后重试。