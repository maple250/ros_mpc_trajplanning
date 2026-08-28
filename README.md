# MPCC - 无人机 MPC 拦截路径规划器（离线仿真版）

基于 **质点模型（Mass Point Model）** 的 C++ MPC 路径规划器，用于无人机对空中机动目标的追踪 / 迎头拦截。

本分支提供 **离线仿真 `mpc_offline_sim`**：零 ROS / Gazebo / PX4 / MAVROS 依赖，在宿主机上直接构建运行，四旋翼动力学在进程内闭环。在线版（ROS1 Noetic + PX4 SITL + Gazebo，容器环境）见 `feat/docker-build-env` 等分支。

## 1. 离线仿真架构

```
offline_sim_node.cpp           # 主程序：飞行状态机 + 三频率节拍（与在线版同构）
├─ 0.5Hz  目标传感器模拟        # 2s 一次观测，含匀速圆周等机动
├─ 10Hz   KF 融合/预测 + MPC    # 在线规划 + 到达检测
└─ 100Hz  控制回路
     └─ 串级 PID -> AccelToAttitudeController（加速度->姿态+推力，替代 PX4 接口）
        -> drone_dynamic 姿态外环 + 角速度内环 + 混控 -> 四旋翼动力学（100Hz）
```

仿真结束弹出 12 个 matplotlib 分析窗口（与在线版共用同一套绘图代码）。

## 2. 算法模块

| 模块 | 说明 |
|---|---|
| `MPC/` | SQP 式 MPC：状态 6 维 (p, v)、输入 3 维 (a)，双积分器线性模型逐段线性化构造 QP；预测时域 N=30，Ts=0.1s；求解失败重置初始猜测，解含 NaN 时输出悬停安全指令 |
| `Cost/` | 拦截代价：在目标速度方向 `n_t` 上投影构造滑动虚拟导引点，动态前瞻距离 `L = 2‖v‖ + 80`；法向 / 切向 / 速度对齐 / 控制能量加权 |
| `KFfilter/` | 9 维卡尔曼滤波 (p, v, a)：10Hz predict + 0.5Hz update |
| `目标预测` | Singer 模型：目标加速度按 `exp(-Ts/τ)`（τ=1s）指数衰减，长时域退化为匀速直线 |
| `Constraints/` | 状态 / 输入 box 约束（位置 ±2000m，速度 ±25m/s，加速度 ±20m/s²）+ 多面体约束 |
| `Interfaces/` | 基于 **HPIPM / blasfeo** 的 QP 求解接口 |
| `controller/` | 串级 PID（位置环 -> 速度环），对 MPC 轨迹做时间插值跟踪 |
| `InnerLoop/` | 加速度指令 -> 期望姿态 + 总推力的转换器（含倾角限幅） |
| `Plotting/` | matplotlib-cpp 事后绘图（程序退出时自动绘制） |

目标参数、代价权重、约束边界均在 `src/mpc_planning/Params/*.json` 中配置；初始场景见 `Params/stateInitialization.json`。四旋翼动力学参数见 `External/drone_dynamic/Params/drone_params.json`。

## 3. 依赖（宿主机，Ubuntu 24.04 验证通过）

- 系统包：`cmake`、`build-essential`、`python3-dev`、`python3-numpy`、`python3-matplotlib`、`python3-tk`（TkAgg 出图需要 X 环境）
- 第三方库位于 `src/mpc_planning/External/`（**不入版本库**）：Eigen、nlohmann/json、matplotlib-cpp、blasfeo、hpipm、drone_dynamic，由 `install.sh` 自动拉取/编译（幂等）
- `drone_dynamic` 默认从 `~/drone_ws/drone_dynamic` 本地克隆，缺失时回退 GitHub（maple250/drone_dynamic）

```bash
sudo apt install cmake build-essential python3-dev python3-numpy \
                 python3-matplotlib python3-tk
bash src/mpc_planning/install.sh
```

## 4. 构建与运行

```bash
# 构建（无需 ROS；若环境里有 ROS/catkin，ROS 在线节点也不会被牵连）
cmake -S src/mpc_planning -B src/mpc_planning/build-native
cmake --build src/mpc_planning/build-native -j$(nproc)

SIM=src/mpc_planning/build-native/mpc_offline_sim
$SIM --selftest    # 转换器三断言（悬停/前倾/倾角限幅）后退出
$SIM --hover-only  # 起飞->悬停5s->+20m 阶跃验证，不接 MPC，不弹窗
$SIM               # 完整拦截仿真，结束弹 12 个绘图窗口
```

用法：`mpc_offline_sim [Params目录] [drone_params.json] [--selftest] [--hover-only]`（参数路径默认为编译期宏，仅在需要换参数时传入）。无头环境跑冒烟可设 `MPLBACKEND=Agg`。

## 5. 已知局限

- 目标做圆形、上下起伏等机动时，规划效果变差（Singer 模型假设加速度指数衰减，难以持续跟踪周期性机动）。

## 6. 默认整定参数（`Params/cost.json`）

```json
"q_l" : 2.7,
"q_c" : 4.5,
"q_v" : 0.3,
"q_a" : 0.005
```
