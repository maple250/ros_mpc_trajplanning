# MPCC — 无人机 MPC 拦截路径规划器

基于 **质点模型（Mass Point Model）** 的 C++ MPC 路径规划器，用于无人机对空中机动目标的追踪 / 迎头拦截。在 ROS1 (Noetic) 下与 PX4 SITL + Gazebo Classic 仿真器联合运行，机架为默认 `iris`，通过 MAVROS 与飞控通信。

## 1. 系统架构

```
mpc_planning_node.cpp          # 主节点：飞行状态机 + 三频率定时器
├─ 0.5Hz  targetSensorTimer    # 模拟目标传感器（2s 一次观测，含转弯机动）
├─ 10Hz   mpcPlannerTimer      # KF 融合/预测 + MPC 在线规划 + 到达检测
└─ 100Hz  controlLoopTimer     # 轨迹时间插值 + 串级 PID -> MAVROS 加速度指令
```

- **飞行状态机**：`WAIT_FOR_CONNECTION → TAKEOFF → TRACKING_MPC → EMERGENCY_HOVER`
  （Offboard 丢失或判定拦截成功后自动切换到位置悬停）
- **多线程**：`AsyncSpinner(3)`，10Hz MPC 求解不阻塞 100Hz 控制回路，`mutex` 保护共享轨迹包

## 2. 算法模块

| 模块 | 说明 |
|---|---|
| `MPC/` | SQP 式 MPC：状态 6 维 (p, v)、输入 3 维 (a)，双积分器线性模型逐段线性化构造 QP；预测时域 N=30，Ts=0.1s；求解失败重置初始猜测，解含 NaN 时输出悬停安全指令 |
| `Cost/` | 拦截代价：在目标速度方向 `n_t` 上投影构造滑动虚拟导引点，动态前瞻距离 `L = 2‖v‖ + 80`；法向 / 切向 / 速度对齐 / 控制能量加权 |
| `KFfilter/` | 9 维卡尔曼滤波 (p, v, a)：10Hz predict + 0.5Hz update |
| `目标预测` | Singer 模型：目标加速度按 `exp(-Ts/τ)`（τ=1s）指数衰减，长时域退化为匀速直线 |
| `Constraints/` | 状态 / 输入 box 约束（位置 ±2000m，速度 ±25m/s，加速度 ±20m/s²）+ 多面体约束 |
| `Interfaces/` | 基于 **HPIPM / blasfeo** 的 QP 求解接口 |
| `controller/` | 串级 PID（位置环 → 速度环），对 MPC 轨迹做时间插值跟踪 |
| `Plotting/` | matplotlib-cpp 事后绘图（节点退出时自动绘制） |

目标参数、代价权重、约束边界均在 `src/mpc_planning/Params/*.json` 中配置；初始场景（目标位置 / 速度等）见 `Params/stateInitialization.json` 与 `mpc_planning_node.cpp`。

## 3. 依赖

- ROS1 Noetic、PX4 SITL (Gazebo Classic)、MAVROS
- 第三方库位于 `src/mpc_planning/External/`（**不入版本库**，需自行放置）：
  Eigen、nlohmann/json、blasfeo、hpipm、matplotlib-cpp
- 部分依赖可由 `src/mpc_planning/install.sh` 拉取（目前仅 matplotlib-cpp 的克隆生效，blasfeo/hpipm 的编译步骤已注释，需按脚本内注释手动编译为静态库）

## 4. 编译与启动

```bash
catkin_make                  # 工作空间根目录
roslaunch mpcplanning intercept_mpc.launch
```

可视化界面可在 `.launch` 文件中修改以下一行开启或关闭（以下为关闭示例，开启改为 `true`）：

```xml
<arg name="gui" value="false"/>
```

## 5. 已知局限

- 目标做圆形、上下起伏等机动时，规划效果变差（Singer 模型假设加速度指数衰减，难以持续跟踪周期性机动）。

## 6. 默认整定参数（`Params/cost.json`）

```json
"q_l" : 2.7,
"q_c" : 4.5,
"q_v" : 0.3,
"q_a" : 0.005
```
