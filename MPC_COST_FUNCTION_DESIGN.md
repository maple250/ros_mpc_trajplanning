# 三维拦截 MPC 规划器：模型与代价函数的数学描述

> 本文档基于当前代码实现（`src/mpc_planning/`）整理，完整给出拦截 MPC 规划器的设计思路、
> 运动学模型、目标预测模型、代价函数各分项的数学表达及其在 QP（二次规划）中的装配形式。
> 所有权重取值以 `Params/cost.json`、`Params/bounds.json`、`config.h` 当前值为准。

---

## 目录

1. [总体设计思路](#1-总体设计思路)
2. [符号与变量定义](#2-符号与变量定义)
3. [本机运动模型与精确 ZOH 离散化](#3-本机运动模型与精确-zoh-离散化)
4. [目标估计与预测模型](#4-目标估计与预测模型)
5. [交战几何量定义](#5-交战几何量定义)
6. [最优控制问题（OCP）的 QP 表述](#6-最优控制问题ocp的-qp-表述)
7. [代价函数设计（核心）](#7-代价函数设计核心)
8. [距离调度权重与分段拦截策略](#8-距离调度权重与分段拦截策略)
9. [约束与变量归一化](#9-约束与变量归一化)
10. [RTI/SQP 实时求解流程](#10-rtisqp-实时求解流程)
11. [参数汇总表](#11-参数汇总表)
12. [实现细节与注意事项](#12-实现细节与注意事项)

---

## 1. 总体设计思路

本规划器面向**三维空间中对机动目标的自主拦截**任务，核心思想是：

**将经典制导律（比例导引 PN / 增广比例导引 APN、追踪法）的先验知识，以变权重软代价的形式嵌入模型预测控制（MPC）框架**，用一系列线性时变 QP 逼近原始非线性拦截问题，从而同时获得：

- **制导律的确定性与鲁棒性**——远场由 APN 项主导，保证接近最优的拦截几何；
- **优化的灵活性**——所有制导先验均为软代价而非硬约束，可与输入惩罚、位置收敛等目标在线权衡；
- **分段拦截策略**——通过相对距离 $\rho$ 调度的 tanh 类开关权重，实现"远场高速逼近 + APN 中制导 → 近场位置误差收敛 + 速度跟踪"的行为切换；
- **几何自适应**——通过视线（LOS）与目标速度夹角 $\theta$ 的高斯权重，区分迎头（head-on）与尾追（tail-chase）态势并调整速度策略。

系统采用**分层异步架构**：

```
目标量测 (0.5 Hz, 位置+速度)
        │
        ▼
9 维 Kalman 滤波器 (10 Hz 预测/更新, 输出 p_t, v_t, a_t)
        │
        ▼
MPC 轨迹规划器 (10 Hz, 预测时域 N=30, Ts=0.1 s, HPIPM 求解 QP)
        │   输出: 预测时域内 p/v/a 轨迹 (TrackPackage)
        ▼
PID 轨迹跟踪控制器 (100 Hz, 轨迹插值 → 加速度指令)
        │
        ▼
MAVROS / 飞控 (加速度 setpoint)
```

拦截判定：当相对距离 $\rho \le 350\,\mathrm{m}$ 时认为目标进入交接范围，规划器停止并悬停，交由末制导接管。

---

## 2. 符号与变量定义

### 2.1 本机（拦截方）状态与输入

$$
x = \begin{bmatrix} p \\ v \end{bmatrix} \in \mathbb{R}^{6}, \qquad
p = \begin{bmatrix} p_x \\ p_y \\ p_z \end{bmatrix}, \quad
v = \begin{bmatrix} v_x \\ v_y \\ v_z \end{bmatrix}, \qquad
u = a = \begin{bmatrix} a_x \\ a_y \\ a_z \end{bmatrix} \in \mathbb{R}^{3}
$$

其中 $p$ 为位置、$v$ 为速度，控制输入 $u$ 为**加速度指令**。各通道动力学解耦，构成带一阶线性空气阻尼的双积分器链。

### 2.2 目标状态

$$
x_t = \begin{bmatrix} p_t \\ v_t \\ a_t \end{bmatrix} \in \mathbb{R}^{9}
$$

由卡尔曼滤波器估计得到；预测时域内按衰减加速度模型外推（见第 4 节）。目标航向单位向量：

$$
\hat{n}_t = \begin{cases} \dfrac{v_t}{\|v_t\|}, & \|v_t\| > 10^{-3} \\[2mm] \mathbf{0}, & \text{否则} \end{cases}
$$

### 2.3 主要符号一览

| 符号 | 含义 | 定义位置 |
|---|---|---|
| $r$ | 视线（LOS）相对位置矢量，$r = p_t - p$ | §5 |
| $\rho$ | 相对距离，$\rho = \|r\|$ | §5 |
| $\hat{r}$ | LOS 方向单位矢量 | §5 |
| $v_{\mathrm{rel}}$ | 相对速度，$v_{\mathrm{rel}} = v - v_t$ | §5 |
| $\dot{\lambda}$ | LOS 转率（角速度大小） | §5 |
| $\theta$ | LOS 与目标速度的夹角（态势角） | §5 |
| $P_{\parallel},\,P_{\perp}$ | 沿 / 垂直目标航向的投影算子 | §5 |
| $T_s$ | 离散步长（$0.1\,\mathrm{s}$） | `config.h` |
| $N$ | 预测时域步数（$30$，即 $3\,\mathrm{s}$） | `config.h` |
| $c$ | 线性空气阻尼系数（$0.2\,\mathrm{s^{-1}}$） | `config.h` |
| $w_{\mathrm{near}}(\rho)$ | 近场调度权重 | §8 |
| $w_{\mathrm{far}}(\rho)$ | 远场调度权重 | §8 |
| $w_{\mathrm{spd}}(\theta)$ | 态势角高斯权重 | §7.2 |

---

## 3. 本机运动模型与精确 ZOH 离散化

### 3.1 连续时间模型

三个通道独立、形式相同的线性定常系统（位置-速度-加速度链 + 一阶阻尼）：

$$
\dot{p} = v, \qquad \dot{v} = u - c\,v, \qquad c = 0.2\,\mathrm{s^{-1}}
$$

阻尼项 $-cv$ 刻画飞行器自然空气阻力：零输入时速度按 $e^{-ct}$ 指数衰减，稳态速度增益为 $1/c$。

写成状态空间形式（$\otimes$ 表示对每个三维通道取单位阵）：

$$
\dot{x} = \mathcal{A} x + \mathcal{B} u, \qquad
\mathcal{A} = \begin{bmatrix} 0 & I_3 \\ 0 & -c\,I_3 \end{bmatrix}, \quad
\mathcal{B} = \begin{bmatrix} 0 \\ I_3 \end{bmatrix}
$$

### 3.2 精确零阶保持（ZOH）离散化

由于模型线性定常，离散化可解析完成。定义三个标量系数：

$$
\alpha \triangleq e^{-c\,T_s}, \qquad
b_v \triangleq \frac{1-\alpha}{c}, \qquad
b_p \triangleq \frac{T_s - b_v}{c}
$$

则精确 ZOH 离散模型为：

$$
\boxed{\;x_{k+1} = A_d\, x_k + B_d\, u_k \;}
$$

$$
A_d = \begin{bmatrix} I_3 & b_v\, I_3 \\ 0 & \alpha\, I_3 \end{bmatrix}, \qquad
B_d = \begin{bmatrix} b_p\, I_3 \\ b_v\, I_3 \end{bmatrix}
$$

**推导要点**：状态转移矩阵 $e^{\mathcal{A}t}$ 的右上块为 $\int_0^{t} e^{-c s}\,\mathrm{d}s = \frac{1-e^{-ct}}{c}$，即得 $A_d$ 的 $(1,2)$ 块 $b_v$；输入矩阵位置块为 $\int_0^{T_s} \frac{1-e^{-c\tau}}{c}\,\mathrm{d}\tau = \frac{T_s}{c} - \frac{1-\alpha}{c^2} = \frac{T_s - b_v}{c}$，即 $b_p$。

物理意义（与欧拉离散对比）：

- $b_v$ 同时是"速度对输入"与"位置对速度"的增益；
- $b_p$ 是"位置对输入"的增益——**欧拉离散下此项恒为 $0$**，即输入要经过两拍才影响位置；精确离散化保留了该耦合，避免了规划轨迹的系统相位滞后；
- 当 $c \to 0$ 时模型退化为纯双积分器的精确离散化：$A_d = \begin{bmatrix} I & T_s I \\ 0 & I \end{bmatrix}$，$B_d = \begin{bmatrix} \tfrac{1}{2}T_s^2 I \\ T_s I \end{bmatrix}$（代码中保留了该退化分支）。

代入数值（$c=0.2$，$T_s=0.1$）：$\alpha \approx 0.9802$，$b_v \approx 0.0990$，$b_p \approx 0.00497$。

---

## 4. 目标估计与预测模型

### 4.1 目标状态估计（卡尔曼滤波）

滤波器状态 $x_t = [p_t^T,\, v_t^T,\, a_t^T]^T \in \mathbb{R}^9$，采用匀加速（CA）运动学传播：

**预测步**（步长 $\mathrm{d}t$）：

$$
\bar{x}_t = F(\mathrm{d}t)\, x_t, \qquad
F(\mathrm{d}t) = \begin{bmatrix} I_3 & \mathrm{d}t\, I_3 & \tfrac{1}{2}\mathrm{d}t^2\, I_3 \\ 0 & I_3 & \mathrm{d}t\, I_3 \\ 0 & 0 & I_3 \end{bmatrix}, \qquad
\bar{P} = F P F^{T} + Q
$$

**更新步**（0.5 Hz 量测 $z = [p_{t,\mathrm{meas}}^T,\, v_{t,\mathrm{meas}}^T]^T$）：

$$
K = \bar{P} H^T \left( H \bar{P} H^T + R \right)^{-1}, \qquad
x_t^{+} = \bar{x}_t + K\left( z - H \bar{x}_t \right), \qquad
P^{+} = (I - K H)\,\bar{P}
$$

过程噪声中加速度分量方差取大值（$50$），使滤波器能通过加速度残差快速响应目标机动；位置/速度分量方差取小值（$0.1$），保持运动学传播的可信度。

### 4.2 预测时域内的目标外推（衰减加速度模型）

MPC 时域内（$k = 0, 1, \dots, N$）采用 Singer 型衰减加速度模型外推目标——机动加速度按时间常数 $\tau_m$ 指数衰减，预测轨迹平滑过渡为匀速直线：

$$
\begin{aligned}
p_{t,k+1} &= p_{t,k} + v_{t,k}\, T_s + \tfrac{1}{2}\, a_{t,k}\, T_s^2 \\
v_{t,k+1} &= v_{t,k} + a_{t,k}\, T_s \\
a_{t,k+1} &= e^{-T_s / \tau_m}\, a_{t,k}, \qquad \tau_m = 1.0\,\mathrm{s}
\end{aligned}
$$

初始值 $p_{t,0}, v_{t,0}, a_{t,0}$ 取自滤波器输出；衰减因子 $e^{-T_s/\tau_m} \approx 0.905$。

该预测模型与代价函数配合：**代价不惩罚沿目标航向的位置滞后**（见 §7.1），因此目标机动的不确定性主要转化为沿航向误差，其对脱靶量的影响由近场项在后续规划周期中持续修正。

---

## 5. 交战几何量定义

以下几何量在每个预测步 $k$ 处、基于该步的本机线性化状态与目标预测状态计算。

**视线（LOS）**：

$$
r = p_t - p, \qquad \rho = \|r\|, \qquad
\hat{r} = \begin{cases} \dfrac{r}{\rho}, & \rho > 10^{-3} \\[2mm] [1,0,0]^T, & \text{否则（数值保护）} \end{cases}
$$

**相对速度与 LOS 转率**：

$$
v_{\mathrm{rel}} = v - v_t
$$

LOS 角速度矢量与转率大小（经典比例导引中的 $\dot{\lambda}$）：

$$
\omega_{\mathrm{LOS}} = \frac{r \times v_{\mathrm{rel}}}{\rho^{2}}, \qquad
\dot{\lambda} = \frac{\| r \times v_{\mathrm{rel}} \|}{\rho^{2}}
$$

**态势角**（LOS 方向与目标速度方向的夹角）：

$$
\theta = \arccos\!\left( \frac{\hat{r}^{T} v_t}{\|v_t\|} \right) \in [0, \pi]
$$

- $\theta \to \pi$：**迎头**（目标朝我方飞来），接近速度快；
- $\theta \to 0$：**尾追**（目标背离我方），接近速度受限于本机最大速度。

当 $\rho$ 或 $\|v_t\|$ 过小无法求解时，按 $\theta = \pi$（迎头）处理，即取最保守的强加速策略。

**投影算子**（沿/垂直目标航向）：

$$
P_{\parallel} = \hat{n}_t\, \hat{n}_t^{T}, \qquad
P_{\perp} = I_3 - \hat{n}_t\, \hat{n}_t^{T}
$$

---

## 6. 最优控制问题（OCP）的 QP 表述

在每个规划周期，以当前本机状态 $x_0$ 为初值，求解如下 OCP（阶段代价的具体构成见 §7）：

$$
\min_{\substack{x_1,\dots,x_N \\ u_0,\dots,u_{N-1}}}
\; \sum_{k=1}^{N} \left( \tfrac{1}{2}\, x_k^{T} Q_k\, x_k + q_k^{T} x_k \right)
\; + \; \sum_{k=0}^{N-1} \left( \tfrac{1}{2}\, u_k^{T} R\, u_k + r_k^{T} u_k \right)
$$

约束条件：

$$
\begin{aligned}
\text{动力学：} \quad & x_{k+1} = A_d\, x_k + B_d\, u_k, && k = 0, \dots, N-1 \\
\text{箱式约束：} \quad & \underline{x} \le x_k \le \overline{x}, && k = 1, \dots, N \\
& \underline{u} \le u_k \le \overline{u}, && k = 0, \dots, N-1
\end{aligned}
$$

说明：

- 阶段代价采用 HPIPM 约定 $\tfrac{1}{2}z^{T} W z + w^{T} z$；$k=0$ 处状态固定（接口中该级 $n_x = 0$，$x_1 = A_d x_0 + B_d u_0$），故其状态代价自然消失；$k=N$ 处无输入（$n_u = 0$），阶段代价兼作终态代价（无单独终端权重）。
- 权重矩阵 $Q_k,\, q_k,\, r_k$ 依赖第 $k$ 步的目标预测状态与态势几何，因此沿时域**逐阶段变化**（时变 QP）；其中速度范数项经线性化后还依赖本机线性化点（见 §7.2），这是外层 SQP 迭代的非线性来源。
- QP 实际在归一化空间中数值求解（见 §9），装配矩阵已完成相似变换。

---

## 7. 代价函数设计（核心）

单阶段概念代价由五个分项构成：

$$
J_k = J_c + J_{\mathrm{spd}} + J_{\mathrm{apn}} + J_{\mathrm{trk}} + J_{u}
$$

| 分项 | 名称 | 作用 | 激活范围 |
|---|---|---|---|
| $J_c$ | 法向位置误差收敛项 | 消除垂直于目标航向的脱靶分量 | 近场（$w_{\mathrm{near}}$） |
| $J_{\mathrm{spd}}$ | 速度大小奖励项 | 迎头态势下鼓励高速逼近 | 迎头（$\theta \approx \pi$） |
| $J_{\mathrm{apn}}$ | APN 速度增量对齐项 | 中制导段按增广比例导引修正弹道 | 远场（$w_{\mathrm{far}}$） |
| $J_{\mathrm{trk}}$ | 速度跟踪二次惩罚项 | 驱动速度指向目标的追踪参考方向 | 近场（$w_{\mathrm{near}}$） |
| $J_{u}$ | 加速度指令惩罚项 | 输入正则化，抑制过机动 | 全程 |

以下逐项给出**概念数学形式**与**QP 实际装配矩阵**（对照 `Cost/cost.cpp`）。

### 7.1 分项一：法向位置误差收敛项 $J_c$

**物理动机**：终端拦截只关心脱靶量。将位置误差投影到垂直于目标航向的平面内惩罚，允许沿航向的位置滞后（追赶目标沿航向位置需要速度对齐，代价高昂且无必要），集中消除法向（横向）偏差——这是脱靶量的直接来源。

$$
e_{\perp} = P_{\perp}\left( p - p_t \right), \qquad
J_c = q_c \; w_{\mathrm{near}}(\rho)\; \big\| e_{\perp} \big\|^{2}
$$

**QP 装配**（写入 $Q$ 的位置块与 $q$ 的位置分量；按实现，完整二次型展开应为 $q^{(c)} = -Q^{(c)} p_t$，代码中额外乘了一次调度权重，见 §12）：

$$
Q^{(c)} = q_c\, w_{\mathrm{near}}\, P_{\perp} \;\; \left(\to Q[0{:}3,\,0{:}3]\right), \qquad
q^{(c)} = -\,Q^{(c)}\, w_{\mathrm{near}}\; p_t \;\; \left(\to q[0{:}3]\right)
$$

### 7.2 分项二：速度大小奖励项 $J_{\mathrm{spd}}$（态势角高斯变权重）

**物理动机**：迎头（$\theta \to \pi$）时接近速度由双方速度叠加决定，本机加速对缩短交战时间收益极大；尾追（$\theta \to 0$）时接近速度受本机最大速度限制，盲目加速收益有限。故用中心在 $\theta = \pi$ 的高斯函数调度奖励强度。

权重函数：

$$
w_{\mathrm{spd}}(\theta) = q_{\mathrm{vmag}} \exp\!\left( -\,\frac{\left(\theta - \pi\right)^{2}}{2\,\sigma_v^{2}} \right)
$$

概念代价（**奖励**速度大小，非惩罚）：

$$
J_{\mathrm{spd}} = -\; w_{\mathrm{spd}}(\theta)\; \|v\|
$$

**线性化**：$\|v\|$ 非线性，在 SQP 线性化点 $v_{\mathrm{lin}}$ 处一阶展开：

$$
\|v\| \;\approx\; \|v_{\mathrm{lin}}\| + \hat{v}_{\mathrm{lin}}^{T}\left( v - v_{\mathrm{lin}} \right), \qquad
\hat{v}_{\mathrm{lin}} = \frac{v_{\mathrm{lin}}}{\|v_{\mathrm{lin}}\|}
$$

丢弃常数项后只剩线性项进入 $q$ 的速度分量：

$$
q^{(\mathrm{spd})} = -\; w_{\mathrm{spd}}\; \hat{v}_{\mathrm{lin}} \;\; \left(\to q[3{:}6]\right), \qquad
\hat{v}_{\mathrm{lin}} \to \hat{r} \;\; \text{当 } \|v_{\mathrm{lin}}\| \le 10^{-3} \text{（沿 LOS 鼓励加速）}
$$

### 7.3 分项三：APN 速度增量对齐项 $J_{\mathrm{apn}}$

**物理动机**：将经典增广比例导引律（Augmented Proportional Navigation）作为软先验注入。由于控制输入即加速度，且 $u_k \approx \Delta v_k / T_s$，奖励输入方向与 APN 指令方向对齐，等价于在速度增量层面复现比例导引的几何收敛性质。

APN 指令（矢量形式）：

$$
a_{\mathrm{APN}} = N_{\mathrm{apn}}\; \|v_{\mathrm{rel}}\|\; \dot{\lambda}\; \hat{n}_{\perp}, \qquad
\hat{n}_{\perp} = \frac{r \times v_{\mathrm{rel}}}{\left\| r \times v_{\mathrm{rel}} \right\|}
$$

其中 $\hat{n}_{\perp}$ 垂直于 LOS、位于交战平面内，$N_{\mathrm{apn}} = 3.5$ 为导引系数。

概念代价（奖励对齐）：

$$
J_{\mathrm{apn}} = -\; w_{\mathrm{far}}(\rho)\; T_s\; a_{\mathrm{APN}}^{T}\, u
$$

**QP 装配**（写入输入线性项；$\|r \times v_{\mathrm{rel}}\| > 10^{-6}$ 时才启用，避免 LOS 转率退化方向数值爆炸）：

$$
r^{(\mathrm{apn})} = -\; w_{\mathrm{far}}(\rho)\; T_s\; a_{\mathrm{APN}}
$$

### 7.4 分项四：速度跟踪二次惩罚项 $J_{\mathrm{trk}}$

**物理动机**：构造一个"指向目标的追踪参考速度" $v_{\mathrm{ref}}$（大小固定为接近速度 $V_{\mathrm{app}}$，方向由投影几何确定），用二次惩罚驱动本机速度对齐，实现近场的逼近/追踪行为。

**参考速度方向的构造**（投影几何，追击/迎头自适应）：将本机位置投影到目标航向直线上，取"投影点 → 目标"的横向矢量作为参考方向：

$$
p_{\mathrm{proj}} = p_t + \left( \hat{n}_t^{T} \left( p - p_t \right) \right) \hat{n}_t, \qquad
d_{\perp} = p_t - p_{\mathrm{proj}}
$$

（$\hat{n}_t = \mathbf{0}$ 时取 $p_{\mathrm{proj}} = p$。）

$$
v_{\mathrm{ref}} =
\begin{cases}
V_{\mathrm{app}}\; \dfrac{d_{\perp}}{\|d_{\perp}\|}, & \|d_{\perp}\| > 10^{-3} \quad \text{（一般：指向目标横向偏差方向）} \\[3mm]
+\,V_{\mathrm{app}}\; \hat{n}_t, & \text{退化且沿航向滞后}\ \Rightarrow\ \text{尾追：沿目标航向追} \\[2mm]
-\,V_{\mathrm{app}}\; \hat{n}_t, & \text{退化且沿航向超前}\ \Rightarrow\ \text{迎头：逆目标航向对飞}
\end{cases}
$$

其中 $V_{\mathrm{app}} = 25\,\mathrm{m/s}$（当前硬编码于 `cost.cpp`）。

概念代价与 QP 装配：

$$
J_{\mathrm{trk}} = q_v\; w_{\mathrm{near}}(\rho)\; \big\| v - v_{\mathrm{ref}} \big\|^{2}
$$

$$
Q^{(\mathrm{trk})} = q_v\, w_{\mathrm{near}}\, I_3 \;\; \left(\to Q[3{:}6,\,3{:}6]\right), \qquad
q^{(\mathrm{trk})} = -\,Q^{(\mathrm{trk})}\, v_{\mathrm{ref}} \;\; \left(\to q[3{:}6]\right)
$$

### 7.5 分项五：加速度指令惩罚项 $J_u$

**物理动机**：输入正则化——抑制不必要的剧烈机动，保证 QP 海森矩阵正定、节省控制余量。

$$
J_u = \tfrac{1}{2}\, q_a\, \|u\|^{2}
\qquad\Longrightarrow\qquad
R = q_a\, I_3
$$

### 7.6 总装配

将各分项叠加（位置/速度/输入块），单阶段 QP 数据为：

$$
Q_k = \begin{bmatrix} q_c\, w_{\mathrm{near}}\, P_{\perp} & 0 \\ 0 & q_v\, w_{\mathrm{near}}\, I_3 \end{bmatrix} + \varepsilon I_6, \qquad
q_k = \begin{bmatrix} -\,q_c\, w_{\mathrm{near}}^{2}\, P_{\perp}\, p_{t,k} \\[1mm] -\,w_{\mathrm{spd}}(\theta_k)\, \hat{v}_{\mathrm{lin},k} \;-\; q_v\, w_{\mathrm{near}}\, v_{\mathrm{ref},k} \end{bmatrix}
$$

$$
R = q_a\, I_3, \qquad
r_k = -\, w_{\mathrm{far}}(\rho_k)\; T_s\; a_{\mathrm{APN},k}
$$

其中 $\varepsilon = 10^{-12}$ 为正则化常数，保证 $Q_k$ 严格正定；$w_{\mathrm{near}},\, w_{\mathrm{far}},\, \theta_k,\, p_{t,k},\, v_{\mathrm{ref},k},\, a_{\mathrm{APN},k}$ 均按第 $k$ 步的目标预测状态与本机线性化状态逐阶段计算。

---

## 8. 距离调度权重与分段拦截策略

两个 tanh 类开关权重实现远/近场行为切换（过渡中心 $\rho_v = \rho_{dv} = 1200\,\mathrm{m}$，过渡宽度 $k_v = k_{dv} = 400\,\mathrm{m}$）：

$$
w_{\mathrm{near}}(\rho) = \tfrac{1}{2}\left( 1 - \tanh\frac{\rho - \rho_v}{k_v} \right)
\qquad\Longrightarrow\qquad
\begin{cases} \rho \ll \rho_v: & w_{\mathrm{near}} \to 1 \quad \text{（近场激活）} \\ \rho \gg \rho_v: & w_{\mathrm{near}} \to 0 \quad \text{（远场关闭）} \end{cases}
$$

$$
w_{\mathrm{far}}(\rho) = \tfrac{1}{2}\left( 1 + \tanh\frac{\rho - \rho_{dv}}{k_{dv}} \right)
\qquad\Longrightarrow\qquad
\begin{cases} \rho \ll \rho_{dv}: & w_{\mathrm{far}} \to 0 \quad \text{（近场关闭）} \\ \rho \gg \rho_{dv}: & w_{\mathrm{far}} \to 1 \quad \text{（远场激活）} \end{cases}
$$

由此形成的**分段拦截策略**：

| 阶段 | 相对距离 | 主导代价项 | 行为 |
|---|---|---|---|
| 中制导（远场） | $\rho \gtrsim 1200\,\mathrm{m}$ | $J_{\mathrm{apn}}$、$J_{\mathrm{spd}}$ | APN 弹道修正 + 迎头高速逼近 |
| 过渡带 | $\rho \approx 1200\,\mathrm{m}$ | 各项渐变切换 | tanh 平滑过渡，无跳变 |
| 末段（近场） | $\rho \lesssim 1200\,\mathrm{m}$ | $J_c$、$J_{\mathrm{trk}}$ | 法向脱靶收敛 + 追踪参考速度对齐 |
| 全程 | — | $J_u$ | 输入正则化 |

注意：调度权重与 LOS 几何均在预测时域内逐阶段评估——近场项作用于预测轨迹的末段阶段、远场项作用于前段阶段，因此**同一次规划内部即可完成中制导→末段的策略过渡**，而非仅靠周期切换。

---

## 9. 约束与变量归一化

### 9.1 箱式约束（物理量纲，`bounds.json`）

$$
\underline{x} \le x_k \le \overline{x}, \qquad
\underline{u} \le u_k \le \overline{u}
$$

| 变量 | 下界 | 上界 | 单位 |
|---|---|---|---|
| $p_x$ | $-2\times10^{5}$ | $2\times10^{4}$ | m |
| $p_y$ | $-2\times10^{5}$ | $2\times10^{4}$ | m |
| $p_z$ | $10$ | $3000$ | m |
| $\|v_i\|$ | $50$ | $50$ | m/s（逐轴） |
| $\|a_i\|$ | $30$ | $30$ | m/s²（逐轴） |

### 9.2 数值归一化

状态与输入按下述对角阵缩放后送入求解器（`normalization.json`）：

$$
\bar{x} = T_x^{-1} x, \qquad \bar{u} = T_u^{-1} u
$$

$$
T_x = \mathrm{diag}\left( 2000,\ 2000,\ 300,\ 25,\ 25,\ 25 \right), \qquad
T_u = \mathrm{diag}\left( 20,\ 20,\ 20 \right)
$$

将 $x = T_x \bar{x}$、$u = T_u \bar{u}$ 代入原始 QP，得到归一化空间中的等价问题：

$$
\bar{Q} = T_x\, Q\, T_x, \qquad \bar{q} = T_x\, q, \qquad
\bar{R} = T_u\, R\, T_u, \qquad \bar{r} = T_u\, r
$$

$$
\bar{A}_d = T_x^{-1} A_d\, T_x, \qquad
\bar{B}_d = T_x^{-1} B_d\, T_u
$$

$$
\bar{\underline{x}} = T_x^{-1}\underline{x}, \quad \bar{\overline{x}} = T_x^{-1}\overline{x}, \quad
\bar{\underline{u}} = T_u^{-1}\underline{u}, \quad \bar{\overline{u}} = T_u^{-1}\overline{u}
$$

求解后将解左乘 $T_x,\ T_u$ 反归一化回物理量纲。

---

## 10. RTI/SQP 实时求解流程

原始问题因速度范数项（§7.2 的 $\hat{v}_{\mathrm{lin}}$）及几何量对线性化点的依赖而呈非线性，采用**实时迭代（RTI）风格的 SQP 方案**（当前配置 $n_{\mathrm{sqp}} = 1$、混合系数 $\alpha_{\mathrm{mix}} = 1.0$，即每周期一次线性化 + 一次 QP 求解）：

**Step 1 — 初始猜测更新**（热启动）：

将上一周期解前移一步，末端用与约束一致的离散模型外推：

$$
\tilde{x}_{i-1} = x_{i}^{\ast}\ (i = 1,\dots,N), \qquad
\tilde{x}_N = A_d\, x_{N-1}^{\ast} + B_d\, u_{N-1}^{\ast}, \qquad \tilde{u}_N = 0
$$

首次运行（或连续求解失败超过阈值 $n_{\mathrm{reset}} = 5$ 次后重置）则以当前状态生成匀速直线猜测。

**Step 2 — SQP 迭代**（$i = 1, \dots, n_{\mathrm{sqp}}$）：

1. 沿时域外推目标状态（§4.2），逐阶段线性化代价（含 $\|v\|$ 一阶展开）；
2. HPIPM 求解时变 QP（OCP 结构，稀疏）；
3. 解混合更新（阻尼，防线性化点跳变）：

$$
x^{(i+1)} = \alpha_{\mathrm{mix}}\, x_{\mathrm{QP}} + \left(1 - \alpha_{\mathrm{mix}}\right) x^{(i)}, \qquad
u^{(i+1)} = \alpha_{\mathrm{mix}}\, u_{\mathrm{QP}} + \left(1 - \alpha_{\mathrm{mix}}\right) u^{(i)}
$$

**Step 3 — 输出与保护**：

组装 $N+1$ 个点的 $p/v/a$ 轨迹包输出给 100 Hz PID 跟踪层；若解出现 NaN，触发安全屏蔽（shield），强制输出当前位置悬停指令。

---

## 11. 参数汇总表

### 11.1 代价权重（`Params/cost.json`）

| 参数 | 值 | 所属分项 | 含义 |
|---|---|---|---|
| $q_c$ | $1\times10^{-4}$ | $J_c$ | 法向位置误差权重 |
| $q_v$ | $1\times10^{-3}$ | $J_{\mathrm{trk}}$ | 速度跟踪权重 |
| $q_a$ | $5\times10^{-4}$ | $J_u$ | 加速度惩罚权重 |
| $q_{\mathrm{vmag}}$ | $1\times10^{-2}$ | $J_{\mathrm{spd}}$ | 速度大小奖励峰值权重 |
| $\sigma_v$ | $1.5$ rad | $J_{\mathrm{spd}}$ | 态势角高斯宽度 |
| $q_{dv}$ | $3\times10^{-3}$ | $J_{\mathrm{apn}}$ | APN 对齐权重 |
| $N_{\mathrm{apn}}$ | $3.5$ | $J_{\mathrm{apn}}$ | APN 导引系数 |
| $\rho_{dv}$ | $1200$ m | $J_{\mathrm{apn}}$ | 远场权重过渡中心 |
| $k_{dv}$ | $400$ m | $J_{\mathrm{apn}}$ | 远场权重过渡宽度 |
| $\rho_v$ | $1200$ m | $J_c,\,J_{\mathrm{trk}}$ | 近场权重过渡中心 |
| $k_v$ | $400$ m | $J_c,\,J_{\mathrm{trk}}$ | 近场权重过渡宽度 |

### 11.2 模型与求解（`config.h`、节点配置）

| 参数 | 值 | 含义 |
|---|---|---|
| $T_s$ | $0.1$ s | 规划步长（10 Hz） |
| $N$ | $30$ | 预测步数（时域 3 s） |
| $c$ | $0.2\,\mathrm{s^{-1}}$ | 线性阻尼系数 |
| $n_{\mathrm{sqp}}$ | $1$ | SQP 迭代次数（RTI 模式） |
| $\alpha_{\mathrm{mix}}$ | $1.0$ | SQP 解混合系数 |
| $n_{\mathrm{reset}}$ | $5$ | 连续失败重置阈值 |
| $V_{\mathrm{app}}$ | $25$ m/s | 参考接近速度（`cost.cpp` 硬编码） |
| $\tau_m$ | $1.0$ s | 目标加速度衰减时间常数（`mpc.cpp` 硬编码） |
| $\varepsilon$ | $10^{-12}$ | $Q$ 正定性正则化 |
| — | $350$ m | 拦截判定/交接距离 |
| — | $0.5$ Hz / $10$ Hz / $100$ Hz | 量测 / 规划 / 跟踪频率 |

---

## 12. 实现细节与注意事项

以下为整理文档时对照代码发现的实现细节，读代码或调参时需注意：

1. **调度权重的注释与代码不一致**：`cost.cpp` 中 $J_{\mathrm{apn}}$ 的注释写 $w_{dv} = q_{dv}\cdot\frac{1}{2}(1-\tanh\cdot)$（近场激活），但**代码实际为 $\frac{1}{2}(1+\tanh\cdot)$（远场激活）**；$J_{\mathrm{trk}}$ 的注释写"距离越大权重越大"（$1+\tanh$），但代码复用的 `w_v_track` 实际为 $\frac{1}{2}(1-\tanh\cdot)$（**近场激活**）。本文档 §8 的分段策略表以**代码实际行为**为准。若需恢复注释所述行为，改 `cost.cpp` 第 76–77 行的符号即可。

2. **$J_c$ 线性项多乘一次权重**：完整二次型展开应为 $q^{(c)} = -Q^{(c)} p_t$，代码为 $-\,Q^{(c)}\, w_{\mathrm{near}}\, p_t$（`cost.cpp` 第 48 行，$w_{\mathrm{near}}$ 出现平方）。在过渡带内这会使等效"吸引位置"从 $p_t$ 向原点偏移；近场 $w_{\mathrm{near}} \approx 1$ 时无影响。

3. **$V_{\mathrm{app}} = 25\,\mathrm{m/s}$ 硬编码**于 `cost.cpp`，未进入 `cost.json`，调参时易遗漏。

4. **$J_{\mathrm{trk}}$ 的退化分支**：$\|d_{\perp}\| \le 10^{-3}$ 时 `vec_proj2target` 为零向量，其后 `dot(n_t) >= 0` 恒真，故参考速度恒取 $+V_{\mathrm{app}}\hat{n}_t$（尾追方向），"迎头取 $-V_{\mathrm{app}}\hat{n}_t$"的分支实际不可达。

5. **阶段 $k=0$ 的状态代价无效**：HPIPM 接口中首级 $n_x = 0$（$x_1 = A_d x_0 + B_d u_0$ 直接作为等式右端），故 `getCost` 在 $k=0$ 装配的 $Q_0,\, q_0$ 不参与求解；末级 $k=N$ 无输入，$r_N$ 无效。

6. **非线性来源单一**：外层 SQP 迭代中，代价对各阶段目标预测状态（随迭代不变）依赖明确；对本机线性化状态的依赖仅来自 $J_{\mathrm{spd}}$ 的 $\hat{v}_{\mathrm{lin}}$。当前 `n_sqp=1, α_mix=1.0` 下等价于单次线性化求解（RTI），该项的雅可比信息通过周期滚动隐式获得。

7. **目标模型的两层结构**：KF 采用 CA 恒加速传播（无衰减），时域外推采用 Singer 衰减（$\tau_m = 1\,\mathrm{s}$）；二者在 $\hat{a}_t$ 处衔接，预测时域末端目标近似匀速直线。

---

*文档生成日期：2026-08-18，对应分支 `feat/docker-build-env`。如代价函数修改，请同步更新本文档。*
