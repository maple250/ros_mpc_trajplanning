# 容器化运行 ros_mpc_trajplanning（Ubuntu 24.04 宿主机）

本项目是 **ROS1 Noetic + Gazebo Classic + PX4 SITL + MAVROS** 栈，官方仅支持 Ubuntu 20.04。
本机为 Ubuntu 24.04，无法直接安装，故用容器（基于 `osrf/ros:noetic-desktop-full`）运行。
Gazebo GUI 通过 **nvidia-container-toolkit** 走 RTX 5060 硬件 OpenGL 加速。

## 目录约定
- 仓库根 = catkin 工作空间（含 `.catkin_workspace`、`src/mpc_planning`）
- 容器内工作空间：`/catkin_ws`，源码包：`/catkin_ws/src/mpc_planning`
- 容器内 PX4：`/opt/PX4-Autopilot`（v1.14.3）

---

## 第 1 步：宿主机安装 Docker + NVIDIA 容器工具（一次性）

> 需 sudo。本机 sudo 非免密，请在终端用 `!` 前缀执行，输出会回到会话：

```bash
! sudo bash docker/host-setup.sh
```

脚本会：安装 Docker Engine + compose 插件 → 安装 nvidia-container-toolkit → 配置 docker runtime → 把当前用户加入 docker 组。

完成后**重新登录**（或 `newgrp docker`）使 docker 组生效，然后验证：

```bash
docker run --rm hello-world                       # Docker 正常
docker run --rm --gpus all ubuntu nvidia-smi      # 能看到 RTX 5060
```

---

## 第 2 步：构建镜像

### 2a. 快速验证项目可编译（不含 PX4，几分钟，验证任务 B）
```bash
docker build --target mpc-built -t mpc-noetic:mpc .
```
此镜像含 ROS Noetic + MAVROS + External 依赖 + 已编译的项目，可用于编译验证和（无 PX4 时的）节点调试，但不能跑完整 iris SITL 仿真。

### 2b. 完整镜像（含 PX4 SITL，首次约 30–60 分钟，镜像约 6–8GB）
```bash
docker build -t mpc-noetic:full .
```
> PX4 编译是长杆。若 PX4 层失败，2a 的 `mpc-noetic:mpc` 镜像仍可用（项目编译已验证）。

---

## 第 3 步：运行

### 进入容器 shell（默认挂载宿主机源码，改代码实时生效）
```bash
./docker/run.sh
# 等价于：docker run --rm -it --runtime=nvidia --gpus all --net=host \
#   -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#   -v <repo>/src/mpc_planning:/catkin_ws/src/mpc_planning \
#   -v mpc_external:/catkin_ws/src/mpc_planning/External mpc-noetic:full
```
- `External/` 用命名卷 `mpc_external` 保留镜像内已构建的 blasfeo/hpipm 等，不会被宿主机的空目录覆盖。
- 源码改动后在容器内重新编译：`cd /catkin_ws && catkin_make`

### 直接启动仿真
```bash
# 无头（不开 Gazebo 窗口，最快验证节点能跑起来）
./docker/run.sh roslaunch mpcplanning intercept_mpc.launch gui:=false

# 带 Gazebo GUI（走 NVIDIA 硬件 OpenGL）
./docker/run.sh roslaunch mpcplanning intercept_mpc.launch gui:=true
```

> 用镜像内烘焙源码（不挂载宿主机源码）：`./docker/run.sh --no-mount roslaunch ...`

---

## 架构说明
- **基础镜像** `osrf/ros:noetic-desktop-full`：Ubuntu 20.04 + ROS Noetic + Gazebo 11 + rviz
- **MAVROS**：apt 安装 + geographiclib 数据集
- **External 依赖**：容器内由 `src/mpc_planning/install.sh` 构建
  - blasfeo（`TARGET=X64_INTEL_HASWELL`，静态库）→ `External/blasfeo/lib/`
  - hpipm（复用预编译 blasfeo，静态库）→ `External/hpipm/lib/`
  - Eigen / nlohmann/json / matplotlib-cpp（头文件库）
- **PX4**：v1.14.3，`DONT_RUN=1 make px4_sitl gazebo-classic`（编译但不启动）
- **`$(find px4)`**：通过 `ROS_PACKAGE_PATH` 指向 `/opt/PX4-Autopilot`（其根有 `package.xml`）

## 排错

### Gazebo GUI 不显示 / GL 报错
NVIDIA 容器内 Gazebo Classic 的 OpenGL 较折腾。若 `gui:=true` 报 GLX/OpenGL 错误，回退到软件渲染：
```bash
./docker/run.sh --env LIBGL_ALWAYS_SOFTWARE=1 \
  roslaunch mpcplanning intercept_mpc.launch gui:=true
```
或直接用 `gui:=false` 跑无头仿真（节点逻辑不受影响）。

### 改了 install.sh / External 依赖后，命名卷仍是旧的
```bash
docker volume rm mpc_external        # 删除后下次 run 会重新从镜像填充
```

### PX4 构建失败
PX4 v1.14 对工具链较敏感。常见原因：Python 包缺失、submodule 没拉全。可先进 `mpc-noetic:mpc` 容器手动复现：
```bash
docker run --rm -it mpc-noetic:mpc bash
# 容器内：
git clone --recursive --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git /opt/PX4
cd /opt/PX4 && DONT_RUN=1 make px4_sitl gazebo-classic -j4
```
根据报错补依赖后，回 Dockerfile 对应层修正重建。

### `$(find px4)` 找不到
确认 `ROS_PACKAGE_PATH` 含 `/opt/PX4-Autopilot`：`rospack find px4` 应输出该路径。entrypoint 已自动设置。

### 内存不足（PX4 编译 OOM）
Dockerfile 已用 `-j4`。仍 OOM 则改 `-j2` 重建，或关闭其他占内存进程。
