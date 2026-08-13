# syntax=docker/dockerfile:1
# ============================================================
#  ROS1 Noetic + Gazebo Classic + PX4 SITL v1.14.3 + MAVROS + MPC 项目
#  在 Ubuntu 24.04 宿主机上通过容器运行 ros_mpc_trajplanning
#
#  分阶段构建：
#    base       : ROS Noetic + Gazebo11 + MAVROS + Python + NVIDIA GL 依赖
#    mpc-built  : + External 依赖(blasfeo/hpipm/...) + catkin_make（验证项目可编译）
#    final      : + PX4 SITL（长编译，仅运行时仿真需要）
#
#  快速验证项目编译（不含 PX4，快）：
#    docker build --target mpc-built -t mpc-noetic:mpc .
#  完整镜像（含 PX4 仿真）：
#    docker build -t mpc-noetic:full .
# ============================================================

FROM osrf/ros:noetic-desktop-full AS base

ENV DEBIAN_FRONTEND=noninteractive

# ---- 1. apt 依赖：MAVROS + Python 绘图 + NVIDIA GL + PX4 构建工具链 ----
# 构建时用 --network=host + --build-arg HTTPS_PROXY=http://127.0.0.1:10801，
# apt 走宿主机代理（archive.ubuntu.com / packages.ros.org 直连被墙）
RUN if [ -n "$HTTPS_PROXY" ]; then \
        printf 'Acquire::http::Proxy "%s";\nAcquire::https::Proxy "%s";\n' \
            "$HTTPS_PROXY" "$HTTPS_PROXY" > /etc/apt/apt.conf.d/99proxy; \
    fi
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-mavros ros-noetic-mavros-extras \
        python3-dev python3-numpy python3-matplotlib python3-pip \
        geographiclib-tools \
        libglvnd0 libgl1 libegl1 libgles2 libglx0 libglib2.0-0 \
        git cmake build-essential libtool autoconf unzip wget \
        protobuf-compiler libopencv-dev libxml2-dev libboost-dev libeigen3-dev \
        vim less \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. MAVROS geographiclib 数据集（约 500MB 下载）----
RUN /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh \
    || echo "[warn] geographiclib 数据集安装失败，MAVROS 可能告警但不影响构建"

# ---- 3. PX4 构建所需 Python 包 ----
RUN pip3 install --no-cache-dir \
        "empy==3.3.4" jinja2 numpy toml packaging pandas future pyyaml

# ---- 4. NVIDIA GL 环境变量（glvnd 选 NVIDIA vendor，运行时由 nvidia-container-toolkit 注入驱动库）----
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display \
    __GLX_VENDOR_LIBRARY_NAME=nvidia \
    __NV_PRIME_RENDER_OFFLOAD=1


# ============================================================
#  mpc-built：构建 External 依赖 + 项目 catkin_make（不依赖 PX4）
# ============================================================
FROM base AS mpc-built

ENV CATKIN_WS=/catkin_ws
WORKDIR $CATKIN_WS
RUN mkdir -p $CATKIN_WS/src

# 拷贝项目源码，然后构建 External 依赖（install.sh 幂等）
COPY src/mpc_planning $CATKIN_WS/src/mpc_planning
RUN cd $CATKIN_WS/src/mpc_planning && bash install.sh

# catkin_make 编译项目（验证 ROS + External 链接成功 = 任务 B）
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make -j$(nproc)"


# ============================================================
#  final：在 mpc-built 上构建 PX4 SITL（长编译，运行时仿真需要）
# ============================================================
FROM mpc-built AS final

ENV PX4_DIR=/opt/PX4-Autopilot
WORKDIR /opt

# clone PX4 v1.14.3（含 sitl_gazebo-classic 子模块，递归）
RUN git clone --recursive --depth 1 --branch v1.14.3 \
        https://github.com/PX4/PX4-Autopilot.git $PX4_DIR \
    || (cd $PX4_DIR && git submodule update --init --recursive)

# PX4 v1.14.3 的完整 Python 构建依赖（kconfiglib/jsonschema/cerberus/lxml/nunavut/sympy 等）
# base 阶段只装了部分，PX4 make 生成 parameters.xml/actuators.json 等会逐个报缺包；
# 改为直接装仓库自带 requirements.txt 一次补齐。放在 clone 之后、make 之前，保留 git clone 层缓存。
RUN pip3 install --no-cache-dir -r $PX4_DIR/Tools/setup/requirements.txt

# sitl_gazebo-classic 的 gst_camera_plugin/gst_video_stream_widget 需要 GStreamer 开发包
# （base 镜像未含；缺失会使 configure 阶段 GSTREAMER_APP_LIBRARIES=NOTFOUND 而失败）
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# 构建 px4_sitl + gazebo-classic 插件；DONT_RUN=1 阻止启动 gazebo
# -j4 限制并行度防止内存(15G)不足；target 用 px4_sitl(v1.14)，失败回退 px4_sitl_default
RUN cd $PX4_DIR \
    && (DONT_RUN=1 make px4_sitl gazebo-classic -j4 \
        || DONT_RUN=1 make px4_sitl_default gazebo-classic -j4)

# ROS_PACKAGE_PATH 使 $(find px4) 解析到 PX4-Autopilot 根
ENV ROS_PACKAGE_PATH=${PX4_DIR}:${PX4_DIR}/Tools/sitl_gazebo-classic

# ---- 入口 ----
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
WORKDIR $CATKIN_WS

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
