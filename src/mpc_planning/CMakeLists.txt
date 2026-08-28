cmake_minimum_required(VERSION 3.10) # 提升至3.10以完美支持 Python3 Find 模块
project(mpcplanning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")

# ==========================================
# 1. 查找 ROS Catkin 依赖包（可选：仅在线版 mpc_planning_node 需要）
#    - 容器/ROS 环境下 catkin_make：两个可执行都构建，行为与历史一致
#    - 宿主机无 ROS 直接 cmake：跳过 ROS 目标，只构建离线仿真 mpc_offline_sim
# ==========================================
find_package(catkin QUIET COMPONENTS
    roscpp
    nav_msgs
    mavros_msgs
    roslib
)
if(catkin_FOUND)
    # 声明这是一个标准的 ROS 包
    catkin_package()
endif()

# ==========================================
# 2. 查找 Python3 (用于 matplotlib-cpp 绘图)
# ==========================================
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# ==========================================
# 3. 头文件路径包含
# ==========================================
include_directories(
    .
    External/blasfeo/lib/include    # install.sh 将 blasfeo 装到 External/blasfeo/lib，头文件在其 include/ 下（扁平）
    External/hpipm/lib/include      # 同理，hpipm 装到 External/hpipm/lib
    External/matplotlib
    External/Eigen
    External/Json/include
    ${Python3_INCLUDE_DIRS}
)
if(catkin_FOUND)
    include_directories(${catkin_INCLUDE_DIRS})
endif()

# ==========================================
# 4. 编译可执行文件 (ROS Node，仅 catkin 环境下构建)
# ==========================================
# 【核心替换】去掉了 main.cpp，换成了 mpcc_ros_node.cpp，可执行文件命名为 mpcc_node
if(catkin_FOUND)
add_executable(mpc_planning_node
        mpc_planning_node.cpp
        KFfilter/KFfilter.cpp
        types.cpp
        types.h
        config.h
        Params/params.cpp
        Interfaces/hpipm_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        MPC/mpc.cpp
        Plotting/plotting.cpp
        Plotting/plotting.h
        controller/controller.cpp
        controller/controller.h
)
# ==========================================
# 5. 链接所有静态库和共享库
# ==========================================
target_link_libraries(mpc_planning_node
        ${catkin_LIBRARIES}
        Python3::Python
        Python3::NumPy
        ${PROJECT_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
        ${PROJECT_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a
)
endif()

# ==========================================
# 6. drone_dynamic 动力学库接入（离线仿真 mpc_offline_sim 用）
#    优先源码编译（install.sh/构建上下文已就位），否则回退预编译静态库
# ==========================================
set(DRONE_DYNAMIC_DIR "${PROJECT_SOURCE_DIR}/External/drone_dynamic" CACHE PATH "drone_dynamic 源码根目录")
if(EXISTS "${DRONE_DYNAMIC_DIR}/CMakeLists.txt")
    add_subdirectory(${DRONE_DYNAMIC_DIR} drone_dynamic_build EXCLUDE_FROM_ALL)
else()
    add_library(drone_dynamic STATIC IMPORTED)
    set_target_properties(drone_dynamic PROPERTIES IMPORTED_LOCATION
        "${DRONE_DYNAMIC_DIR}/build/libdrone_dynamic.a")
endif()
include_directories(${DRONE_DYNAMIC_DIR})   # 以 <Model/drone_dynamics.h> 等包含

# ==========================================
# 7. 离线仿真可执行（无 ROS 依赖：不链 catkin 库）
#    drone_dynamic 动力学 + KF + MPC + PID 进程内闭环，退出时弹 12 个绘图窗口
# ==========================================
add_executable(mpc_offline_sim
        OfflineSim/offline_sim_node.cpp
        InnerLoop/AccelToAttitudeController.cpp
        InnerLoop/AccelToAttitudeController.h
        KFfilter/KFfilter.cpp
        types.cpp
        Params/params.cpp
        Interfaces/hpipm_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        MPC/mpc.cpp
        Plotting/plotting.cpp
        controller/controller.cpp
)
target_link_libraries(mpc_offline_sim
        drone_dynamic
        Python3::Python
        Python3::NumPy
        ${PROJECT_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
        ${PROJECT_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a
)
# 离线版无 roslib：参数目录用编译期绝对路径，运行期可用 argv[1]/argv[2] 覆盖
target_compile_definitions(mpc_offline_sim PRIVATE
        MPC_PARAMS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Params"
        DRONE_PARAMS_JSON="${DRONE_DYNAMIC_DIR}/Params/drone_params.json")
