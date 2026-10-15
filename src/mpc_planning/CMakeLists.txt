cmake_minimum_required(VERSION 3.10) # 提升至3.10以完美支持 Python3 Find 模块
project(mpcplanning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")

# ==========================================
# 1. 查找 Python3 (用于 matplotlib-cpp 绘图)
# ==========================================
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# ==========================================
# 2. 头文件路径包含
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

# ==========================================
# 3. drone_dynamic 动力学库接入（离线仿真 mpc_offline_sim 用，install.sh 已拉取源码）
# ==========================================
set(DRONE_DYNAMIC_DIR "${PROJECT_SOURCE_DIR}/External/drone_dynamic" CACHE PATH "drone_dynamic 源码根目录")
add_subdirectory(${DRONE_DYNAMIC_DIR} drone_dynamic_build EXCLUDE_FROM_ALL)
include_directories(${DRONE_DYNAMIC_DIR})   # 以 <Model/drone_dynamics.h> 等包含

# ==========================================
# 4. 离线仿真可执行（无外部中间件依赖：drone_dynamic 动力学 + KF + MPC + PID 进程内闭环，
#    退出时弹 12 个绘图窗口）
# ==========================================
add_executable(mpc_offline_sim
        OfflineSim/offline_sim_node.cpp
        INDI/INDIAttitudeController.cpp
        INDI/INDIAttitudeController.h
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
# 参数目录用编译期绝对路径，运行期可用 argv[1]/argv[2] 覆盖
target_compile_definitions(mpc_offline_sim PRIVATE
        MPC_PARAMS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Params"
        DRONE_PARAMS_JSON="${DRONE_DYNAMIC_DIR}/Params/drone_params.json")
