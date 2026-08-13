cmake_minimum_required(VERSION 3.10) # 提升至3.10以完美支持 Python3 Find 模块
project(mpcplanning)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")

# ==========================================
# 1. 查找 ROS Catkin 依赖包
# ==========================================
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    mavros_msgs
    roslib
)

# 声明这是一个标准的 ROS 包
catkin_package()

# ==========================================
# 2. 查找 Python3 (用于 matplotlib-cpp 绘图)
# ==========================================
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

# ==========================================
# 3. 头文件路径包含
# ==========================================
include_directories(
    .
    ${catkin_INCLUDE_DIRS}
    External/blasfeo/lib/include    # install.sh 将 blasfeo 装到 External/blasfeo/lib，头文件在其 include/ 下（扁平）
    External/hpipm/lib/include      # 同理，hpipm 装到 External/hpipm/lib
    External/matplotlib
    External/Eigen
    External/Json/include
    ${Python3_INCLUDE_DIRS}
)

# ==========================================
# 4. 编译可执行文件 (ROS Node)
# ==========================================
# 【核心替换】去掉了 main.cpp，换成了 mpcc_ros_node.cpp，可执行文件命名为 mpcc_node
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
