# MPCC
This is a C++ implementation of the MPC path-planner(Mass Point Model). 
1.仿真形式：MPC规划器（C++ Version）和ros1下的gazebo仿真器联合仿真，无人机模型为默认的iris；
2.启动方式：roslaunch mpcplanning intercept_mpc.launch
3.可视化界面可在.launch文件中修改以下一行开启或关闭（以下为开启示例）：
        <arg name="gui" value="true"/>
4.规划效果在目标做圆形、上下起伏等机动时变差。
5.默认整定参数为：

  "q_l" : 2.7,
  "q_c" : 4.5,
  "q_v" : 0.3,
  "q_a" : 0.005
