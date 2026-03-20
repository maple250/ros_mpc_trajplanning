#ifndef MPCC_CONFIG_H
#define MPCC_CONFIG_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace mpcc{

#define gravity 9.81
#define NX 6
#define NU 3
constexpr double Ts = 0.1; // 规划步长

#define NB 24 //max number of bounds
#define NPC 1 //number of polytopic constraints:不等式约束组成的可行域
#define NS 0 //软约束3

static constexpr int N = 30;//编译时就初始化的常量
static constexpr double INF = 1E5;
struct StateInputIndex{
    int px = 0;
    int py = 1;
    int pz = 2;
    int vx = 3;
    int vy = 4;
    int vz = 5;

    int ax = 0;
    int ay = 1;
    int az = 2;
};
static const StateInputIndex si_index;

}
#endif //MPCC_CONFIG_H
