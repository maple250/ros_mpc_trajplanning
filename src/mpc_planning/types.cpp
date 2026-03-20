#include "types.h"
namespace mpcc{
//输入一个State结构对象，输出一个StateVector类型Eigen向量
StateVector stateToVector(const State &x)
{
    StateVector xk;
    xk(0) = x.px;
    xk(1) = x.py;
    xk(2) = x.pz;
    xk(3) = x.vx;
    xk(4) = x.vy;
    xk(5) = x.vz;
    return xk;
}
//输入一个Input结构对象，输出一个InputVector类型Eigen向量
InputVector inputToVector(const Input &u)
{
    InputVector uk = {u.ax,u.ay,u.az};
    return uk;
}
//输入一个StateVector类型Eigen向量，输出一个State结构对象
State vectorToState(const StateVector &xk)
{
    State x;
    x.px    = xk(0);
    x.py     = xk(1);
    x.pz     = xk(2);
    x.vx     = xk(3);
    x.vy     = xk(4);
    x.vz     = xk(5);
    return x;
}
//输入一个InputVector类型Eigen向量，输出一个Input结构对象
Input vectorToInput(const InputVector &uk)
{
    Input u;
    u.ax    = uk(0);
    u.ay    = uk(1);
    u.az    = uk(2);
    return u;
}
//输入一个状态数组，输出一个State结构对象
State arrayToState(double *xk)
{
    State x;
    x.px     = xk[0];
    x.py     = xk[1];
    x.pz     = xk[2];
    x.vx     = xk[3];
    x.vy     = xk[4];
    x.vz     = xk[5];
    return x;
}
//输入一个输入数组，输出一个Input结构对象
Input arrayToInput(double *uk)
{
    Input u;
    u.ax    = uk[0];
    u.ay    = uk[1];
    u.az    = uk[2];
    return u;
}

}