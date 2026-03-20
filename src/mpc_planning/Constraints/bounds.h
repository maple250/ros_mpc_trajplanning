//类特点：成员变量精细分类、存储参数（结构体形式），成员函数直接输出某一类参数
#ifndef MPCC_BOUNDS_H
#define MPCC_BOUNDS_H

#include "config.h"
#include "types.h"
#include "Params/params.h"

namespace mpcc{
class Bounds {
public:
    Bounds();
    Bounds(BoundsParam bounds_param);//实例化时，需要传入BoundsParam类对象

    Bounds_x getBoundsLX() const;
    Bounds_x getBoundsUX() const;

    Bounds_u getBoundsLU() const;
    Bounds_u getBoundsUU() const;

    Bounds_s getBoundsLS() const;
    Bounds_s getBoundsUS() const;

private:

    Bounds_x u_bounds_x_;
    Bounds_x l_bounds_x_;

    Bounds_u u_bounds_u_;
    Bounds_u l_bounds_u_;

    Bounds_s u_bounds_s_;
    Bounds_s l_bounds_s_;
};
}
#endif //MPCC_BOUNDS_H
