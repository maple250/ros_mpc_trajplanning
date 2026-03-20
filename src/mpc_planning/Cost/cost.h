#ifndef MPCC_COST_H
#define MPCC_COST_H

#include "config.h"
#include "types.h"
#include "Params/params.h"
// #include "Spline/arc_length_spline.h"

namespace mpcc{
struct CostMatrix{
    Q_MPC Q;
    R_MPC R;
    S_MPC S;//soft constraint
    f_MPC f;
    r_MPC r;
    Z_MPC Z;
    z_MPC z;
};
class Cost {
public:
    CostMatrix getCost(const TargetState &target_pred, const Eigen::Vector3d &n_t, 
                         const State &x, const Input &u, int k) const;
    Cost(const PathToJson &path);
    Cost();
    
private:
    CostParam cost_param_;
};
}
#endif //MPCC_COST_H
