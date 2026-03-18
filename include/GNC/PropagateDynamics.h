#ifndef __PROPAGATE_DYNAMICS_H__
#define __PROPAGATE_DYNAMICS_H__

#include "Common.h"
#include <BasicLinearAlgebra.h>

namespace PropagateDynamics{
    void PropagateLinear(Common::StateVector& NextState, const Common::StateVector CurrentState, const Common::IMUData IMUData);
    void PropagateQuaternion(Common::StateVector& NextState, const Common::StateVector CurrentState);

    class RocketDynamics{

    };

    class ParachuteDynamics{

    };

    class ParagliderDynamics{

    };
};

#endif // __PROPAGATE_DYNAMICS_H__