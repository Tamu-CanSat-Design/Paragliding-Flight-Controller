#include "Common.h"
#include "GNC/PathPlanner.h"
#include "GNC/MatrixUtils.h"

#include <arduino.h>
#include <BasicLinearAlgebra.h>

PathPlanner::PathPlanner(const Common::StateVector state, const Common::DropZone& drop)
    : Phase(EnergyBleed), Energy{} {
    float dx = drop.x - state.x;
    float dy = drop.y - state.y;
    Energy.DistStraight = sqrt(dx * dx + dy * dy);
    Energy.DistReq      = state.z * AverageGlideRatio;
    Energy.EnergySurplus = Energy.DistReq - Energy.DistStraight;

    if (Energy.EnergySurplus > 0.0f) {
        Phase = EnergyBleed;
    } else {
        Phase = Approach;
    }

    MatrixUtils::StateToMatrix(state, StateMatrix);

}

void PathPlanner::UpdateState(const Common::StateVector state){
    MatrixUtils::StateToMatrix(state, StateMatrix);
};

void PathPlanner::DetermineState(){

};

void PathPlanner::UpdateGlideRatio(){

};