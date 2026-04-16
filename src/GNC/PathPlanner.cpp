#include "GNC/PathPlanner.h"

PathPlanner::PathPlanner(const Common::StateVector state, const Common::DropZone& drop)
    : Energy{}, Phase(EnergyBleed) {
    float dx = drop.x - state.x;
    float dy = drop.y - state.y;
    float vx = state.vx, vy = state.vy, vz = state.vz;
    float v_i = sqrt(vx*vx+vy*vy+vz*vz);
    float v_f = 3; //need to define desired final velocity (if any)
    float G = GNC::GlideRatio.average(); 
    float g = GNC::STANDARD_GRAVITY;
    Energy.DistStraight = sqrt(dx * dx + dy * dy);

    Energy.EnergyAval = g * abs(-state.z-2)+ 1/2*(v_i*v_i - v_f*v_f);

    float EnergyTrans = g*Energy.DistStraight/G;
    
    Energy.EnergySurplus = Energy.EnergyAval - EnergyTrans;

    if (Energy.EnergySurplus > 0.0f) {
        Phase = EnergyBleed;
    } else {
        Phase = Approach;
    }
};

void PathPlanner::UpdateState(const Common::StateVector state, const Common::DropZone& drop){
    float dx = drop.x - state.x;
    float dy = drop.y - state.y;
    float vx = state.vx, vy = state.vy, vz = state.vz;
    float v_i = sqrt(vx*vx+vy*vy+vz*vz);
    float v_f = 3; //need to define desired final velocity (if any)
    float G = GNC::GlideRatio.average(); 
    float g = GNC::STANDARD_GRAVITY;
    Energy.DistStraight = sqrt(dx * dx + dy * dy);

    Energy.EnergyAval = g * abs(-state.z-2)+ 1/2*(v_i*v_i - v_f*v_f);

    float EnergyTrans = g*Energy.DistStraight/G;
    
    Energy.EnergySurplus = Energy.EnergyAval - EnergyTrans;

    if (Energy.EnergySurplus > 0.0f) {
        Phase = EnergyBleed;
    } else {
        Phase = Approach;
    }
};