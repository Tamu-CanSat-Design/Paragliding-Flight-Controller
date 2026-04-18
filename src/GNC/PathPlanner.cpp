#include "GNC/PathPlanner.h"

PathPlanner::PathPlanner(const Common::StateVector state, const Common::Vector3 waypoint)
    : Energy{}, Phase(EnergyBleed) {
    float dx = waypoiny.x - state.x;
    float dy = waypoint.y - state.y;
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

void PathPlanner::UpdateState(const Common::StateVector state, const Common::Vector3 waypoint){
    float dx = waypoint.x - state.x;
    float dy = waypoint.y - state.y;
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


Common::Vector3 PathPlanner::CalculateWaypoint(const Common::DropZone drop){
    Common::Vector3 waypoint = {drop.x, drop.y,0.0f};
    return waypoint;
};