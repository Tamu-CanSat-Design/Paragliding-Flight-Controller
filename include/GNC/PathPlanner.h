#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "Common.h"
#include "GNC/GNC.h"



class PathPlanner{
    private:

        

        struct EnergyValues{
            float DistReq{0.0f};
            float DistStraight{0.0f};
            float EnergySurplus{0.0f};
            float EnergyAval{0.0f};

            EnergyValues() = default;
            EnergyValues(float req, float straight, float surplus):
                DistReq(req), DistStraight(straight), EnergySurplus(surplus) {}
        };        

        EnergyValues Energy;
        GNC::GliderPhase Phase;


    public:

        PathPlanner() : Phase(GNC::GliderPhase::EnergyBleed), Energy{} {}

        PathPlanner(const Common::StateVector state, const Common::Vector3 waypoint);

        void UpdateState(const Common::StateVector state, const Common::Vector3 waypoint);

        Common::Vector3 CalculateWaypoint(const Common::DropZone drop);
};

#endif // PATHPLANNER_H