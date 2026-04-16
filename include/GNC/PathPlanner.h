#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "Common.h"
#include "GNC/GNC.h"

//Constraints positions, headings, curvature



class PathPlanner{
    private:

        enum GliderPhase{
            EnergyBleed,
            Approach,
            Abort,
        };

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
        GliderPhase Phase;


    public:

        PathPlanner() : Phase(EnergyBleed), Energy{} {}

        PathPlanner(const Common::StateVector state, const Common::DropZone& drop);

        void UpdateState(const Common::StateVector state, const Common::DropZone& drop);

        Common::Vector3 CalculateWaypoint();
};

#endif // PATHPLANNER_H