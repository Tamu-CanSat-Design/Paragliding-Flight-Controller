#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "Common.h"
#include "GNC/MatrixUtils.h"
#include <BasicLinearAlgebra.h>

#define MAX_R 10f
#define GLIDE_RATIO 5f

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

            EnergyValues() = default;
            EnergyValues(float req, float straight, float surplus):
                DistReq(req), DistStraight(straight), EnergySurplus(surplus) {}
        };        

        BLA::Matrix<13> StateMatrix;
        EnergyValues Energy;
        GliderPhase Phase;

        float AverageGlideRatio;



    public:

        PathPlanner(): Phase(EnergyBleed),Energy{}, AverageGlideRatio(0){
            StateMatrix.Fill(0);
        }

        PathPlanner(const Common::StateVector state, const Common::DropZone& drop);

        void UpdateState(const Common::StateVector state);
        void DetermineState();
        void UpdateGlideRatio();


        void GenerateWaypoint();
        void GetWaypoint();

};

#endif // PATHPLANNER_H