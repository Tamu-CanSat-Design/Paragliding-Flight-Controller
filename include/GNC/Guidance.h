#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "Common.h"
#include "GNC/GNC.h"

class Guidance{
    private:

    float MaxTurnRadius = 6; //m
    float P_L = 0.5; //s
    float Zeta_L = 0.707f;
    float R = 8.0f; //Will need to be adaptive eventually for approach
    float phi_max = (float)PI/4;

    float L_1;
    float chi;
    float chi_cp;
    float chi_L;
    float eta;
    float eta_L;
    float a_ref;
    float v_g;
    float k_L;
    float q_L;
    float gamma;
    float e_t;
    float phi_ref;

    public:
    Guidance() : L_1(0.0), R(0), chi(0), chi_L(0), chi_cp(0), eta(0), a_ref(0), v_g(0), k_L(0), q_L(0), gamma(0), e_t(0), phi_ref(0) {}

    Guidance(Common::StateVector state, Common::Vector3 waypoint);
    float CalculateRoll(Common::StateVector state, Common::Vector3 waypoint);
    
    float GetRoll() const{return phi_ref;};


};

#endif // GUIDANCE_H

