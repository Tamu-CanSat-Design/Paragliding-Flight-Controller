#include "GNC/Guidance.h"

Guidance::Guidance(Common::StateVector state, Common::Vector3 waypoint)
    : L_1(0.0), R(0), chi(0), chi_L(0), chi_cp(0), eta(0), a_ref(0), v_g(0), k_L(0), q_L(0), gamma(0), e_t(0), phi_ref(0){
    float v_x = state.vx, v_y = state.vy;
    float x = state.x, y = state.y;
    float x_f = waypoint.x, y_f = waypoint.y;

    v_g = sqrt(v_x*v_x + v_y * v_y);
    q_L = P_L * Zeta_L / (float)PI;
    k_L = 4*Zeta_L;

    L_1 = q_L * v_g;
    float cp_vector[2] = {x-x_f, y-y_f};
    e_t = sqrt(cp_vector[0]*cp_vector[0] + cp_vector[1]*cp_vector[1]) - R; 

    if (L_1 < fabsf(e_t)){
        L_1 = fabsf(e_t);
    }

    chi_cp = atan2(-cp_vector[1],-cp_vector[0]);
    
    gamma = acos((L_1*L_1 + (e_t + R) * (e_t + R) - R*R)/(2 * L_1 * (e_t + R)));

    chi_L = GNC::wrap_pi(chi_cp - gamma);

    chi = atan2(v_y,v_x);

    eta = constrain(GNC::wrap_pi(chi_L-chi), -PI/2, PI/2);

    a_ref = k_L * v_g / q_L * sin(eta);
    phi_ref = constrain(atan(a_ref/GNC::STANDARD_GRAVITY), -phi_max, phi_max);
};

float Guidance::CalculateRoll(Common::StateVector state, Common::Vector3 waypoint){
    float v_x = state.vx, v_y = state.vy;
    float x = state.x, y = state.y;
    float x_f = waypoint.x, y_f = waypoint.y;

    v_g = sqrt(v_x*v_x + v_y * v_y);
    q_L = P_L * Zeta_L / (float)PI;
    k_L = 4*Zeta_L;

    L_1 = q_L * v_g;
    float cp_vector[2] = {x-x_f, y-y_f};
    e_t = sqrt(cp_vector[0]*cp_vector[0] + cp_vector[1]*cp_vector[1]) - R; 

    if (L_1 < fabsf(e_t)){
        L_1 = fabsf(e_t);
    }

    chi_cp = atan2(-cp_vector[1],-cp_vector[0]);
    
    gamma = acos((L_1*L_1 + (e_t + R) * (e_t + R) - R*R)/(2 * L_1 * (e_t + R)));

    chi_L = GNC::wrap_pi(chi_cp - gamma);

    chi = atan2(v_y,v_x);

    eta = constrain(GNC::wrap_pi(chi_L-chi), -PI/2, PI/2);

    a_ref = k_L * v_g / q_L * sin(eta);
    phi_ref = constrain(atan(a_ref/GNC::STANDARD_GRAVITY), -phi_max, phi_max);
    return phi_ref;
};