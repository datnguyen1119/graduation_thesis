#include "impedance_control.hpp"
#include <cmath>

ImpedanceControl::ImpedanceControl(const ImpedanceParams& params)
    : params_(params), q_prev_(0.0), dq_prev_(0.0), initialized_(false) {
}

void ImpedanceControl::SetReferencePosition(double q_ref) {
    params_.q_ref = q_ref;
}

void ImpedanceControl::SetParameters(const ImpedanceParams& params) {
    params_ = params;
}

double ImpedanceControl::ComputeTorque(double q_current, double dq_current, double tau_external) {
    // Impedance control law: tau = M*ddq_desired + B*(dq_desired - dq) + K*(q_desired - q)
    // For simplicity, assuming desired acceleration = 0, desired velocity = 0
    // tau = -K*(q - q_ref) - B*dq + tau_external

    double position_error = q_current - params_.q_ref;
    double velocity_error = dq_current;  // assuming desired velocity = 0

    double tau_impedance = -params_.stiffness * position_error - params_.damping * velocity_error;

    // Add external torque (e.g., gravity compensation)
    return tau_impedance + tau_external;
}