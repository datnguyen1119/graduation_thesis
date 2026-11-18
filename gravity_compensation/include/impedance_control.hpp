#pragma once
#include <vector>
#include <memory>

class ImpedanceControl {
public:
    struct ImpedanceParams {
        double stiffness;      // K - spring constant
        double damping;        // B - damping coefficient
        double mass;           // M - inertia
        double q_ref;          // reference position
        double dt;             // time step

        ImpedanceParams() : stiffness(1.0), damping(0.1), mass(0.1), q_ref(0.0), dt(0.01) {}
    };

    explicit ImpedanceControl(const ImpedanceParams& params = ImpedanceParams());
    ~ImpedanceControl() = default;

    void SetReferencePosition(double q_ref);
    void SetParameters(const ImpedanceParams& params);
    double ComputeTorque(double q_current, double dq_current, double tau_external = 0.0);

private:
    ImpedanceParams params_;
    double q_prev_;
    double dq_prev_;
    bool initialized_;
};