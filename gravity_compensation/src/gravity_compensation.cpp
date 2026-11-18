#include "gravity_compensation.hpp"

GravityCompParams::GravityCompParams(int num_joints) {
    mass_scaling.resize(num_joints, 1.0);
    friction_comp.resize(num_joints, 0.0);
    torque_limits.resize(num_joints, 100.0); // some default
    gravity_vector = Eigen::Vector3d(0.0, 0.0, -9.81);
}

ArmGravityComp::ArmGravityComp(const std::string& urdf_path) {
    // Build the Pinocchio model from URDF
    pinocchio::urdf::buildModel(urdf_path, model_);
    
    // Initialize parameters
    params_ = GravityCompParams(model_.nv);
    
    // Set gravity vector
    model_.gravity.setZero();
    model_.gravity.linear() = params_.gravity_vector;
    
    // Initialize joint state vectors
    q_ = Eigen::VectorXd::Zero(model_.nq);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    a_ = Eigen::VectorXd::Zero(model_.nv);
    tau_gravity_ = Eigen::VectorXd::Zero(model_.nv);
}

ArmGravityComp::~ArmGravityComp() {
    // Destructor - Pinocchio handles memory management
}

void ArmGravityComp::UpdatePositions(const std::vector<double>& positions) {
    // Update joint positions
    for (size_t i = 0; i < positions.size() && i < q_.size(); ++i) {
        q_[i] = positions[i];
        std::cout << "Joint " << i << " position updated to " << positions[i] << std::endl;
    }
}

void ArmGravityComp::ComputeGravityCompensation() {
    // Create data structure for computations
    pinocchio::Data data(model_);
    
    // Compute gravity torques using Recursive Newton-Euler Algorithm
    // With v=0 and a=0, this gives only gravity compensation torques
    tau_gravity_ = pinocchio::rnea(model_, data, q_, v_, a_);
    // tau_gravity_ = -tau_gravity_; // Negate to get compensation torques
}

double ArmGravityComp::GetGravityTorque(int joint) const {
    if (joint >= 0 && joint < tau_gravity_.size()) {
        return tau_gravity_[joint];
    }
    return 0.0; // Return 0 for invalid joint index
}

std::vector<double> ArmGravityComp::GetGravityTorques() const {
    return std::vector<double>(tau_gravity_.data(), tau_gravity_.data() + tau_gravity_.size());
}