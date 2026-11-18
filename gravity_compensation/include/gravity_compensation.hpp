#pragma once
#include <rclcpp/rclcpp.hpp>
#include <can_odrive_interface/odrive_interface.hpp>
#include <memory>
#include <string>
#include <vector>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/frame.hpp>

struct GravityCompParams {
    std::vector<double> mass_scaling;      
    std::vector<double> friction_comp;     
    std::vector<double> torque_limits;     
    Eigen::Vector3d gravity_vector;        
    
    GravityCompParams(int num_joints = 3);
};

class ArmGravityComp {
public:
    explicit ArmGravityComp(const std::string& urdf_path);
    ~ArmGravityComp();
    
    void UpdatePositions(const std::vector<double>& positions);
    void ComputeGravityCompensation();
    double GetGravityTorque(int joint) const;
    std::vector<double> GetGravityTorques() const;
    // void SetCalibrationParams(const GravityCompParams& params);
    const Eigen::VectorXd& GetJointPositions() const { return q_; }
    // int GetDOF() const { return reduced_model_.nv; }
    // std::vector<std::string> GetJointNames() const;
    // void PrintStatus() const;



private:
    // void createReducedModel();
    
    pinocchio::Model model_;         // Full model
    // pinocchio::Model reduced_model_; // Reduced model
    // pinocchio::Data reduced_data_;   // Data for reduced model
    
    Eigen::VectorXd q_;           
    Eigen::VectorXd v_;           
    Eigen::VectorXd a_;           
    Eigen::VectorXd tau_gravity_; 
    
    GravityCompParams params_;
};