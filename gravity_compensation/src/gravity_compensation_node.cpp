#include <rclcpp/rclcpp.hpp>
#include <can_odrive_interface/odrive_interface.hpp>
#include "gravity_compensation.hpp"
#include "impedance_control.hpp"
#include <memory>
#include <vector>
#include <cmath>

class GravityCompensationNode : public rclcpp::Node {
public:
    static constexpr double REV_TO_RAD = M_PI / 4.0;

    GravityCompensationNode() : Node("gravity_compensation_node") {
        // Declare parameters with defaults
        this->declare_parameter<std::string>("urdf_path", "/home/dat/Workspace/rehab_robot_ver2_modified/src/rehab_robot_ver2/urdf/rehab_robot_ver2.urdf");
        this->declare_parameter<int>("num_joints", 3);
        this->declare_parameter<double>("impedance_stiffness", 1.0);
        this->declare_parameter<double>("impedance_damping", 0.1);
        this->declare_parameter<double>("impedance_mass", 0.1);
        this->declare_parameter<double>("reference_position", 0.0);

        std::string urdf_path = this->get_parameter("urdf_path").as_string();
        num_joints_ = this->get_parameter("num_joints").as_int();

        if (urdf_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "URDF path parameter not set!");
            return;
        }

        // Initialize gravity compensation
        gravity_comp_ = std::make_unique<ArmGravityComp>(urdf_path);

        // Initialize impedance control
        ImpedanceControl::ImpedanceParams imp_params;
        imp_params.stiffness = this->get_parameter("impedance_stiffness").as_double();
        imp_params.damping = this->get_parameter("impedance_damping").as_double();
        imp_params.mass = this->get_parameter("impedance_mass").as_double();
        imp_params.q_ref = this->get_parameter("reference_position").as_double();
        impedance_ctrl_ = std::make_unique<ImpedanceControl>(imp_params);

        // Initialize ODrive interface
        odrive_interface_ = std::make_unique<can_odrive_interface::ODriveInterface>();

        // Initialize torque mode for elbow joint (node 2)
        odrive_interface_->InitTorqueMode(2);

        // Create control loop timer (100 Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GravityCompensationNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Gravity Compensation + Impedance Control Node started with %d joints, controlling elbow joint (node 2)", num_joints_);
    }

private:
    void controlLoop() {
        std::vector<double> positions(num_joints_);
        std::vector<double> velocities(num_joints_);

        // Read positions and velocities from all ODrive nodes
        for (int i = 0; i < num_joints_; i++) {
            odrive_interface_->ReadEncoder(i);

            float turn = 0.0f;
            float vel = 0.0f;
            odrive_interface_->get_encoder_data(i, turn, vel);

            // Convert to radians
            positions[i] = turn * REV_TO_RAD;
            velocities[i] = vel * REV_TO_RAD;  // assuming vel is in rev/s, convert to rad/s
        }

        // Update joint positions
        gravity_comp_->UpdatePositions(positions);

        // Compute gravity compensation torques
        gravity_comp_->ComputeGravityCompensation();
        auto gravity_torques = gravity_comp_->GetGravityTorques();

        // Compute impedance control torque for elbow joint (node 2)
        double tau_gravity = gravity_torques[2];
        double tau_impedance = impedance_ctrl_->ComputeTorque(positions[2], velocities[2], tau_gravity);

        // Send combined torque command to elbow joint (node 2)
        // odrive_interface_->SendTorqueCommand(2, static_cast<float>(tau_impedance));
        odrive_interface_->SendTorqueCommand(2, static_cast<float>(tau_gravity));
    }

    std::unique_ptr<ArmGravityComp> gravity_comp_;
    std::unique_ptr<ImpedanceControl> impedance_ctrl_;
    std::unique_ptr<can_odrive_interface::ODriveInterface> odrive_interface_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    int num_joints_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GravityCompensationNode>());
    rclcpp::shutdown();
    return 0;
}