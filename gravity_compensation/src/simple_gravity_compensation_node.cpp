// #include <rclcpp/rclcpp.hpp>
// #include <can_odrive_interface/odrive_interface.hpp>
// #include <memory>
// #include <vector>
// #include <cmath>

// class SimpleGravityCompensationNode : public rclcpp::Node {
// public:
//     static constexpr double REV_TO_RAD = M_PI / 4.0;
    
//     // Gravity compensation parameters
//     static constexpr double MASS = 0.586;           // kg
//     static constexpr double GRAVITY = 9.81;         // m/s^2
//     static constexpr double LENGTH = 0.179;         // m
//     static constexpr int JOINT_ID = 2;              // joint 2 (elbow)

//     SimpleGravityCompensationNode() : Node("simple_gravity_compensation_node") {
//         // Declare parameters
//         this->declare_parameter<int>("num_joints", 3);
//         num_joints_ = this->get_parameter("num_joints").as_int();

//         // Initialize ODrive interface
//         odrive_interface_ = std::make_unique<can_odrive_interface::ODriveInterface>();

//         // Initialize torque mode for joint 2
//         odrive_interface_->InitTorqueMode(JOINT_ID);

//         // Create control loop timer (100 Hz)
//         control_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(10),
//             std::bind(&SimpleGravityCompensationNode::controlLoop, this));

//         RCLCPP_INFO(this->get_logger(), "Simple Gravity Compensation Node started for joint %d", JOINT_ID);
//         RCLCPP_INFO(this->get_logger(), "Parameters: m=%.3f kg, g=%.3f m/s^2, l=%.3f m", MASS, GRAVITY, LENGTH);
//     }

// private:
//     void controlLoop() {
//         // Read position from joint 2
//         odrive_interface_->ReadEncoder(JOINT_ID);

//         float turn = 0.0f;
//         float vel = 0.0f;
//         odrive_interface_->get_encoder_data(JOINT_ID, turn, vel);

//         // Convert revolutions to radians
//         double theta = turn * REV_TO_RAD;  // Current angle in radians

//         // Calculate gravity compensation torque: tau = mgl * sin(theta)
//         double tau_gravity = MASS * GRAVITY * LENGTH * std::sin(theta);

//         // Send torque command to joint 2
//         odrive_interface_->SendTorqueCommand(JOINT_ID, static_cast<float>(tau_gravity));

//         // Optional: Log every 100 cycles (1 second at 100 Hz)
//         cycle_count_++;
//         if (cycle_count_ % 100 == 0) {
//             RCLCPP_INFO(this->get_logger(), 
//                 "Joint 2 - Angle: %.3f rad (%.1f deg), Gravity Torque: %.3f Nm",
//                 theta, theta * 180.0 / M_PI, tau_gravity);
//         }
//     }

//     std::unique_ptr<can_odrive_interface::ODriveInterface> odrive_interface_;
//     rclcpp::TimerBase::SharedPtr control_timer_;
//     int num_joints_;
//     int cycle_count_ = 0;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SimpleGravityCompensationNode>());
//     rclcpp::shutdown();
//     return 0;
// }

//----------------------------------------------------------------
// #include <rclcpp/rclcpp.hpp>
// #include <can_odrive_interface/odrive_interface.hpp>
// #include <memory>
// #include <vector>
// #include <cmath>
// #include <algorithm>

// class SimpleGravityCompensationNode : public rclcpp::Node {
// public:
//     static constexpr double REV_TO_RAD = M_PI / 4.0;

//     // Gravity compensation parameters
//     // static constexpr double MASS = 0.586;           // kg
//     // static constexpr double MASS = 2.12;
//     static constexpr double MASS = 2.0;
//     static constexpr double GRAVITY = 9.81;         // m/s^2
//     static constexpr double LENGTH = 0.179;         // m
//     static constexpr int JOINT_ID = 2;              // joint 2 (elbow)

//     SimpleGravityCompensationNode() : Node("simple_gravity_compensation_node") {
//         // Declare parameters
//         this->declare_parameter<int>("num_joints", 3);
//         num_joints_ = this->get_parameter("num_joints").as_int();

//         // "Transparent impedance" parameters
//         this->declare_parameter<double>("stiffness_k", 0.0);         // Nm/rad (0 => no spring)
//         this->declare_parameter<double>("damping_b", 0.15);          // Nm/(rad/s)
//         this->declare_parameter<double>("max_torque", 3.0);          // Nm, safety clamp

//         // Optional: keep it from "pulling back" by updating q_ref when nearly static
//         this->declare_parameter<bool>("auto_update_q_ref", true);
//         this->declare_parameter<double>("dq_static_thresh", 0.10);   // rad/s
//         this->declare_parameter<int>("static_count_to_update", 20);  // cycles (20 @100Hz = 0.2s)

//         // Simple velocity low-pass filter
//         this->declare_parameter<double>("dq_lpf_alpha", 0.2);        // 0..1 (higher = less filtering)

//         // Sign knobs (useful when your theta definition is opposite)
//         this->declare_parameter<double>("gravity_sign", 1.0);        // +1 or -1
//         this->declare_parameter<double>("torque_sign", 1.0);         // +1 or -1 (final output sign)

//         // Initialize ODrive interface
//         odrive_interface_ = std::make_unique<can_odrive_interface::ODriveInterface>();

//         // Initialize torque mode for joint 2
//         odrive_interface_->InitTorqueMode(JOINT_ID);

//         // Create control loop timer (100 Hz)
//         control_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(10),
//             std::bind(&SimpleGravityCompensationNode::controlLoop, this));

//         RCLCPP_INFO(this->get_logger(), "Simple Gravity + Transparent Impedance Node started for joint %d", JOINT_ID);
//         RCLCPP_INFO(this->get_logger(), "Params: m=%.3f kg, g=%.3f, l=%.3f m", MASS, GRAVITY, LENGTH);
//     }

// private:
//     static double clamp(double x, double lo, double hi) {
//         return std::max(lo, std::min(hi, x));
//     }

//     void controlLoop() {
//         // Read position from joint 2
//         odrive_interface_->ReadEncoder(JOINT_ID);

//         float turn = 0.0f;
//         float vel_turn_s = 0.0f; // typically turns/s from ODrive-style estimate
//         odrive_interface_->get_encoder_data(JOINT_ID, turn, vel_turn_s);

//         // Convert to rad / rad/s
//         const double q  = static_cast<double>(turn) * REV_TO_RAD;
//         const double dq_raw = static_cast<double>(vel_turn_s) * REV_TO_RAD;

//         // LPF dq
//         const double alpha = clamp(this->get_parameter("dq_lpf_alpha").as_double(), 0.0, 1.0);
//         if (!dq_initialized_) {
//             dq_filt_ = dq_raw;
//             dq_initialized_ = true;
//         } else {
//             dq_filt_ = alpha * dq_raw + (1.0 - alpha) * dq_filt_;
//         }
//         const double dq = dq_filt_;

//         // Initialize q_ref at first run to avoid step
//         if (!qref_initialized_) {
//             q_ref_ = q;
//             qref_initialized_ = true;
//         }

//         // Auto update q_ref when "static" so it doesn't pull back after you reposition
//         const bool auto_update = this->get_parameter("auto_update_q_ref").as_bool();
//         const double dq_thresh = std::abs(this->get_parameter("dq_static_thresh").as_double());
//         const int static_need = std::max<int>(
//             1,
//             static_cast<int>(this->get_parameter("static_count_to_update").as_int())
//         );


//         if (auto_update && std::abs(dq) < dq_thresh) {
//             static_count_++;
//             if (static_count_ >= static_need) {
//                 q_ref_ = q;              // “float” equilibrium moves with the arm
//                 static_count_ = 0;
//             }
//         } else {
//             static_count_ = 0;
//         }

//         // Gravity torque
//         const double gsign = this->get_parameter("gravity_sign").as_double();
//         const double tau_gravity = gsign * (MASS * GRAVITY * LENGTH * std::sin(q));
//         std::cout << "Gravity torque: " << tau_gravity << std::endl;

//         // Transparent impedance torque (spring-damper around q_ref)
//         const double K = this->get_parameter("stiffness_k").as_double(); // keep near 0 for transparency
//         const double B = this->get_parameter("damping_b").as_double();   // small damping to feel “light” but stable

//         const double tau_imp = -K * (q - q_ref_) - B * dq;

//         // Total torque + safety clamp + optional final sign
//         const double max_tau = std::abs(this->get_parameter("max_torque").as_double());
//         const double tsign   = this->get_parameter("torque_sign").as_double();

//         double tau_cmd = tau_gravity + tau_imp;
//         tau_cmd = clamp(tau_cmd, -max_tau, +max_tau);
//         tau_cmd *= tsign;

//         if (!std::isfinite(tau_cmd)) {
//             // fail safe
//             tau_cmd = 0.0;
//         }

//         // Send torque command
//         odrive_interface_->SendTorqueCommand(JOINT_ID, static_cast<float>(tau_cmd));

//         // Log every 1s
//         cycle_count_++;
//         if (cycle_count_ % 100 == 0) {
//             RCLCPP_INFO(this->get_logger(),
//                 "q=%.3f rad (%.1f deg), dq=%.3f rad/s, q_ref=%.3f, tau_g=%.3f, tau_imp=%.3f, tau_cmd=%.3f",
//                 q, q * 180.0 / M_PI, dq, q_ref_, tau_gravity, tau_imp, tau_cmd);
//         }
//     }

//     std::unique_ptr<can_odrive_interface::ODriveInterface> odrive_interface_;
//     rclcpp::TimerBase::SharedPtr control_timer_;

//     int num_joints_;
//     int cycle_count_ = 0;

//     // Impedance state
//     bool qref_initialized_ = false;
//     double q_ref_ = 0.0;
//     int static_count_ = 0;

//     // dq filter state
//     bool dq_initialized_ = false;
//     double dq_filt_ = 0.0;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SimpleGravityCompensationNode>());
//     rclcpp::shutdown();
//     return 0;
// }


//----------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <can_odrive_interface/odrive_interface.hpp>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

struct JointConfig {
  std::string name;
  int id = 2;

  // model
  double mass = 2.0;  double gravity = 9.81;  double length = 0.179;
  // initial angle (radians) used as reference for gravity calculation
  double initial_angle = 0.0;
  // gear reduction (motor:joint). Use >1.0 for reduction (e.g. 15.0 means motor turns 15 for 1 joint rev)
  double gear_ratio = 1.0;

  // control
  double stiffness_k = 1.5;  double damping_b = 0.15;  double max_torque = 3.0;

  bool auto_update_q_ref = true;
  double dq_static_thresh = 0.10;
  int static_count_to_update = 20;

  double dq_lpf_alpha = 0.2;

  double gravity_sign = 1.0;
  double torque_sign = 1.0;

  // runtime state
  bool qref_initialized = false;
  double q_ref = 0.0;
  int static_count = 0;

  bool dq_initialized = false;
  double dq_filt = 0.0;
};

class SimpleGravityCompensationNode : public rclcpp::Node {
public:
  static constexpr double REV_TO_RAD = M_PI / 4.0;

  SimpleGravityCompensationNode() : Node("simple_gravity_compensation_node") {
    // Common params
    this->declare_parameter<int>("control_rate_hz", 100);
    // Which joint should run in torque mode (others will be put to position mode at 0)
    this->declare_parameter<int>("working_joint", 2);

    // Declare params per joint (prefix)
    declareJointParams("base_joint", 0);
    declareJointParams("shoulder_joint", 1);
    declareJointParams("elbow_joint", 2);

    // Create ODrive interface
    odrive_interface_ = std::make_unique<can_odrive_interface::ODriveInterface>();

    // Load configs from params
    joints_.clear();
    joints_.reserve(3);

    JointConfig base;     base.name = "base_joint";     loadJointParams(base);
    JointConfig shoulder; shoulder.name = "shoulder_joint"; loadJointParams(shoulder);
    JointConfig elbow;    elbow.name = "elbow_joint";    loadJointParams(elbow);

    joints_.push_back(base);
    joints_.push_back(shoulder);
    joints_.push_back(elbow);

    // Init modes for each joint id: the `working_joint` runs torque mode,
    // the others are put into position mode and commanded to position 0.
    const int working_joint = this->get_parameter("working_joint").as_int();
    for (const auto& jc : joints_) {
      if (jc.id == working_joint) {
        odrive_interface_->InitTorqueMode(jc.id);
        RCLCPP_INFO(this->get_logger(),
                    "Init torque mode: %s id=%d m=%.3f l=%.3f max_tau=%.3f",
                    jc.name.c_str(), jc.id, jc.mass, jc.length, jc.max_torque);
      } else {
        odrive_interface_->InitPositionMode(jc.id);
        // send position 0 (turns) to hold other joints steady
        odrive_interface_->SendPositionCommand(jc.id, 0.0f, 0, 0);
        RCLCPP_INFO(this->get_logger(),
                    "Init position mode: %s id=%d pos=0.0",
                    jc.name.c_str(), jc.id);
      }
    }

    // Timer
    const int hz = std::max<int>(
      1,
      static_cast<int>(this->get_parameter("control_rate_hz").as_int())
    );

    const int period_ms = std::max<int>(1, static_cast<int>(1000.0 / hz));


    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SimpleGravityCompensationNode::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Node started. control_rate_hz=%d", hz);
  }

private:
  static double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
  }

  void declareJointParams(const std::string& joint, int default_id) {
    const std::string p = joint + ".";

    this->declare_parameter<int>(p + "id", default_id);

    this->declare_parameter<double>(p + "mass", 2.0);
    this->declare_parameter<double>(p + "gravity", 9.81);
    this->declare_parameter<double>(p + "length", 0.179);

    // initial angle for gravity compensation (radians)
    this->declare_parameter<double>(p + "initial_angle", 0.0);

    // gear reduction (motor:joint)
    this->declare_parameter<double>(p + "gear_ratio", 1.0);

    this->declare_parameter<double>(p + "stiffness_k", 0.0);
    this->declare_parameter<double>(p + "damping_b", 0.15);
    this->declare_parameter<double>(p + "max_torque", 3.0);

    this->declare_parameter<bool>(p + "auto_update_q_ref", true);
    this->declare_parameter<double>(p + "dq_static_thresh", 0.10);
    this->declare_parameter<int>(p + "static_count_to_update", 20);

    this->declare_parameter<double>(p + "dq_lpf_alpha", 0.2);

    this->declare_parameter<double>(p + "gravity_sign", 1.0);
    this->declare_parameter<double>(p + "torque_sign", 1.0);
  }

  void loadJointParams(JointConfig& jc) {
    const std::string p = jc.name + ".";

    jc.id = this->get_parameter(p + "id").as_int();

    jc.mass = this->get_parameter(p + "mass").as_double();
    jc.gravity = this->get_parameter(p + "gravity").as_double();
    jc.length = this->get_parameter(p + "length").as_double();

    jc.initial_angle = this->get_parameter(p + "initial_angle").as_double();

    jc.gear_ratio = this->get_parameter(p + "gear_ratio").as_double();

    jc.stiffness_k = this->get_parameter(p + "stiffness_k").as_double();
    jc.damping_b = this->get_parameter(p + "damping_b").as_double();
    jc.max_torque = this->get_parameter(p + "max_torque").as_double();

    jc.auto_update_q_ref = this->get_parameter(p + "auto_update_q_ref").as_bool();
    jc.dq_static_thresh = this->get_parameter(p + "dq_static_thresh").as_double();
    jc.static_count_to_update = this->get_parameter(p + "static_count_to_update").as_int();

    jc.dq_lpf_alpha = this->get_parameter(p + "dq_lpf_alpha").as_double();

    jc.gravity_sign = this->get_parameter(p + "gravity_sign").as_double();
    jc.torque_sign = this->get_parameter(p + "torque_sign").as_double();
  }

  void controlLoop() {
    for (auto& jc : joints_) {
      // Read encoder
      odrive_interface_->ReadEncoder(jc.id);

      float turn = 0.0f;
      float vel_turn_s = 0.0f;
      odrive_interface_->get_encoder_data(jc.id, turn, vel_turn_s);

      const double gear = std::max(1e-6, jc.gear_ratio);
      const double q = static_cast<double>(turn) * REV_TO_RAD / gear;
      const double dq_raw = static_cast<double>(vel_turn_s) * REV_TO_RAD / gear;

      // dq LPF
      const double alpha = clamp(jc.dq_lpf_alpha, 0.0, 1.0);
      if (!jc.dq_initialized) {
        jc.dq_filt = dq_raw;
        jc.dq_initialized = true;
      } else {
        jc.dq_filt = alpha * dq_raw + (1.0 - alpha) * jc.dq_filt;
      }
      const double dq = jc.dq_filt;

      // init q_ref
      if (!jc.qref_initialized) {
        jc.q_ref = q;
        jc.qref_initialized = true;
      }

      // auto update q_ref
      const double dq_thresh = std::abs(jc.dq_static_thresh);
      const int static_need = std::max(1, jc.static_count_to_update);

      if (jc.auto_update_q_ref && std::abs(dq) < dq_thresh) {
        jc.static_count++;
        if (jc.static_count >= static_need) {
          jc.q_ref = q;
          jc.static_count = 0;
        }
      } else {
        jc.static_count = 0;
      }

      // gravity torque (use joint initial angle as reference)
      const double tau_g = jc.gravity_sign * (jc.mass * jc.gravity * jc.length * std::sin(q + jc.initial_angle));

      // impedance torque
      const double tau_imp = -jc.stiffness_k * (q - jc.q_ref) - jc.damping_b * dq;

      // total (joint torque) + clamp + sign
      const double max_tau = std::abs(jc.max_torque);
      double tau_cmd = tau_g + tau_imp;
      tau_cmd = clamp(tau_cmd, -max_tau, +max_tau);
      tau_cmd *= jc.torque_sign;

      if (!std::isfinite(tau_cmd)) tau_cmd = 0.0;

      // convert joint torque to motor torque before sending (motor_torque = joint_torque / gear_ratio)
      const double motor_tau = tau_cmd / std::max(1e-6, jc.gear_ratio);
      odrive_interface_->SendTorqueCommand(jc.id, static_cast<float>(motor_tau));

      // log mỗi 1s cho từng joint
      cycle_count_++;
      if (cycle_count_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "[%s id=%d] q=%.3f dq=%.3f q_ref=%.3f tau_g=%.3f tau_imp=%.3f tau_cmd=%.3f",
          jc.name.c_str(), jc.id, q, dq, jc.q_ref, tau_g, tau_imp, tau_cmd);
      }
    }
  }

  std::unique_ptr<can_odrive_interface::ODriveInterface> odrive_interface_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::vector<JointConfig> joints_;
  int cycle_count_ = 0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleGravityCompensationNode>());
  rclcpp::shutdown();
  return 0;
}
