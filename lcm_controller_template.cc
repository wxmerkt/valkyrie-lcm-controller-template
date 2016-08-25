// Copyright 2016 Wolfgang Merkt
//
// Barebones controller template for controller receiving robot state
// information via LCM, running a control loop and publishing control commands
// via LCM
// To be used with LCM2ROSControl

#include <iostream>
#include <memory>

#include <bot_core/timestamp.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/joint_angles_t.hpp"
#include "lcmtypes/bot_core/joint_state_t.hpp"
#include "lcmtypes/bot_core/atlas_command_t.hpp"

#include "lcmtypes/drc/behavior_transition_t.hpp"

// LCM Handlers
class LCMControllerTemplate {
 private:
  std::shared_ptr<lcm::LCM> lcm_;

 public:
  LCMControllerTemplate(std::shared_ptr<lcm::LCM> lcm_in) : lcm_(lcm_in) {
    desired_torque_ = bot_core::joint_angles_t();
    executed_torque_ = bot_core::joint_state_t();
    measured_feedback_ = bot_core::joint_state_t();
  }
  ~LCMControllerTemplate() {}
  bot_core::joint_angles_t desired_torque_;
  bot_core::joint_state_t executed_torque_;
  bot_core::joint_state_t measured_feedback_;

  void measured_feedback_handler(const lcm::ReceiveBuffer *rbuf,
                                 const std::string &channel,
                                 const bot_core::joint_state_t *msg) {
    // std::cout << "Received measured feedback" << std::endl;
    measured_feedback_ = *msg;

    run_control_command();
  }

  void desired_torque_handler(const lcm::ReceiveBuffer *rbuf,
                              const std::string &channel,
                              const bot_core::joint_angles_t *msg) {
    // std::cout << "Received desired torque (for synchronicity)" << std::endl;
    desired_torque_ = *msg;
  }

  void executed_torque_handler(const lcm::ReceiveBuffer *rbuf,
                               const std::string &channel,
                               const bot_core::joint_state_t *msg) {
    // std::cout << "Received executed torques" << std::endl;
    executed_torque_ = *msg;
  }

  void transition_to_position_control(double transition_duration_s = 1.0) {
    drc::behavior_transition_t msg = drc::behavior_transition_t();
    msg.utime = bot_timestamp_now();
    msg.behavior = 1;
    msg.transition_duration_s = transition_duration_s;

    lcm_->publish("ROBOT_BEHAVIOR", &msg);
  }

  void transition_to_force_control(double transition_duration_s = 1.0) {
    drc::behavior_transition_t msg = drc::behavior_transition_t();
    msg.utime = bot_timestamp_now();
    msg.behavior = 1;
    msg.transition_duration_s = transition_duration_s;

    lcm_->publish("ROBOT_BEHAVIOR", &msg);
  }

  void run_control_command() {
    std::cout << "Run control command" << std::endl;

    // Getting example position, velocity, torque
    // leftElbowPitch is e.g. [2]
    int num_joints = 1;
    std::cout << measured_feedback_.joint_name[2]
              << ":\t position: " << measured_feedback_.joint_position[2]
              << "\t velocity: " << measured_feedback_.joint_velocity[2]
              << "\t effort: " << measured_feedback_.joint_effort[2]
              << std::endl;

    // TODO: When in force mode, you need to command ALL joints of the actuated
    // components, i.e. PD on the joints that are not part of the controller

    double position = measured_feedback_.joint_position[2];
    double velocity = measured_feedback_.joint_velocity[2];

    double example_pd = 30.0 * (0.0 - position) - 10.0 * velocity;
    bot_core::atlas_command_t msg = bot_core::atlas_command_t();
    msg.utime = bot_timestamp_now();
    msg.num_joints = num_joints;
    msg.joint_names.resize(msg.num_joints);

    msg.position.resize(msg.num_joints);
    msg.velocity.resize(msg.num_joints);
    msg.effort.resize(msg.num_joints);

    msg.k_q_p.resize(msg.num_joints);
    msg.k_q_i.resize(msg.num_joints);
    msg.k_qd_p.resize(msg.num_joints);
    msg.k_f_p.resize(msg.num_joints);
    msg.ff_qd.resize(msg.num_joints);
    msg.ff_qd_d.resize(msg.num_joints);
    msg.ff_f_d.resize(msg.num_joints);
    msg.ff_const.resize(msg.num_joints);
    msg.k_effort.resize(msg.num_joints);  // only used in sim

    msg.desired_controller_period_ms =
        2;  // set desired controller rate (ms), only used in sim

    msg.joint_names[0] = "leftElbowPitch";
    msg.position[0] = 0.0;
    msg.velocity[0] = 0.0;
    msg.effort[0] = example_pd;

    lcm_->publish("ROBOT_COMMAND", &msg);
  }
};

// Main
int main() {
  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good()) std::cerr << "ERROR: lcm is not good()" << std::endl;

  LCMControllerTemplate handler(lcm);

  // Desired torque, i.e. applied to the robot (LCM2ROSCONTROL_lcm_torque)
  lcm->subscribe("LCM2ROSCONTROL_lcm_torque",
                 &LCMControllerTemplate::desired_torque_handler, &handler);

  // Commanded torque, i.e. requested via ROBOT_COMMAND
  // (LCM2ROSCONTROL_lcm_commanded)
  lcm->subscribe("LCM2ROSCONTROL_lcm_commanded",
                 &LCMControllerTemplate::executed_torque_handler, &handler);

  // Measured position/velocities/torque (LCM2ROSCONTROL_lcm_pose)
  lcm->subscribe("LCM2ROSCONTROL_lcm_pose",
                 &LCMControllerTemplate::measured_feedback_handler, &handler);

  // TODO F/T and IMU
  // FORCE_TORQUE: six_axis_force_torque_t
  // IMU_leftTorsoImu : ins_t
  // IMU_pelvisRearImu : ins_t

  while (0 == lcm->handle())
    ;

  return 0;
}
