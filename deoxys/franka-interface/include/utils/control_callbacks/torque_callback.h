// Copyright 2022 Yifeng Zhu

#include <chrono>
#include <franka/model.h>
#include <franka/robot.h>

#include "utils/control_utils.h"
#include "utils/robot_utils.h"
#include "utils/shared_memory.h"
#include "utils/shared_state.h"

#ifndef UTILS_CONTROL_CALLBACKS_TORQUE_CALLBACK_H_
#define UTILS_CONTROL_CALLBACKS_TORQUE_CALLBACK_H_

namespace control_callbacks {

// Decay the previously commanded torque toward zero. Avoids
// controller_torque_discontinuity on the abrupt non-zero → zero step
// libfranka would otherwise see at session end.
inline std::array<double, 7> RampTorqueDown(
    const franka::RobotState &robot_state,
    double decay = 0.8) {
  std::array<double, 7> tau_decay;
  for (int i = 0; i < 7; i++) {
    tau_decay[i] = robot_state.tau_J_d[i] * decay;
  }
  return franka::limitRate(franka::kMaxTorqueRate, tau_decay,
                           robot_state.tau_J_d);
}

inline bool TorqueIsNearZero(const std::array<double, 7> &tau,
                             double thresh = 0.05) {
  double max_abs = 0.0;
  for (double t : tau) {
    max_abs = std::max(max_abs, std::abs(t));
  }
  return max_abs < thresh;
}

std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
CreateTorqueFromCartesianSpaceCallback(
    const std::shared_ptr<SharedMemory> &global_handler,
    const std::shared_ptr<robot_utils::StatePublisher> state_publisher,
    const franka::Model &model, std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info, const int &policy_rate,
    const int &traj_rate) {
  return [&global_handler, &state_publisher, &model, &current_state_info,
          &goal_state_info, &policy_rate,
          &traj_rate](const franka::RobotState &robot_state,
                      franka::Duration period) -> franka::Torques {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();

    current_state_info->joint_positions =
        Eigen::VectorXd::Map(robot_state.q.data(), 7);
    Eigen::Affine3d current_T_EE_in_base_frame(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    current_state_info->pos_EE_in_base_frame
        << current_T_EE_in_base_frame.translation();
    current_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(current_T_EE_in_base_frame.linear());

    if (!global_handler->running) {
      auto tau_decay = RampTorqueDown(robot_state);
      franka::Torques output(tau_decay);
      if (TorqueIsNearZero(tau_decay)) {
        return franka::MotionFinished(output);
      }
      return output;
    }

    std::array<double, 7> tau_d_array{};
    if (global_handler->time == 0.) {
      global_handler->traj_interpolator_ptr->Reset(
          0., current_state_info->pos_EE_in_base_frame,
          current_state_info->quat_EE_in_base_frame,
          goal_state_info->pos_EE_in_base_frame,
          goal_state_info->quat_EE_in_base_frame, policy_rate, traj_rate,
          global_handler->traj_interpolator_time_fraction);
    }
    global_handler->time += period.toSec();

    Eigen::Vector3d desired_pos_EE_in_base_frame;
    Eigen::Quaterniond desired_quat_EE_in_base_frame;

    global_handler->traj_interpolator_ptr->GetNextStep(
        global_handler->time, desired_pos_EE_in_base_frame,
        desired_quat_EE_in_base_frame);

    state_publisher->UpdateNewState(robot_state, &model);

    tau_d_array = global_handler->controller_ptr->Step(
        robot_state, desired_pos_EE_in_base_frame,
        desired_quat_EE_in_base_frame);

    std::array<double, 7> tau_d_rate_limited = franka::limitRate(
        franka::kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);
    control_utils::TorqueSafetyGuardFn(tau_d_rate_limited,
                                       global_handler->min_torque,
                                       global_handler->max_torque);
    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    // global_handler->logger->debug("{0} microseconds" , time.count());

    return tau_d_rate_limited;
  };
}

std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
CreateTorqueFromJointSpaceCallback(
    const std::shared_ptr<SharedMemory> &global_handler,
    const std::shared_ptr<robot_utils::StatePublisher> state_publisher,
    const franka::Model &model, std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info, const int &policy_rate,
    const int &traj_rate) {
  return [&global_handler, &state_publisher, &model, &current_state_info,
          &goal_state_info, &policy_rate,
          &traj_rate](const franka::RobotState &robot_state,
                      franka::Duration period) -> franka::Torques {
    std::chrono::high_resolution_clock::time_point t1 =
        std::chrono::high_resolution_clock::now();

    current_state_info->joint_positions =
        Eigen::VectorXd::Map(robot_state.q.data(), 7);
    Eigen::Affine3d current_T_EE_in_base_frame(
        Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    current_state_info->pos_EE_in_base_frame
        << current_T_EE_in_base_frame.translation();
    current_state_info->quat_EE_in_base_frame =
        Eigen::Quaterniond(current_T_EE_in_base_frame.linear());

    if (!global_handler->running) {
      auto tau_decay = RampTorqueDown(robot_state);
      franka::Torques output(tau_decay);
      if (TorqueIsNearZero(tau_decay)) {
        return franka::MotionFinished(output);
      }
      return output;
    }

    std::array<double, 7> tau_d_array{};
    if (global_handler->time == 0.) {
      global_handler->traj_interpolator_ptr->Reset(
          0., current_state_info->pos_EE_in_base_frame,
          current_state_info->quat_EE_in_base_frame,
          goal_state_info->pos_EE_in_base_frame,
          goal_state_info->quat_EE_in_base_frame, policy_rate, traj_rate,
          global_handler->traj_interpolator_time_fraction);
    }
    global_handler->time += period.toSec();

    Eigen::Matrix<double, 7, 1> desired_q;

    global_handler->traj_interpolator_ptr->GetNextStep(global_handler->time,
                                                       desired_q);

    state_publisher->UpdateNewState(robot_state, &model);

    tau_d_array = global_handler->controller_ptr->Step(robot_state, desired_q);

    std::array<double, 7> tau_d_rate_limited = franka::limitRate(
        franka::kMaxTorqueRate, tau_d_array, robot_state.tau_J_d);
    control_utils::TorqueSafetyGuardFn(tau_d_rate_limited,
                                       global_handler->min_torque,
                                       global_handler->max_torque);
    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    // global_handler->logger->debug("{0} microseconds" , time.count());

    return tau_d_rate_limited;
  };
}

} // namespace control_callbacks

#endif // UTILS_CONTROL_CALLBACKS_TORQUE_CALLBACK_H_
