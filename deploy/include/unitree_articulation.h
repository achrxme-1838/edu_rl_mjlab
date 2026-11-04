// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"
#include <functional>
#include <array>
#include <optional>

namespace unitree
{

// Use the same policy -> SDK mapping for reading sensors and writing targets.
template <typename LowCmd>
void write_joint_position_targets(LowCmd& command, const std::vector<int>& ids,
                                 const std::vector<float>& targets)
{
    if (ids.size() != targets.size()) throw std::runtime_error("Joint target size mismatch");
    for (std::size_t i = 0; i < ids.size(); ++i) {
        command.motor_cmd().at(ids[i]).q() = targets[i];
    }
}

template <typename LowStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_, std::function<std::array<float, 2>()> scales = {})
    : lowstate(lowstate_), read_scales_(std::move(scales))
    {
        data.joystick = &*joystick_snapshot;
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        // Newer SDK joystick types are copy-constructible but not assignable.
        data.joystick = &joystick_snapshot.emplace(lowstate->joystick);
        if (read_scales_) data.velocity_scales = read_scales_();
        // base_angular_velocity
        for(int i(0); i<3; i++) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyroscope()[i];
        }
        // project_gravity_body
        data.root_quat_w = Eigen::Quaternionf(
            lowstate->msg_.imu_state().quaternion()[0],
            lowstate->msg_.imu_state().quaternion()[1],
            lowstate->msg_.imu_state().quaternion()[2],
            lowstate->msg_.imu_state().quaternion()[3]
        );
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        // joint positions and velocities
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state().at(data.joint_ids_map[i]).q();
            data.joint_vel[i] = lowstate->msg_.motor_state().at(data.joint_ids_map[i]).dq();
        }
    }

    LowStatePtr lowstate;
    std::optional<unitree::common::UnitreeJoystick> joystick_snapshot{std::in_place};
private:
    std::function<std::array<float, 2>()> read_scales_;
};

}
