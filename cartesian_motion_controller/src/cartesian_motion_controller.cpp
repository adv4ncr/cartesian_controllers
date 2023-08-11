////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include <algorithm>
#include <cartesian_motion_controller/cartesian_motion_controller.h>

namespace cartesian_motion_controller
{

CartesianMotionController::CartesianMotionController()
: Base::CartesianControllerBase()
{
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianMotionController::init(const std::string & controller_name)
{
  const auto ret = Base::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  return controller_interface::return_type::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // m_target_frame_subscr = get_node()->create_subscription<CmdType>(
  //   get_node()->get_name() + std::string("/target_frame"),
  //   3,
  //   std::bind(&CartesianMotionController::targetFrameCallback, this, std::placeholders::_1));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);

  // reset command buffer if a command came through callback when controller was inactive
  //m_target_frame_buffer = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  // Reset simulation with real joint state
  m_current_frame = Base::m_ik_solver->getEndEffectorPose();

  // Start where we are
  m_target_frame = m_current_frame;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianMotionController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Stop drifting by sending zero joint velocities
  Base::computeJointControlCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  Base::writeJointControlCmds();
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE
controller_interface::return_type CartesianMotionController::update_and_write_commands(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianMotionController::update_and_write_commands()
#endif
{



  // Get new pose from subscriber
  // const auto _target = m_target_frame_buffer.readFromRT();

  // // No command received yet
  // if (!_target || !(*_target))
  // {
  //   RCLCPP_WARN_ONCE(get_node()->get_logger(), "[MOTION][update] no command received yet");
  //   return controller_interface::return_type::OK;
  // }

  // if(period.nanoseconds() > 4500000)
  // {
  //   RCLCPP_WARN(get_node()->get_logger(), "[MOTION][update] last period: %lf", period.nanoseconds()/1e6);
  // }

  m_use_pref_frame = false;
  for(size_t i = 0; i < reference_interfaces_.size(); i++)
  {
    if(std::isnan(reference_interfaces_[i])) {m_use_pref_frame = true;}
  }

  if(m_use_pref_frame)
  {
    m_target_frame = m_current_frame;
    RCLCPP_ERROR(get_node()->get_logger(), "[MOTION][update] using previous frame");
  }
  else
  {
    m_target_frame = KDL::Frame(
      KDL::Rotation::Quaternion(
        reference_interfaces_[3],
        reference_interfaces_[4],
        reference_interfaces_[5],
        reference_interfaces_[6]),
      KDL::Vector(
        reference_interfaces_[0],
        reference_interfaces_[1],
        reference_interfaces_[2])
    );
  }
  
  // RCLCPP_WARN(get_node()->get_logger(), "[MOTION][update] set pos: %lf %lf %lf", m_target_frame.p.x(), m_target_frame.p.y(), m_target_frame.p.z());
  // std::array<double, 3> _rpy;
  // m_target_frame.M.GetRPY(_rpy[0], _rpy[1], _rpy[2]);
  // RCLCPP_WARN_ONCE(get_node()->get_logger(), "[MOTION][update] set agl: %lf %lf %lf", _rpy[0], _rpy[1], _rpy[2]);
  


  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Forward Dynamics turns the search for the according joint motion into a
  // control process. So, we control the internal model until we meet the
  // Cartesian target motion. This internal control needs some simulation time
  // steps.
  for (int i = 0; i < Base::m_iterations; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    auto internal_period = rclcpp::Duration::from_seconds(0.02);

    // Compute the motion error = target - current.
    ctrl::Vector6D error = computeMotionError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error,internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianMotionController::
computeMotionError()
{
  // Compute motion error wrt robot_base_link
  m_current_frame = Base::m_ik_solver->getEndEffectorPose();

  // Transformation from target -> current corresponds to error = target - current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle    = error_kdl.M.GetRotAngle(rot_axis);   // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle    = std::clamp(angle,-max_angle,max_angle);
  distance = std::clamp(distance,-max_distance,max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error(0) = error_kdl.p.x();
  error(1) = error_kdl.p.y();
  error(2) = error_kdl.p.z();
  error(3) = rot_axis(0);
  error(4) = rot_axis(1);
  error(5) = rot_axis(2);

  return error;
}

// void CartesianMotionController::targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target)
// {
//   if (target->header.frame_id != Base::m_robot_base_link)
//   {
//     auto& clock = *get_node()->get_clock();
//     RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
//         clock, 3000,
//         "Got target pose in wrong reference frame. Expected: %s but got %s",
//         Base::m_robot_base_link.c_str(),
//         target->header.frame_id.c_str());
//     return;
//   }

//   m_target_frame_buffer.writeFromNonRT(target);
// }

// Chainable controller functions

bool CartesianMotionController::on_set_chained_mode(bool chained_mode)
{
  RCLCPP_WARN(get_node()->get_logger(), "[MOTION][on_set_chained_mode] Chained mode: %s", chained_mode ? "true" : "false");

  return true;
}

controller_interface::return_type CartesianMotionController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

} // namespace

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_motion_controller::CartesianMotionController, controller_interface::ChainableControllerInterface)
