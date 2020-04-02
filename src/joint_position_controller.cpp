/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>

namespace panda_simulator {

class JointPositionController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
  bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n) {
    std::vector<std::string> joint_names;
    if (!n.getParam("joint_names", joint_names)) {
      ROS_ERROR("Could not read joint names from param server");
      return false;
    }

    // retrieve gains
    if (!n.getParam("gains", gains_vec_)) {
      ROS_ERROR("Could not read joint gains from param server");
      return false;
    }

    for (auto &joint_name : joint_names) {
      joint_handles_.push_back(hw->getHandle(joint_name));
    }

    for (auto &joint_handle : joint_handles_) {
      command_.push_back(joint_handle.getPosition());
    }

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>(std::string("command"), 1,
                                                            &JointPositionController::setCommandCallback, this);

    return true;
  }

  void update(const ros::Time &time, const ros::Duration &period) {
    for (size_t i = 0; i < joint_handles_.size(); i++) {
      double error = command_.at(i) - joint_handles_.at(i).getPosition();
      double commanded_effort = error * gains_vec_.at(i);

      joint_handles_.at(i).setCommand(commanded_effort);
    }
  }

  void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg) { command_ = msg->data; }

  void starting(const ros::Time &time) {}

  void stopping(const ros::Time &time) {}

private:
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<double> gains_vec_;
  std::vector<double> command_;
  ros::Subscriber sub_command_;
};

PLUGINLIB_EXPORT_CLASS(panda_simulator::JointPositionController, controller_interface::ControllerBase);

} // namespace panda_simulator