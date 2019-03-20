// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/blackboard/blackboard_local.h"
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"
#include "nav2_lifecycle/lifecycle_node.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_tasks/navigate_to_pose_task.hpp"

namespace nav2_bt_navigator
{

class BtNavigator : public nav2_lifecycle::LifecycleNode
{
public:
  BtNavigator();
  ~BtNavigator();

protected:
  // The lifecycle node interface
  nav2_lifecycle::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  nav2_lifecycle::CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  // The NavigateToPose interface (a task server that implements the NavigateToPose command)
  std::unique_ptr<nav2_tasks::NavigateToPoseTaskServer> task_server_;
  nav2_tasks::TaskStatus navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command);

private:
  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The goal (on the blackboard) to be passed to ComputePath
  std::shared_ptr<nav2_tasks::ComputePathToPoseCommand> goal_;

  // The path (on the blackboard) to be returned from ComputePath and sent to the FollowPath task
  std::shared_ptr<nav2_tasks::ComputePathToPoseResult> path_;

  // The XML string that defines the Behavior Tree to create
  std::string xml_string_;

  // The wrapper class for the BT functionality
  std::unique_ptr<NavigateToPoseBehaviorTree> bt_;

  // The complete behavior tree that results from parsing the incoming XML
  BT::Tree tree_;
};

}  // namespace nav2_bt_navigator

#endif  // NAV2_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
