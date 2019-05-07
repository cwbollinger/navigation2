#ifndef BT_ROS_LOGGER_H
#define BT_ROS_LOGGER_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "nav2_msgs/msg/bt_node_status.hpp"

namespace BT
{
/**
 * @brief AddRosLoggerToTree. Give  the root node of a tree,
 * a simple callback is subscribed to any status change of each node.
 *
 *
 * @param root_node
 * @return Important: the returned shared_ptr must not go out of scope,
 *         otherwise the logger is removed.
 */

class RosLogger : public StatusChangeLogger
{

  public:
    RosLogger(TreeNode* root_node, const rclcpp::Node::SharedPtr & node, const std::string & topic_name);
    ~RosLogger() override;

    virtual void callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                          NodeStatus status) override;

    virtual void flush() override;
  private:
    nav2_msgs::msg::BTNodeStatus msg_;
    rclcpp::Publisher<nav2_msgs::msg::BTNodeStatus>::SharedPtr pub_;
};

}   // end namespace

#endif   // BT_ROS_LOGGER_H

