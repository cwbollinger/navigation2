#include "nav2_tasks/bt_ros_logger.h"

namespace BT
{

RosLogger::RosLogger(TreeNode* root_node, const rclcpp::Node::SharedPtr & node, const std::string & topic_name) : StatusChangeLogger(root_node)
{
    pub_ = node->create_publisher<nav2_msgs::msg::BTNodeStatus>(topic_name.c_str());
}

RosLogger::~RosLogger()
{
}

void RosLogger::callback(Duration timestamp, const TreeNode& node, NodeStatus prev_status,
                         NodeStatus status)
{
    using namespace std::chrono;

    msg_.node_name = node.name();
    msg_.status = toStr(status, true);
    msg_.prev_status = toStr(prev_status, true);
    msg_.header.stamp.sec = duration_cast<seconds>(timestamp).count();
    pub_->publish(msg_);
    //msg_->header->stamp->nanosec = duration_cast<nanoseconds>(timestamp).count();
    //double since_epoch = duration<double>(timestamp).count();
    //printf("[%.3f]: %s%s %s -> %s",
    //       since_epoch, node.name().c_str(),
    //       &whitespaces[std::min(ws_count, node.name().size())],
    //       toStr(prev_status, true).c_str(),
    //       toStr(status, true).c_str() );
    //std::cout << std::endl;
}

void RosLogger::flush()
{
}

} // end namespace
