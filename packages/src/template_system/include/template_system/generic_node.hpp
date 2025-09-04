#ifndef TEMPLATE_SYSTEM_GENERIC_NODE_HPP_
#define TEMPLATE_SYSTEM_GENERIC_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <map>

namespace template_system
{

struct TimerConfig
{
  double period;
  std::string callback;
};

struct SubscriptionConfig
{
  std::string topic;
  int buffer_size;
  std::string callback;
};

struct PublisherConfig
{
  std::string topic;
};

struct VariableConfig
{
  std::string type;
  std::string name;
};

struct CallbackConfig
{
  double execution_time;
  std::vector<std::string> publishers_to_publish;
  std::vector<std::string> variables_to_write;
  std::vector<std::string> variables_to_read;
};

struct NodeConfig
{
  std::string name;
  std::vector<TimerConfig> timers;
  std::vector<SubscriptionConfig> subscriptions;
  std::vector<PublisherConfig> publishers;
  std::vector<VariableConfig> variables;
  std::map<std::string, CallbackConfig> callbacks;
};

struct ExecutorConfig
{
  std::string type; // "single-threaded" or "multi-threaded"
  std::vector<std::string> nodes;
};

struct SystemConfig
{
  ExecutorConfig executor;
  std::vector<NodeConfig> nodes;
};

class GenericNode : public rclcpp::Node
{
public:
  explicit GenericNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void load_config();
  void timer_callback(const std::string& callback_name);
  void subscription_callback(const std_msgs::msg::String::SharedPtr msg, const std::string& callback_name);
  void execute_callback(const CallbackConfig& callback);
  void simulate_execution_time(double execution_time);

  NodeConfig config_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  std::map<std::string, std_msgs::msg::Header> variables_;
};

NodeConfig load_node_config(const std::string& config_file);

} // namespace template_system

#endif // TEMPLATE_SYSTEM_GENERIC_NODE_HPP_
