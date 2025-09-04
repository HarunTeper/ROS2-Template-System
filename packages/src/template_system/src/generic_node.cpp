#include "template_system/generic_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <iostream>
#include <filesystem>

namespace template_system
{

GenericNode::GenericNode(const rclcpp::NodeOptions& options)
  : Node("generic_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Creating generic node component");
  
  // Load configuration and initialize node
  load_config();
}

void GenericNode::load_config()
{
  // Get the full config file path parameter
  std::string config_file_path = this->declare_parameter<std::string>("config_file_path", "");
  
  if (config_file_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "config_file_path parameter not provided");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file_path.c_str());
  
  // Load the node configuration
  config_ = load_node_config(config_file_path);
  
  // Set the node name from config
  // Note: In component mode, we can't change the node name after construction,
  // but we can log it for verification
  RCLCPP_INFO(this->get_logger(), "Configured as node: %s", config_.name.c_str());

  // Initialize variables
  for (const auto& var : config_.variables) {
    variables_[var.name] = std_msgs::msg::Header();
    variables_[var.name].stamp = this->now();
    variables_[var.name].frame_id = config_.name + "_" + var.name;
    RCLCPP_INFO(this->get_logger(), "Initialized variable: %s", var.name.c_str());
  }

  // Create publishers
  for (const auto& pub : config_.publishers) {
    auto publisher = this->create_publisher<std_msgs::msg::String>(pub.topic, 10);
    publishers_[pub.topic] = publisher;
    RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", pub.topic.c_str());
  }

  // Create subscriptions
  for (const auto& sub : config_.subscriptions) {
    auto subscription = this->create_subscription<std_msgs::msg::String>(
      sub.topic,
      sub.buffer_size,
      [this, callback_name = sub.callback](const std_msgs::msg::String::SharedPtr msg) {
        this->subscription_callback(msg, callback_name);
      }
    );
    subscriptions_.push_back(subscription);
    RCLCPP_INFO(this->get_logger(), "Created subscription for topic: %s", sub.topic.c_str());
  }

  // Create timers
  for (const auto& timer : config_.timers) {
    auto timer_obj = this->create_wall_timer(
      std::chrono::duration<double>(timer.period),
      [this, callback_name = timer.callback]() {
        this->timer_callback(callback_name);
      }
    );
    timers_.push_back(timer_obj);
    RCLCPP_INFO(this->get_logger(), "Created timer with period: %f seconds", timer.period);
  }
}

void GenericNode::timer_callback(const std::string& callback_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Timer callback: %s", callback_name.c_str());
  
  auto callback_it = config_.callbacks.find(callback_name);
  if (callback_it != config_.callbacks.end()) {
    execute_callback(callback_it->second);
  } else {
    RCLCPP_WARN(this->get_logger(), "Callback not found: %s", callback_name.c_str());
  }
}

void GenericNode::subscription_callback(const std_msgs::msg::String::SharedPtr msg, const std::string& callback_name)
{
  RCLCPP_DEBUG(this->get_logger(), "Subscription callback: %s", callback_name.c_str());
  (void)msg; // To avoid unused variable warning 
  auto callback_it = config_.callbacks.find(callback_name);
  if (callback_it != config_.callbacks.end()) {
    execute_callback(callback_it->second);
  } else {
    RCLCPP_WARN(this->get_logger(), "Callback not found: %s", callback_name.c_str());
  }
}

void GenericNode::execute_callback(const CallbackConfig& callback)
{
  auto start_time = this->now();
  
  // Read variables at the beginning
  for (const auto& var_name : callback.variables_to_read) {
    auto it = variables_.find(var_name);
    if (it != variables_.end()) {
      RCLCPP_DEBUG(this->get_logger(), "Reading variable: %s", var_name.c_str());
    }
  }

  // Simulate execution time
  simulate_execution_time(callback.execution_time);

  // Write to variables at the end
  for (const auto& var_name : callback.variables_to_write) {
    auto it = variables_.find(var_name);
    if (it != variables_.end()) {
      it->second.stamp = this->now();
      RCLCPP_DEBUG(this->get_logger(), "Writing to variable: %s", var_name.c_str());
    }
  }

  // Publish to specified topics at the end
  for (const auto& topic : callback.publishers_to_publish) {
    auto pub_it = publishers_.find(topic);
    if (pub_it != publishers_.end()) {
      auto message = std_msgs::msg::String();
      message.data = std::string("Message from ") + this->get_name() + " at " + std::to_string(this->now().seconds());
      pub_it->second->publish(message);
      RCLCPP_DEBUG(this->get_logger(), "Published to topic: %s", topic.c_str());
    }
  }

  auto end_time = this->now();
  RCLCPP_DEBUG(this->get_logger(), "Callback execution took: %f seconds", 
               (end_time - start_time).seconds());
}

void GenericNode::simulate_execution_time(double execution_time)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  auto target_duration = std::chrono::duration<double>(execution_time);
  
  // Busy wait loop to simulate execution time
  while (true) {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = current_time - start_time;
    
    if (elapsed >= target_duration) {
      break;
    }
  }
}

NodeConfig load_node_config(const std::string& config_file)
{
  NodeConfig node_config;
  
  try {
    YAML::Node yaml_config = YAML::LoadFile(config_file);
    
    // For single node config files, load directly from root
    node_config.name = yaml_config["name"].as<std::string>();
    
    // Load timers
    if (yaml_config["timers"]) {
      for (const auto& timer_yaml : yaml_config["timers"]) {
        TimerConfig timer;
        timer.period = timer_yaml["period"].as<double>();
        timer.callback = timer_yaml["callback"].as<std::string>();
        node_config.timers.push_back(timer);
      }
    }
    
    // Load subscriptions
    if (yaml_config["subscriptions"]) {
      for (const auto& sub_yaml : yaml_config["subscriptions"]) {
        SubscriptionConfig sub;
        sub.topic = sub_yaml["topic"].as<std::string>();
        sub.buffer_size = sub_yaml["buffer_size"].as<int>();
        sub.callback = sub_yaml["callback"].as<std::string>();
        node_config.subscriptions.push_back(sub);
      }
    }
    
    // Load publishers
    if (yaml_config["publishers"]) {
      for (const auto& pub_yaml : yaml_config["publishers"]) {
        PublisherConfig pub;
        pub.topic = pub_yaml["topic"].as<std::string>();
        node_config.publishers.push_back(pub);
      }
    }
    
    // Load variables
    if (yaml_config["variables"]) {
      for (const auto& var_yaml : yaml_config["variables"]) {
        VariableConfig var;
        var.type = var_yaml["type"].as<std::string>();
        var.name = var_yaml["name"].as<std::string>();
        node_config.variables.push_back(var);
      }
    }
    
    // Load callbacks
    if (yaml_config["callbacks"]) {
      for (const auto& callback_yaml : yaml_config["callbacks"]) {
        std::string callback_name = callback_yaml.first.as<std::string>();
        CallbackConfig callback;
        
        auto callback_data = callback_yaml.second;
        callback.execution_time = callback_data["execution_time"].as<double>();
        
        if (callback_data["publishers_to_publish"]) {
          for (const auto& pub : callback_data["publishers_to_publish"]) {
            callback.publishers_to_publish.push_back(pub.as<std::string>());
          }
        }
        
        if (callback_data["variables_to_write"]) {
          for (const auto& var : callback_data["variables_to_write"]) {
            callback.variables_to_write.push_back(var.as<std::string>());
          }
        }
        
        if (callback_data["variables_to_read"]) {
          for (const auto& var : callback_data["variables_to_read"]) {
            callback.variables_to_read.push_back(var.as<std::string>());
          }
        }
        
        node_config.callbacks[callback_name] = callback;
      }
    }
    
  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading node YAML config: " << e.what() << std::endl;
  }
  
  return node_config;
}

} // namespace template_system

// Register the component
RCLCPP_COMPONENTS_REGISTER_NODE(template_system::GenericNode)
