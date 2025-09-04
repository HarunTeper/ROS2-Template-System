#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def load_system_config(context, *args, **kwargs):
    """Load system configuration and create nodes dynamically"""
    
    # Get launch configurations
    system_config_file = LaunchConfiguration('system_config_file').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    config_dir = LaunchConfiguration('config_dir').perform(context)
    
    # Build full path to system config file
    system_config_path = os.path.join(config_dir, system_config_file)
    
    print(f"Loading system config from: {system_config_path}")
    
    # Load system configuration
    with open(system_config_path, 'r') as file:
        system_config = yaml.safe_load(file)
    
    actions = []
    
    # Create executors and nodes based on configuration
    executors = system_config.get('executors', [])
    nodes_config = system_config.get('nodes', [])
    
    # Create a map of node name to config
    node_config_map = {node['name']: node for node in nodes_config}
    
    for executor in executors:
        executor_name = executor['name']
        executor_type = executor['type']
        executor_nodes = executor.get('nodes', [])
        
        print(f"Creating {executor_type} executor '{executor_name}' with nodes: {executor_nodes}")
        
        # Create composable nodes for this executor
        composable_nodes = []
        
        for node_name in executor_nodes:
            if node_name in node_config_map:
                node_config = node_config_map[node_name]
                config_file = node_config.get('config_file', f"{node_name}_config.yaml")
                # Build full path to config file
                full_config_path = os.path.join(config_dir, config_file)
                node_namespace = node_config.get('namespace', '')
                
                # Combine global and node-specific namespace
                full_namespace = ""
                if namespace:
                    if node_namespace:
                        full_namespace = f"{namespace}/{node_namespace}"
                    else:
                        full_namespace = namespace
                elif node_namespace:
                    full_namespace = node_namespace
                
                composable_node = ComposableNode(
                    package='template_system',
                    plugin='template_system::GenericNode',
                    name=node_name,
                    namespace=full_namespace,
                    parameters=[{
                        'config_file_path': full_config_path
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
                composable_nodes.append(composable_node)
                print(f"  - Added node '{node_name}' with namespace '{full_namespace}' and config '{config_file}'")
        
        if composable_nodes:
            # Create container for this executor
            container_name = f"{executor_name}_container"
            
            if executor_type == "single-threaded":
                executor_options = {
                    'use_global_arguments': False,
                    'enable_logging': True,
                }
            else:  # multi-threaded
                executor_options = {
                    'use_global_arguments': False,
                    'enable_logging': True,
                    'num_threads': 4,  # Configurable if needed
                }
            
            container = ComposableNodeContainer(
                name=container_name,
                namespace="",  # Don't set namespace on container since nodes already have it
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=composable_nodes,
                output='screen',
                parameters=[executor_options]
            )
            
            actions.append(container)
    
    return actions

def generate_launch_description():
    # Declare launch arguments
    system_config_file_arg = DeclareLaunchArgument(
        'system_config_file',
        default_value='system.yaml',
        description='System configuration YAML file name (without path)'
    )
    
    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value='/home/ubuntu/workspace/config',
        description='Directory containing configuration files'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for the nodes'
    )
    
    # Use OpaqueFunction to dynamically generate actions based on config
    dynamic_nodes = OpaqueFunction(function=load_system_config)
    
    return LaunchDescription([
        system_config_file_arg,
        config_dir_arg,
        namespace_arg,
        dynamic_nodes
    ])
