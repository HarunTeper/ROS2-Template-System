#!/usr/bin/env python3

"""
Template System Configuration Validator

This script validates YAML configuration files for the template system
to ensure they have the correct structure and required fields.
"""

import yaml
import sys
import os
from pathlib import Path

def validate_node_config(config_path):
    """Validate a single node configuration file."""
    
    print(f"Validating node configuration: {config_path}")
    
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
    except yaml.YAMLError as e:
        print(f"❌ YAML parsing error: {e}")
        return False
    except FileNotFoundError:
        print(f"❌ File not found: {config_path}")
        return False
    
    # Check for required top-level fields
    if 'name' not in config:
        print("❌ Missing required 'name' field")
        return False
    
    node_name = config['name']
    
    # Check for required lists
    required_lists = ['timers', 'subscriptions', 'publishers', 'variables']
    for field in required_lists:
        if field not in config:
            print(f"❌ Missing '{field}' list")
            return False
        
        if not isinstance(config[field], list):
            print(f"❌ '{field}' must be a list")
            return False
    
    # Validate timers
    for j, timer in enumerate(config['timers']):
        if 'period' not in timer or 'callback' not in timer:
            print(f"❌ Timer {j+1}: Missing 'period' or 'callback'")
            return False
        
        if not isinstance(timer['period'], (int, float)):
            print(f"❌ Timer {j+1}: 'period' must be a number")
            return False
    
    # Validate subscriptions
    for j, sub in enumerate(config['subscriptions']):
        required_sub_fields = ['topic', 'buffer_size', 'callback']
        for field in required_sub_fields:
            if field not in sub:
                print(f"❌ Subscription {j+1}: Missing '{field}'")
                return False
        
        if not isinstance(sub['buffer_size'], int):
            print(f"❌ Subscription {j+1}: 'buffer_size' must be an integer")
            return False
    
    # Validate publishers
    for j, pub in enumerate(config['publishers']):
        if 'topic' not in pub:
            print(f"❌ Publisher {j+1}: Missing 'topic'")
            return False
    
    # Validate variables
    for j, var in enumerate(config['variables']):
        if 'type' not in var or 'name' not in var:
            print(f"❌ Variable {j+1}: Missing 'type' or 'name'")
            return False
    
    # Validate callbacks
    if 'callbacks' in config:
        for callback_name, callback in config['callbacks'].items():
            if 'execution_time' not in callback:
                print(f"❌ Callback '{callback_name}': Missing 'execution_time'")
                return False
            
            if not isinstance(callback['execution_time'], (int, float)):
                print(f"❌ Callback '{callback_name}': 'execution_time' must be a number")
                return False
            
            # Optional fields - check if present that they're lists
            optional_lists = ['publishers_to_publish', 'variables_to_write', 'variables_to_read']
            for field in optional_lists:
                if field in callback and not isinstance(callback[field], list):
                    print(f"❌ Callback '{callback_name}': '{field}' must be a list")
                    return False
    
    print(f"✅ Node configuration for '{node_name}' is valid!")
    return True

def validate_system_config(config_path):
    """Validate a system configuration file."""
    
    print(f"Validating system configuration: {config_path}")
    
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
    except yaml.YAMLError as e:
        print(f"❌ YAML parsing error: {e}")
        return False
    except FileNotFoundError:
        print(f"❌ File not found: {config_path}")
        return False
    
    # Validate system section
    if 'system' in config:
        system = config['system']
        if 'name' not in system:
            print("❌ Missing 'name' in system section")
            return False
    
    # Validate executors section
    if 'executors' not in config:
        print("❌ Missing 'executors' section")
        return False
    
    executors = config['executors']
    if not isinstance(executors, list):
        print("❌ 'executors' must be a list")
        return False
    
    if len(executors) == 0:
        print("❌ At least one executor must be defined")
        return False
    
    # Track node names referenced by executors
    referenced_nodes = set()
    
    for i, executor in enumerate(executors):
        print(f"  Validating executor {i+1}...")
        
        if 'name' not in executor:
            print(f"❌ Executor {i+1}: Missing 'name'")
            return False
            
        if 'type' not in executor:
            print(f"❌ Executor {i+1}: Missing 'type'")
            return False
            
        if executor['type'] not in ['single-threaded', 'multi-threaded']:
            print(f"❌ Executor {i+1}: Invalid type '{executor['type']}'")
            return False
            
        if 'nodes' not in executor:
            print(f"❌ Executor {i+1}: Missing 'nodes' list")
            return False
            
        if not isinstance(executor['nodes'], list):
            print(f"❌ Executor {i+1}: 'nodes' must be a list")
            return False
            
        for node_name in executor['nodes']:
            referenced_nodes.add(node_name)
    
    # Validate nodes section
    if 'nodes' not in config:
        print("❌ Missing 'nodes' section")
        return False
    
    nodes = config['nodes']
    if not isinstance(nodes, list):
        print("❌ 'nodes' must be a list")
        return False
    
    if len(nodes) == 0:
        print("❌ At least one node must be defined")
        return False
    
    # Track defined node names
    defined_nodes = set()
    
    for i, node in enumerate(nodes):
        print(f"  Validating node {i+1}...")
        
        if 'name' not in node:
            print(f"❌ Node {i+1}: Missing 'name'")
            return False
            
        node_name = node['name']
        defined_nodes.add(node_name)
        
        if 'config_file' not in node:
            print(f"❌ Node '{node_name}': Missing 'config_file'")
            return False
    
    # Check that all referenced nodes are defined
    for node_name in referenced_nodes:
        if node_name not in defined_nodes:
            print(f"❌ Executor references undefined node: '{node_name}'")
            return False
    
    print("✅ System configuration is valid!")
    return True

def main():
    """Main validation function."""
    
    if len(sys.argv) > 1:
        # Validate specific file
        config_file = sys.argv[1]
        if not validate_config(config_file):
            sys.exit(1)
    else:
        # Validate all config files in the config directory
        config_dir = Path(__file__).parent / "packages" / "src" / "template_system" / "config"
        
        if not config_dir.exists():
            print(f"❌ Config directory not found: {config_dir}")
            sys.exit(1)
        
        yaml_files = list(config_dir.glob("*.yaml"))
        
        if not yaml_files:
            print(f"❌ No YAML files found in: {config_dir}")
            sys.exit(1)
        
        print(f"Found {len(yaml_files)} configuration files to validate:")
        
        all_valid = True
        for yaml_file in yaml_files:
            print(f"\n{'='*50}")
            if not validate_config(yaml_file):
                all_valid = False
        
        print(f"\n{'='*50}")
        if all_valid:
            print("✅ All configuration files are valid!")
        else:
            print("❌ Some configuration files have errors!")
            sys.exit(1)

if __name__ == "__main__":
    main()
