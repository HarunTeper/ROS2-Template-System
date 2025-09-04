# ROS2-Template-System

A component-based template system for creating synthetic ROS 2 systems through YAML configuration. This system provides a generic ROS 2 node that can be configured to simulate different behaviors and deployed using Docker Compose.

## Features

- **Component-Based Architecture**: Uses ROS 2 components for dynamic node loading
- **YAML Configuration**: Separate configuration files for system topology and individual nodes
- **Docker Compose**: Containerized deployment with simple command execution
- **Flexible Node Behavior**: Configurable timers, publishers, subscribers, and variables
- **Namespace Support**: Configurable ROS namespaces for multi-robot scenarios

## Quick Start

### 1. Prerequisites

- Docker and Docker Compose
- X11 forwarding for GUI applications (optional)

### 2. Configure the System

Edit the `.env` file to configure your system:

```bash
# System configuration
SYSTEM_CONFIG_FILE=system.yaml           # Defines system topology
TEMPLATE_NAMESPACE=template_ns            # ROS namespace
ROS_DOMAIN_ID=0                          # ROS domain ID
```

### 3. Run the Template System

```bash
# Run the template system
docker compose run template_system
```

That's it! The system will:

1. Build the container if needed
2. Load the system configuration from `system.yaml`
3. Create multiple executors with nodes as defined in the configuration
4. Run until you press Ctrl+C

## Configuration

The template system uses two types of configuration files:

### 1. System Configuration (`system.yaml`)

Defines the overall system topology - which executors to create and which nodes run in each executor:

```yaml
system:
  name: "multi_node_template_system"

executors:
  - name: "executor_1"
    type: "single-threaded"
    nodes: ["node1"]
  - name: "executor_2" 
    type: "single-threaded"
    nodes: ["node2"]
  - name: "executor_3"
    type: "single-threaded"
    nodes: ["node3"]

nodes:
  - name: "node1"
    config_file: "node1_config.yaml"
  - name: "node2"
    config_file: "node2_config.yaml"
  - name: "node3"
    config_file: "node3_config.yaml"
```

### 2. Node Configuration Files

Each node has its own configuration file (e.g., `node1_config.yaml`) that defines its behavior:

```yaml
name: "node1"
timers:
  - period: 1.0
    callback: "timer_callback_1"
subscriptions: []
publishers:
  - topic: "/topic1"
variables:
  - type: "std_msgs::Header"
    name: "var1"
callbacks:
  timer_callback_1:
    execution_time: 0.1
    publishers_to_publish: ["/topic1"]
    variables_to_write: ["var1"]
    variables_to_read: []
```

## Customizing Your System

### 1. Modify System Topology

Edit `packages/src/template_system/config/system.yaml` to:
- Change number of executors
- Assign different nodes to executors
- Add or remove nodes

### 2. Modify Node Behavior

Edit individual node config files in `packages/src/template_system/config/` to:
- Change timer periods
- Add/remove publishers and subscribers
- Modify callback execution times
- Configure topic names

### 3. Change Namespace

Edit `.env` file:
```bash
TEMPLATE_NAMESPACE=my_robots  # Changes namespace from template_ns to my_robots
```

## Package Structure

## Template System Architecture

The `template_system` package (located in `packages/src/template_system/`) provides:

- **Generic Node Component**: A configurable ROS 2 component that adapts its behavior based on YAML configuration
- **Launch File**: Dynamic launch file that reads system configuration and creates the appropriate executors and nodes
- **Configuration Files**: YAML files defining system topology and individual node behaviors

## Monitoring Your System

### View Running Topics

```bash
# List all topics
docker exec -it template_system ros2 topic list

# Monitor a specific topic
docker exec -it template_system ros2 topic echo /template_ns/topic1
```

### View Computation Graph

```bash
# Launch RQT graph tool
docker compose run rqt_graph
```

### Check Node Information

```bash
# List running nodes
docker exec -it template_system ros2 node list

# Get node info
docker exec -it template_system ros2 node info /template_ns/node1
```

## Development

### Building in Development Mode

```bash
# Enter development container
docker compose run development

# Inside container - build the package
colcon build --packages-select template_system --symlink-install

# Test the launch file
ros2 launch template_system multi_node_system.launch.py
```

### Validating Configuration

Use the included validation script:

```bash
python3 validate_configs.py
```

## Configuration Reference

### System Configuration Fields

- `system.name`: System identifier
- `executors[]`: List of executor definitions
  - `name`: Executor identifier
  - `type`: "single-threaded" or "multi-threaded"
  - `nodes[]`: List of node names to run in this executor
- `nodes[]`: List of node definitions
  - `name`: Node identifier (must match names used in executors)
  - `config_file`: Path to individual node configuration file

### Node Configuration Fields

- `name`: Node name
- `timers[]`: Timer definitions
  - `period`: Timer period in seconds
  - `callback`: Callback name to execute
- `subscriptions[]`: Subscription definitions
  - `topic`: Topic name to subscribe to
  - `buffer_size`: Queue size for messages
  - `callback`: Callback name to execute on message receipt
- `publishers[]`: Publisher definitions
  - `topic`: Topic name to publish to
- `variables[]`: Variable definitions
  - `type`: Variable type (currently supports "std_msgs::Header")
  - `name`: Variable identifier
- `callbacks{}`: Callback definitions
  - `execution_time`: Simulated processing time in seconds
  - `publishers_to_publish[]`: Topics to publish to during callback
  - `variables_to_write[]`: Variables to update during callback
  - `variables_to_read[]`: Variables to read during callback

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `SYSTEM_CONFIG_FILE` | System topology configuration file | `system.yaml` |
| `TEMPLATE_NAMESPACE` | ROS namespace for all nodes | `template_ns` |
| `ROS_DOMAIN_ID` | ROS 2 domain ID | `0` |
| `ROS_DISTRO` | ROS 2 distribution | `humble` |
| `RMW_IMPLEMENTATION` | ROS middleware | `rmw_zenoh_cpp` |

## Troubleshooting

### Container Issues

```bash
# Rebuild the container
docker compose build template_system

# Clean up containers
docker system prune
```

### Network Issues

```bash
# Check ROS_DOMAIN_ID conflicts
echo $ROS_DOMAIN_ID

# Use different domain ID
ROS_DOMAIN_ID=1 docker compose run template_system
```

### Configuration Issues

```bash
# Validate YAML configuration
python3 validate_configs.py

# Check configuration loading
docker exec -it template_system cat /home/ubuntu/workspace/packages/install/template_system/share/template_system/config/system.yaml
```

## Contributing

1. Fork the repository
2. Create feature branch  
3. Make changes in development container
4. Test with template system
5. Submit pull request

## License

[Specify your license here]
