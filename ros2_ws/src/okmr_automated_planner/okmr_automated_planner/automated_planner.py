#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from okmr_automated_planner.state_machine_factory import StateMachineFactory
from okmr_utils.logging import make_green_log 

class AutomatedPlannerNode(Node):
    """
    Main node for the Automated Planner.
    Initializes the state machine based on YAML configuration.
    """
    
    DEFAULT_PARAMS = [
        {
            'name': 'state_timeout_check_period', 
            'value': 0.5, 
            'descriptor': 'Frequency in seconds to check for state timeouts'
        },
        {
            'name': 'config_base_path', 
            'value': '', 
            'descriptor': 'Base path for config files (e.g. share/state_machine_configs/dev)'
        },
        {
            'name': 'root_config', 
            'value': 'root.yaml', 
            'descriptor': 'Root configuration file name'
        }
    ]

    def __init__(self):
        # allow_undeclared_parameters=True allows the state machines to dynamically declare their own parameters later
        super().__init__("automated_planner", allow_undeclared_parameters=True)
        
        for param in self.DEFAULT_PARAMS:
            self._declare_parameter_with_log(
                param['name'], 
                param['value'], 
                param['descriptor']
            )

    def _declare_parameter_with_log(self, name, value, description):
        """Helper to declare a ROS 2 parameter with a descriptor and debug log."""
        descriptor = ParameterDescriptor(
            type=Parameter.Type.from_parameter_value(value),
            description=description
        )
        self.declare_parameter(name, value, descriptor)
        self.get_logger().debug(f"Registered parameter: {name}")


def main(args=None):
    rclpy.init(args=args)
    node = AutomatedPlannerNode()
    
    try:
        config_base_path = str(node.get_parameter("config_base_path").value)
        root_config = str(node.get_parameter("root_config").value)

        node.get_logger().info(f"Config Base Path: {config_base_path}")
        node.get_logger().info(f"Root Config: {make_green_log(root_config)}")

        try:
            root_state_machine = StateMachineFactory.createMachineFromConfig(
                root_config, 
                node, 
                config_base_path
            )
        except Exception as e:
            node.get_logger().fatal(f"Failed to create state machine: {e}")
            raise e

        machine_thread = threading.Thread(
            target=root_state_machine.initialize, 
            daemon=True
        )
        machine_thread.start()
        
        node.get_logger().info(make_green_log("Root State Machine Started"))

        while rclpy.ok():
            # Check if machine has reached a terminal state
            if root_state_machine.is_aborted() or root_state_machine.is_done():
                node.get_logger().info("State machine finished or aborted. Exiting.")
                break
            
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()