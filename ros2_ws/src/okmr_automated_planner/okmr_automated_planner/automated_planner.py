#!/usr/bin/env python3

import rclpy
import threading
from rclpy.node import Node
from rclpy.parameter import Parameter
from okmr_automated_planner.state_machine_factory import StateMachineFactory
from rcl_interfaces.msg import ParameterDescriptor
from okmr_utils.logging import make_green_log

class AutomatedPlannerNode(Node):
    #hardcoded parameters to declare on startup
    PARAMETERS = [
        {'name': 'state_timeout_check_period', 'value': 0.5, 'descriptor': 'How often to check if the current state should timeout'},
        {'name': 'config_share_path', 'value': '', 'descriptor': 'Base path for all configuration files, usually share directory/state_machine_configs'},
        {'name': 'config_folder', 'value': '', 'descriptor': 'Actual machine name. i.e dev, competition.'},
        {'name': 'root_config', 'value': 'root.yaml', 'descriptor': 'Root configuration file name (relative to config_base_path)'}
    ]

    def __init__(self):
        super().__init__("automated_planner", allow_undeclared_parameters=True)
        
        # Declare all parameters from the list
        for param in self.PARAMETERS:
            self.declare_ros2_parameter(param['name'], param['value'], param['descriptor'])

    def declare_ros2_parameter(self, name, value=None, description=None, ignore_override=False):
        """
        Declare a parameter and automatically create a getter method for it.
        allows you to call get_{parameter_name}() to fetch value 
        """
        
        if self.has_parameter(name):
            return self.get_parameter(name)
        
        descriptor = ParameterDescriptor(type=Parameter.Type.from_parameter_value(value),
                                                      description=description)
        result = self.declare_parameter(name, value, descriptor, ignore_override)

        self.get_logger().debug(f"Registering parameter: {name}")
        
        return result
    
def main():
    rclpy.init()
    master_node = AutomatedPlannerNode()
    config_share_path = master_node.get_parameter("config_share_path").value
    config_folder = master_node.get_parameter("config_folder").value
    root_config = master_node.get_parameter("root_config").value
    root_state_machine = None

    master_node.get_logger().info(f"Using config base path: {config_share_path}")
    master_node.get_logger().info(f"Using root config: {make_green_log(root_config)}")

    try:
        root_state_machine = StateMachineFactory.createMachineFromConfig(
            master_node,
            root_config,
            config_share_path,
            config_folder
        )
        # root_state_machine.fail_callback =  
        #root_state_machine.done_callback = lamda: 
    except Exception as e:
        master_node.get_logger().fatal(f"Error when parsing config: \n{str(e)}")
        rclpy.shutdown()
        return
    
    #used for drawing the graph
    #dot_graph = robot.get_graph()
    #dot_graph.draw('auv_fsm.dot', prog='dot')

    threading.Thread(target=root_state_machine.initialize, daemon=True).start()
    master_node.get_logger().info(make_green_log("Started root state machine"))
    master_done = False
    try:
        while rclpy.ok() and not root_state_machine.is_aborted() and not root_state_machine.is_done():
            rclpy.spin_once(master_node, timeout_sec=0.1)
    except Exception as e:
        master_node.get_logger().error(f"{str(e)}")
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
