import threading
from transitions import Machine
from rcl_interfaces.msg import SetParametersResult
from okmr_automated_planner.state_node import StateNode
from okmr_automated_planner.movement_command_action_client import (
    MovementCommandActionClient,
)
from typing import cast


class BaseStateMachine(Machine):
    """
    Base class for all state machines in the OKMR automated planner.
    Inherits from transitions.Machine and adds ROS 2 integration,
    resource management, and parameter handling.
    """

    mandatory_states = ["uninitialized", "initializing", "done", "aborted"]
    mandatory_transitions = [
        {"trigger": "initialize", "source": "uninitialized", "dest": "initializing"},
        {"trigger": "initialize", "source": "done", "dest": "initializing"},
        {"trigger": "finish", "source": "*", "dest": "done"},
        {"trigger": "abort", "source": "*", "dest": "aborted"},
    ]

    PARAMETERS = []

    def __init__(
        self,
        name,
        ros_node,
        states,
        transitions,
        success_callback=None,
        fail_callback=None,
        *args,
        **kwargs,
    ):
        self.state_cls = StateNode
        self.machine_name = name
        self.ros_node = ros_node
        self.initial_states_config = states
        self.transitions = transitions

        self.success_callback = success_callback
        self.fail_callback = fail_callback

        self._subscriptions = []
        self._publishers = []
        self._timers = {}
        self._clients = []

        self.movement_client = MovementCommandActionClient(self.ros_node)

        self.initial_start_time = None
        self.state_start_time = None

        self.record_initial_start_time()
        self.record_state_start_time()

        # Method to execute immediately after a state transition is complete
        self.queued_method = None

        self.add_mandatory_states()
        self.add_mandatory_transitions()

        super().__init__(
            model=self,
            states=self.initial_states_config,
            transitions=self.transitions,
            initial="uninitialized",
            after_state_change="post_state_change",
            before_state_change="pre_state_change",
            *args,
            **kwargs 
        )

        self.current_sub_machine = None

        period = self.get_global_parameter("state_timeout_check_period")
        self.add_timer("state_timeout_check", period, self.state_timeout_check_callback)

        for param in self.PARAMETERS:
            self.ros_node.declare_ros2_parameter(
                f'{self.machine_name}.{param["name"]}',
                param["value"],
                param["descriptor"],
            )

        self.ros_node.add_on_set_parameters_callback(self._on_parameters_updated)

    def get_local_parameter(self, name):
        """
        Get the value of a parameter scoped to this state machine.
        Example: If machine_name is 'gate' and name is 'speed', looks for 'gate.speed'.
        """
        return self.ros_node.get_parameter(f"{self.machine_name}.{name}").value

    def get_global_parameter(self, name):
        """Get the value of a global ROS 2 parameter."""
        return self.ros_node.get_parameter(f"{name}").value

    def get_current_state_node(self):
        """Return the StateNode object corresponding to the current state."""
        return cast(StateNode, self.get_state(self.state))

    def state_timeout_check_callback(self):
        """
        Timer callback to check if the current state has exceeded its timeout duration.
        Triggers 'abort' if the timeout is reached.
        """
        time_since_state_start = self.get_time_since_state_start()
        timeout = self.get_current_state_node().timeout

        if timeout > 0 and time_since_state_start >= timeout:
            self.ros_node.get_logger().warn(
                f"State Timed Out after {time_since_state_start:.2f}s "
                + f"\t Machine: {self.machine_name} \t State: {self.state}"
            )
            self.abort()

    def add_mandatory_transitions(self):
        """Ensure standard transitions (initialize, finish, abort) exist."""
        all_triggers = [transition["trigger"] for transition in self.transitions]
        
        for transition in self.mandatory_transitions:
            if transition["trigger"] not in all_triggers:
                self.transitions.append(transition)

    def add_mandatory_states(self):
        """Ensure standard states (uninitialized, done, aborted) exist."""
        state_names = [state.name for state in self.initial_states_config]
        
        for state in self.mandatory_states:
            if state not in state_names:
                self.initial_states_config.append(StateNode(state))

    def record_initial_start_time(self):
        """Record the ROS time when the machine was initialized."""
        self.initial_start_time = self.ros_node.get_clock().now()

    def record_state_start_time(self):
        """Record the ROS time when the current state was entered."""
        self.state_start_time = self.ros_node.get_clock().now()

    def get_time_since_initial_start(self):
        """Return seconds elapsed since the state machine started."""
        return (
            self.ros_node.get_clock().now() - self.initial_start_time
        ).nanoseconds / 1e9

    def get_time_since_state_start(self):
        """Return seconds elapsed since the current state began."""
        return (
            self.ros_node.get_clock().now() - self.state_start_time
        ).nanoseconds / 1e9

    def on_completion(self):
        """
        Hook called when the state machine finishes or aborts.
        Override in subclasses to perform cleanup (e.g. disabling object detection).
        """
        pass

    def pre_state_change(self):
        """Internal callback executed before a state transition."""
        self.log_pre_state_change()

    def post_state_change(self):
        """
        Internal callback executed after a state transition.
        Updates timing and executes queued methods.
        """
        self.record_state_start_time()
        self.log_post_state_change()

        # Check completion, otherwise run queued method if exists
        if not self.check_completion() and self.queued_method:
            self.warn_auto_queued_method()
            method = self.queued_method
            self.queued_method = None
            method()

    def log_pre_state_change(self):
        """Log debug info before changing state."""
        self.ros_node.get_logger().debug(
            f"Pre State Change  \t Machine: {self.machine_name} \t From: {self.state}"
        )

    def log_post_state_change(self):
        """Log debug info after changing state."""
        self.ros_node.get_logger().debug(
            f"Post State Change \t Machine: {self.machine_name} \t To: {self.state}"
        )

    def warn_auto_queued_method(self):
        """Logs a warning when a method is auto-queued, helpful for debugging flow."""
        if self.queued_method is None:
            return

        queued_func_name = "Unknown"
        try:
            queued_func_name = self.queued_method.func.__self__.name
        except AttributeError:
            try:
                queued_func_name = self.queued_method.__name__
            except AttributeError:
                pass
        
        self.ros_node.get_logger().warn(
            f"Auto Queued: ({self.machine_name}) {self.state} -> {queued_func_name}"
        )

    def check_completion(self):
        """
        Checks if the machine has reached a terminal state (done/aborted).
        Handles cleanup and callbacks if so.
        """
        if self.is_done() or self.is_aborted():
            self.on_completion()
            self.cleanup_ros2_resources()

            # Abort any running sub-machine
            if self.current_sub_machine and not (
                self.current_sub_machine.is_aborted()
                or self.current_sub_machine.is_done()
            ):
                self.current_sub_machine.abort()
                self.current_sub_machine = None

            # Execute completion callbacks
            if self.is_done() and self.success_callback:
                self.success_callback()
            elif self.is_aborted() and self.fail_callback:
                self.fail_callback()
            return True
        return False

    def start_current_state_sub_machine(
        self, success_callback=None, fail_callback=None
    ):
        """
        Starts the sub-state machine associated with the current state (defined in config).
        """
        current_state = cast(StateNode, self.get_state(self.state))
        
        sub_machine = current_state.sub_machine
        self._start_sub_machine(sub_machine, success_callback, fail_callback)

    def _start_sub_machine(
        self, sub_machine, success_callback=None, fail_callback=None
    ):
        """Internal method to initialize and run a sub-machine."""
        self.current_sub_machine = sub_machine
        sub_machine.success_callback = success_callback
        sub_machine.fail_callback = fail_callback
        threading.Thread(target=sub_machine.initialize, daemon=True).start()


    def _on_parameters_updated(self, params):
        """
        Callback for dynamic parameter updates. 
        Automatically updates class instance variables that match parameter names.
        """
        prefix = f"{self.machine_name}."

        for param in params:
            # Only process parameters belonging to this state machine
            if param.name.startswith(prefix):
                # Extract variable name (e.g., 'machine.speed' -> 'speed')
                local_name = param.name[len(prefix):]

                # Update the attribute if it exists
                if hasattr(self, local_name):
                    setattr(self, local_name, param.value)
                    self.ros_node.get_logger().info(
                        f"Live Parameter Update: {self.machine_name}.{local_name} = {param.value}"
                    )

        return SetParametersResult(successful=True)

    # -------------------------------------------------------------------------
    # ROS 2 Resource Management
    # -------------------------------------------------------------------------

    def add_subscription(self, msg_type, topic, callback):
        """Create and track a ROS 2 subscription."""
        sub = self.ros_node.create_subscription(msg_type, topic, callback, 10)
        self._subscriptions.append(sub)
        return sub

    def remove_subscription(self, topic):
        """Remove and destroy a subscription by topic name."""
        matching_subs = [
            sub
            for sub in self._subscriptions
            if sub.topic_name == topic or sub.topic_name == "/" + topic
        ]
        for sub in matching_subs:
            try:
                self._subscriptions.remove(sub)
                self.ros_node.destroy_subscription(sub)
            except Exception:
                pass

    def add_publisher(self, msg_type, topic):
        """Create and track a ROS 2 publisher."""
        pub = self.ros_node.create_publisher(msg_type, topic, 10)
        self._publishers.append(pub)
        return pub

    def publish_on_topic(self, msg_type, topic_name, msg):
        """Publish a message, creating the publisher if it doesn't exist."""
        matching_pubs = [
            pub
            for pub in self._publishers
            if pub.topic_name == topic_name or pub.topic_name == "/" + topic_name
        ]
        
        if len(matching_pubs) > 0:
            pub = matching_pubs[0]
        else:
            pub = self.add_publisher(msg_type, topic_name)
        pub.publish(msg)

    def add_timer(self, name: str, duration, callback):
        """Create and track a ROS 2 timer."""
        timer = self.ros_node.create_timer(duration, callback)
        self._timers[f"{self.machine_name}_{name}"] = timer
        return timer

    def remove_timer(self, name):
        """Remove and destroy a timer by name."""
        key = f"{self.machine_name}_{name}"
        if key in self._timers:
            try:
                self.ros_node.destroy_timer(self._timers[key])
                self._timers.pop(key)
            except Exception as e:
                self.ros_node.get_logger().error(f"Failed to destroy timer {name}: {e}")

    def add_service_client(self, service_type, service_name):
        """Create and track a ROS 2 service client."""
        cli = self.ros_node.create_client(service_type, service_name)
        self._clients.append({service_name: cli})
        return cli

    def send_service_request(self, service_type, service_name, srv_msg, done_callback):
        """Send an async service request."""
        cli = None
        # Check if client already exists
        for client_dict in self._clients:
            if service_name in client_dict:
                cli = client_dict[service_name]
                break
        
        if cli is None:
            cli = self.add_service_client(service_type, service_name)
            
        cli.call_async(srv_msg).add_done_callback(done_callback)

    def cleanup_ros2_subscriptions(self):
        """Destroy all tracked subscriptions."""
        for sub in self._subscriptions:
            try:
                self.ros_node.destroy_subscription(sub)
            except Exception:
                pass
        self._subscriptions.clear()

    def cleanup_ros2_publishers(self):
        """Destroy all tracked publishers."""
        for pub in self._publishers:
            try:
                self.ros_node.destroy_publisher(pub)
            except Exception:
                pass
        self._publishers.clear()

    def cleanup_ros2_timers(self):
        """Destroy all tracked timers."""
        for name in self._timers:
            try:
                self.ros_node.destroy_timer(self._timers[name])
            except Exception as e:
                self.ros_node.get_logger().error(f"Failed to destroy timer: {name}, {e}")
        self._timers.clear()

    def cleanup_ros2_clients(self):
        """Clear service clients (destruction handled by node usually)."""
        self._clients.clear()

    def cleanup_movement_client(self):
        """Clean up the movement action client."""
        self.movement_client.cleanup()

    def cleanup_ros2_resources(self):
        """
        Cleans up all ROS 2 resources specific to this state machine instance.
        """
        self.cleanup_ros2_subscriptions()
        self.cleanup_ros2_publishers()
        self.cleanup_ros2_timers()
        self.cleanup_ros2_clients()
        self.cleanup_movement_client()