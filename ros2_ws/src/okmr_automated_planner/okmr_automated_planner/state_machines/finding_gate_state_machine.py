from okmr_msgs.msg import MovementCommand, GoalVelocity, BoundingBox
from okmr_msgs.srv import SetInferenceCamera, ChangeModel
from geometry_msgs.msg import Vector3
from okmr_automated_planner.base_state_machine import BaseStateMachine


class FindingGateStateMachine(BaseStateMachine):
    """State machine for finding and centering on the gate"""
    # Defined in finding_gate.yaml

    PARAMETERS = [
        {
            "name": "scan_speed",
            "value": 30.0,
            "descriptor": "angular velocity while scanning for gate (degrees/sec)",
        },
        {
            "name": "scan_angle",
            "value": 360.0,
            "descriptor": "angle to cover while searching for gate (degrees)",
        },
        {
            "name": "image_width",
            "value": 640.0,
            "descriptor": "camera image width in pixels",
        },
        {
            "name": "image_height",
            "value": 480.0,
            "descriptor": "camera image height in pixels",
        },
        {
            "name": "camera_horizontal_fov",
            "value": 90.0,
            "descriptor": "camera horizontal field of view in degrees",
        },
        {
            "name": "centering_threshold_pixels",
            "value": 50.0,
            "descriptor": "maximum pixel offset from center to be considered centered",
        },
        {
            "name": "min_box_size_pixels",
            "value": 20.0,
            "descriptor": "minimum bounding box dimension to be considered valid",
        },
        {
            "name": "detection_timeout_sec",
            "value": 2.0,
            "descriptor": "maximum age of cached detection before considering it stale",
        },
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # parameters
        self.scan_speed = self.get_local_parameter("scan_speed")
        self.scan_angle = self.get_local_parameter("scan_angle")
        self.image_width = self.get_local_parameter("image_width")
        self.image_height = self.get_local_parameter("image_height")
        self.camera_horizontal_fov = self.get_local_parameter("camera_horizontal_fov")
        self.centering_threshold_pixels = self.get_local_parameter("centering_threshold_pixels")
        self.min_box_size_pixels = self.get_local_parameter("min_box_size_pixels")
        self.detection_timeout_sec = self.get_local_parameter("detection_timeout_sec")

        self.cached_gate_bounding_box = None
        self.last_detection_time = None
        self.image_center_x = self.image_width / 2.0
        self.image_center_y = self.image_height / 2.0

        # Subscribe to bounding box detections
        self.detection_subscription = self.ros_node.create_subscription(
            BoundingBox, "/bounding_box", self.detection_callback, 10
        )
        self._subscriptions.append(self.detection_subscription)

    def set_inference_camera(self, camera_mode):
        """Request camera mode change via service"""
        request = SetInferenceCamera.Request()
        request.camera_mode = camera_mode
        self.send_service_request(
            SetInferenceCamera, 
            "/set_inference_camera", 
            request, 
            lambda f: None
        )

    def change_model(self, model_id):
        """Request model change via service"""
        request = ChangeModel.Request()
        request.model_id = model_id
        self.send_service_request(
            ChangeModel, 
            "/change_model", 
            request, 
            lambda f: None
        )

    def detection_callback(self, msg):
        """Process incoming bounding box detections"""
        # Validate bounding box dimensions
        if msg.box_width < self.min_box_size_pixels or msg.box_height < self.min_box_size_pixels:
            return
        
        # Validate coordinates are within image bounds
        if (msg.x_coordinate < 0 or msg.x_coordinate > self.image_width or
            msg.y_coordinate < 0 or msg.y_coordinate > self.image_height):
            return
        
        self.cached_gate_bounding_box = msg
        self.last_detection_time = self.ros_node.get_clock().now()
        
        # Calculate offset from image center
        offset_x = abs(msg.x_coordinate - self.image_center_x)
        offset_y = abs(msg.y_coordinate - self.image_center_y)
        
        # If object is off-center and we're scanning, switch to following
        if offset_x > self.centering_threshold_pixels or offset_y > self.centering_threshold_pixels:
            if not self.is_following_detection():
                self.follow_detection()

    def is_detection_stale(self):
        """Check if cached detection is too old"""
        if self.last_detection_time is None:
            return True
        
        time_since_detection = (
            self.ros_node.get_clock().now() - self.last_detection_time
        ).nanoseconds / 1e9
        
        return time_since_detection > self.detection_timeout_sec

    def on_enter_initializing(self):
        """Initialize object detection for gate"""
        self.change_model(ChangeModel.Request.GATE)
        self.set_inference_camera(SetInferenceCamera.Request.FRONT_CAMERA)
        self.queued_method = self.initializing_done

    def on_enter_scanning_cw(self):
        """Rotate clockwise while scanning for gate"""
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_VELOCITY
        movement_msg.goal_velocity = GoalVelocity()
        movement_msg.goal_velocity.twist.angular.z = -self.scan_speed
        movement_msg.goal_velocity.duration = self.scan_angle / abs(self.scan_speed)
        movement_msg.goal_velocity.integrate = True
        movement_msg.timeout_sec = self.scan_angle / abs(self.scan_speed) * 2

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.scanning_cw_done,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send scanning CW command")
            self.queued_method = self.abort

    def on_enter_scanning_ccw(self):
        """Rotate counter-clockwise while scanning for gate"""
        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.SET_VELOCITY
        movement_msg.goal_velocity = GoalVelocity()
        movement_msg.goal_velocity.twist.angular.z = self.scan_speed
        movement_msg.goal_velocity.duration = self.scan_angle / abs(self.scan_speed)
        movement_msg.goal_velocity.integrate = True
        movement_msg.timeout_sec = self.scan_angle / abs(self.scan_speed) * 2

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.scanning_ccw_done,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send scanning CCW command")
            self.queued_method = self.abort

    def on_enter_following_detection(self):
        """Turn towards detected gate to center it in view"""
        if self.cached_gate_bounding_box is None or self.is_detection_stale():
            self.ros_node.get_logger().warn(
                "No valid bounding box cached, resuming scan"
            )
            self.resume_scan()
            return

        offset_x = self.cached_gate_bounding_box.x_coordinate - self.image_center_x
        
        # Normalize offset to [-1, 1] range
        normalized_offset = offset_x / self.image_center_x
        
        # Calculate rotation angle based on camera FOV
        # If normalized_offset = 1.0 (at edge), rotation = FOV/2
        max_rotation_angle = self.camera_horizontal_fov / 2.0
        calculated_yaw_rotation = max_rotation_angle * normalized_offset

        movement_msg = MovementCommand()
        movement_msg.command = MovementCommand.MOVE_RELATIVE
        movement_msg.rotation = Vector3(x=0.0, y=0.0, z=calculated_yaw_rotation)
        movement_msg.timeout_sec = 15.0

        success = self.movement_client.send_movement_command(
            movement_msg,
            on_success=self.check_centered_on_gate,
            on_failure=self.handle_movement_failure,
        )

        if not success:
            self.ros_node.get_logger().error("Failed to send look at gate command")
            self.queued_method = self.abort

    def check_centered_on_gate(self):
        """Verify gate is now centered in view"""
        if self.cached_gate_bounding_box is None or self.is_detection_stale():
            self.ros_node.get_logger().warn("Detection lost, resuming scan")
            self.resume_scan()
            return
        
        # Calculate current offset from center
        offset_x = abs(self.cached_gate_bounding_box.x_coordinate - self.image_center_x)
        offset_y = abs(self.cached_gate_bounding_box.y_coordinate - self.image_center_y)
        
        # Check if centered within threshold
        if offset_x < self.centering_threshold_pixels and offset_y < self.centering_threshold_pixels:
            self.ros_node.get_logger().info("Gate successfully centered")
            self.following_detection_done()
        else:
            self.ros_node.get_logger().info(
                f"Gate not centered (offset: x={offset_x:.0f}, y={offset_y:.0f}), resuming scan"
            )
            self.resume_scan()

    def on_completion(self):
        """Clean up when state machine completes"""
        self.set_inference_camera(SetInferenceCamera.Request.DISABLED)
        self.ros_node.get_logger().info("FindingGate state machine completed")

    def handle_movement_failure(self):
        """Handle movement action failure"""
        self.ros_node.get_logger().error("Movement action failed")
        self.abort()