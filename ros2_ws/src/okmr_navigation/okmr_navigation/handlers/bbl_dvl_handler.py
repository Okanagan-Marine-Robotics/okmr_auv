from okmr_msgs.action import Movement
from okmr_msgs.msg import GoalPose, MovementCommand, GoalVelocity
from okmr_msgs.msg._dvl import Dvl
from okmr_navigation.handlers.get_pose_twist_accel import get_current_pose, get_current_twist
from okmr_navigation.navigator_action_server import NavigatorActionServer
from okmr_navigation.handlers.set_velocity_handler import *
from okmr_navigation.handlers.freeze_handler import *

DIFF_ALLOWED = 0.3
FRONT_IDX = [0, 3]
BACK_IDX = [1, 2]

def handle_bbl_dvl(goal_handle):
    node = NavigatorActionServer.get_instance()
    
    dvl_sub = None
    
    try:
        dvl_sub = node.create_subscription(Dvl, '/dvl', _dvl_callback, 10)
            # should probably add a destroy somewhere - and uncache the sub above idk
    finally:
        pass

def _dvl_callback(msg):
    beams = msg.beam_distances
    node = NavigatorActionServer.get_instance()

    
    front_measurements = []
    back_measurements = []
    for i in front_idx:
        front_measurements.append(beams[i])
    for i in back_idx:
        back_measurements.append(beams[i])
    
    if(min(front_measurements) < min(back_measurements)):
        _send_velocity()
        node.get_logger().debug("Front aligned against wall. - 0.1m/s velocity")
        return
            
    if(back_measurements[0] - back_measurements[1] >= DIFF_ALLOWED):
        _send_velocity()
        node.get_logger().debug("Diff between beams is too large. - 0.1m/s velocity")
        return
    #msg = MovementCommand();
    #msg.command = MovementCommand.FREEZE
    execute_freeze()
    node.get_logger().debug("sub aligned - sent freeze pose")

        
def _send_velocity():
    node = NavigatorActionServer.get_instance()
    msg = MovementCommand()
    msg.command = MovementCommand.SET_VELOCITY
    msg.goal_velocity.twist.angular.z = 0.1
    handle_set_velocity(msg)
