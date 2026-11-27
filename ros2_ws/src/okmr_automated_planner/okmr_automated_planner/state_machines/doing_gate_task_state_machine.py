from okmr_automated_planner.base_state_machine import BaseStateMachine
from okmr_msgs.msg import MovementCommand
from okmr_msgs.msg import GoalVelocity
from okmr_msgs.srv import Status
from geometry_msgs.msg import Vector3
from okmr_automated_planner.state_machines.sideways_scan_state_machine import SidewaysScanStateMachine


class DoingGateTaskStateMachine(BaseStateMachine):

    PARAMETERS = [
        {
            "name": "approaching_distance",
            "value": 2.0,
            "descriptor": "distance for approaching_gate"
        },
        {
            "name": "passing_distance",
            "value": 0.2,
            "descriptor": "distance for passing gate"
        }
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.approaching_distance = self.get_local_parameter("approaching_distance")
        self.passing_distance = self.get_local_parameter("passing_distance")
            
    def on_enter_initializing(self):
        self.queued_method = self.initialized
        pass

    def on_enter_approaching_gate(self):
        self.move_relative(self.approaching_gate_done, x=self.approaching_distance)

    def on_enter_passing_gate(self):
        self.move_relative(self.passing_gate_done,x=self.passing_distance)