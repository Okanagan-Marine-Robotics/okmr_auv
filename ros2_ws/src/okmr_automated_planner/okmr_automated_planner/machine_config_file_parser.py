import yaml
from okmr_automated_planner.state_node import StateNode
from okmr_automated_planner.base_state_machine import BaseStateMachine

class MachineConfigFileParser:
    _allowed_state_keys = ['name', 'timeout', 'config_path', 'movement', 'task']
    _allowed_transition_keys = ['trigger', 'source', 'dest']

    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.name = ""
        self._states = []#list of disctionaries [{name: 'a', timeout: 10, config_path: "xyz/abc"}, ...]
        self._transitions = [] #list of dictionaries {trigger: 'doneA', source: 'a', dest: 'b'}
        self._load_yaml()

    def _load_yaml(self):
        with open(self.yaml_path, 'r') as file:
            config = yaml.safe_load(file)

        self.name = next(iter(config))
        config = config[self.name]

        for state in config.get('states', []):
            if self.isValidStateDict(state):
                self._states.append(state)
        
        for transition in config.get('transitions', []):
            if self.isValidTransitionDict(transition):
                self._transitions.append(transition)

    def isValidStateDict(self, state):
        if 'name' not in state:
            raise ValueError(f"{self.name}\tState dictionary missing required 'name' field: {state}")
        
        if 'task' in state and 'movement' in state:
            raise ValueError(f"{self.name}\tState '{state['name']}'. Subtasks cannot have a movement in their state definition.")

        for key in state:
            if key not in self._allowed_state_keys:
                raise ValueError(f"{self.name}\tInvalid state key '{key}' in state {state['name'] if 'name' in state else state}. Allowed keys: {self._allowed_state_keys}")
        return True
    
    def isValidTransitionDict(self, transition):
        #transition dict must contain all of, and only the required keys
        for key in self._allowed_transition_keys:
            if key not in transition:
                raise ValueError(f"{self.name}\tTransition missing required key '{key}': {transition}")

        state_names = [state['name'] for state in self._states]
        mandatory_state_names = BaseStateMachine.mandatory_states #these states may be auto added later
        for key in transition:
            if key not in self._allowed_transition_keys:
                raise ValueError(f"{self.name}\tInvalid transition key '{key}' in transition {transition}. Allowed keys: {self._allowed_transition_keys}")
            elif key in ['source', 'dest'] and transition[key] != '*' and transition[key] not in state_names and transition[key] not in mandatory_state_names:
                raise ValueError(f"{self.name}\tTransition {transition['trigger']} references unknown state '{transition[key]}'")
        
        return True

    def get_states(self):
        return self._states

    def get_transitions(self):
        return self._transitions
