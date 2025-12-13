from random import uniform
from math import pi
from constants import *


class FiniteStateMachine(object):
    """
    A finite state machine.
    """
    def __init__(self, state):
        self.state = state

    def change_state(self, new_state):
        self.state = new_state

    def update(self, agent):
        self.state.check_transition(agent, self)
        self.state.execute(agent)


class State(object):
    """
    Abstract state class.
    """
    def __init__(self, state_name):
        """
        Creates a state.

        :param state_name: the name of the state.
        :type state_name: str
        """
        self.state_name = state_name

    def check_transition(self, agent, fsm):
        """
        Checks conditions and execute a state transition if needed.

        :param agent: the agent where this state is being executed on.
        :param fsm: finite state machine associated to this state.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")

    def execute(self, agent):
        """
        Executes the state logic.

        :param agent: the agent where this state is being executed on.
        """
        raise NotImplementedError("This method is abstract and must be implemented in derived classes")


class MoveForwardState(State):
    def __init__(self):
        super().__init__("MoveForward")
        self.execution_count = 0

    def check_transition(self, agent, state_machine):
        if agent.get_bumper_state() is True: 
            state_machine.change_state(GoBackState())
        elif self.execution_count * SAMPLE_TIME > MOVE_FORWARD_TIME:
            state_machine.change_state(MoveInSpiralState())
        

    def execute(self, agent):
        self.execution_count += 1
        agent.set_velocity(FORWARD_SPEED, 0)
        


class MoveInSpiralState(State):
    def __init__(self):
        super().__init__("MoveInSpiral")
        self.execution_count = 0
        self.spiral_radius = INITIAL_RADIUS_SPIRAL
    
    def check_transition(self, agent, state_machine):
        if agent.get_bumper_state() is True: 
            state_machine.change_state(GoBackState())
        elif self.execution_count * SAMPLE_TIME > MOVE_IN_SPIRAL_TIME:
            state_machine.change_state(MoveForwardState())
        

    def execute(self, agent):
        self.execution_count += 1
        self.spiral_radius += SPIRAL_FACTOR * SAMPLE_TIME
        agent.set_velocity(FORWARD_SPEED, FORWARD_SPEED / self.spiral_radius)
        


class GoBackState(State):
    def __init__(self):
        super().__init__("GoBack")
        self.execution_count = 0

    def check_transition(self, agent, state_machine):
        if self.execution_count * SAMPLE_TIME > GO_BACK_TIME:
            state_machine.change_state(RotateState())
        

    def execute(self, agent):
        self.execution_count += 1
        agent.set_velocity(BACKWARD_SPEED, 0)
        


class RotateState(State):
    def __init__(self):
        super().__init__("Rotate")
        self.execution_count = 0
        self.rotate_angle = uniform(-pi, pi)

    def check_transition(self, agent, state_machine):
        if self.execution_count * SAMPLE_TIME > self.rotate_angle / ANGULAR_SPEED:
            state_machine.change_state(MoveForwardState()) 
        
    
    def execute(self, agent):
        self.execution_count += 1
        agent.set_velocity(0, ANGULAR_SPEED)
        
