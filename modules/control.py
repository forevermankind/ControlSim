from abc import ABC, abstractmethod
import numpy as np

class Controller(ABC):

    @abstractmethod
    def __init__(self, setpoint):
        self.setpoint = setpoint

    @ abstractmethod
    def get_control_action(self, state_vector):
        pass


class PID(Controller):

    def __init__(self, K_p=1, K_i=1, K_d=1, setpoint=0):
        super().__init__(setpoint)
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        # self.control_variable_indices = control_var_inds # which variables we're controlling
        self.integral_error = 0
        self.error_history = []
        self.num_data_points = 0
        self.MAX_FORCE = 5 # N

    def get_control_action(self, state_vector, dt):
        # calculate error and save where it's needed
        # NOTE: dt in seconds
        error = state_vector[2] - self.setpoint
        self.integral_error += error*dt
        self.error_history.append(error)
        self.num_data_points += 1

        # compute derivative term
        if self.num_data_points == 1:
            derivative_error = error/dt
        else:
            derivative_error = (error - self.error_history[-1])/dt

        # compute control force
        control_force = self.K_p*error + self.K_p*self.integral_error + self.K_d*derivative_error
        # print(f"error: {error}, integral error: {self.integral_error}, control force: {control_force}")


        if np.abs(control_force) > self.MAX_FORCE:
            control_force = self.MAX_FORCE * np.sign(control_force)

        return control_force


class FullStateFeedback(Controller):

    def __init__(self, K, setpoint=np.array([0,0,0,0]).T):
        super().__init__(setpoint)
        self.K = K

    def get_control_action(self, state_vector, dt):
        # return super().get_control_action(state_vector)
        control_force = -self.K@(self.setpoint - state_vector) # TODO: NOT SURE THIS IS RIGHT (should I take the difference from the setpoint?)
        print(f"state vector: {state_vector}, control_force: {control_force}")
        return control_force