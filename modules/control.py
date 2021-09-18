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

    def __init__(self, K, setpoint=np.array([0,0,0,0]).T, energy_opts=None):
        super().__init__(setpoint)
        self.K = K

        # how much energy (relative to the target state) do we want before we switch
        # from energy-based control to the standard control law
        if energy_opts is not None:
            self.energy_function = energy_opts['energy function']
            self.energy_factor = energy_opts['energy factor']
            self.init_kick_angle_tol = energy_opts['angle tolerance']
            self.init_kick_rate_tol = energy_opts['rate tolerance']
            self.target_energy = self.energy_function(setpoint)
            self.swing_up_control = True
        else:
            self.energy_function = None
            self.swing_up_control = False

    def get_control_action(self, state_vector, dt):
        # return super().get_control_action(state_vector)

        control_force = -self.K@(state_vector - self.setpoint)

        if self.swing_up_control:
            current_energy = self.energy_function(state_vector)

            angle, angular_rate = state_vector[2:]

            if current_energy < self.energy_factor*self.target_energy:
                if np.abs(angle - np.pi) < self.init_kick_angle_tol and np.abs(angular_rate) < self.init_kick_rate_tol:
                    angular_rate = 10 # initial kick

                control_force = (current_energy - self.target_energy)*angular_rate*np.cos(angle)

        return control_force