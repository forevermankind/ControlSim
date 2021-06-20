import numpy as np
from scipy.integrate import RK45
from modules.graphics import Point, physical_to_pixel_coords, pixel_to_physical_coords



class State:

    class Cart:

        class Linkage:
            def __init__(self, mass, length, angle=0, base=None):
                self.mass = mass # kg
                self.length = length # m
                self.base = base # link to the Linkage/cart to which it's attached?
                self.inertia_base = (self.mass*self.length**2)/3 # kg.m^2
                self.inertia_cg = (self.mass*self.length**2)/12 # kg.m^2
                self.angle = angle # rad - initialise in vertical position (NOTE: POSITIVE CLOCKWISE)
                self.angular_velocity = 0 # rad/s (NOTE: POSITIVE CLOCKWISE)

            @property
            def relative_tip_position(self):
                # get position of end of linkage arm relative to the base as a
                # numpy array
                return np.array([np.sin(self.angle), np.cos(self.angle)])*self.length # mm

        def __init__(self, cart_mass, init_pos, display_object, parent):
            self.parent = parent
            self.display_obj = display_object
            self.linkages = []
            self.num_linkages = 0
            self.mass = cart_mass # kg
            self.position = init_pos.x # m - initially at centre of screen, positive right
            self.velocity = 0 # m/s - positive right
            self.controller = None

        @property
        def position(self):
            return self._position
        @position.setter
        def position(self, new_x_pos):
            # new_x_pos is a float
            self._position = new_x_pos
            self.display_obj.x = self._position * 1e3

        def add_linkage(self, linkage_mass, linkage_length, linkage_angle):
            if self.num_linkages == 0:
                parent = self
            else:
                parent = self.linkages[-1]
            self.linkages.append(self.Linkage(linkage_mass, linkage_length, linkage_angle, base=parent))
            self.parent.total_mass += linkage_mass # kg
            self.num_linkages += 1

    def __init__(self, cart_mass, cart_init_pos, cart_display_obj):
        self.create_cart(cart_mass, cart_init_pos, cart_display_obj)
        self.time = 0 # ms

    def create_cart(self, cart_mass, cart_init_pos, cart_display_obj):
        self.cart = self.Cart(cart_mass, cart_init_pos, cart_display_obj, parent=self)
        self.total_mass = cart_mass # kg

    def assign_new_state(self, state_vec):
        # TODO: EXTEND TO N LINKAGES
        pendulum = self.cart.linkages[0]
        self.cart.position = state_vec[0]
        self.cart.velocity = state_vec[1]
        pendulum.angle = state_vec[2]
        pendulum.angular_velocity = state_vec[3]

    def assign_controller(self, controller):
        self.controller = controller

    def get_current_state_vector(self):
        # get [x, xdot, theta, thetadot]
        # TODO: GENERALISE TO N LINKAGE ARMS
        if self.cart.num_linkages > 1:
            raise NotImplementedError("Physics model only supports a single inverted pendulum.")
        pendulum = self.cart.linkages[0]
        state_vector = np.array([self.cart.position, self.cart.velocity, pendulum.angle, pendulum.angular_velocity]).T

        return state_vector

    def get_state_derivative(self, state_vector, control_force=0):
        # TODO: generalise to N linkage arms, currently only works for 1
        # return 
        if self.cart.num_linkages > 1:
            raise NotImplementedError("Physics model only supports a single inverted pendulum.")
        
        # constants
        g = 9.81 # m/s^2 TODO: define this better
        mu_c = 0.1 # cart to track sliding friction coefficient
        mu_p = 0

        M = self.total_mass # kg
        pendulum = self.cart.linkages[0]
        cart_vel = state_vector[1] # m/s
        theta = state_vector[2] # rad
        omega = state_vector[3] # rad/s
        J = pendulum.inertia_base # kg.m^2
        l_hat = pendulum.length/2 # m
        m = pendulum.mass # kg

        # TODO: MODEL FRICTION
        cart_friction_force = 0
        pend_friction_moment = 0

        angular_accel = (M*m*g*l_hat*np.sin(theta) - m*l_hat*np.cos(theta)*(control_force + m*l_hat*(omega**2)*np.sin(theta)))/(M*(m*l_hat**2 + J) - (m*l_hat*np.cos(theta))**2) # rad/s^2
        cart_accel = (((m*l_hat)**2)*g*np.sin(theta)*np.cos(theta) - (m*l_hat**2 + J)*(control_force + m*l_hat*omega**2*np.sin(theta)))/((m*l_hat*np.cos(theta))**2 - M*(m*l_hat**2 + J)) # m/s^2

        state_gradient = np.array([cart_vel, cart_accel, omega, angular_accel]).T
        return state_gradient

    def get_pivot_points(self, window):
        # return a list of Point instances representing pivot positions, for use
        # in drawing the current cart-pole configuration
        pivots = [Point(self.cart.display_obj.centerx, self.cart.display_obj.y)]
        for linkage_num in range(self.cart.num_linkages):
            pivot_physical_coords = pixel_to_physical_coords(window, *pivots[linkage_num].coords) + self.cart.linkages[linkage_num].relative_tip_position
            pivot_pos = physical_to_pixel_coords(window, *pivot_physical_coords)
            pivots.append(Point(*pivot_pos))
        return pivots

    def advance_step(self, dt=10, apply_control=False):
        # propagate all positions forward one step
        # save these results in their appropriate locations
        init_state = self.get_current_state_vector()
        # NOTE: dt assumed in milliseconds
        dt = dt * 1e-3

        if not apply_control or self.controller is None:
            self.control_force = 0
        else:
            self.control_force = self.controller.get_control_action(init_state, dt)

        # TODO: UPGRADE TO INCLUDE CONTROLLER
        # TODO: EXTEND TO MULTIPLE LINKAGES
        # TODO: UPGRADE INTEGRATION SCHEME
        # state_derivative = self.get_state_derivative(control_force)
        f = lambda t, state_vec: self.get_state_derivative(state_vec, self.control_force)

        # state_vec = state_vec + state_derivative*dt
        t0 = self.time
        t_bound = t0 + dt
        integrator = RK45(f, t0, init_state, t_bound, vectorized=True)
        integrator.step()
        state_vec = integrator.y

        # correct angle so it's in the range [-pi,pi]
        state_vec[2] = np.mod(state_vec[2] + np.pi, 2*np.pi) - np.pi
        # print(f"theta: {np.rad2deg(state_vec[2])}deg")

        # assign results
        self.assign_new_state(state_vec)

