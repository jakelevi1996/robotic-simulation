import numpy as np
import matplotlib.pyplot as plt
import logging

def sind(x): return np.sin(np.radians(x))
def cosd(x): return np.cos(np.radians(x))
def asind(x): return np.degrees(np.arcsin(x))
def acosd(x): return np.degrees(np.arccos(x))

class TwoLegRobot():
    def __init__(
        self, L=0.1, M=0.1, dt=1e-3,
        # t_lift=0.2, t_step=1.6,
        y_lift=0.01, x_step=-0.08, speed=0.05,
        x0_init=0, y0_init=0,
        theta0_init=30, theta1_init=120,
        # foot1_clamped=True, 
    ):
        """Initialise a robot with both feet horizontal,
        IE links 1 and 4 are vertical, and theta1 + theta2 + theta3 = 180
        """
        self.L = L
        self.M = M
        self.dt = dt
        # self.t_lift = t_lift
        # self.t_step = t_step
        self.y_lift = y_lift
        self.x_step = x_step
        self.speed = speed
        self.motion_counter = 0

        # Initialise traces:
        # Array to store joint-angles over time:
        self.theta = np.zeros([3, 1])
        self.theta[0, 0] = theta0_init
        self.theta[1, 0] = theta1_init
        self.theta[2, 0] = 180 - theta0_init - theta1_init
        # Array to store the x-values for each joint over time:
        self.x = np.zeros([5, 1])
        self.x[0, 0] = x0_init
        self.x[1, 0] = x0_init
        self.x[2, 0] = self.x[1, 0] + L*sind(theta0_init)
        self.x[3, 0] = self.x[2, 0] + L*sind(theta0_init + theta1_init)
        self.x[4, 0] = self.x[3, 0]
        # Array to store the y-values for each joint over time:
        self.y = np.zeros([5, 1])
        self.y[0, 0] = y0_init
        self.y[1, 0] = y0_init + L
        self.y[2, 0] = self.y[1, 0] + L*cosd(theta0_init)
        self.y[3, 0] = self.y[2, 0] + L*cosd(theta0_init + theta1_init)
        self.y[4, 0] = self.y[3, 0] - L
        # Velocities:
        self.thetadot = np.zeros([3, 1])
        self.xdot = np.zeros([5, 1])
        self.ydot = np.zeros([5, 1])
        # Accelerations:
        self.thetadotdot = np.zeros([3, 1])
        self.xdotdot = np.zeros([5, 1])
        self.ydotdot = np.zeros([5, 1])
        # Torques:
        self.torque = np.zeros([3, 1])
    
    def cubic_motion_trajectory(self, start_pos, distance, T):
        """Calculate motion trajectory with given start point, end point, and
        time-period, using cubic motion control (IE quadratic velocity) with
        zero start and end-velocity
        """
        N = int(T / self.dt)
        t = np.linspace(0, T, N)
        return distance*(t**2)*(3*T - 2*t)/(T**3) + start_pos
    
    def quadratic_velocity_trajectory(self, ):
        pass
    
    def linear_acceleration_trajectory(self, ):
        pass

    def affectors_to_angles(self, x, y, theta):
        # Calculate distance between end affectors:
        alpha = np.sqrt((x[4] - x[0])**2 + (y[4] - y[0])**2)
        # Calculate angles:
        theta[1] = 2 * acosd(alpha / (2*self.L))
        theta[0] = acosd((y[4] - y[0]) / alpha) - theta[1] / 2
        theta[2] = 180 - theta[0] - theta[1]
        return theta
    
    def angles_to_central_joint(self, x, y, theta):
        x[2] = x[1] + self.L * sind(theta[0])
        y[2] = y[1] + self.L * cosd(theta[0])
        return x, y

    def move_affector(
        self, joint0x=None, joint0y=None, joint4x=None, joint4y=None, T=0
    ):
        """Move each affector by the specified distance in time T. Positive
        distances correspond to positive directions in the x and y axes (IE a
        negative x-value corresponds to moving left).
        
        This function should be called using one of its wrappers.
        """
        # Number of time steps:
        N = int(T / self.dt)
        # Arrays to store trajectories during motion:
        x = np.zeros([5, N])
        y = np.zeros([5, N])
        theta = np.zeros([3, N])
        # Horizontal positions of joints 0, 1, 3 and 4 remain fixed:
        # (NB implicit broadcasting is used here)
        if joint0x is None:
            x[0] = self.x[0, -1]
            x[1] = self.x[1, -1]
        else:
            x[0] = self.cubic_motion_trajectory(self.x[0, -1], joint0x, T)
            x[1] = x[0]
        if joint4x is None:
            x[4] = self.x[4, -1]
            x[3] = self.x[3, -1]
        else:
            x[4] = self.cubic_motion_trajectory(self.x[4, -1], joint4x, T)
            x[3] = x[4]
        if joint0y is None:
            y[0] = self.y[0, -1]
            y[1] = self.y[1, -1]
        else:
            y[0] = self.cubic_motion_trajectory(self.y[0, -1], joint0y, T)
            y[1] = y[0] + self.L
        if joint4y is None:
            y[4] = self.y[4, -1]
            y[3] = self.y[3, -1]
        else:
            y[4] = self.cubic_motion_trajectory(self.y[4, -1], joint4y, T)
            y[3] = y[4] + self.L

        # Calculate trajectories of joint angles and central joint:
        theta = self.affectors_to_angles(x, y, theta)
        x, y = self.angles_to_central_joint(x, y, theta)
        # Update total trajectories:
        self.x = np.append(self.x, x, axis=1)
        self.y = np.append(self.y, y, axis=1)
        self.theta = np.append(self.theta, theta, axis=1)
        self.motion_counter += 1
    
    def lift_left(self, y_lift):
        """Lift joints 0 and 1 vertically upwards by a distance y_lift using a
        cubic motion trajectory
        """
        t_lift = abs(y_lift / self.speed)
        self.move_affector(joint0y=y_lift, T=t_lift)

    def step_left(self, x_step):
        """Step joints 0 and 1 horizontally across by a distance x_step using a
        cubic motion trajectory
        """
        t_step = abs(x_step / self.speed)
        self.move_affector(joint0x=x_step, T=t_step)
    
    def lift_right(self, y_lift):
        """Lift joints 3 and 4 vertically upwards by a distance y_lift using a
        cubic motion trajectory
        """
        t_lift = abs(y_lift / self.speed)
        self.move_affector(joint4y=y_lift, T=t_lift)

    def step_right(self, x_step):
        """Step joints 3 and 4 horizontally across by a distance x_step using a
        cubic motion trajectory
        """
        t_step = abs(x_step / self.speed)
        self.move_affector(joint4x=x_step, T=t_step)
    
    def take_horizontal_step(self, y_lift, x_step):
        self.lift_left(y_lift)
        self.step_left(x_step)
        self.lift_left(-y_lift)
        self.lift_right(y_lift)
        self.step_right(x_step)
        self.lift_right(-y_lift)

    def walk_distance(self, distance):
        """NB this method currently only supports negative distances (IE
        towards the left in the xy-plane); to support positive distances, need
        to test for the sign of `distance`
        """
        x_init = self.x[0, -1]
        # Keep taking full steps until a full step would be too much:
        while self.x[0, -1] + self.x_step - x_init > distance:
            self.take_horizontal_step(self.y_lift, self.x_step)
        # Take a final step of the right length:
        final_step_length = distance + x_init - self.x[0, -1]
        self.take_horizontal_step(self.y_lift, final_step_length)
    
    def navigate_step(self, step_height):
        self.lift_left(step_height + self.y_lift)
        self.step_left(self.x_step)
        self.lift_left(-self.y_lift)
        self.lift_right(step_height + self.y_lift)
        self.step_right(self.x_step)
        self.lift_right(-self.y_lift)



if __name__ == "__main__":

    r = TwoLegRobot()
    
    r.walk_distance(-0.49)
    r.navigate_step(0.05)
    remaining_distance = -1.0 - r.x[0, -1]
    r.walk_distance(remaining_distance)

    print(r.x.shape)
