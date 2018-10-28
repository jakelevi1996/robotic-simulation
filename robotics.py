import numpy as np
import matplotlib.pyplot as plt

def sind(x):
    return np.sin(np.radians(x))

def cosd(x):
    return np.cos(np.radians(x))

class Robot():
    def __init__(
        self, L=0.1, M=0.1, dt=1e-3,
        t_lift=0.2, t_step=1.6,
        y_lift=0.01, x_step=0.08,
        x1_init=0, y1_init=0,
        theta1_init=30, theta2_init=120,
        # foot1_clamped=True, 
    ):
        """Initialise a robot with both feet horizontal,
        IE links 1 and 4 are vertical, and theta1 + theta2 + theta3 = 180
        """
        self.L = L
        self.M = M
        self.dt = dt
        self.t_lift = t_lift
        self.t_step = t_step
        self.y_lift = y_lift
        self.x_step = x_step

        # Initialise traces:
        self.theta1 = [theta1_init]
        self.theta2 = [theta2_init]
        self.theta3 = [180 - theta1_init - theta2_init]
        self.x1 = [x1_init]
        self.y1 = [y1_init]
        self.x2 = [x1_init]
        self.y2 = [y1_init + L]
        self.x3 = [self.x2[0] + L*sind(theta1_init)]
        self.y3 = [self.y2[0] + L*cosd(theta1_init)]
        self.x4 = [self.x3[0] + L*sind(theta1_init + theta2_init)]
        self.y4 = [self.y3[0] + L*cosd(theta1_init + theta2_init)]
        self.x5 = [self.x4[0]]
        self.y5 = [self.y4[0] - L]
        self.torque1 = []
        self.torque2 = []
        self.torque3 = []
    
    def set_angles_using_affectors(self):
        pass
    
    def set_joint_3(self):
        pass

    def lift_left(self):
        N = int(self.t_lift / self.dt)
        y1_init = self.y1[-1]
        for n in range(N):
            t = n * self.dt
            # Joints 4 and 5 remain fixed:
            self.x4.append(self.x4[-1])
            self.y4.append(self.y4[-1])
            self.x5.append(self.x5[-1])
            self.y5.append(self.y5[-1])
            # Horizontal positions of joints 1 and 2 remain fixed:
            self.x1.append(self.x1[-1])
            self.x2.append(self.x2[-1])
            # Vertical positions of joints 1 and 2 given by cubic equation:
            self.y1.append(
                self.y_lift * pow(t, 2) * (
                    3 * self.t_lift - 2 * t
                ) / pow(self.t_lift, 3) + y1_init
            )
            self.y2.append(self.y1[-1] + self.L)

            self.set_angles_using_affectors()
            self.set_joint_3()


    def take_step(self):
        self.lift_left()
        # self.step_left()
        # self.drop_left()
        # self.lift_right()
        # self.step_right()
        # self.drop_right()

def plot_frame(robot, frame=-1, filename="robot_pos"):
    plt.plot(
        [r.x1[frame], r.x2[frame], r.x3[frame], r.x4[frame], r.x5[frame]],
        [r.y1[frame], r.y2[frame], r.y3[frame], r.y4[frame], r.y5[frame]]
    )
    plt.grid(True)
    plt.axis('equal')
    plt.savefig(filename)


if __name__ == "__main__":
    # a = np.array([0])
    # print(a)
    # np.append(a, [1])
    # print(a)
    # a = [0]
    # a.append(1)
    # print(a)
    r = Robot()
    print(r.x1, r.x2, r.x3, r.x4, r.x5)
    print(r.y1, r.y2, r.y3, r.y4, r.y5)
    plot_frame(r)
    r.lift_left()
    plot_frame(r, filename="robot_post2")