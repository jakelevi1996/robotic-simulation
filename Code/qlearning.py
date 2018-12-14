import numpy as np
import matplotlib.pyplot as plt

def discrete_boltzman(T, arg_list):
    probs = np.exp(np.array(arg_list) / T)
    probs = probs / probs.sum()
    print(probs)
    return probs

class QlearningRobot():
    def __init__(self, nx=200, ny=50, dx=0.01, dy=0.01, T=1e2, L=0.1):
        # Boltzman temperature:
        self.T = T
        # Length of links:
        self.L = L
        # Size of discrete space:
        self.nx = nx
        self.ny = ny
        # Resolution of discrete space:
        self.dx = dx
        self.dy = dy
        # Set initial positions and Q-values:
        self.reset_position()
        self.reset_q_tables()

    def reset_position(self):
        # x-pos of left foot starts off in centre of discrete space:
        # self.x_left = self.nx // 2
        # self.x_left = 0
        self.x_left = -5
        # y-pos of left foot starts off on the floor:
        self.y_left = 0
        # x-pos of right foot starts a distance of L in front of left foot:
        self.x_right = self.x_left + int(np.ceil(self.L / self.dx))
        # y-pos of right foot starts off on the floor:
        self.y_right = 0
    
    def reset_q_tables(self):
        # Q-tables for coordinates of each foot:
        self.Q_left = np.zeros([self.nx, self.ny])
        self.Q_right = np.zeros([self.nx, self.ny])

    def choose_action(self):
        # Store old coordinates (used later for updating Q-tables):
        self.old_left_xy = [self.x_left, self.y_left]
        self.old_right_xy = [self.x_right, self.y_right]

        if self.y_left == 0 and self.y_right == 0:
            # Both feet are on the floor, so lift one up:
            probs = discrete_boltzman(self.T, [
                self.Q_left[self.x_left, self.y_left + 1],
                self.Q_right[self.x_right, self.y_right + 1]
            ])
            action = np.random.choice(2, p=probs)
            if action == 0: self.y_left += 1
            else: self.y_right += 1
        
        elif self.y_left == 0:
            # Left foot is on the floor, so move the right foot:
            probs = discrete_boltzman(self.T, [
                self.Q_right[self.x_right, self.y_right + 1],
                self.Q_right[self.x_right, self.y_right - 1],
                self.Q_right[self.x_right + 1, self.y_right],
                self.Q_right[self.x_right - 1, self.y_right],
            ])
            action = np.random.choice(4, p=probs)
            if action == 0: self.y_right += 1
            elif action == 1: self.y_right -= 1
            elif action == 2: self.x_right += 1
            else: self.x_right -= 1
        
        else:
            # Right foot is on the floor, so move the left foot:
            probs = discrete_boltzman(self.T, [
                self.Q_left[self.x_left, self.y_left + 1],
                self.Q_left[self.x_left, self.y_left - 1],
                self.Q_left[self.x_left + 1, self.y_left],
                self.Q_left[self.x_left - 1, self.y_left],
            ])
            action = np.random.choice(4, p=probs)
            if action == 0: self.y_left += 1
            elif action == 1: self.y_left -= 1
            elif action == 2: self.x_left += 1
            else: self.x_left -= 1
        
    def choose_optimal_action(self):
        # Store old coordinates (used later for updating Q-tables):
        self.old_left_xy = [self.x_left, self.y_left]
        self.old_right_xy = [self.x_right, self.y_right]

        if self.y_left == 0 and self.y_right == 0:
            # Both feet are on the floor, so lift one up:
            action = np.argmin([
                self.Q_left[self.x_left, self.y_left + 1],
                self.Q_right[self.x_right, self.y_right + 1]
            ])
            # action = np.random.choice(2, p=probs)
            print(action)
            if action == 0: self.y_left += 1
            else: self.y_right += 1
        
        elif self.y_left == 0:
            # Left foot is on the floor, so move the right foot:
            action = np.argmin([
                self.Q_right[self.x_right, self.y_right + 1],
                self.Q_right[self.x_right, self.y_right - 1],
                self.Q_right[self.x_right + 1, self.y_right],
                self.Q_right[self.x_right - 1, self.y_right],
            ])
            print(action)
            # action = np.random.choice(4, p=probs)
            if action == 0: self.y_right += 1
            elif action == 1: self.y_right -= 1
            elif action == 2: self.x_right += 1
            else: self.x_right -= 1
        
        else:
            # Right foot is on the floor, so move the left foot:
            action = np.argmin([
                self.Q_left[self.x_left, self.y_left + 1],
                self.Q_left[self.x_left, self.y_left - 1],
                self.Q_left[self.x_left + 1, self.y_left],
                self.Q_left[self.x_left - 1, self.y_left],
            ])
            print(action)
            # action = np.random.choice(4, p=probs)
            if action == 0: self.y_left += 1
            elif action == 1: self.y_left -= 1
            elif action == 2: self.x_left += 1
            else: self.x_left -= 1
        
    
    def reward_function(self):
        motor_distance_x = self.dx * (self.x_right - self.x_left)
        motor_distance_y = self.dy * (self.y_right - self.y_left)
        motor_distance = np.linalg.norm([motor_distance_x, motor_distance_y])
        # Motors are too far apart, but links are rigid:
        if motor_distance > 2 * self.L: return -1
        # Feet are colliding with each other:
        elif motor_distance_x < self.L: return -1
        # Return sum of x_coordinates, to encourage motion in +x direction:
        else: return self.x_left + self.x_right
    
    def update_q_tables(self, alpha, gamma, reward):
        # Update Q-values for left foot:
        self.Q_left[self.old_left_xy] = sum([
            (1 - alpha) * self.Q_left[self.old_left_xy],
            alpha * (reward + gamma * self.Q_left[self.x_left, self.y_left])
        ])
        # Update Q-values for right foot:
        self.Q_right[self.old_right_xy] = sum([
            (1 - alpha) * self.Q_right[self.old_right_xy],
            alpha * (reward + gamma * self.Q_right[self.x_right, self.y_right])
        ])

    def print_coords(self):
        print(self.x_left, self.y_left, self.x_right, self.y_right)
    
    def q_learn(
        self, n_episodes=500, n_t_steps=500, n_evals=5, alpha=0.1, gamma=0.1
    ):
        self.reset_q_tables()
        plt.figure(figsize=[8, 6])
        for _ in range(n_episodes):
            self.reset_position()
            print("\n***New episode***")
            self.print_coords()
            reward_list = []
            for _ in range(n_t_steps):
                self.choose_action()
                self.print_coords()
                reward = self.reward_function()
                print(reward)
                reward_list.append(reward)
                self.update_q_tables(alpha, gamma, reward)
                if reward == -1: break
            plt.plot(reward_list, 'b', alpha=0.2)
        
        for _ in range(n_evals):
            self.reset_position()
            reward_list = []
            for _ in range(n_t_steps):
                self.choose_optimal_action()
                reward = self.reward_function()
                reward_list.append(reward)
                self.update_q_tables(alpha, gamma, reward)
                if reward == -1: break
            plt.plot(reward_list, 'r', alpha=0.6)

        plt.grid(True)
        plt.xlabel("t")
        plt.ylabel("Reward")
        plt.title("Results of Q-learning")
        plt.savefig("qlearn")
        plt.close()





if __name__ == "__main__":
    r = QlearningRobot()
    r.print_coords()
    r.choose_action()
    r.print_coords()
    r.q_learn()