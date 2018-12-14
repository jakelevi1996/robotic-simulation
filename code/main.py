import numpy as np
import logging

from robots import TwoLegRobot
import traces
import plotting

def locomotion_problem_0():
    logging.info("Creating robot...")
    r = TwoLegRobot()
    # r = TwoLegRobot(speed=0.5)
    # r = TwoLegRobot(y_lift=0.005)

    logging.info("Navigating terrain...")
    # r.walk_distance(-0.08)
    r.walk_distance(-0.49)
    r.navigate_step(0.05)
    remaining_distance = -1.0 - r.x[0, -1]
    r.walk_distance(remaining_distance)

    return r

def locomotion_problem_1():
    logging.info("Creating robot...")
    
    r = TwoLegRobot()

    logging.info("Navigating terrain...")
    r.walk_distance(-0.29)
    
    r.navigate_step(0.05)
    remaining_distance = -0.39 - r.x[0, -1]
    r.walk_distance(remaining_distance)
    r.navigate_step(0.05)
    remaining_distance = -0.7 - r.x[0, -1]
    r.walk_distance(remaining_distance)

    return r

def locomotion_problem_2():
    logging.info("Creating robot...")
    
    r = TwoLegRobot()

    logging.info("Navigating terrain...")
    r.walk_distance(-0.49)
    
    r.navigate_step(0.05)
    r.walk_distance(-0.05)
    r.navigate_step(-0.05)
    remaining_distance = -1.05 - r.x[0, -1]
    r.walk_distance(remaining_distance)

    return r
    

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    
    # r = locomotion_problem_0()
    # r = locomotion_problem_1()
    # r = locomotion_problem_2()
    r = TwoLegRobot()
    # r.navigate_terrain([-0.3, -0.4], [0.05, 0.05], x_end_goal=-0.7)
    r.navigate_terrain(
        [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6],
        [0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
    )


    logging.info("Calculating derivatives, torques etc...")
    xdot, ydot, thetadot = traces.derivatives(
        r.x, r.y, np.radians(r.theta), r.dt
    )
    xdotdot, ydotdot, thetadotdot = traces.derivatives(
        xdot, ydot, thetadot, r.dt
    )
    ke, pe, dPE = traces.energies(r.x, r.y, xdot, ydot, r.M, r.g)
    torque = traces.static_torques(
        r.x, r.M, r.g, r.foot1_clamped, r.foot2_clamped
    )
    power = traces.instantaneous_power(
        torque, thetadot, r.foot1_clamped, r.foot2_clamped
    )
    total_energy = traces.energy_consumption(power, r.dt)
    max_torques = traces.max_torques(torque)

    logging.info("Plotting trajectory...")
    plotting.plot_robot_trajectory(
        r.x, r.y, r.L, r.dt,
        # T=10,
        # x_ground = [-0.8, -.4, -.4, -.3, -.3, .2],
        # y_ground_top = [0.1, 0.1, .05, .05, 0, 0],
        # y_ground_bottom = [-.1, -.1, -.1, -.1, -.1, -.1]
        # T=11,
        # x_ground = [-1.1, -.6, -.6, -.5, -.5, .2],
        # y_ground_top = [0.0, 0.0, .05, .05, 0, 0],
        # y_ground_bottom = [-.1, -.1, -.1, -.1, -.1, -.1]
        T=10.5,
        x_ground = [
            -1.1, -.6, -.6, -.5, -.5, -.4, -.4,
            -.3, -.3, -.2, -.2, -.1, -.1, .2
        ],
        y_ground_top = [
            0.3, 0.3, 0.25, 0.25, 0.2, 0.2, 0.15,
            0.15, 0.1, 0.1, 0.05, 0.05, 0.0, 0.0
        ],
        y_ground_bottom = [
            -.1, -.1, -.1, -.1, -.1, -.1, -.1,
            -.1, -.1, -.1, -.1, -.1, -.1, -.1, 
        ]
    )
    xlims = None
    # xlims = [0, 2]

    logging.info("Plotting positions...")
    plotting.plot_traces(
        r.dt, r.x[1:4], "images/x", "Horizontal joint-positions",
        "x_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.y[1:4], "images/y", "Vertical joint-positions",
        "y_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.theta[0:2], "images/theta", "Joint-angles",
        "theta_", xlims=xlims
    )

    logging.info("Plotting velocities...")
    plotting.plot_traces(
        r.dt, xdot[1:4], "images/xdot", "Horizontal joint-velocities",
        "xdot_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, ydot[1:4], "images/ydot", "Vertical joint-velocities",
        "ydot_", 1, xlims=xlims
    )
    # plotting.plot_traces(
    #     r.dt, thetadot[0:2], "images/thetadot", "Joint anglular velocities",
    #     "thetadot_", xlims=xlims
    # )
    plotting.plot_traces(
        r.dt, thetadot, "images/thetadot", "Joint anglular velocities",
        "thetadot_", xlims=xlims
    )

    logging.info("Plotting accelerations...")
    plotting.plot_traces(
        r.dt, xdotdot[1:4], "images/xdotdot",
        "Horizontal joint-accelerations", "xdotdot_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, ydotdot[1:4], "images/ydotdot",
        "Vertical joint-accelerations", "ydotdot_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, thetadotdot[0:2], "images/thetadotdot",
        "Joint anglular accelerations", "thetadotdot_", xlims=xlims
    )

    logging.info("Plotting energies, torques, power...")
    plotting.plot_traces(
        r.dt, [ke, pe], "images/energy",
        "Energy", legend_entries=["Kinetic", "Potential"], xlims=xlims
    )

    plotting.plot_traces(
        r.dt, torque, "images/torque", "Motor torques",
        "torque_", xlims=xlims
    )

    plotting.plot_traces(
        r.dt, power, "images/power", "Instantaneous power",
        "motor_", xlims=xlims
    )

    print("Time taken to reach goal = {} s".format(r.dt*r.x.shape[1]))
    print("Maximum torques required = {} Nm".format(max_torques))
    print("Energy consumption = {:.3f} J".format(total_energy))
    print("Total change in GPE = {:.3f} J".format(dPE))
    print("Efficiency = {:.3f} %".format(100.0 * dPE / total_energy))
    