from robots import TwoLegRobot
import numpy as np
import plotting
import logging

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    logging.info("Creating robot...")
    r = TwoLegRobot()
    # r = TwoLegRobot(speed=0.5)
    
    logging.info("Navigating terrain...")
    r.walk_distance(-0.49)
    r.navigate_step(0.05)
    remaining_distance = -1.0 - r.x[0, -1]
    r.walk_distance(remaining_distance)

    logging.info("Creating plots...")
    plotting.plot_robot_trajectory(r.x, r.y, r.L, r.dt)
    xlims = None
    # xlims = [0, 2]
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

    logging.info("Calculating derivatives...")
    r.set_vels_and_accs()

    logging.info("Plotting velocities...")
    plotting.plot_traces(
        r.dt, r.xdot[1:4], "images/xdot", "Horizontal joint-velocities",
        "xdot_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.ydot[1:4], "images/ydot", "Vertical joint-velocities",
        "ydot_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.thetadot[0:2], "images/thetadot", "Joint anglular velocities",
        "thetadot_", xlims=xlims
    )

    logging.info("Plotting accelerations...")
    plotting.plot_traces(
        r.dt, r.xdotdot[1:4], "images/xdotdot",
        "Horizontal joint-accelerations", "xdotdot_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.ydotdot[1:4], "images/ydotdot",
        "Vertical joint-accelerations", "ydotdot_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.thetadotdot[0:2], "images/thetadotdot",
        "Joint anglular accelerations", "thetadotdot_", xlims=xlims
    )

    logging.info("Plotting energies...")
    plotting.plot_traces(
        r.dt, [r.kinetic_energy, r.potential_energy], "images/energy",
        "Energy", legend_entries=["Kinetic", "Potential"], xlims=xlims
    )

    print("Time taken to reach goal = {}s".format(r.dt*r.x.shape[1]))
