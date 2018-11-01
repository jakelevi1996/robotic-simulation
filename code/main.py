import numpy as np
import logging

from robots import TwoLegRobot
import traces
import plotting

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    logging.info("Creating robot...")
    # r = TwoLegRobot()
    r = TwoLegRobot(speed=0.5)
    # r = TwoLegRobot(y_lift=0.005)

    logging.info("Navigating terrain...")
    # r.walk_distance(-0.08)
    r.walk_distance(-0.49)
    r.navigate_step(0.05)
    remaining_distance = -1.0 - r.x[0, -1]
    r.walk_distance(remaining_distance)

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
    plotting.plot_robot_trajectory(r.x, r.y, r.L, r.dt)
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
