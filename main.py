from robots import TwoLegRobot
import plotting

if __name__ == "__main__":

    r = TwoLegRobot()
    
    r.walk_distance(-0.49)
    r.navigate_step(0.05)
    remaining_distance = -1.0 - r.x[0, -1]
    r.walk_distance(remaining_distance)

    plotting.plot_robot_trajectory(r.x, r.y, r.L, r.dt)
    xlims = None
    # xlims = [0, 2]
    plotting.plot_traces(
        r.dt, r.x[1:4], "x", "Horizontal joint-positions", "x_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.y[1:4], "y", "Vertical joint-positions", "y_", 1, xlims=xlims
    )
    plotting.plot_traces(
        r.dt, r.theta[0:2], "theta", "Joint-angles", "theta_", xlims=xlims
    )

    print(r.x.shape)
