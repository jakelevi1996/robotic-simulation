import numpy as np

def cubic_motion_trajectory(start_pos, distance, T, N):
    """Calculate motion trajectory with given start point, end point,
    time-period, and number of points, using cubic motion control (IE
    quadratic velocity) with zero start and end-velocity
    """
    t = np.linspace(0, T, N)
    return distance*(t**2)*(3*T - 2*t)/(T**3) + start_pos

def differentiate(x, dt):
    xdot = np.zeros(x.shape)
    xdot[:, 1:] = (x[:, 1:] - x[:, :-1]) / dt
    return xdot

def derivatives(x, y, theta, dt):
    """Use this function to calculate velocties and accelerations of horizontal
    joint-positions, vertical joint-positions, and joint-angles
    """
    xdot = differentiate(x, dt)
    ydot = differentiate(y, dt)
    thetadot = differentiate(np.radians(theta), dt)
    return xdot, ydot, thetadot

def energies(x, y, xdot, ydot, M, g):
    # Calculate kinetic energy
    kinetic_energy = 0.5 * M * sum([
        np.square(xdot[1]),
        np.square(ydot[1]),
        np.square((xdot[1] + xdot[2])/2),
        np.square((ydot[1] + ydot[2])/2),
        np.square((xdot[2] + xdot[3])/2),
        np.square((ydot[2] + ydot[3])/2),
        np.square(xdot[3]),
        np.square(ydot[3]),
    ])
    # Calculate relative potential energy
    potential_energy = M * g * (1.5*y[1] + y[2] + 1.5*y[3])
    potential_energy -= potential_energy.min()

    return kinetic_energy, potential_energy

def static_torques(x, M, g, foot1_clamped, foot2_clamped):
    """Calculate static torques, assuming kinetic energy, angular momentum
    etc are negligible, and the only relevant moments are due to gravity

    Sign convention: clockwise_positive (consistent with theta)
    """
    error_msg = "Can't calculate static torque when neither foot is clamped"
    if not all(foot1_clamped | foot2_clamped): raise ValueError(error_msg)
    
    # Initialise torque array
    torque = np.zeros([3, x.shape[1]])

    # Indices when foot 1 is clamped and foot 2 is not clamped:
    inds = foot1_clamped & np.invert(foot2_clamped)
    torque[0, inds] = M * g * (2.5*x[1, inds] - x[2, inds] - 1.5*x[3, inds])
    torque[1, inds] = M * g * (1.5*x[2, inds] - 1.5*x[3, inds])

    # Indices when foot 1 is not clamped and foot 2 is clamped:
    inds = np.invert(foot1_clamped) & foot2_clamped
    torque[2, inds] = M * g * (2.5*x[3, inds] - x[2, inds] - 1.5*x[1, inds])
    torque[1, inds] = M * g * (1.5*x[2, inds] - 1.5*x[1, inds])

    return torque

def instantaneous_power(torque, thetadot, foot1_clamped, foot2_clamped):
    """Calculate instantaneous power, found by multiplying torque by angular
    velocity. NB angular velocity must be flipped when foot 1 is not clamped
    and foot 2 is clamped, because although the sign convention remains
    consistent, in this case the link which is considered stationary and the
    link which is considered rotating relative to that stationary link are
    exchanged, and so signs must be flipped to maintain a consistent sign
    convention and return correct power values.
    """
    # Indices when foot 1 is not clamped and foot 2 is clamped:
    inds = ~foot1_clamped & foot2_clamped
    # Create array which is flipped when foot 1 is not clamped and foot 2 is
    # clamped and is otherwise the same:
    thetadot_flipped = np.zeros(thetadot.shape)
    thetadot_flipped[:, inds] = -thetadot[:, inds]
    thetadot_flipped[:, ~inds] = thetadot[:, ~inds]

    return torque * thetadot_flipped

def get_energy_consumption(instantaneous_power, dt):
    # Integrate power over time (but don't let negative powers 'charge up the
    # batteries'):
    return np.sum(np.maximum(instantaneous_power, 0)) * dt

def get_max_torques(torque):
    return (abs(torque)).max(axis=1)
