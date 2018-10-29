# robotic-simulation

This repository is primarily for coursework submission to 4M20: 'Robotics' coursework 1.

## TODO

- Add speeds (use numerical differentiation ?? Quantify dependence on dt ??)
- Include equation of motion to model torques: if using exclusively joint positions, velocities, and accelerations, then don't need to use any angles, and don't need to assume that one foot is stuck to the floor ????? In general this will need to be ITO angles (for Euler-Lagrange equations to get torques), so might need to use Jacobians; or might just need speeds of one joint for reference
- Calculating and storing both x0 and x1 and both x3 and x4 uses unnecessary time and space (although it is convenient)
