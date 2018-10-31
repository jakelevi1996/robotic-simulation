# robotic-simulation

This repository is primarily for coursework submission to 4M20: 'Robotics' coursework 1.

## File description

The top-level directory contains the following files and folders:

- **Code** folder:
  - `robots.py`: Module containing the class `TwoLegRobot` and its methods for navigation (EG `navigate_step`)
  - `traces.py`: Module containing functions which calculate traces such as `cubic_motion_trajectory` and `instantaneous_power`
  - `plotting.py`: functions for plotting robot trajectories (EG `plot_robot_trajectory`) and traces (EG `plot_traces`)
  - `main.py`: main script for using these modules
- **Images** folder:
  - Some images, go have a look
- `README.md`: supposedly a complete description of the repository
- `Coursework description.pdf`: Description of the coursework task
- `.gitignore`: don't commit these files

## TODO

- Add usage description and images to readme
- Include equation of motion to model *dynamic* torques (instead of static approximation): might be helpful to use numerically differentiated traces
- Calculating and storing both x0 and x1 and both x3 and x4 uses unnecessary time and space (although it is convenient)
- Animate trajectory
