# robotic-simulation

This repository is primarily for coursework submission to 4M20: 'Robotics' coursework 1.

## TODO

- Change lists to np.ndarrays, with first dim referring to joint-number, 2nd dim referring to time-step. To set new time-step, append 5 zeros to end of x-array and y-array, then set each joint for the new time step individually. Likewise for speeds, accelerations, torques. Can make entire array for a given action, then concatenate onto the total
- Add speeds