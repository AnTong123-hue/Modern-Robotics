## Capstone Project summary
---
This capstone project follows NorthWestern University's [Mobile Manipulation Capstone Project](https://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone). Milestones of the project can be summarized:
- [x] Write an odometry function to calculate next configuration of the mobile robot from current configuration and commanded joint velocities and wheel velocities
- [x] Write an trajectory generator to generate twist type or cartesian type trajectory for end-effector to satisfy eight tasks
- [x] Write a feedforward control function to calculate required twist command to minimize configuration error between current configuration and desired configuration belong to trajectory
- [x] Write a main function that uses all features above to execute pick and place task

## How to run this project
Execute the script src/FeedforwardControl.py with two arguments:
    -d --data: choose between "best", "overshoot" or "newtask" to generate different simulated data
                    + best: the controller use best PI gains
                    + overshoot: the controller uses overshoot PI gains
                    + newtask: the initial configuration of mobile manipulator is differed
    -c --controller: choose one of the following controller 
                    + FPI: Feedforward PI control (This is the best controller for the system)
                    + F: Feedforward control (This is the worst controller for the system)
                    + PI: PI control

**Example:** Run the simulator with zeros configuration and best PI gain controller.
```
pip install -e mobile_manipulation
python src/code.py -d best -c FPI
```

## FAQ
Enjoy :joy