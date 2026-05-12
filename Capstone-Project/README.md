Execute the script src/FeedforwardControl.py with two arguments:
    -d --data: choose between original data from course or my modified data
    -c --controller: choose one of the following controller 
                    + FPI: Feedforward PI control (This is the best controller for the system)
                    + F: Feedforward control (This is the worst controller for the system)
                    + PI: PI control

Example:

```python src/FeedForwardControl.py -d modified -c PI```

Using this command, you are running the PI control of my modified initial configuration of end-effector

After the script is completed, please check its generated **log file** and **error plot** in *./error log* dir and its generated simulation configs data in *./results dir*

Enjoy !!!