This program determine wheether a planar rigid body with specificied set of stationary point contacts is in form closure
Input:
    + stationary_contacts.csv: List of stationary point contacts on the body 
        - Position of contacts (catersian)
        - Direction of contacts (roll-pitch-yaw) angle
    + spatial or planar form closure select: Choose operation between spatial and planar setting. (Default is planar)
Output:
    + Form Closure check result: 1 represents form closure while 0 represents none form closure
    + Solution to linear program: vector k of nonnegative wrench magnitudes at the contacts solved from linear programming

Run command: python .\form_closure_check.py -s .\stationary_contacts.csv