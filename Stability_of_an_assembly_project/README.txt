Program specification:
- Purpose: determine whether an assembly of planar rigid bodies in friction contact with each other can remain standing in gravity
- Input:
    + A description of the static mass of N bodies: (x, y, m) 
        x, y: location of the center of mass 
        m: the total mass
    + A description of the contacts: (b1, b2, x, y, alpha, u)
        b1, b2: the first and the second body involved in the contact
        x, y: position of the contact
        alpha: planar angle of the contact
        u: friction coefficient. 
- Output:
    yes/no: is it posible for the assembly to remain standing 

In this directory create your CSV file and run this command in terminal
    python.exe .\code.py -b {body_file_name} -c {contact_file_name}

Log file will be created in result_month_day_hour_min_second.log format in results directory