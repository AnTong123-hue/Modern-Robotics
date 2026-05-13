import csv
import numpy as np
from scipy.optimize import linprog
import argparse
import os
from datetime import datetime

def F_ext_extract(bodies_file, rslt_file_path, g=9.8):
    '''
    Arguments:
        - bodies_file: csv file storing position and mass propertiy of each body
    Return:
        - F_ext      : ndarray(n,) - Mass of each body
        - cm_pos     : ndarary(n, 2) - Position of center of mass of each body
    '''
    bodies_csv = []
    with open(bodies_file) as csv_file:
        bodies_csv_read = csv.reader(csv_file)
        for line in bodies_csv_read:
            bodies_csv.append(line)
    
    # Change format of bodies csv to float
    F_ext = []
    cm_pos = []
    with open(rslt_file_path, 'w', encoding='utf-8') as log_file:
        log_file.write('================================ Input data ==================================\n')
        log_file.write('External wrenches (Gravity force only):\n')
        for i in range(len(bodies_csv)):
            body_info = bodies_csv[i]
            body_info_flt = []
            for j in range(len(body_info)):
                body_info_flt.append(float(body_info[j]))
            cm_pos.append(body_info_flt[0:2])
            F_ext.extend([0,0,- g * body_info_flt[-1]])
            log_file.write(f'-- Bodies {i+1} - Position: {cm_pos[-1]} - Wrench: {body_info_flt[-1]}\n')


    return F_ext, cm_pos

def F_wrench_extract(contacts_file, cm_pos, rslt_file_path):
    '''
    Arguments:
        - contacts_file: csv file storing information of contacts (bodies, position, angle, fric_coef)

    Return:
        - F_bodies_contacts: ndarray(n, m, 3)
            + n : number of bodies
            + m : number of contacts for each body
            + 3 : dimension of contact wrench 
    '''
    contacts_csv = []
    with open(contacts_file) as csv_file:
        contacts_csv_read = csv.reader(csv_file)
        for line in contacts_csv_read:
            contacts_csv.append(line)
    contacts_arr = []

    contacts_arr = [list(map(float, row)) for row in contacts_csv]
    
    # Define contacts for each body - if body is named in the first columne and + if body is named in the second column
    # Calculate total number of contacts
    total_ct = 0
    with open(rslt_file_path, 'a', encoding='utf-8') as log_file:
        log_file.write('Contact informations:\n')
        for j in range(len(contacts_arr)):
            contact = contacts_arr[j]
            u = contact[-1]
            if (u == 0):
                total_ct += 1
            else:
                total_ct += 2
            log_file.write(f'--First body: {contact[0]} - Second body: {contact[1]} - Position: {contact[2:4]} - Angle: {contact[4]} - Fric_coef: {contact[5]}\n')


    F_contacts = np.zeros((3 * len(cm_pos), total_ct))

    for i in range(len(cm_pos)):
        cm_x, cm_y = cm_pos[i]
        ct_idx = 0
        for j in range(len(contacts_arr)):
            contact = contacts_arr[j]
            ct_px = contact[2] - cm_x
            ct_py = contact[3] - cm_y
            alpha, u = contact[4:]
            fric_angle = np.arctan(u)
            if u == 0:
                # only one contact wrench is calculated
                if (i+1) in contacts_arr[:2]:
                    F_contact = np.array([ct_px * np.sin(alpha) - ct_py * np.cos(alpha), np.cos(alpha), np.sin(alpha)]).T
                    if contact[0] == (i+1):
                        F_contacts[3*i:3*i+3, ct_idx] = -F_contact
                    elif contact[1] == (i+1):
                        F_contacts[3*i:3*i+3, ct_idx] = F_contact
                ct_idx += 1
            else:
                # two contact wrenches are calculated
                alpha_1 = alpha - fric_angle
                alpha_2 = alpha + fric_angle
                F_contact_1 = np.array([ct_px * np.sin(alpha_1) - ct_py * np.cos(alpha_1), np.cos(alpha_1), np.sin(alpha_1)]).T
                F_contact_2 = np.array([ct_px * np.sin(alpha_2) - ct_py * np.cos(alpha_2), np.cos(alpha_2), np.sin(alpha_2)]).T
                if contact[0] == (i+1):
                    F_contacts[3*i:3*i+3, ct_idx]   = -F_contact_1 
                    F_contacts[3*i:3*i+3, ct_idx+1] = -F_contact_2
                if contact[1] == (i+1):
                    F_contacts[3*i:3*i+3, ct_idx]   = F_contact_1
                    F_contacts[3*i:3*i+3, ct_idx+1] = F_contact_2
                ct_idx += 2
        
    with open(rslt_file_path, 'a', encoding='utf-8') as log_file:
        log_file.write('============================== Processed data ================================\n')
        log_file.write(f'Wrench matrix for {len(cm_pos)} bodies and {total_ct} contact wrenches:\n')
        for i in range(F_contacts.shape[0]):
            log_file.write(f"Body {i // 3}: | {' '.join(map(str, np.round(F_contacts[i],2)))} |\n")


    return F_contacts, total_ct;

def check_force_closure(F_ext, F_contacts, total_ct):
    '''
    Solving equality equation for each body
        F_ext[i] + sum(x * F_bodies_contacts[i]) = 0 and x >= 0
    Where 
        + The number of equality constraints is n * 3
        + The number of inequality constraints is m  (m is the total number of contact) 
    '''
    c = [1 for i in range(total_ct)]
    b_ub = [-1 for i in range(total_ct)]
    A_ub = -np.eye(total_ct)
    A_ub.tolist()
    print("Inequality Conditions:")
    print("A_ub: ")
    print(A_ub)
    print("b_ub: ")
    print(b_ub)
    A_eq = F_contacts
    np.set_printoptions(precision=2, suppress=True)
    print("Equality Conditions:")
    print("A_eq: ")
    print(A_eq)
    A_eq.tolist()
    b_eq = - np.array(F_ext)
    b_eq.tolist()
    print("b_eq: ")
    print(b_eq)
    
    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, method='highs')
    return res.success, res.x

def main(bodies_file, contacts_file):
    '''
    Arguments:
        - bodies_file: csv file storing position and mass propertiy of each body
        - contacts_file: csv file storing information of contacts (bodies, position, angle, fric_coef)
    Return:
        - force_closure   : binary - 1 represents form closure
        - vec_k         : ndarray(n,) - nonnegative magnitude of each contact wrenches 
    '''
    now = datetime.now()
    formated = now.strftime("_%m_%d_%H_%M_%S")
    rslt_dir = "./results"
    rslt_file = f"result{formated}.log"
    print(rslt_file)
    if not os.path.exists(rslt_dir):
        os.makedirs(rslt_dir)

    rslt_file_path = os.path.join(rslt_dir, rslt_file)

    F_ext, cm_pos = F_ext_extract(bodies_file, rslt_file_path)
    F_contacts, total_ct = F_wrench_extract(contacts_file, cm_pos, rslt_file_path)
    force_closure, vec_k = check_force_closure(F_ext, F_contacts, total_ct)

    if force_closure:
        print("With these contacts, object is in form closure!!")
        print("Magnetude for each contacts are:")
        print(vec_k)
    else:
        print("With these contact, object is not in form closure!!")

    with open(rslt_file_path, 'a', encoding='utf-8') as log_file:
        log_file.write('================================= Results ===================================\n')
        if force_closure:
            log_file.write("With these contacts, object is in form closure!!\n")
            log_file.write("Magnetude for each contacts are:")
            log_file.write(f"{vec_k}\n")
        else:
            log_file.write("With these contact, object is not in form closure!!")

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bodies", help="Bodies' mass properties csv file")
    parser.add_argument("-c", "--contacts", help="Contacts' properties csv file")
    args = parser.parse_args()

    if args.bodies and args.contacts:   
        main(args.bodies, args.contacts)
    else:
        print("Not enough input argument !!!")