import csv
import numpy as np
from scipy.optimize import linprog
import sys
import argparse

def csv_extraction(input_file):
    '''
    Arguments:
        - input_file: csv file storing position and direction information of contacts
    Return:
        - F         : ndarray(n,3) - Contact wrenches matrix. F_i = [m_iz, f_ix, f_iy].T
            + f_ix = cos(contact_angle)
            + f_iy = sin(contact_angle)
            + m_iz = p_ix * sin(contact_angle) - p_iy * cos(contact_angle)
    '''
    # Read raw data from CSV file
    contacts_csv = []
    with open(input_file) as csv_file:
        contacts_csv_reader = csv.reader(csv_file)
        for line in contacts_csv_reader:
            contacts_csv.append(line)
    
    # Change format of contacts csv to float
    contacts_csv_float = []
    for i in range(len(contacts_csv)):
        contacts_csv_float_line = []
        for j in range(len(contacts_csv[i])):
            contacts_csv_float_line.append(float(contacts_csv[i][j]))
        contacts_csv_float.append(contacts_csv_float_line)
    
    # Convert to F matrix 
    F = np.zeros((3, len(contacts_csv_float)))
    print(F.shape)
    for i in range(len(contacts_csv_float)):
        px, py, alpha = contacts_csv_float[i]
        F[:, i] = np.array([px*np.sin(alpha) - py*np.cos(alpha), np.cos(alpha), np.sin(alpha)]).T

    F.tolist()
    print(F)
    return F

def form_closure_check(F):
    '''
    Arguments:
        - F             : ndarray(n,3) - Contact wrenches matrix
    Return:
        - form_closed   : binary - 1 represents form closure
        - vec_k         : ndarray(n,) - nonnegative magnitude of each contact wrenches 
    '''
    c = [1 for i in range(len(F[0]))]
    b_ub = [-1 for i in range(len(F[0]))]
    A_ub = -np.eye(len(F[0]))
    A_ub.tolist()
    print("Inequality Conditions:")
    print("A_ub: ")
    print(A_ub)
    print("b_ub: ")
    print(b_ub)
    A_eq = F
    # A_eq = np.array(F).T
    # A_eq = A_eq.tolist()
    b_eq = [0 for i in range(len(F))]
    print("Equality Conditions:")
    print("A_eq: ")
    print(A_eq)
    print("b_eq: ")
    print(b_eq)


    res = linprog(c, A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, method='highs')
    return res.success, res.x


def main(input_file):
    '''
    Arguments:
        - input_file: csv file storing position and direction information of contacts
    Return:
        - form_closed   : binary - 1 represents form closure
        - vec_k         : ndarray(n,) - nonnegative magnitude of each contact wrenches 
    '''
    F = csv_extraction(input_file)
    form_closed, vec_k = form_closure_check(F)

    if form_closed:
        print("With these contacts, object is in form closure!!")
        print("Magnetude for each contacts are:")
        print(vec_k)
    else:
        print("With these contact, object is not in form closure!!")
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--source", help="Source file")
    args = parser.parse_args()

    if args.source:   
        main(args.source)
    else:
        print("Please add input argument")