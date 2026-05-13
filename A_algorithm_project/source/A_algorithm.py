# Import library
import csv
import numpy as np
import logging
import os

logging.basicConfig(filename='test.log', level=logging.INFO, format='%(levelname)s: %(message)s', filemode='w')

def open_list_add(open_list, total_estimated_cost, nbr):
    if (open_list.shape[0] == 0):
        open_list = np.insert(open_list, 0, nbr)
    else:
        nbr_inserted = 0
        for i in range(open_list.shape[0]):
            if total_estimated_cost[nbr] < total_estimated_cost[open_list[i]-1]:
                open_list = np.insert(open_list, i, nbr)
                nbr_inserted = 1
                break
        if (not nbr_inserted):
            open_list = np.append(open_list, nbr)
    return open_list

def path_to_current(parent, current_node, start_node):
    parent_node = parent[current_node]
    if (parent_node == start_node):
        return [int(parent_node)+1, int(current_node)+1]
    else:
        return path_to_current(parent, parent_node, start_node) + [int(current_node)+1] 

def main():
    # ======= Set up log file =====================
    # ======= Reading edges.csv & nodes.csv =======
    edges_csv = []
    with open('../copeliasim_input_files/edges.csv') as edges_file:
        print("1. Read data from edges.csv")
        csv_edge_file = csv.reader(edges_file)
        for line in csv_edge_file:
            # Clean comment line
            if (line[0][0] != '#'):
                edges_csv.append(line)
                print(line)


    nodes_csv = []
    with open('../copeliasim_input_files/nodes.csv') as nodes_file:
        print("2. Read data from nodes.csv")
        csv_node_file = csv.reader(nodes_file)
        for line in csv_node_file:
            # Clean comment line
            if (line[0][0] != '#'):
                nodes_csv.append(line)
                print(line)

    # ======= Structurize datas ===================
    # Turn edges csv into edges matrix for easy searching
    # ID1, ID2, edge
    nodes_num = int(nodes_csv[-1][0])
    print(f"Number of nodes: {nodes_num}")
    edges_mat = np.zeros((nodes_num, nodes_num), dtype = np.float64)
    for i in range(len(edges_csv)):
        edge = edges_csv[i]
        edges_mat[int(edge[0])-1, int(edge[1])-1] = float(edge[-1])
        edges_mat[int(edge[1])-1, int(edge[0])-1] = float(edge[-1])

    print("Edges matrix: ")
    print(edges_mat)

    # Initialize cost vectors
    past_cost = np.zeros(nodes_num, dtype = np.float64)
    cost_to_go = np.zeros(nodes_num, dtype=np.float64)
    total_estimated_cost = np.zeros(nodes_num, dtype=np.float64)
    # Past cost of node 1 is 0 at the begining while costs of node greater than 1 are inf
    past_cost[:] = 1e10
    past_cost[0] = 0
    # Retrieve optimistic cost to go from nodes.csv
    for i in range(len(nodes_csv)):
        node_info = nodes_csv[i]
        cost_to_go[i] = float(node_info[-1])

    # Calculate total estimated cost
    total_estimated_cost = past_cost + cost_to_go
    # Initialized parent array where -1 value indicates node without parent
    parent = np.full(nodes_num, -1, dtype=np.int32)

    # ======= Excute A* Algorithm ================
    start_node  = 0
    end_node    = nodes_num-1
    open_list   = np.array([start_node])
    closed_list = np.array([])
    goal_list   = np.array([end_node])
    path_found  = 0
    print("\n================================================Start of A* algorithm=================================================")

    while (open_list.nonzero()):
        current_node = open_list[0]
        open_list    = open_list[1:]
        print(f"========================== Current checked node: {current_node} =============================================")
        np.append(closed_list, current_node)
        if (np.sum(goal_list == current_node)):
            path_found = 1
            print(f"End of A* algorithm. Showing parent status:")
            print(parent)
            path = path_to_current(parent, current_node, start_node)
            break
        nbr_list = np.where(edges_mat[current_node]>0)[0]
        print("-- Current neighbor list: ", nbr_list)
        for nbr in nbr_list:
            
            if (np.sum(closed_list == nbr) == 0):
                tentative_past_cost = past_cost[current_node] + edges_mat[current_node, nbr]
                print(f"---- Check neighbor {nbr} of node {current_node} with pastcost = {past_cost[nbr]:.2f} and tentative_past_cost = {tentative_past_cost:.2f}")
                if (tentative_past_cost < past_cost[nbr]):
                    past_cost[nbr] = tentative_past_cost
                    parent[nbr]  = current_node
                    # Move nbr in sorted list
                    print(f"------ Update {nbr} into OPEN list")
                    open_list = open_list_add(open_list, total_estimated_cost, nbr)
                    total_estimated_cost[nbr] = past_cost[nbr] + cost_to_go[nbr]

    if path_found:
        print("Optimal path found by A* algorithm", path)
    else:
        print("Optimal path not found !!!!")

    # ======= Export result to CSV ================
    with open('../copeliasim_input_files/my_path.csv', 'w', newline='') as save_file:
        writer = csv.writer(save_file)
        writer.writerow(path)

if __name__ == "__main__":

    main()

