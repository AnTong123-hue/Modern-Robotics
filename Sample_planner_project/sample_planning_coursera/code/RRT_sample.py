import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

#========================= Extract information from obstacles.csv ==============================
obs_csv = []
with open("../results/obstacles.csv") as input_file:
    obs_csv_file = csv.reader(input_file)
    for line in obs_csv_file:
        if (line[0][0] != "#"):
            data = []
            for i in range(len(line)):
                data.append(float(line[i]))
            obs_csv.append(data)

print("Extracted information from obstacles.csv")
print(obs_csv)



#========================= Define function to save csv file ===================================

def save_result(csvdata, filename, opt=1):
    with open(f"../results/{filename}.csv", 'w', newline='') as save_file:
        writer = csv.writer(save_file)
        if (opt == 1):
            writer.writerows(csvdata)
        else:
            writer.writerow(csvdata)

#========================= Define colision check function =====================================
def Euclidian_dist(node1, node2):
    x1, y1 = node1
    x2, y2 = node2
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)

def colision_check(node1, node2, obs_csv, print_opt=0):
    """
    Arguments:
        node1: (x1, y1) - position of node 1
        node2: (x2, y2) - position of node 2
        obs_csv: list of all obstacles in C-space [[x_obs, y_obs, r_obs]]
    Return:
        obs_colided: 1 - colided, 0 - not colided
    """
    x1, y1 = node1
    x2, y2 = node2
    m = (y2 - y1) / (x2 - x1)
    c = y1 + m * (- x1)   
    # Analytically check whether distance from 
    for i in range(len(obs_csv)):


        x_obs, y_obs, d_obs = obs_csv[i]
        r_obs = d_obs / 2
        # From 2 equations
        # y = mx + c                                (1)
        # (x-x_obs)^2 + (y-y_obs)^2 = r_obs^2       (2)
        # Find the intersections of (1) and (2)
        delta = (m*(c-y_obs)-x_obs)**2 - (1+m**2)*(x_obs**2+(c-y_obs)**2 - r_obs**2)
        if (print_opt == 1):
            print(f"Check node1: [{node1[0]}, {node1[1]}] and node2: [{node2[0]}, {node2[1]}] with obstacles [{x_obs}, {y_obs}, {d_obs}]")
            print(f"Delta = {delta}")
        if (delta == 0):
            x_intersect = -(m*(c-y_obs)-x_obs) / (1+m**2)
            if (x_intersect <= max(x1, x2) and x_intersect >= min(x1, x2)):
                return 1
        elif (delta > 0):
            x_intersect1 = -(m*(c-y_obs)-x_obs) / (1+m**2) + np.sqrt(delta) / (1+m**2)
            x_intersect2 = -(m*(c-y_obs)-x_obs) / (1+m**2) - np.sqrt(delta) / (1+m**2)
            if (   (x_intersect1 <= max(x1, x2) and x_intersect1 >= min(x1, x2))
                or (x_intersect2 <= max(x1, x2) and x_intersect2 >= min(x1, x2))):
                return 1

    return 0

#========================= Define sampling function ===========================================
def deterministic_sampling(boundary, node_goal, likelihood):
    """
    Arguments: 
        boundary  : list(list) [[xmin, xmax], [ymin, ymax], [zmin, zmax], ...] - indicates boundary of each dimension of C-space
        goal_set  : list(list) [goal1, goal2, goal3, ...]                      - indicates goal set used for deterministc sampling method
        likelihood: float                                                      - indicates the probability of choosing sample from X_goal
    Return:
        node_samp    : list [x, y, z, ...]                                  - indicates configuration of "random" sample 
    """
    x_min, x_max = boundary[0]
    y_min, y_max = boundary[1]
    N = 100
    seed0 = np.random.randint(0, N)
    if seed0 < (0.1 * N):
        node_samp = node_goal
    else:
        node_samp = [np.random.uniform(x_min, x_max), np.random.uniform(y_min, y_max)]
    return node_samp

#========================= Define find nearest nbr function ===================================
def find_nearest_nbr(nodes_csv, node_samp, obs_csv):
    """
    Arguments: 
        - nodes_csv     : list(list) [[ID, x, y, ctg]]  - List of nodes occupied by previous search
        - node_samp     : list       [x, y]             - Configuration of "random" sample node
        - obs_csv       : list(list) [[x, y, r]]        - List of obstacles with position and radius information

    Returns:
        - node_nearest  : list       [x, y]             - Configuration of nearest node from tree to sample node
    """
    dist_arr = np.zeros((len(nodes_csv))) # >0 : colision-free distance
                                          # 0  : 2 nodes are obstructed by at least one obstacle

    for i in range(len(nodes_csv)):
        node_tree = [nodes_csv[i][1], nodes_csv[i][2]]
        dist_arr[i] = Euclidian_dist(node_tree, node_samp)
    print(dist_arr)
    i_nearest = np.argmin(dist_arr)
    node_nearest = [nodes_csv[i_nearest][1], nodes_csv[i_nearest][2]]
    return i_nearest, node_nearest

#========================= Define local planer function ======================================
def straight_local_planner(node_nearest, node_samp, d):
    """
    Arguments:
        - node_nearest  : list [x_nearest, y_nearest]    - Configuration of nearest node to sample node
        - node_samp     : list [x_samp, y_samp]]         - Configuration of sample node
        - d             : float                          - Distance between 2 nodes
    Return:
        - node_new      : list [x_new, y_new]            - Configuration of new node occupied
    """
    # Straight line parameter
    x_nearest, y_nearest = node_nearest
    x_samp, y_samp = node_samp
    node_dist = Euclidian_dist(node_nearest, node_samp)
    if (node_dist < d):
        node_new = node_samp
    else:
        x_new = x_nearest + (d / node_dist) * (x_samp - x_nearest)
        y_new = y_nearest + (d / node_dist) * (y_samp - y_nearest)
        node_new = [x_new, y_new]
    return node_new    

def path_to_current(parent, current_ID, start_ID):
    parent_node = parent[current_ID]
    if (parent_node == start_ID):
        return [int(parent_node)+1, int(current_ID)+1]
    else:
        return path_to_current(parent, parent_node, start_ID) + [int(current_ID)+1] 

def plot_rrt(nodes_csv, node_start, node_goal, parent, path_csv, obs_csv):

    figure, axes = plt.subplots()

    for i in range(len(nodes_csv)):
        if parent[i] != -1:
            plt.plot([nodes_csv[i][1], nodes_csv[parent[i]][1]], [nodes_csv[i][2], nodes_csv[parent[i]][2]], 'go-')
    plt.plot(node_start[0], node_start[1], 'yo', markersize=12, label="START")
    plt.plot(node_goal[0], node_goal[1], 'bo', markersize=12, label="GOAL")

    # Plot optimal path
    for i in range(len(path_csv)-1):
        node_i = path_csv[i] - 1
        node_j = path_csv[i+1] - 1
        plt.plot([nodes_csv[node_i][1], nodes_csv[node_j][1]], [nodes_csv[node_j][2], nodes_csv[node_j][2]], 'r+-', linewidth=2, markersize=12)

    # Plot the obstacles
    for i in range(len(obs_csv)):
        x_obs, y_obs, d_obs = obs_csv[i]
        obs_circle = plt.Circle(( x_obs, y_obs ), d_obs / 2)
        axes.add_artist( obs_circle )
    
    axes.set_aspect( 1 )

    plt.legend()
    plt.grid()
    plt.xlim((-0.5, 0.5))
    plt.ylim((-0.5, 0.5))
    plt.show()


#========================= Define RRT algorithm =============================================
def RRT_algorithm(boundary, obs_csv, node_start, node_goal, d, likelihood=0.1, max_size=100):
    """
    Arguments: 
        boundary    : list(list) [[xmin, xmax], [ymin, ymax], [zmin, zmax], ...] - indicates boundary of each dimension of C-space
        obs_csv     : list(list) [[x, y, r]]                                     - List of obstacles with position and radius information
        node_start  : list [x_start, y_start]                                    - Start node of tree
        node_goal   : list(list) [goal1, goal2, goal3, ...]                      - indicates goal set used for deterministc sampling method
        d           : float                                                      - indicates distance between 2 nodes in constructed tree
        likelihood  : float                                                      - indicates the probability of choosing sample from X_goal

    Return:
        nodes_csv    : list of [ID, x, y, ctg ]                                   - indicates information of nodes
        edge_csv    : list of [ID1, ID2, edge]                                   - indicates connected edges in tree
        path_csv    : list
    """
    nodes_csv = [[1, node_start[0], node_start[1], Euclidian_dist(node_start, node_goal)]]
    edge_csv = []
    path_csv = []
    parent   = [-1]
    success  = 0
    count    = 0
    while (len(nodes_csv) < max_size and not success):
        count += 1
        print(f"===================================== Iteration {count} - Current tree size {len(nodes_csv)}=============================================")
        node_samp    = deterministic_sampling(boundary, node_goal, likelihood)
        print("-- Step 1: Random Sample Node: ", node_samp)
        i_nearest, node_nearest = find_nearest_nbr(nodes_csv, node_samp, obs_csv)
        print("-- Step 2: Find nearest node to sample node from current tree: ", node_nearest)
        node_new     = straight_local_planner(node_nearest, node_samp, d)
        print("-- Step 3: Calculate new node: ", node_new)
        if (not colision_check(node_nearest, node_new, obs_csv)):
            new_ID = len(nodes_csv) + 1
            nodes_csv.append([new_ID, node_new[0], node_new[1], Euclidian_dist(node_new, node_goal)])
            edge_csv.append([i_nearest + 1, new_ID, Euclidian_dist(node_new, node_nearest)])
            parent.append(int(i_nearest))
            if (node_new == node_goal):
                success = 1
                # Recursively find path from x_start to x_goal
                path_csv = path_to_current(parent, new_ID - 1, 0)
        else:
            print("---- Node is collided with obstacles")
    save_result(nodes_csv, "nodes")
    save_result(edge_csv, "edges")

    if (success):
        print("== PATH FOUND ==: ", path_csv)
        save_result(path_csv, "path", 0)
    else:
        print("== FAILED TO FIND PATH ==")
    print("Parent of nodes: ", parent)
    plot_rrt(nodes_csv, node_start, node_goal, parent, path_csv, obs_csv)

    
    return nodes_csv, edge_csv, path_csv

#========================= Execute RRT alogrithm ============================================

#========================= Initiate variables and parameters ==================================
node_start = [-0.5, -0.5]
node_goal  = [ 0.5,  0.5]
node_check = [-0.3, -0.2]
boundary   = [[-0.5, 0.5], [-0.5, 0.5]]
# 10% of samples are chosen near X goal set0
likelihood = 0.1

# Maximum tree size
tree_max_size  = 1000
tree_size      = 0

nodes_csv = [] # Each element represents [ID, x, y, ctg]
edges_csv = [] # Each element represents [ID1, ID2, Edge]
path_csv  = []
parent    = [] # Element i represent parent node of node i+1

if __name__ == "__main__":
    # Test colision function
    # check = colision_check(node_start, node_check, obs_csv, 1)
    # print("Check result:", check)

     nodes_csv, edges_csv, path_csv = RRT_algorithm(boundary, obs_csv, node_start, node_goal, d=0.02, likelihood=0.1, max_size=1000)
    