import random
import math
import csv

class Obstacle:
	def __init__(self, x,y,d):
		self.x = x
		self.y = y
		self.d = d
		pass

def RRT(x_start, x_goal, maxLen, obstacles):
    T = []
    edges = []
    T.append(x_start)
    x_nearest = x_start
    dist_old = math.inf
    while (len(T)<maxLen):
        x_samp = sampling_uniform(x_goal)
        for x in T:
            dist_new = distance(x, x_samp)
            if dist_new < dist_old:
                dist_old = dist_new
                x_nearest = x
        x_new = point_at_distance(distance(x_nearest,x_samp), x_nearest, x_samp)
        if not check_path_for_collisions(obstacles, x_nearest, x_new):
            T.append(x_new)
            edges.append([x_nearest,x_new,distance(x_nearest, x_new)])
            if x_new == x_goal:
                 return T, edges, "SUCCESS"
    return T, edges, "FAILURE"

def sampling_uniform(x_goal):
    if random.random() < 0.1:
        return x_goal
    else: 
        xvalue = random.uniform(-0.5,0.5)
        yvalue = random.uniform(-0.5,0.5)
        return [xvalue, yvalue]

def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5

def check_path_for_collisions(obstacles, start_point, end_point):
    x1, y1 = start_point
    x2, y2 = end_point

    # Calculate the coefficients of the line passing through the start and end points
    m = (y2 - y1) / (x2 - x1) if x2 != x1 else math.inf
    q = y1 - m * x1 if x2 != x1 else None

    # Check for intersection between the line and each circle
    for obstacle in obstacles:
        # Calculate the distance between the center of the circle and the line
        dist_to_line = abs(m*obstacle.x - obstacle.y + q) / math.sqrt(m**2 + 1)

        # Calculate the distance between the closest point on the line and the center of the circle
        closest_x = (obstacle.x + m*obstacle.y - m*q) / (m**2 + 1)
        closest_y = m * closest_x + q

        if x1 <= closest_x <= x2 or x2 <= closest_x <= x1:
            if y1 <= closest_y <= y2 or y2 <= closest_y <= y1:
                if dist_to_line <= obstacle.d / 2:
                    return True

    return False


def point_at_distance(d, starting_point, ending_point):
    x1, y1 = starting_point
    x2, y2 = ending_point
    
    # Calculate the length of the line
    line_length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Calculate the coordinates of the point at distance d along the line
    x_point = x1 + (d * (x2 - x1)) / line_length
    y_point = y1 + (d * (y2 - y1)) / line_length
    
    return [x_point, y_point]

def create_matrix(input_list):
    matrix = []
    for i, element in enumerate(input_list):
        row = [i+1, element[0], element[1]]
        matrix.append(row)
    return matrix

def write_matrix_to_csv(matrix, file_path):
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in matrix:
            writer.writerow(row)

def assemble_goal_distance_matrix(nodes, goal):
    goal_distance_matrix = []
    for node in nodes:
        node_id = node[0]
        node_x = node[1]
        node_y = node[2]
        distance_to_goal = ((node_x - goal[0])**2 + (node_y - goal[1])**2)**0.5
        goal_distance_matrix.append([node_id, node_x, node_y, distance_to_goal])
    return goal_distance_matrix

def assemble_edge_matrix(nodes, edges):
    edge_matrix = []
    for edge in edges:
        start_point = edge[0]
        end_point = edge[1]
        distance = edge[2]
        
        start_id = None
        end_id = None
        
        # Find the IDs of the starting and ending nodes in the nodes matrix
        for node in nodes:
            if node[1] == start_point[0] and node[2] == start_point[1]:
                start_id = node[0]
            if node[1] == end_point[0] and node[2] == end_point[1]:
                end_id = node[0]
            
            if start_id is not None and end_id is not None:
                break
        
        if start_id is not None and end_id is not None:
            edge_matrix.append([start_id, end_id, distance])
        else:
            print(f"Warning: Edge not connected to nodes in the 'nodes' matrix.")
    
    return edge_matrix


x_start = [-0.5,-0.5]
x_goal = [0.5, 0.5]
maxLen = 10000
obstacles = []

with open('obstacles.csv', newline='') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	for row in csv_reader:
		row = list(row)
		if row[0][0] != '#':
				x = float(row[0])
				y = float(row[1])
				d = float(row[2])
				obstacle = Obstacle(x,y,d)
				obstacles.append(obstacle)
                    
T, edges, status = RRT(x_start, x_goal, maxLen, obstacles)
print(status)

nodes = create_matrix(T)
nodes_matrix = assemble_goal_distance_matrix(nodes, x_goal)
write_matrix_to_csv(nodes_matrix, "nodes.csv")

edges_matrix = assemble_edge_matrix(nodes, edges)
write_matrix_to_csv(edges_matrix, "edges.csv")