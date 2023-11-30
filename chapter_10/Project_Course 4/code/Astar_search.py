import csv
import numpy as np

class Node:
	def __init__(self, ID, x, y, h):
		self.ID = ID
		self.x = x
		self.y = y
		self.h = h
		pass

class Edge:
	def __init__(self, ID1, ID2, c):
		self.ID1 = ID1
		self.ID2 = ID2
		self.c = c
		pass

class Obstacle:
	def __init__(self, x,y,d):
		self.x = x
		self.y = y
		self.d = d
		pass

edges = []
nodes = []
obstacles = []

with open('nodes.csv', newline='') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	for row in csv_reader:
		row = list(row)
		if row[0][0] != '#':
				ID = int(row[0])
				x = float(row[1])
				y = float(row[2])
				h = float(row[3])
				node = Node(ID,x,y,h)
				nodes.append(node)

with open('edges.csv', newline='') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	for row in csv_reader:
		row = list(row)
		if row[0][0] != '#':
				ID1 = int(row[0])
				ID2 = int(row[1])
				c = float(row[2])
				edge = Edge(ID1,ID2,h)
				edges.append(edge)

with open('obstacles.csv', newline='') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	for row in csv_reader:
		row = list(row)
		if row[0][0] != '#':
				x = float(row[0])
				y = float(row[1])
				d = float(row[2])
				obstacle = Obstacle(ID1,ID2,h)
				obstacles.append(obstacle)

def cost_func(current, nbr):
	cost = None
	for j in range(len(edges)):
		if edges[j].ID1 == current.ID:
			if edges[j].ID2 == nbr.ID:
				cost = edges[j].c
		elif edges[j].ID2 == current.ID:
			if edges[j].ID1 == nbr.ID:
				cost = edges[j].c
	return cost

def A_star(nodes,edges):
	nodes[0].past_cost = 0
	nodes[0].est_total_cost = nodes[0].past_cost + nodes[0].h
	nodes[0].parent = None
	for node in range(1, len(nodes)):
		nodes[node].past_cost = float('inf')
	goal = nodes[len(nodes)-1]	
	OPEN = [nodes[0]]
	CLOSED = []

	while OPEN != []:
		nbr = []
		current = OPEN.pop(0)
		CLOSED.append(current)
		if current == goal:
			return "Success", CLOSED
		for i in range(len(edges)):
			if edges[i].ID1 == current.ID:
				nbr.append(nodes[edges[i].ID2-1])
			elif edges[i].ID2 == current.ID:
				nbr.append(nodes[edges[i].ID1-1])
		for neighbor in nbr:
			if neighbor not in CLOSED:
				cost = cost_func(current, neighbor)
				if cost == None:
					continue
				tentative_past_cost = current.past_cost + cost
				if tentative_past_cost < neighbor.past_cost:
					neighbor.past_cost = tentative_past_cost
					neighbor.parent = current
					neighbor.est_total_cost = neighbor.past_cost + neighbor.h
					OPEN.append(neighbor)
					OPEN.sort(key=lambda neighbor: neighbor.est_total_cost)
	return "Failure",CLOSED

def main():
	steps = []
	parents = []
	node_parents = []
	path = []
	indexes = []

	[result, CLOSED] = A_star(nodes,edges)
	
	parents.append(None)
	for k in range(len(CLOSED)):
		steps.append(CLOSED[k].ID)
	for k in range(1,len(CLOSED)):
		parents.append(CLOSED[k].parent.ID)
	
	new_step = steps[len(steps)-1]
	path.append(new_step)
	new_idx = steps.index(new_step)
	new_parent = parents[new_idx]
	while new_idx:
		path.insert(0,new_parent)
		new_idx = steps.index(new_parent)
		new_parent = parents[new_idx]
	print(path)

	with open('path.csv', 'w', newline='') as file:
		writer = csv.writer(file)
		writer.writerow(path)
	

main()