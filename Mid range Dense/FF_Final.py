import matplotlib.pyplot as plt
import math
import numpy
from time import time

show_animation = True
ox, oy = [], []

# Distance between two points
def dbtp(x1,y1,x2,y2):
	return math.sqrt(((x2 - x1)**2) + ((y2 - y1)**2))

#this function returns the array of nodes through which robot will move
def dijkstra(graph, initial, end):
	# shortest paths is a dict of nodes
	# whose value is a tuple of (previous node, weight)
	shortest_paths = {initial: (None, 0)}
	current_node = initial
	visited = set()
	weights = {}

	for i in range(end+1):
		for j in range(end+1):
			if(graph[i][j] != 0):
				weights[(i,j)] = graph[i][j]
	while current_node != end:
		visited.add(current_node)
		destinations = []
		for i in range(end+1):
			if(graph[current_node][i] != 0):
				destinations.append(i)
		weight_to_current_node = shortest_paths[current_node][1]

		for next_node in destinations:
			weight = weights[(current_node, next_node)] + weight_to_current_node
			if next_node not in shortest_paths:
				shortest_paths[next_node] = (current_node, weight)
			else:
				current_shortest_weight = shortest_paths[next_node][1]
				if current_shortest_weight > weight:
					shortest_paths[next_node] = (current_node, weight)
		
		next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
		if not next_destinations:
			return "Route Not Possible"
		# next node is the destination with the lowest weight
		current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
	
	# Work back through destinations in shortest path
	path = []
	while current_node is not None:
		path.append(current_node)
		next_node = shortest_paths[current_node][0]
		current_node = next_node
	# Reverse path
	path = path[::-1]
	return path

# Move the robot from source(sx,sy) in the direction of target(gx,gy)
def move_robot(sx, sy, gx, gy):
	u = dbtp(sx,sy+1,gx,gy)
	ur = dbtp(sx+1,sy+1,gx,gy)
	r = dbtp(sx+1,sy,gx,gy)
	rd = dbtp(sx,sy-1,gx,gy)
	d = dbtp(sx,sy-1,gx,gy)
	dl = dbtp(sx-1,sy-1,gx,gy)
	l = dbtp(sx-1,sy,gx,gy)
	lu = dbtp(sx-1,sy+1,gx,gy)

	m_d = min(u,ur,r,rd,d,dl,l,lu)
	
	if(dbtp(sx,sy+1,gx,gy) == m_d):
		sy = sy + 1
	elif(dbtp(sx+1,sy+1,gx,gy) == m_d):
		sx = sx + 1
		sy = sy + 1
	elif(dbtp(sx+1,sy,gx,gy) == m_d):
		sx = sx + 1
	elif(dbtp(sx+1,sy-1,gx,gy) == m_d):
		sx = sx + 1
		sy = sy - 1
	elif(dbtp(sx,sy-1,gx,gy) == m_d):
		sy = sy - 1
	elif(dbtp(sx-1,sy-1,gx,gy) == m_d):
		sx = sx - 1
		sy = sy - 1
	elif(dbtp(sx-1,sy,gx,gy) == m_d):
		sx = sx - 1
	elif(dbtp(sx-1,sy+1,gx,gy) == m_d):
		sx = sx - 1
		sy = sy + 1
	return sx, sy

# check if direct path from (sx,sy) to (gx,gy) exist or not

def direct_path(sx,sy,gx,gy):
	if(sx == gx and sy == gy):
		return True
	for i in range(len(ox)):
		if((sx == ox[i] and sy == oy[i]) or (gx == ox[i] and gy == oy[i])):
			return False
	while True: #((sx != gx) and (sy != gy))
		sx, sy = move_robot(sx, sy, gx, gy)
		if(sx == gx and sy == gy):
			return True
		for i in range(len(ox)):
			if(ox[i] == sx and oy[i] == sy):
				return False

def main():
	init = time()
	# start and goal position
	sx = -5.0  # [m]
	sy = -5.0  # [m]
	gx = 50.0  # [m]
	gy = 50.0  # [m]
	grid_size = 2.0  # [m]
	robot_radius = 1.0  # [m]
	no_of_fireflies = 5
	# set obstacle positions
	
	for i in range(-10, 60):
		ox.append(i)
		oy.append(-10.0)
	for i in range(-10, 60):
		ox.append(60.0)
		oy.append(i)
	for i in range(-10, 61):
		ox.append(i)
		oy.append(60.0)
	for i in range(-10, 61):
		ox.append(-10.0)
		oy.append(i)
	for i in range(-10, 40):
		ox.append(20.0)
		oy.append(i)
	for i in range(0, 40):
		ox.append(40.0)
		oy.append(60.0 - i)

	for i in range(0, 12):
		ox.append(i)
		oy.append(12.0)
	for i in range(0, 12):
		ox.append(i)
		oy.append(28.0)
	for i in range(0, 12):
		ox.append(i)
		oy.append(40.0)
	for i in range(-2, 35):
		ox.append(i)
		oy.append(50.0)
	for i in range(20, 35):
		ox.append(i)
		oy.append(30.0)
	for i in range(-10, 10):
		ox.append(40.0)
		oy.append(i)
	for i in range(40, 55):
		ox.append(i)
		oy.append(40.0)
	found = False
	print("Finding Path..")
	Final_x = []
	Final_y = []
	path = []
	while not found:
		FF_x = []	# x coordinates of fireflies(random)
		FF_y = []	# y coordinates of fireflies(random)
		FF_x.append(sx)	# insert source
		FF_y.append(sy)	#insert source
		# insert random points
		for _ in range(no_of_fireflies - 2):
			FF_x.append(numpy.random.randint(-9, 59)) # grid(x = -10, y = 60) then randint(x-1, y-1) = (-9, 59)
			FF_y.append(numpy.random.randint(-9, 59))
		FF_x.append(gx) # insert target
		FF_y.append(gy)	#insert target

		#adjacency matrix for graph representation
		rows, cols = (no_of_fireflies, no_of_fireflies)
		
		adj = [[0 for i in range(cols)] for j in range(rows)]
		
		for i in range(no_of_fireflies):
			for j in range(no_of_fireflies):
				if(i == j):
					continue
				if(direct_path(FF_x[i],FF_y[i],FF_x[j],FF_y[j]) and direct_path(FF_x[j],FF_y[j],FF_x[i],FF_y[i])):
					adj[i][j] = dbtp(FF_x[i],FF_y[i],FF_x[j],FF_y[j])

		path = dijkstra(adj, 0, no_of_fireflies - 1)
		if path == "Route Not Possible":
			no_of_fireflies = no_of_fireflies + 10
		else:
			found = True
			Final_x = FF_x.copy()
			Final_y = FF_y.copy()

	# prints firefly locations
	#for i in range(len(Final_x)):
	#		print(i, " = ", Final_x[i], Final_y[i])

	print("Path Found!")
	#print(path)
	for i in range(len(path) - 1):
		print('[', Final_x[path[i]],',', Final_y[path[i]], "] -> ", end = '')
	print("[", Final_x[path[len(path) - 1]],',', Final_y[path[len(path) - 1]], "]")
	print("Execution time = ", time() - init)
	plt.plot(ox, oy, ".k")
	plt.plot(sx, sy, "og")
	plt.plot(gx, gy, "xb")
	for i in range(1, len(Final_x) - 1):
		plt.plot(Final_x[i], Final_y[i], "bo")
	for i in range(len(path) - 1):
		x_vals = [Final_x[path[i]], Final_x[path[i + 1]]]
		y_vals = [Final_y[path[i]], Final_y[path[i + 1]]]
		plt.plot(x_vals, y_vals)
		sx = Final_x[path[i]]
		sy = Final_y[path[i]]
		while(True):
			if(sx == Final_x[path[i + 1]] and sy == Final_y[path[i + 1]]):
				break
			sx, sy = move_robot(sx,sy,Final_x[path[i + 1]],Final_y[path[i + 1]])
			plt.plot(sx, sy, ".g")
			plt.pause(0.005)
	plt.grid(True)
	plt.axis("equal")
	plt.draw()
	plt.show()

if __name__ == '__main__':
	main()