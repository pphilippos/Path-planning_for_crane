#!/bin/env python
# Author: Philippos Papaphilippou
#
# Notes: For crane, which is headless, go to draw_results = True and make it False
#
# Algorithm stages: 1) Dynamic programming for target order (TSP)
#			 		2) A* to avoid obstacles (with heatmap heuristic)


import sys
import copy
import math
import time
sqrt2 = math.sqrt(2)

ntargets = 0
nodes_list = []
obsta_list = []
N=0
obstacle_heatmap = None
min_path = None

def distance(p1,p2):
	return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+0.0)

def in_obstacle_border(y,x):
	for n in obsta_list:
		if (abs(n[0]-y)<=1) and (abs(n[1]-x)<=1):
			return True
	return False 

# distance details from here http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html	 
def diagonal2(p1,p2):
    dx = abs(p1[0]-p2[0])
    dy = abs(p1[1]-p2[1])
    return min(dx, dy)*(sqrt2) + abs(dx - dy)

		
''' # (Depreciated) Recursive / exhaustive - good for upto 10 points
min_length = sys.maxint
def recursive():
	global min_length
	global min_path
	min_path = None
	min_length = sys.maxint
	recu_shortest_ham_path([(0,0)],0)
	return list(min_path)
		
def recu_shortest_ham_path(path,length):
	global min_length
	global min_path
	global ntargets
	current = path[-1]
	
	if len(path)==ntargets+1:
		if length<min_length:
			min_length = length
			min_path = path
		return length
	
	for n in nodes_list:
		if n in path: continue
		
		recu_shortest_ham_path(list(path)+[n],length+distance(current,n))
'''
		
# Based on a Held-Karp implementation here https://github.com/CarlEkerot/held-karp
def dynamic_programming():	
	global min_path
	import itertools

	distances = [[distance (nodes_list[i],nodes_list[j]) for i in range(ntargets)] for j in range(ntargets)]
	subset_costs = {}

	for end in range(ntargets):
		subset_costs[(tuple([end]),end)]=(distance(nodes_list[end],(0,0)),-1,end)
		
	for subset_size in range(2, ntargets+1):
		for subset in itertools.combinations(range(ntargets), subset_size):
			
			for m in subset:
				subset0 = list(subset)
				subset0.remove(m)
				subset0 = tuple(subset0)
				
				min_dist = sys.maxsize
				
				for k in subset0:
					new_dist = subset_costs[(subset0,k)][0]+distances[k][m]
					
					if new_dist<min_dist:
						subset_costs[(subset, m)] = (new_dist, k, m)
						min_dist = new_dist

	min_dist = sys.maxsize
	min_sol = None 
	for end in range(ntargets):
		sol = subset_costs[(tuple(range(len(nodes_list))),end)]
		if sol[0] < min_dist:
			min_dist, min_sol = sol[0], sol
		

	# Trace solution
	solution, nodes = [], list(range(len(nodes_list)))
	while (min_sol[1]!=-1):
		solution.append(min_sol[2])
		nodes.remove(min_sol[2])
		min_sol = subset_costs[(tuple(nodes),min_sol[1])]
	solution.append(min_sol[2])	
	min_path =[(0,0)] + [nodes_list[n] for n in solution[::-1]]
	return list(min_path)

# AI algorithm to avoid obstacles, with heatmap
def a_star(start, goal):

	queue = [(0,[start],0,0)]
	min_cost=dict()
	start_time=time.time()
	
	while (len(queue)!=0) and (time.time()-start_time<5): #5 sec timeout		
		candidate = queue[0]
		queue=queue[1:]

		path = candidate[1]
		current = path[-1]
		length = candidate[2]
		hops = candidate[3]
		y,x = current[0],current[1]

		if current == goal:
			return path
		count=0
		for j in (-1,0,1):
			for i in (-1,0,1):
				if (j,i) == (0,0): continue 
				y0, x0 = y+j,x+i
				n=(y0,x0)
				if (-1 in n) or (N in n) or (n in path): continue
				if not in_obstacle_border(y0,x0):	
					d = diagonal2((y0,x0), goal)				
					
					if n in min_cost:
						if min_cost[n]<d+length:
							continue
					min_cost[n]=d+length
					
					
					# HEURISTICS (subject to fine-tune)	
					score = d*(0.1+0.9*obstacle_heatmap[y][x]) #+ obstacle_heatmap[y][x]*5	
					
					# Insertion sort
					for k in range(len(queue)+1):
						if (k == len(queue)) or (queue[k][0]>=score):
							queue.insert(k, (score,list(path)+[n],length+distance(current,n),hops+1))
							count +=1
							break

def find_result(filename):
	global ntargets
	global nodes_list
	global obsta_list
	global N
	global obstacle_heatmap
	
	nodes_list = []
	obsta_list = []
	
	array = [["-" for i in range(50)] for j in range(50)]
	for line in open(filename):
		line = line.split(",")
		if len(line)==0:continue
		y, x = int(line[0]), int(line[1])
		
		if line[2].strip()=="1":
			array[y][x]='o'
			nodes_list.append((y,x))
		else:
			array[y][x]='x'
			obsta_list.append((y,x))
	N = 50
	
	draw_results = True # If False, no matplotlib or a computer moniter are needed
	debug = False 
	
	ntargets = len(nodes_list)
	
	dynamic_programming() #recursive()
	if debug: print(min_path)
	
	# Build heatmap for heuristics
	obstacle_heatmap = [[0 for i in range(N)] for j in range(N)]
	min_ = sys.maxsize
	max_ = -sys.maxsize
	for y in range(N):
		for x in range(N):
			for n in obsta_list:
				if n!=(y,x):
					obstacle_heatmap[y][x]+=1.0/(distance(n,(y,x))**1.7)
			if (y,x) not in obsta_list:
				if obstacle_heatmap[y][x]>max_:max_=obstacle_heatmap[y][x]
				if obstacle_heatmap[y][x]<min_:min_=obstacle_heatmap[y][x]

	# Normalize heatmap		
	for y in range(N):
		for x in range(N):		
			if (y,x) in obsta_list:
				obstacle_heatmap[y][x]=1
			else:
				obstacle_heatmap[y][x]=(obstacle_heatmap[y][x]-min_)/(max_-min_+0.000001)

	quantized_path=[(0,0)]

	if debug: print("starting A*")
	
	# Run A* for every 2 consecutive targets in the shortest path list
	for n in range(len(min_path)-1):
		if debug: print("%d/%d"%(n, len(min_path)-1))
		start, goal = min_path[n], min_path[n+1]
	
		min_path_=a_star(start,goal)
		if not min_path_ is None:
			quantized_path+=min_path_
		else: 
			quantized_path+=[goal]#min_path[n+1] = min_path[n]
	
	if debug: print("%d/%d"%(len(min_path)-1, len(min_path)-1))

	# Remove unnecessary points 	
	comp_path = [(0,0,0)]
	for n in range(1,len(quantized_path)-1):
		prev, cur, next = quantized_path[n-1], quantized_path[n], quantized_path[n+1]
		if ((prev[0]-cur[0]!=cur[0]-next[0]) or (prev[1]-cur[1]!=cur[1]-next[1])) and (prev!=cur):
			if cur in nodes_list:
				comp_path.append(tuple(list(cur)+[1]))
			else:
				comp_path.append(tuple(list(cur)+[0]))
	comp_path.append(tuple(list(quantized_path[-1])+[1]))
	
	if draw_results:
		import matplotlib.pyplot as plt
		import numpy as np
			
		plt.xlabel('x')
		plt.ylabel('y')
		plt.title('')
		plt.plot(np.array(min_path).T[0], np.array(min_path).T[1], 'r--' , label="best path (TSP)")
		plt.plot(np.array(comp_path).T[0], np.array(comp_path).T[1], "b-", label="A* avoid obstacles")
		plt.plot(np.array(nodes_list).T[0], np.array(nodes_list).T[1], "bo", mfc='none', label="targets")
		plt.plot(np.array(obsta_list).T[0], np.array(obsta_list).T[1], "rx", label="obstacles")
		plt.grid(True)

		fig = plt.gcf();
		ax = fig.add_subplot(1,1,1)   
		major_ticks = np.arange(0, N+1, 10)                                              
		minor_ticks = np.arange(0, N+1, 1)                                               
		ax.set_xticks(minor_ticks, minor=True)                                           
		ax.set_yticks(minor_ticks, minor=True)
		ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)      

		plt.grid(which='minor',color='#dddddd', linestyle=':', linewidth=1)

		plt.ylim([0,N])
		plt.xlim([0,N])

		ax = fig.add_subplot(1,1,1) 
		import matplotlib.patches as patches

		# Paint Heatmap
		for y in range(N):
			for x in range(N):
				ax.add_patch( patches.Rectangle( (y-0.5,x-0.5), 1, 1, fill=True, linewidth=0,facecolor="#106d00", alpha=obstacle_heatmap[y][x]))

		# White boxes for targets and obstacles			
		for n in nodes_list+obsta_list:
			ax.add_patch( patches.Rectangle( (n[0]-1,n[1]-1), 2, 2, fill=True,facecolor="#ffffff",edgecolor="#000000"))

		plt.savefig("picture_csv.svg")
		plt.show()
		
	return comp_path[1:]

print(find_result("course_.csv"))
