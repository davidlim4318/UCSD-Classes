#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: sonia martinez
@modifier: David Lim
@modified: 04/13/25
"""

# Please do not distribute or publish solutions to this
# exercise. You are free to use these problems for educational purposes, please refer to the source.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

from mazemods import maze
from mazemods import makeMaze
from mazemods import collisionCheck
from mazemods import makePath
from mazemods import getPathFromActions
from mazemods import getCostOfActions
from mazemods import stayWestCost
from mazemods import stayEastCost

# State class with coordinate, parent coordinate, and cost2come attributes
class State:
    def __init__(self, coord, parent, cost2come):
        self.coord = coord
        self.parent = parent
        self.cost2come = cost2come

# Depth first search algorithm
def depthFirstSearch(xI,xG,n,m,O):
    """
    Searches the deepest nodes in the search tree first with a depth first search algorithm.
    Returns a list of actions and a path that reaches the goal.  
    Also returns the cost of the path using the getCostOfActions function.
    Also returns the number of visited nodes in the search.
    """
    A = [(-1,0),(1,0),(0,-1),(0,1)]  # checks west, east, south, north (priority reveresed in DFS)
    Q = []  # empty queue list
    V = []  # expanded nodes list (visited nodes list is sum of both lists)
    s = State(xI,xI,None)  # define initial state with parent being itself
    Q.append(s)  # append Q with initial state
    while Q:  # while Q is not empty
        s = Q[-1]  # extract LAST element from Q (last-in-first-out in DFS)
        Q.pop(-1)  # remove element from Q
        V.append(s)  # append V with state to be expanded
        if s.coord == xG:  # if state is goal state
            actions = extract_actions(s,V)  # back calculate action list to goal following lineage
            path = getPathFromActions(xI,actions)  # calculate path from actions
            simplecost = getCostOfActions(xI,actions,O)
            westcost = stayWestCost(xI,actions,O)
            eastcost = stayEastCost(xI,actions,O)
            explored = len(V)
            print_results(actions,path,simplecost,westcost,eastcost,explored)  # print results
            showPath(xI,xG,path[:-1],n,m,O)  # plot path (excluding goal state so green square not hidden)
            plt.show()
            return actions, path, simplecost, westcost, eastcost, explored
        else:  # if state is not goal state
          for a in A:  # iterate through all potential actions
              if not collisionCheck(s.coord,a,O):  # if action is legal
                  s_prime = State(tuple(s.coord[i] + a[i] for i in range(len(s.coord))),s.coord,None)  # define new state
                  if not any(s_prime.coord == s.coord for s in V + Q):  # if state not visited
                    Q.append(s_prime)  # append Q with new state
                    # else resolve parents (step 13 does not apply in DFS)
    explored = len(V)
    print('Path not found!')
    print('Explored: %d' % explored)
    return [], [], [], [], [], explored

def breadthFirstSearch(xI,xG,n,m,O):
    """
    Searches the shallowest nodes in the search tree first [p 85] with a breadth first search algorithm.
    Returns a list of actions and a path that reaches the goal. 
    Also returns the cost of the path using the getCostOfActions function
    Also returns the number of visited nodes in the search.
    """
    A = [(-1,0),(1,0),(0,-1),(0,1)]  # checks west, east, south, north (order doesn't matter in BFS)
    Q = []  # empty queue list
    V = []  # expanded nodes list (visited nodes list is sum of both lists)
    s = State(xI,xI,None)  # define initial state with parent being itself
    Q.append(s)  # append Q with initial state
    while Q:  # while Q is not empty
        s = Q[0]  # extract FIRST element from Q (first-in-first-out in BFS)
        Q.pop(0)  # remove element from Q
        V.append(s)  # append V with state to be expanded
        if s.coord == xG:  # if state is goal state
            actions = extract_actions(s,V)  # back calculate action list to goal following lineage
            path = getPathFromActions(xI,actions)  # calculate path from actions
            simplecost = getCostOfActions(xI,actions,O)
            westcost = stayWestCost(xI,actions,O)
            eastcost = stayEastCost(xI,actions,O)
            explored = len(V)
            print_results(actions,path,simplecost,westcost,eastcost,explored)  # print results
            showPath(xI,xG,path[:-1],n,m,O)  # plot path (excluding goal state so green square not hidden)
            plt.show()
            return actions, path, simplecost, westcost, eastcost, explored
        else:  # if state is not goal state
          for a in A:  # iterate through all potential actions
              if not collisionCheck(s.coord,a,O):  # if action is legal
                  s_prime = State(tuple(s.coord[i] + a[i] for i in range(len(s.coord))),s.coord,None)  # define new state
                  if not any(s_prime.coord == s.coord for s in V + Q):  # if state not visited
                    Q.append(s_prime)  # append Q with new state
                    # else resolve parents (step 13 does not apply in BFS)
    explored = len(V)
    print('Path not found!')
    print('Explored: %d' % explored)
    return [], [], [], [], [], explored

def DijkstraSearch(xI,xG,n,m,O,cost):
    """
    Searches the nodes with least cost first a Dijkstra search algorithm.
    Returns a list of actions and a path that reaches the goal. 
    Also returns the cost of the path using either the stayWestCost or stayEastCost function.
    Also returns the number of visited nodes in the search.
    """
    A = [(-1,0),(1,0),(0,-1),(0,1)]  # checks west, east, south, north (order doesn't matter here)
    Q = []  # empty queue list
    V = []  # expanded nodes list (visited nodes list is sum of both lists)
    s = State(xI,xI,0)  # define initial state with parent being itself
    Q.append(s)  # append Q with initial state
    while Q:  # while Q is not empty
        Q = sorted(Q, key=lambda s: s.cost2come)  # sort queue in order of ascending cost
        s = Q[0]  # extract first element from Q, the lowest cost state
        Q.pop(0)  # remove element from Q
        V.append(s)  # append V with state to be expanded
        if s.coord == xG:  # if state is goal state
            actions = extract_actions(s,V)  # back calculate action list to goal following lineage
            path = getPathFromActions(xI,actions)  # calculate path from actions
            simplecost = getCostOfActions(xI,actions,O)
            westcost = stayWestCost(xI,actions,O)
            eastcost = stayEastCost(xI,actions,O)
            explored = len(V)
            print_results(actions,path,simplecost,westcost,eastcost,explored)  # print results
            print('Calculated cost to come: %d' % s.cost2come)
            showPath(xI,xG,path[:-1],n,m,O)  # plot path (excluding goal state so green square not hidden)
            plt.show()
            return actions, path, simplecost, westcost, eastcost, explored
        else:  # if state is not goal state
          for a in A:  # iterate through all potential actions
              if not collisionCheck(s.coord,a,O):  # if action is legal
                  coord = tuple(s.coord[i] + a[i] for i in range(len(s.coord)))  # compute new state coordinates
                  if cost == 'west':
                      cost2come = s.cost2come + coord[0]**2  # increment cost
                  elif cost == 'east':
                      cost2come = s.cost2come + (n - 1 - coord[0])**2  # increment cost (-1 coordinate adjustment)
                  else:
                      cost2come = s.cost2come + 1  # if no cost function specified, use default cost
                  idx = next((i for i, s in enumerate(Q+V) if s.coord == coord), None)  # search visited states, return index
                  if idx is not None:  # if state was visited
                      if idx + 1 > len(Q):  # if state is in V
                          idx = idx - len(Q)  # update index
                          if cost2come < V[idx].cost2come:  # if cost is lower
                              V[idx].cost2come = cost2come  # update cost
                              V[idx].parent = s.coord  # update parent
                      else:  # if state is in Q
                          if cost2come < Q[idx].cost2come:  # if cost is lower
                              Q[idx].cost2come = cost2come  # update cost
                              Q[idx].parent = s.coord # update parent
                  else:  # if state is unvisited
                      s_prime = State(coord,s.coord,cost2come)  # define new state
                      Q.append(s_prime)  # append Q with new state
    explored = len(V)
    print('Path not found!')
    print('Explored: %d' % explored)
    return [], [], [], [], [], explored

# costsum = costsum + nextx[0]**2
# sorted(student_objects, key=lambda student: student.age)

def nullHeuristic(state,goal):
   """
   A heuristic function estimates the cost from the current state to the nearest
   goal.  This heuristic is trivial.

   """
   return 0

#def aStarSearch(xI,xG,n,m,O,heuristic=nullHeuristic):
"Search the node that has the lowest combined cost and heuristic first."
"""The function uses a function heuristic as an argument. We have used
  the null heuristic here first, you should redefine heuristics as part of 
  the homework. 
  Your algorithm also needs to return the total cost of the path using
  getCostofActions functions. 
  Finally, the algorithm should return the number of visited
  nodes during the search."""
"*** YOUR CODE HERE ***"
  

""""""

# Back calculates actions following lineage
def extract_actions(s,V):
    actions = []  # action list
    # P = [s.coord]
    while s.parent != s.coord:  # while state not initial state
      a = tuple(s.coord[i] - s.parent[i] for i in range(len(s.coord)))  # calculate action from parent to state
      actions.insert(0,a)  # insert action to beginning of list
      s = next((p for p in V if p.coord == s.parent))  # extract parent
      # P.insert(0,s.parent)
    return actions

# Prints all results
def print_results(actions,path,simplecost,westcost,eastcost,explored):
    print(f'Actions: {actions}')
    print(f'Path: {path}')
    print('Basic cost: %d' % simplecost)
    print('Stay west Cost: %d' % westcost)
    print('Stay east cost: %d' % eastcost)
    print('Explored: %d' % explored)

# Plots the path
def showPath(xI,xG,path,n,m,O):
    gridpath = makePath(xI,xG,path,n,m,O)
    fig, ax = plt.subplots(1, 1) # make a figure + axes
    ax.imshow(gridpath) # Plot it
    ax.invert_yaxis() # Needed so that bottom left is (0,0)

if __name__ == '__main__':
    # Run test using maze (loads n,m,O)
    # from smallMaze import *; xG = (20,1)
    # from mediumMaze import *; xG = (34,16)
    from bigMaze import *

    maze(n,m,O); plt.show()  # prints the maze
    
    xI = (1,1)
    
    # depthFirstSearch(xI,xG,n,m,O)
    # breadthFirstSearch(xI,xG,n,m,O)
    # DijkstraSearch(xI,xG,n,m,O,'west')

    '''
    # Sample collision check
    x, u = (5,4), (1,0)
    testObs = [[6,6,4,4]]
    collided = collisionCheck(x,u,testObs)
    print('Collision!' if collided else 'No collision!')
    
    # Sample path plotted to goal
    xI = (1,1)
    xG = (20,1)
    actions = [(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(0,1),
               (1,0),(1,0),(1,0),(0,-1),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0)]
    path = getPathFromActions(xI,actions)
    showPath(xI,xG,path,n,m,O)
    
    # Cost of that path with various cost functions
    simplecost = getCostOfActions(xI,actions,O)
    westcost = stayWestCost(xI,actions,O)
    eastcost = stayEastCost(xI,actions,O)
    print('Basic cost was %d, stay west cost was %d, stay east cost was %d' %
          (simplecost,westcost,eastcost))

    plt.show()
    '''