#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: sonia martinez
@modifier: David Lim
@modified: 04/14/25
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
    def __init__(self, coord, parent, cost2come, cost2go):
        self.coord = coord
        self.parent = parent
        self.cost2come = cost2come
        self.cost2go = cost2go

# Depth first search algorithm
def depthFirstSearch(xI,xG,n,m,O):
    """
    Searches the deepest nodes in the search tree first (a depth first search algorithm).
    Returns a list of actions and a path that reaches the goal.  
    Also returns the cost of the path using the getCostOfActions function.
    Also returns the number of visited nodes in the search.
    """
    A = [(-1,0),(1,0),(0,-1),(0,1)]  # checks west, east, south, north (priority reveresed in DFS)
    Q = []  # empty queue list
    V = []  # expanded nodes list (visited nodes list is sum of both lists)
    s = State(xI, xI, None, None)  # define initial state with parent being itself
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
            queued = len(Q)
            print_results(actions,path,simplecost,westcost,eastcost,explored,queued)  # print results
            showPath(xI,xG,path[:-1],n,m,O)  # plot path (excluding goal state so green square not hidden)
            plt.show()
            return actions, path, simplecost, westcost, eastcost, explored, queued
        else:  # if state is not goal state
          for a in A:  # iterate through all potential actions
              if not collisionCheck(s.coord,a,O):  # if action is legal
                  s_prime = State(tuple(s.coord[i] + a[i] for i in range(len(s.coord))), s.coord, None, None)  # define new state
                  if not any(s_prime.coord == s.coord for s in V + Q):  # if state not visited
                    Q.append(s_prime)  # append Q with new state
                    # else resolve parents (step 13 does not apply in DFS)
    explored = len(V)
    queued = len(Q)
    print_results([],[],[],[],[],explored,queued)
    return [], [], [], [], [], explored, queued

def breadthFirstSearch(xI,xG,n,m,O):
    """
    Searches the shallowest nodes in the search tree first [p 85] (a breadth first search algorithm).
    Returns a list of actions and a path that reaches the goal. 
    Also returns the cost of the path using the getCostOfActions function.
    Also returns the number of visited nodes in the search.
    """
    A = [(-1,0),(1,0),(0,-1),(0,1)]  # checks west, east, south, north (order doesn't matter in BFS)
    Q = []  # empty queue list
    V = []  # expanded nodes list (visited nodes list is sum of both lists)
    s = State(xI, xI, None, None)  # define initial state with parent being itself
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
            queued = len(Q)
            print_results(actions,path,simplecost,westcost,eastcost,explored,queued)  # print results
            showPath(xI,xG,path[:-1],n,m,O)  # plot path (excluding goal state so green square not hidden)
            plt.show()
            return actions, path, simplecost, westcost, eastcost, explored, queued
        else:  # if state is not goal state
          for a in A:  # iterate through all potential actions
              if not collisionCheck(s.coord,a,O):  # if action is legal
                  s_prime = State(tuple(s.coord[i] + a[i] for i in range(len(s.coord))), s.coord, None, None)  # define new state
                  if not any(s_prime.coord == s.coord for s in V + Q):  # if state not visited
                    Q.append(s_prime)  # append Q with new state
                    # else resolve parents (step 13 does not apply in BFS)
    explored = len(V)
    queued = len(Q)
    print_results([],[],[],[],[],explored,queued)
    return [], [], [], [], [], explored

def westCost(coord):  # function that computes cost of tuple, favoring states in the west
    return coord[0]**2

def eastCost(coord):  # function that computes cost of tuple, favoring states in the east
    return (n - 1 - coord[0])**2

def DijkstraSearch(xI,xG,n,m,O,cost=westCost):
    """
    Searches the nodes with least cost first (a Dijkstra search algorithm).
    The function needs a cost function as an argument.
    Returns a list of actions and a path that reaches the goal. 
    Also returns the cost of the path using either the stayWestCost or stayEastCost function.
    Also returns the number of visited nodes in the search.
    """
    A = [(-1,0),(1,0),(0,-1),(0,1)]  # checks west, east, south, north (order doesn't matter here)
    Q = []  # empty queue list
    V = []  # expanded nodes list (visited nodes list is sum of both lists)
    s = State(xI, xI, 0, None)  # define initial state with parent being itself
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
            queued = len(Q)
            print_results(actions,path,simplecost,westcost,eastcost,explored,queued)  # print results
            print('Calculated cost to come: %d' % s.cost2come)
            showPath(xI,xG,path[:-1],n,m,O)  # plot path (excluding goal state so green square not hidden)
            plt.show()
            return actions, path, simplecost, westcost, eastcost, explored, queued
        else:  # if state is not goal state
          for a in A:  # iterate through all potential actions
              if not collisionCheck(s.coord,a,O):  # if action is legal
                  coord = tuple(s.coord[i] + a[i] for i in range(len(s.coord)))  # compute new state coordinates
                  cost2come = s.cost2come + cost(coord)  # increment cost
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
                      s_prime = State(coord, s.coord, cost2come, None)  # define new state
                      Q.append(s_prime)  # append Q with new state
    explored = len(V)
    queued = len(Q)
    print_results([],[],[],[],[],explored,queued)
    return [], [], [], [], [], explored

def nullHeuristic(state,goal):
   """
   A heuristic function estimates the cost from the current state to the nearest goal.  This heuristic is trivial.
   """
   return 0

def manhattanHeuristic(coord,xG):  # computes the manhattan distance between two tuples
   return sum(abs(coord[i] - xG[i]) for i in range(len(coord)))

def euclideanHeuristic(coord,xG):  # computes the euclidean distance between two tuples
   return sum((coord[i] - xG[i])**2 for i in range(len(coord)))**0.5

def aStarSearch(xI,xG,n,m,O,heuristic=nullHeuristic):
    """
    Searches the node that has the lowest combined cost and heuristic first.
    The function uses a function heuristic as an argument.
    Also returns the cost of the path using the getCostOfActions function.
    Also returns the number of visited nodes in the search.
    """
    A = [(-1,0),(1,0),(0,-1),(0,1)]  # checks west, east, south, north (order doesn't matter here)
    Q = []  # empty queue list
    V = []  # expanded nodes list (visited nodes list is sum of both lists)
    cost2go = heuristic(xI,xG)  # compute cost to go of initial state
    s = State(xI, xI, 0, cost2go)  # define initial state with parent being itself
    Q.append(s)  # append Q with initial state
    while Q:  # while Q is not empty
        Q = sorted(Q, key=lambda s: s.cost2come+s.cost2go)  # sort queue in order of ascending sum of costs
        s = Q[0]  # extract first element from Q, the lowest cost sum state
        Q.pop(0)  # remove element from Q
        V.append(s)  # append V with state to be expanded
        if s.coord == xG:  # if state is goal state
            actions = extract_actions(s,V)  # back calculate action list to goal following lineage
            path = getPathFromActions(xI,actions)  # calculate path from actions
            simplecost = getCostOfActions(xI,actions,O)
            westcost = stayWestCost(xI,actions,O)
            eastcost = stayEastCost(xI,actions,O)
            explored = len(V)
            queued = len(Q)
            print_results(actions,path,simplecost,westcost,eastcost,explored,queued)  # print results
            print('Predicted cost to go from the start: %d' % heuristic(xI,xG))
            showPath(xI,xG,path[:-1],n,m,O)  # plot path (excluding goal state so green square not hidden)
            plt.show()
            return actions, path, simplecost, westcost, eastcost, explored, queued
        else:  # if state is not goal state
          for a in A:  # iterate through all potential actions
              if not collisionCheck(s.coord,a,O):  # if action is legal
                  coord = tuple(s.coord[i] + a[i] for i in range(len(s.coord)))  # compute new state coordinates
                  cost2come = s.cost2come + 1  # increment cost to come
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
                      cost2go = heuristic(coord,xG)  # compute cost to go of new state
                      s_prime = State(coord, s.coord, cost2come, cost2go)  # define new state
                      Q.append(s_prime)  # append Q with new state
    explored = len(V)
    queued = len(Q)
    print('Path not found!')
    print('Explored: %d' % explored)
    return [], [], [], [], [], explored, queued

# Back calculates actions to goal following lineage
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
def print_results(actions,path,simplecost,westcost,eastcost,explored,queued):
    print(f'Actions: {actions}')
    print(f'Path: {path}')
    print('Basic cost: %d' % simplecost)
    print('Stay west Cost: %d' % westcost)
    print('Stay east cost: %d' % eastcost)
    print('Explored: %d' % explored)
    print('Queued: %d' % queued)

# Plots the path
def showPath(xI,xG,path,n,m,O):
    gridpath = makePath(xI,xG,path,n,m,O)
    fig, ax = plt.subplots(1, 1) # make a figure + axes
    ax.imshow(gridpath) # Plot it
    ax.invert_yaxis() # Needed so that bottom left is (0,0)

if __name__ == '__main__':
    # Run test using maze (loads n,m,O)
    'Choose a maze:'
    # from smallMaze import *; xG = (20,1)
    from mediumMaze import *; xG = (34,16)
    # from bigMaze import *; xG = (35,35)

    # maze(n,m,O); plt.show()  # prints the maze
    
    xI = (1,1)
    
    'Choose a method:'
    # depthFirstSearch(xI,xG,n,m,O)
    # breadthFirstSearch(xI,xG,n,m,O)
    # DijkstraSearch(xI,xG,n,m,O,westCost)
    # DijkstraSearch(xI,xG,n,m,O,eastCost)
    aStarSearch(xI,xG,n,m,O,manhattanHeuristic)
    # aStarSearch(xI,xG,n,m,O,euclideanHeuristic)