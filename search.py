# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """

    # print("Start:", problem.getStartState())
    # print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    # print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    def dfsRec(currentState, visitedNodes, currentPath):
        if currentState in visitedNodes:
            return None
        visitedNodes.append(currentState)
        isGoalState = problem.isGoalState(currentState)
        if isGoalState == True:
            return currentPath
        successors = problem.getSuccessors(currentState)
        for successor in successors:
            cPath = currentPath.copy()
            if successor[1] is not None:
                # print(type(successor[1]))
                cPath.append(successor[1])
                res = dfsRec(successor[0], visitedNodes, cPath)
                if res is not None:
                    return res

    startState = problem.getStartState()

    result = dfsRec(startState, [], [])

    return result

def breadthFirstSearch(problem: SearchProblem):
    q = util.Queue()
    q.push((problem.getStartState(), []))
    visited = {}
    found = False
    while q.isEmpty() is False:
        current = q.pop()
        if problem.isGoalState(current[0]):
            return current[1] 
        if visited.get(current[0]) == True:
            continue
        visited[current[0]] = True
        succs = problem.getSuccessors(current[0])
        for succ in succs:
            resC = current[1].copy()
            resC.append(succ[1])
            q.push((succ[0], resC))
    return None



def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited = set()
    q = util.PriorityQueue()
    q.push((problem.getStartState(), []), 0)

    flag = False
    costs = {}

    while not q.isEmpty():
        flag = False
        node, way = q.pop()
        if problem.isGoalState(node):
            return way
        elif node in visited:
            flag = True
        
        if flag == False:
            visited.add(node)
            for successor, action, stepCost in problem.getSuccessors(node):
                if successor not in visited:
                    if successor not in q.heap:
                        if costs.get(str(way)): 
                            # print("GET COST IS: " + str(costs.get(str(way))) + " + " + str(stepCost))
                            q.push((successor, way + [action]), costs.get(str(way)) + stepCost)
                            costs[str(way + [action])] = costs.get(str(way)) + stepCost
                        else:
                            # print("NOT GETCOST IS: " + str(costs.get(str(way))) +  " + " + str(stepCost))
                            q.push((successor, way + [action]), stepCost)
                            costs[str(way + [action])] = stepCost
    return 0
    


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = set()
    q = util.PriorityQueue()
    q.push((problem.getStartState(), []), 0)

    flag = False
    costs = {}

    while not q.isEmpty():
        flag = False
        node, way = q.pop()
        if problem.isGoalState(node):
            return way
        elif node in visited:
            flag = True
        
        if flag == False:
            visited.add(node)
            for successor, action, stepCost in problem.getSuccessors(node):
                if successor not in visited:
                    if successor not in q.heap:
                        if costs.get(str(way)): 
                            q.push((successor, way + [action]), costs.get(str(way)) + stepCost + heuristic(successor, problem))
                            costs[str(way + [action])] = costs.get(str(way)) + stepCost
                        else:
                            q.push((successor, way + [action]), stepCost + heuristic(successor, problem))
                            costs[str(way + [action])] = stepCost
    return 0

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
