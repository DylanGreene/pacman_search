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

def depthFirstSearch(problem, start=None, path=[]):
    """
    Search the deepest nodes in the search tree first.
    """
    if start == None:
        start = (problem.getStartState(), None, None)
    path = path + [start]

    if problem.isGoalState(start[0]): # base case
        return [x[1] for x in path][1:]
    for n in problem.getSuccessors(start[0]): # build the path
        if n[0] not in [x[0] for x in path]:
            current = depthFirstSearch(problem, n, path)
            if current:
                return current
    return None

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    frontier = []
    frontier.append([(problem.getStartState(), None, None)])

    while len(frontier) > 0: # bfs using a queue
        path = frontier.pop(0)

        if problem.isGoalState(path[-1][0]):
            return [x[1] for x in path][1:]
        for node in problem.getSuccessors(path[-1][0]):
            if node[0] not in [x[0] for x in path]:
                current = list(path)
                current.append(node)
                frontier.append(current)
    return None

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    import heapq

    start = problem.getStartState()
    g = {}
    g[start] = 0
    f = {}
    f[start] = heuristic(start, problem)

    open_set = [] # a priority queue
    heapq.heappush(open_set, (f[start], start))

    closed_set = set()
    came_from = {}

    while open_set:
        current = heapq.heappop(open_set)[1]
        if problem.isGoalState(current):
            path = [] # reconstruct path
            path.append(came_from[current][1])
            while current in came_from and came_from[current][0] != start:
                current = came_from[current][0]
                path.append(came_from[current][1])
            return list(reversed(path))

        closed_set.add(current)
        for neighbor in problem.getSuccessors(current):
            if neighbor[0] in closed_set:
                continue

            # calculate and update costs 
            tentative_g = g[current] + neighbor[2]
            if neighbor[0] in g and tentative_g >= g[neighbor[0]]:
                continue

            came_from[neighbor[0]] = (current, neighbor[1])
            g[neighbor[0]] = tentative_g
            f[neighbor[0]] = g[neighbor[0]] + heuristic(neighbor[0], problem)
            if not open_set or neighbor[0] not in [node for _, node in open_set]:
                heapq.heappush(open_set, (f[neighbor[0]], neighbor[0]))

    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
