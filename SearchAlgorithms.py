from queue import PriorityQueue
import copy
import collections
import queue
import time
from collections import deque

class Node:
    id = None  # Unique value for each node.
    up = None  # Represents value of neighbors (up, down, left, right).
    down = None
    left = None
    right = None
    checked=False
    previousNode = None  # Represents value of neighbors.
    edgeCost = None  # Represents the cost on the edge from any parent to this node.
    gOfN = None  # Represents the total edge cost
    hOfN = None  # Represents the heuristic value
    heuristicFn = None  # Represents the value of heuristic function
    maze2d = []
    maze2d_str = ""
    x= None
    y= None

    def __init__(self, value, indx1, indx2, maze=None):
        self.maze2d.clear
        self.value = value
        self.x = indx1
        self.y = indx2
        self.id = indx1*10 + indx2
        self.maze2d = maze
        if (maze != None):
            self.maze2dtost()

    def __gt__(self, other):
        if (self.x > other.x):
            return True
        else:
            return False
    #        self.idx()

    # def mazed(self, maze):
    def maze2dtost(self):
        for l in self.maze2d:
            for c in l:
                self.maze2d_str += c
            self.maze2d_str += "\n"
        self.maze2d_str += "\n"


    def idx(self):
        if self.x - 1 >= 0 and self.maze2d[self.x - 1][self.y] != '#':
            # self.up = self.maze2d[self.x - 1][self.y]
            self.up = Node(self.value, self.x - 1, self.y, self.makemove(-1, 0))

        if self.x + 1 < len(self.maze2d) and self.maze2d[self.x + 1][self.y] != '#':
            # self.down = self.maze2d[self.x + 1][self.y]

            self.down = Node(self.value, self.x + 1, self.y, self.makemove(1, 0))

        if self.y - 1 >= 0 and self.maze2d[self.x][self.y - 1] != '#':
            # self.left = self.maze2d[self.x][self.y - 1]
            self.left = Node(self.value, self.x, self.y - 1, self.makemove(0, -1))

        if self.y + 1 < len(self.maze2d[0]) and self.maze2d[self.x][self.y + 1] != '#':
            # self.right = self.maze2d[self.x][self.y + 1]
            self.right = Node(self.value, self.x, self.y + 1, self.makemove(0, 1))

    def makemove(self, m_x, m_y):
        tmpmaze = copy.deepcopy(self.maze2d)
        tmpmaze[self.x + m_x][self.y + m_y], tmpmaze[self.x][self.y] = tmpmaze[self.x][self.y], tmpmaze[self.x + m_x][
            self.y + m_y]
        return tmpmaze


class SearchAlgorithms:
    ''' * DON'T change Class, Function or Parameters Names and Order
        * You can add ANY extra functions,
          classes you need as long as the main
          structure is left as is '''
    path = []  # Represents the correct path from start node to the goal node.
    fullPath = []  # Represents all visited nodes from the start node to the goal node.
    totalCost = -1  # Represents the total cost in case using UCS, AStar (Euclidean or Manhattan)
    maze_str = None
    maze2d = []
    value = []

  ############
    q1_isvisited=set()
    return_Value=None
##############
    def __init__(self, mazeStr, heristicValue=None):
        self.value = heristicValue
        self.maze_str = mazeStr  # q2
        self.mazeStrTo2d()

    def mazeStrTo2d(self):

        # q2
        #    x\y0_1_2_3_4_5_6
        #    0: S . . # . . .
        #    1: . # . . . # .
        #    2: . # . . . . .
        #    3: . . # # . . .
        #    4: # . # E . # .
        # 'S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.'

        self.maze2d.clear()
        maze1d = self.maze_str.split(" ")
        for s in maze1d:
            # print(s)
            self.maze2d.append(s.split(","))

    def getpath(self, startnode, endnode=None):
        route = []
        route.append(self.make_cord(startnode.id))
        while startnode.previousNode !=-1:
            route.append(self.make_cord(startnode.previousNode.id))
            startnode=startnode.previousNode
        route = route[::-1]
        if endnode is not None:
            route.append(self.make_cord(endnode.id))
            while endnode.previousNode != -1:
                route.append(self.make_cord(endnode.previousNode.id))
                endnode = endnode.previousNode

        return route
    def isvalid(self, maze, x, y):
        return not ((x < 0 or x >= len(maze) or y < 0 or y >= len(maze[0])) or (maze[x][y] == '#'))

    def getfullpathbyid(self,startvisited,endvisisted=None):
        T_route=[]
        e_route=[]
        for s in startvisited:
            T_route.append(self.make_cord(s))
        if endvisisted is not None:
            for e in endvisisted:
                e_route.append(self.make_cord(e))
            e_route=e_route[::-1]
            for e2 in e_route:
                T_route.append(e2)
        return T_route
    def make_cord(self,n):
        xycord=[]
        xycord.append(n//10)
        xycord.append(n%10)
        return xycord
    def DLS(self,limit):
        # Fill the correct path in self.path
        # self.fullPath should contain the order of visited nodes
        ######
        ######
        c_s_x = 0
        c_s_y = 0
        ########
        c_e_x = 4
        c_e_y = 3
        self.fullPath.clear()
        self.path.clear()
        StartNode = Node(self.maze2d[c_s_x][c_s_y], c_s_x, c_s_y, self.maze2d)
        StartNode.previousNode = -1
        self.q1_isvisited.add(StartNode.id)
        self.q1_isfinished=-1
        EndNode = Node(self.maze2d[c_e_x][c_e_y], c_e_x, c_e_y, self.maze2d)
        EndNode.previousNode = -1
        res = self.recursiveDLS(StartNode, EndNode, limit)
        if res == "Cut-off":
            return "Cut-off", self.fullPath
        elif self.path.__len__() == 0:
            return "failure", self.fullPath
        else:
            return self.path, self.fullPath



    def recursiveDLS(self, StartNode, EndNode, limit):
        self.fullPath.append(StartNode.maze2d)
        #print(StartNode.maze2d_str)
        StartNode.idx()
        if (StartNode.id == EndNode.id):
            self.path = self.getpath(StartNode)
            self.fullPath=self.getfullpathbyid(self.q1_isvisited)
            self.q1_isfinished=1
            self.return_value= self.path
            return self.return_value
        elif limit == 0:
            self.q1_isfinished=1
            self.return_Value="Cut-off"
            return self.return_Value

        if StartNode.down is not None and StartNode.down.id not in self.q1_isvisited:
            StartNode.down.previousNode = StartNode
            self.q1_isvisited.add( StartNode.down.id)
            if self.q1_isfinished==1:
                return  self.return_Value
            else:
                self.recursiveDLS(StartNode.down, EndNode, limit - 1)

        if StartNode.right is not None and StartNode.right.id not in self.q1_isvisited:
            StartNode.right.previousNode = StartNode
            self.q1_isvisited.add( StartNode.right.id)
            if self.q1_isfinished==1:
                return self.return_Value
            else:
                self.recursiveDLS(StartNode.right, EndNode, limit - 1)

        if StartNode.up is not None and StartNode.up.id not in self.q1_isvisited:
            StartNode.up.previousNode = StartNode
            self.q1_isvisited.add( StartNode.up.id)
            if self.q1_isfinished==1:
                return  self.return_Value
            else:
                self.recursiveDLS(StartNode.up, EndNode, limit - 1)

        if StartNode.left is not None and StartNode.left.id not in self.q1_isvisited:
            StartNode.left.previousNode = StartNode
            self.q1_isvisited.add( StartNode.left.id)
            if self.q1_isfinished==1:
                return  self.return_Value
            else:
                self.recursiveDLS(StartNode.left, EndNode, limit - 1)
    def BDS(self):
        # Fill the correct path in self.path
        # self.fullPath should contain the order of visited nodes
        self.path.clear()
        self.fullPath.clear()
        ######
        c_s_x = 0
        c_s_y = 0
        ########
        c_e_x = 4
        c_e_y = 3

        StartNode = Node(self.maze2d[c_s_x][c_s_y], c_s_x, c_s_y, self.maze2d)
        StartNode.previousNode = -1
        EndNode = Node(self.maze2d[c_e_x][c_e_y], c_e_x, c_e_y, self.maze2d)
        EndNode.previousNode = -1
        start_queue = collections.deque([StartNode])
        start_queue_id=[]
        start_queue_id.append(StartNode.id)

        end_queue = collections.deque([EndNode])
        end_queue_id=[]
        end_queue_id.append(EndNode.id)

        start_visited = set()
        end_visited = set()
        start_visited.add(StartNode.id)
        end_visited.add(EndNode.id)

        while start_queue.__len__() > 0 and end_queue.__len__() > 0:
            if len(start_queue) > 0:

                StartNode = start_queue.popleft()
                start_queue_id.pop(0)
                self.fullPath.append(StartNode.maze2d)
                #print("Startnode:", StartNode.id)
                #print(StartNode.maze2d_str)


                StartNode.idx()
                if StartNode.id == EndNode.id or StartNode.id in end_queue_id:
                    #print("here")
                    if StartNode.id in end_queue_id:
                        for n in end_queue:
                            if n.id == StartNode.id:
                                EndNode = n
                                #self.fullPath.append(self.make_cord(n.id))
                                end_visited.add(n.id)
                                #print(n.maze2d_str)
                                break


                    self.path = self.getpath(StartNode, EndNode)
                    self.fullPath=self.getfullpathbyid(start_visited,end_visited)

                    #for Ns in self.path:
                      #  print(Ns)
                    return self.path, self.fullPath
                # return  # succ break
                if StartNode.up is not None and StartNode.up.id not in start_visited:
                    StartNode.up.previousNode = StartNode
                    start_visited.add(StartNode.up.id)
                    start_queue.append(StartNode.up)
                    start_queue_id.append(StartNode.up.id)
                if StartNode.down is not None and StartNode.down.id not in start_visited:
                    StartNode.down.previousNode = StartNode
                    start_visited.add(StartNode.down.id)
                    start_queue.append(StartNode.down)
                    start_queue_id.append(StartNode.down.id)


                if StartNode.left is not None and StartNode.left.id not in start_visited:
                    StartNode.left.previousNode = StartNode
                    start_visited.add(StartNode.left.id)
                    start_queue.append(StartNode.left)
                    start_queue_id.append(StartNode.left.id)


                if StartNode.right is not None and StartNode.right.id not in start_visited:
                    StartNode.right.previousNode = StartNode
                    start_visited.add(StartNode.right.id)
                    start_queue.append(StartNode.right)
                    start_queue_id.append(StartNode.right.id)


            if len(end_queue) > 0:
                EndNode = end_queue.popleft()
                end_queue_id.pop(0)
                #self.fullPath.append(EndNode.maze2d)

                #print("endnode:", EndNode.id)
                #print(EndNode.maze2d_str)

                if StartNode.id == EndNode.id or EndNode.id in start_queue_id:
                    if EndNode.id in start_queue_id:
                        for n in start_queue_id:
                            if n.id == EndNode.id:
                                StartNode = n
                                start_visited.add(n.id)

                                break
                    #print("here")
                    self.path = self.getpath(StartNode, EndNode)
                    self.fullPath=self.getfullpathbyid(start_visited,end_visited)
                    #for Ns in self.path:
                     #   print(Ns.maze2d_str)
                    return self.path, self.fullPath

                EndNode.idx()
                if EndNode.up is not None and EndNode.up.id not in end_visited:
                    EndNode.up.previousNode = EndNode
                    end_visited.add(EndNode.up.id)
                    end_queue.append(EndNode.up)
                    end_queue_id.append(EndNode.up.id)

                if EndNode.down is not None and EndNode.down.id not in end_visited:
                    EndNode.down.previousNode = EndNode
                    end_visited.add(EndNode.down.id)
                    end_queue.append(EndNode.down)
                    end_queue_id.append(EndNode.down.id)

                if EndNode.left is not None and EndNode.left.id not in end_visited:
                    EndNode.left.previousNode = EndNode
                    end_visited.add(EndNode.left.id)
                    end_queue.append(EndNode.left)
                    end_queue_id.append(EndNode.left.id)

                if EndNode.right is not None and EndNode.right.id not in end_visited:
                    EndNode.right.previousNode = EndNode
                    end_visited.add(EndNode.right.id)
                    end_queue.append(EndNode.right)
                    end_queue_id.append(EndNode.right.id)

        return self.path, self.fullPath

    def BFS(self,source):
        self.path.clear()
        self.fullPath.clear()
        value = self.value
        maze2d = self.maze2d
        fullPath = self.fullPath
        Q = PriorityQueue()

        closed = [[-1 for x in range(len(maze2d[0]))] for y in range(len(maze2d))]
        Q.put((value[source.x][source.y], source))
        x_path = [0, 0, 1, -1]
        y_path = [-1, 1, 0, 0]
        while Q.qsize() > 0:

            curNode = Q.get()

            if closed[curNode[1].x][curNode[1].y] < curNode[0] and closed[curNode[1].x][curNode[1].y] != -1:
                continue

            closed[curNode[1].x][curNode[1].y] = curNode[0]

            if maze2d[curNode[1].x][curNode[1].y] == "E":
                if self.totalCost == -1 or self.totalCost > curNode[0]:
                    self.totalCost = curNode[0]
                continue

            for i in range(4):
                X = curNode[1].x + x_path[i]
                Y = curNode[1].y + y_path[i]
                if (self.isvalid(maze2d, X, Y)):
                    Q.put((curNode[0] + value[X][Y], Node(0,X, Y)))
                    fullPath.append([X, Y])
        return self.fullPath, self.fullPath, self.totalCost


def main():

    print( "x\y0_1_2_3_4_5_6\n0: S . . # . . .\n1: . # . . . # .\n2: . # . . . . .\n3: . . # # . . .\n4: # . # E . # .\n")

    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.')

    path, fullPath = searchAlgo.DLS(50)
    print('**DFS**\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\n\n')

    #######################################################################################
    print( "x\y0_1_2_3_4_5_6\n0: S . . # . . .\n1: . # . . . # .\n2: . # . . . . .\n3: . . # # . . .\n4: # . # E . # .\n")
    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.')
    path, fullPath = searchAlgo.BDS()
    print('**BFS**\nPath is: ' + str(path) + '\nFull Path is: ' + str(fullPath) + '\n\n')

    #######################################################################################
    print( "x\y0_1_2_3_4_5_6\n0: S . . # . . .\n1: . # . . . # .\n2: . # . . . . .\n3: . . # # . . .\n4: # . # E . # .\n")
    searchAlgo = SearchAlgorithms('S,.,.,#,.,.,. .,#,.,.,.,#,. .,#,.,.,.,.,. .,.,#,#,.,.,. #,.,#,E,.,#,.',
                                  [[0, 15, 2, 100, 60, 35, 30],
                                   [3, 100, 2, 15, 60, 100, 30],
                                   [2, 100, 2, 2, 2, 40, 30],
                                   [2, 2, 100, 100, 3, 15, 30],
                                   [100, 2, 100, 0, 2, 100, 30]])
    searchAlgo.path , searchAlgo.fullPath , searchAlgo.totalCost =searchAlgo.BFS(Node(0,0,0))
    print('** UCS **\nfull Path is:')
    print(searchAlgo.fullPath)
    print('Path is:')
    print(searchAlgo.path)
    print ("Total Cost :")
    print(searchAlgo.totalCost)


    #######################################################################################


main()
