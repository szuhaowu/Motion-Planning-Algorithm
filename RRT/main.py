# Include here your student identification number e.g. A123456
import matplotlib.pyplot as plt
import numpy as np
import math
from auxiliary_functions import inCollision

# node class which has the x and y position along with the parent
# node index
class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

# RRT class implementation
class RRT():
    def __init__(self, start, goal, obstacle_list):
        self.start = Node(start[0], start[1])  # start node for the RRT
        self.goal = Node(goal[0], goal[1])     # goal node for the RRT
        self.obstacle_list = obstacle_list     # list of obstacles 
        self.node_list = []    # list of nodes added while creating the RRT


    def planning(self, animation=True):
        self.node_list = [self.start]
        while self.goal.parent is None:
            # Random Sampling
            # We are choosing the goal node with 0.1 probability, this
            # gives a bias to RRT to search towards the goal. Increasing
            # the bias may take longer time to converge to goal if the 
            # path has lot of obstacles in its path. Tune this parameter to
            # see the differences 
            if np.random.rand() > 0.1: 
                random_point = np.random.sample((2, 1))*10.1  #0~10.1
            else:
                random_point = np.asarray([self.goal.x, self.goal.y],
                                          dtype=float)
            
        # once the goal node has a parent this means the tree has a path
        # to the start node.
        # Edit below this line at your own risk. This will take care of creating a
        # path from goal to start. 
            #TODO
            nearest_index = self.getNearestNode(random_point)
            random_point = self.getQnearby(random_point,self.node_list[nearest_index])
            isCollision = inCollision(self.node_list[nearest_index],random_point,obstacle_list)
            if isCollision:
                continue
            else:
                if random_point[0] == self.goal.x and random_point[1] == self.goal.y:
                    print('goal')
                    self.connectNewNodeToNearestFreeNode(self.goal,nearest_index)
                    self.node_list.append(self.goal)
                    if animation:
                        self.drawGraph(random_point)
                else:
                    
                    # creating a node from the point
                    new_node = Node(random_point[0], random_point[1])
                    # set the parent as index no of the node in the self.node_list
                    new_node.parent = 0 # setting the parent of new node to start node 
                                        # as 0 refers to the first node in self.node_list
                    self.connectNewNodeToNearestFreeNode(new_node,nearest_index)
                    self.node_list.append(new_node) # storing the nodes in a list
                    if animation:
                        self.drawGraph(random_point)
        
        
        path = [[self.goal.x, self.goal.y]]
        prev_node_index = len(self.node_list) - 1
        while self.node_list[prev_node_index].parent is not None:
            node = self.node_list[prev_node_index]
            path.append([node.x, node.y])
            prev_node_index = node.parent
        path.append([self.start.x, self.start.y])
        path.pop(0)

        return path

    # input: node as defined by node class
    # input: point defined as a list (x,y)
    # output: distance between the node and point
    def calcDistNodeToPoint(self, node, point):
        # TODO: implement a method to calculate distance
        pointx,pointy = point[0],point[1]
        return math.sqrt(math.pow(pointx-node.x,2)+math.pow(pointy-node.y,2))

    # input: random_point which you sampled as (x,y)
    # output: index of the node in self.node_list
    def getNearestNode(self, random_point):
        # TODO: implement a method to find the closest node to the random_point
        diff = np.zeros((len(self.node_list),))
        pos = [random_point[0],random_point[1]]
        for i in range(len(self.node_list)):
            diff[i] = self.calcDistNodeToPoint(self.node_list[i],pos)
        nearest_index = np.argmin(diff)
        return nearest_index
    
    #TODO
    def getQnearby(self, random_point, node):
        new = random_point
        pos = [random_point[0],random_point[1]]
        if self.calcDistNodeToPoint(node,pos) > 0.25:    
            ratio = 0.25/self.calcDistNodeToPoint(node,pos)
            diffx,diffy = random_point[0]-node.x,random_point[1]-node.y
            new[0] = node.x + ratio*diffx
            new[1] = node.y + ratio*diffy  
        return new

    
    #TODO
    def connectNewNodeToNearestFreeNode(self, new_node,nearest_index):
        new_node.parent = nearest_index
        

    # edit this function at your own risk 
    def drawGraph(self, random_point=None):
        plt.clf()
        # draw random point
        if random_point is not None:
            plt.plot(random_point[0], random_point[1], '.')
        # draw the tree
        for node in self.node_list:
            if node.parent is not None:
                plt.plot([node.x, self.node_list[node.parent].x], [
                         node.y, self.node_list[node.parent].y], "-g")
        # draw the obstacle
        for obstacle in self.obstacle_list:
            obstacle_draw = plt.Polygon(obstacle, fc="b")
            plt.gca().add_patch(obstacle_draw)
        # draw the start and goal points
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis([0, 10, 0, 10])
        plt.grid(True)
        plt.pause(0.01)
        
    

# You can define any other methods in for the rrt object (i.e. functions
# that apply to the rrt object attributes) and which can help you
# complete the assignment, for example to obtain q nearby

#
#if __name__ == '__main__':
#    # Define obstacle polygon in the counter clockwise direction
#    obstacle_list = [[[3, 3], [4, 3], [4, 4], [3, 4]],
#                     [[8, 8], [7, 6], [9, 9]]]
#    # Set Initial parameters
#    rrt = RRT(start=[2, 2], goal=[5, 5], obstacle_list=obstacle_list)
#    show_animation = True
#    path = rrt.planning(animation=show_animation)
#    # Draw final path
#    if show_animation:
#        rrt.drawGraph()
#        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
#        plt.grid(True)
#        plt.show()
