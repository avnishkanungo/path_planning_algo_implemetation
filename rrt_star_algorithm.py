from env_creation import generate_random_environment, display_and_save_environment,sampleAPoint, sampleNonObstaclePoint, treeNode
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import random
import math


def env_setup():
    grid_size = (20, 20)  # Grid size (height, width)
    obstacle_density = 0.05  # Density of obstacles


    # Generate random environment
    environment = generate_random_environment(grid_size, obstacle_density)
    # x = sampleNonObstaclePoint(environment)

    return environment

class RRTStarAlgorithm:
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])         #The RRT (root position)
        self.goal = treeNode(goal[0], goal[1])                 #goal position
        self.nearestNode = self.randomTree                                #nearest node            
        self.iterations = min(numIterations, 100)              #number of iterations to run
        self.grid = grid                                       #the map
        self.rho = stepSize                                    #length of each branch 
        self.path_distance = 0                                 #total path distance  
        self.nearestDist = 100                                 #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                  #number of waypoints
        self.Waypoints = []                                    #the waypoints
        self.searchRadius = self.rho*2                          #the radius to search for finding neighbouring vertices 
        self.neighbouringNodes = []                             #neighbouring nodes  
        self.goalArray = np.array([goal[0],goal[1]])            #goal as an array
        self.goalCosts = [10000]                                #the costs to the goal (ignore first value)

    def addChild(self, newNode):
        if (newNode.locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            self.nearestNode.children.append(newNode)
            newNode.parent = self.nearestNode

        #steer a distance stepSize from start location to end location (keep in mind the grid limits) (DONE)
    def steerToPoint(self, locationStart:treeNode, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]-1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0]-1
        return point 

    def isObstacle(self, locationStart:treeNode, locationEnd):
        u_hat = self.unitVector(locationStart,locationEnd)
        val = np.array([0,0])
        for i in range(self.rho):
            val[0] = min(self.grid.shape[1]-1,locationStart.locationX+i*u_hat[0])
            val[1] = min(self.grid.shape[0]-1,locationStart.locationY+i*u_hat[1])
            if self.grid[round(val[1]), round(val[0])] == 1:
                return True
        return False 

    def unitVector(self, locationStart: treeNode, locationEnd):
        v = np.array([locationEnd[0]-locationStart.locationX, locationEnd[1]-locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat  

    def findNearest(self, root, point):
        if not root:
            return
        dist = self.distance(root,point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
        # pass
    
    def findNeighbouringNodes(self,root,point):
        if not root:
            return
        dist = self.distance(root,point) #find distance between root and point (dist)
        if dist<=self.searchRadius: #add root to neighbouringNodes if dist is less than or equal to searchRadius
            self.neighbouringNodes.append(root)
        #recursive call
        for child in root.children:
            self.findNeighbouringNodes(child, point) 

    def distance(self, node1:treeNode, point):
        # dist = np.sqrt((node1.locationX-point[0])**2 + (node1.locationY-point[1])**2)
        dist = np.linalg.norm(np.array([node1.locationX, node1.locationY]) - point)
        return dist

    def goalFound(self,point):
        if self.distance(self.goal,point) <= self.rho:
            return True
        return False
    
    def resetNearestValues(self):
        self.nearestNode = self.randomTree
        self.nearestDist = 100
        self.neighbouringNodes = []
        
    
    # def retraceRRTPath(self,goal):
    #     if goal.locationX == self.randomTree.locationX:
    #         return
    #     self.numWaypoints += 1
    #     self.Waypoints.insert(0,np.array([goal.locationX,goal.locationY]))
    #     self.path_distance += self.rho
    #     self.retraceRRTPath(goal.parent)
    def retracePath(self):
        self.numWaypoints = 0
        self.Waypoints = []
        goalCost = 0
        goal = self.goal
        while goal.locationX != self.randomTree.locationX:
            self.numWaypoints+=1  #add 1 to numWaypoints
            loc = np.array([goal.locationX,goal.locationY]) #extract the X Y location of goal in a numpy array 
            self.Waypoints.insert(0,loc) #insert this array to waypoints (from the beginning)
            goalCost += self.distance(goal,np.array([goal.parent.locationX, goal.parent.locationY])) #add distance between the node and it's parent to goalCost (goalCost keeps increasing)        
            goal = goal.parent  #set the node to it's parent
        self.goalCosts.append(goalCost)  

    #find unique path length from root of a node (cost) (DONE)
    def findPathDistance(self, node):
        costFromRoot = 0
        currentNode = node
        while currentNode.locationX != self.randomTree.locationX:
            costFromRoot += self.distance(currentNode, np.array([currentNode.parent.locationX, currentNode.parent.locationY])) 
            currentNode = currentNode.parent   
        return costFromRoot  

if __name__ == "__main__":
    env = env_setup()
    

    start = sampleNonObstaclePoint(env)
    goal = sampleNonObstaclePoint(env)
    numIterations = 70
    stepSize = 2
    goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

    fig = plt.figure("RRT Algorithm")
    plt.imshow(env, cmap='gray_r', origin='upper')
    # plt.title('Randomized 2D Environment')
    # plt.axis('off')  # Hide axis

    # # Display root, point, and point2
    # plt.scatter(start[0], start[1], c='g', marker='o', s=10)  # green circle for root
    # plt.scatter(goal[0], goal[1], c='r', marker='o', s=10)  # red circle for point
    print(start[0])
    plt.plot(start[0],start[1],'ro')
    plt.plot(goal[0],goal[1],'bo')
    ax = fig.gca()
    ax.add_patch(goalRegion)
    plt.xlabel('X-axis $(m)$')
    plt.ylabel('Y-axis $(m)$')

    # plt.show()

    rrtStar = RRTStarAlgorithm(start, goal, numIterations, env, stepSize)
    plt.pause(2)

    for i in range(rrtStar.iterations):
        rrtStar.resetNearestValues()
        print(f"Iteration number:{i+1}")
        point = sampleAPoint(env)
        # if i == 0:
        #     rrt.findNearest(rrt.randomTree,point)
        # else:
        rrtStar.findNearest(rrtStar.nearestNode,point)
        
        new = rrtStar.steerToPoint(rrtStar.nearestNode,point)

        if not rrtStar.isObstacle(rrtStar.nearestNode,new):
            rrtStar.findNeighbouringNodes(rrtStar.randomTree, new)
            min_cost_node = rrtStar.nearestNode
            # if min_cost_node == rrtStar.randomTree:
            #     min_cost = 0
            # else:
            min_cost = rrtStar.findPathDistance(min_cost_node)
            min_cost =+ rrtStar.distance(rrtStar.nearestNode, new)
            
            for node in rrtStar.neighbouringNodes:
                cost = rrtStar.findPathDistance(node)
                cost+= rrtStar.distance(node,new)
                if not rrtStar.isObstacle(node,new) and cost < min_cost:
                    min_cost_node = node
                    min_cost = cost
            rrtStar.nearestNode = min_cost_node
            newNode = treeNode(new[0],new[1])
            rrtStar.addChild(newNode)
            # plt.pause(0.01)
            plt.plot([rrtStar.nearestNode.locationX,new[0]],[rrtStar.nearestNode.locationY,new[1]],'go',linestyle='--')
            plt.pause(0.05)
            for node in rrtStar.neighbouringNodes:
                cost = min_cost
                cost+=rrtStar.distance(node,new)
                if not rrtStar.isObstacle(node,new) and cost < rrtStar.findPathDistance(node):
                    node.parent = newNode
            
            #if goal found, and the projected cost is lower, then append to path let it sample more (DONE)
            point = np.array([newNode.locationX, newNode.locationY])
            if rrtStar.goalFound(point):
                projectedCost = rrtStar.findPathDistance(newNode) + rrtStar.distance(rrtStar.goal, point)
                if projectedCost < rrtStar.goalCosts[-1]:
                    rrtStar.addChild(rrtStar.goal)
                    plt.plot([rrtStar.nearestNode.locationX, rrtStar.goalArray[0]], [rrtStar.nearestNode.locationY, rrtStar.goalArray[1]],'go', linestyle="--") 
                    #retrace and plot, this method finds waypoints and cost from root
                    rrtStar.retracePath()
                    print("Goal Cost: ", rrtStar.goalCosts)
                    plt.pause(0.25)
                    rrtStar.Waypoints.insert(0,start)
                    #plot the waypoints
                    for i in range(len(rrtStar.Waypoints)-1):
                        plt.plot([rrtStar.Waypoints[i][0], rrtStar.Waypoints[i+1][0]], [rrtStar.Waypoints[i][1], rrtStar.Waypoints[i+1][1]],'ro', linestyle="--")
                        plt.pause(0.01)

    print("Goal Costs: ", rrtStar.goalCosts[1:-1])
       
    # rrt.Waypoints.insert(0,start)
    # print("Number of waypoints: ", rrt.numWaypoints)
    # print("Path Distance (m): ", rrt.path_distance)    
    # print("Waypoints: ", rrt.Waypoints)

    # #plot the waypoints in red (DONE)
    # for i in range(len(rrt.Waypoints)-1):
    #     plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]],'ro', linestyle="--")
    #     plt.pause(0.10)

